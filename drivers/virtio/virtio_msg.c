// SPDX-License-Identifier: GPL-2.0+
/*
 * Virtio message transport.
 *
 * Copyright (C) 2025 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * The virtio-msg transport encapsulates virtio operations as discrete message
 * exchanges rather than relying on PCI or memory-mapped I/O regions. It
 * separates bus-level functionality (e.g., device enumeration, hotplug events)
 * from device-specific operations (e.g., feature negotiation, virtqueue setup),
 * ensuring that a single, generic transport layer can be reused across multiple
 * bus implementations (like ARM Firmware Framework (FF-A), IPC, etc.).
 *
 * This file implements the generic Virtio message transport layer.
 */

#define pr_fmt(fmt) "virtio-msg: " fmt

#include <linux/err.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ring.h>
#include <uapi/linux/virtio_msg.h>

#include "virtio_msg_internal.h"

#define to_virtio_msg_device(_dev)	\
	container_of(_dev, struct virtio_msg_device, vdev)

static void msg_prepare(struct virtio_msg *vmsg, bool bus, u8 msg_id,
			u16 dev_id, u16 payload_size)
{
	u16 size = sizeof(*vmsg) + payload_size;

	memset(vmsg, 0, size);

	if (bus) {
		vmsg->type = VIRTIO_MSG_TYPE_BUS;
	} else {
		vmsg->type = VIRTIO_MSG_TYPE_TRANSPORT;
		vmsg->dev_id = cpu_to_le16(dev_id);
	}

	vmsg->msg_id = msg_id;
	vmsg->msg_size = cpu_to_le16(size);
}

static void transport_msg_prepare(struct virtio_msg_device *vmdev, u8 msg_id,
				  u16 payload_size)
{
	msg_prepare(vmdev->request, false, msg_id, vmdev->dev_id, payload_size);
}

/**
 * virtio_msg_prepare - Initialize a virtio message for bus transfer
 * @vmsg: Pointer to the virtio message structure to initialize
 * @msg_id: Message ID to set in the message header
 * @payload_size: Size of the payload (in bytes)
 *
 * Prepares a virtio_msg structure for transmission over the bus transport
 * (VIRTIO_MSG_TYPE_BUS). The payload buffer follows the header.
 */
void virtio_msg_prepare(struct virtio_msg *vmsg, u8 msg_id, u16 payload_size)
{
	msg_prepare(vmsg, true, msg_id, 0, payload_size);
}
EXPORT_SYMBOL_GPL(virtio_msg_prepare);

static int virtio_msg_xfer(struct virtio_msg_device *vmdev)
{
	int ret;

	memset(vmdev->response, 0, vmdev->msg_size);

	ret = vmdev->ops->transfer(vmdev, vmdev->request, vmdev->response);
	if (ret)
		dev_err(&vmdev->vdev.dev, "Transfer request failed (%d)\n", ret);

	return ret;
}

static inline int virtio_msg_send(struct virtio_msg_device *vmdev)
{
	int ret;

	ret = vmdev->ops->transfer(vmdev, vmdev->request, NULL);
	if (ret)
		dev_err(&vmdev->vdev.dev, "Send request failed (%d)\n", ret);

	return ret;
}

static int virtio_msg_get_device_info(struct virtio_msg_device *vmdev)
{
	struct get_device_info_resp *payload = virtio_msg_payload(vmdev->response);
	struct virtio_device *vdev = &vmdev->vdev;
	u32 num_feature_bits;
	int ret;

	static_assert(sizeof(*vmdev->response) + sizeof(*payload) <
		      VIRTIO_MSG_MIN_SIZE);

	transport_msg_prepare(vmdev, VIRTIO_MSG_DEVICE_INFO, 0);

	ret = virtio_msg_xfer(vmdev);
	if (ret)
		return ret;

	vdev->id.device = le32_to_cpu(payload->device_id);
	if (vdev->id.device == 0) {
		/*
		 * A virtio device with ID 0 is a placeholder and indicates no
		 * valid device function is present.
		 */
		return -ENODEV;
	}

	vdev->id.vendor = le32_to_cpu(payload->vendor_id);
	vmdev->config_size = le32_to_cpu(payload->config_size);
	num_feature_bits = le32_to_cpu(payload->num_feature_bits);

	/* Linux supports 64 feature bits */
	if (num_feature_bits != 64) {
		dev_err(&vdev->dev, "Incompatible num_feature_bits (%u)\n",
			num_feature_bits);
		return -EINVAL;
	}

	return 0;
}

static u64 virtio_msg_get_features(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct get_features *req_payload = virtio_msg_payload(vmdev->request);
	struct get_features_resp *res_payload = virtio_msg_payload(vmdev->response);
	__le32 *features;
	int ret;

	static_assert(sizeof(*vmdev->request) + sizeof(*req_payload) <
		      VIRTIO_MSG_MIN_SIZE);
	static_assert(sizeof(*vmdev->response) + sizeof(*res_payload) <
		      VIRTIO_MSG_MIN_SIZE);

	transport_msg_prepare(vmdev, VIRTIO_MSG_GET_DEV_FEATURES,
			      sizeof(*req_payload));

	/* Linux supports 64 feature bits */
	req_payload->num = cpu_to_le32(2);
	req_payload->index = 0;

	ret = virtio_msg_xfer(vmdev);
	if (ret)
		return 0;

	features = (__le32 *)res_payload->features;
	return ((u64)(le32_to_cpu(features[1])) << 32) | le32_to_cpu(features[0]);
}

static int virtio_msg_finalize_features(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct set_features *payload = virtio_msg_payload(vmdev->request);
	__le32 *features = (__le32 *)payload->features;

	static_assert(sizeof(*vmdev->request) + sizeof(*payload) <
		      VIRTIO_MSG_MIN_SIZE);

	/* Give virtio_ring a chance to accept features */
	vring_transport_features(vdev);

	transport_msg_prepare(vmdev, VIRTIO_MSG_SET_DRV_FEATURES, sizeof(*payload));

	/* Linux supports 64 feature bits */
	payload->num = cpu_to_le32(2);
	payload->index = 0;

	features[0] = cpu_to_le32((u32)vmdev->vdev.features);
	features[1] = cpu_to_le32(vmdev->vdev.features >> 32);

	return virtio_msg_xfer(vmdev);
}

static void virtio_msg_get(struct virtio_device *vdev, unsigned int offset,
			   void *buf, unsigned int len)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct get_config *req_payload = virtio_msg_payload(vmdev->request);
	struct get_config_resp *res_payload = virtio_msg_payload(vmdev->response);
	u32 i = 0, max;

	static_assert(sizeof(*vmdev->request) + sizeof(*req_payload) <
		      VIRTIO_MSG_MIN_SIZE);
	static_assert(sizeof(*vmdev->response) + sizeof(*res_payload) <
		      VIRTIO_MSG_MIN_SIZE);

	if (offset + len > vmdev->config_size) {
		dev_err(&vmdev->vdev.dev,
			"Invalid config read operation: %u: %u: %u\n", offset,
			len, vmdev->config_size);
		return;
	}

	/* Maximum payload size available in the response message buffer */
	max = vmdev->msg_size - sizeof(*vmdev->response) - sizeof(*res_payload);

	transport_msg_prepare(vmdev, VIRTIO_MSG_GET_CONFIG, sizeof(*req_payload));

	while (i != len) {
		u32 size = min(max, len - i);

		req_payload->offset = cpu_to_le32(offset + i);
		req_payload->size = cpu_to_le32(size);

		if (virtio_msg_xfer(vmdev))
			return;

		/* Buffer holds the data in little endian */
		memcpy(buf + i, res_payload->config, size);
		i += size;
	}

	vmdev->generation_count = le32_to_cpu(res_payload->generation);
}

static void virtio_msg_set(struct virtio_device *vdev, unsigned int offset,
			   const void *buf, unsigned int len)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct set_config *payload = virtio_msg_payload(vmdev->request);
	u32 i = 0, max;

	static_assert(sizeof(*vmdev->request) + sizeof(*payload) <
		      VIRTIO_MSG_MIN_SIZE);

	if (offset + len > vmdev->config_size) {
		dev_err(&vmdev->vdev.dev,
			"Invalid config write operation: %u: %u: %u\n", offset,
			len, vmdev->config_size);
		return;
	}

	/* Maximum payload size available in the request message buffer */
	max = vmdev->msg_size - sizeof(*vmdev->request) - sizeof(*payload);

	transport_msg_prepare(vmdev, VIRTIO_MSG_SET_CONFIG, sizeof(*payload));
	payload->generation = cpu_to_le32(vmdev->generation_count);

	while (i != len) {
		u32 size = min(max, len - i);

		payload->offset = cpu_to_le32(offset + i);
		payload->size = cpu_to_le32(size);

		/* Buffer holds the data in little endian */
		memcpy(payload->config, buf + i, size);
		i += size;

		if (virtio_msg_xfer(vmdev))
			return;
	}
}

static u32 virtio_msg_generation(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct get_config *req_payload = virtio_msg_payload(vmdev->request);
	struct get_config_resp *res_payload = virtio_msg_payload(vmdev->response);

	transport_msg_prepare(vmdev, VIRTIO_MSG_GET_CONFIG, sizeof(*req_payload));
	req_payload->offset = cpu_to_le32(0);
	req_payload->size = cpu_to_le32(0);

	if (virtio_msg_xfer(vmdev))
		return 0;

	vmdev->generation_count = le32_to_cpu(res_payload->generation);
	return vmdev->generation_count;
}

static u8 virtio_msg_get_status(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct get_device_status_resp *payload = virtio_msg_payload(vmdev->response);

	static_assert(sizeof(*vmdev->response) + sizeof(*payload) <
		      VIRTIO_MSG_MIN_SIZE);

	transport_msg_prepare(vmdev, VIRTIO_MSG_GET_DEVICE_STATUS, 0);

	if (virtio_msg_xfer(vmdev))
		return 0;

	return (u8)le32_to_cpu(payload->status);
}

static void virtio_msg_set_status(struct virtio_device *vdev, u8 status)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	struct set_device_status *payload = virtio_msg_payload(vmdev->request);

	static_assert(sizeof(*vmdev->request) + sizeof(*payload) <
		      VIRTIO_MSG_MIN_SIZE);

	transport_msg_prepare(vmdev, VIRTIO_MSG_SET_DEVICE_STATUS, sizeof(*payload));
	payload->status = cpu_to_le32(status);

	if (virtio_msg_xfer(vmdev))
		dev_warn(&vmdev->vdev.dev, "Set status %u failed\n", status);
}

static void virtio_msg_vq_reset(struct virtqueue *vq)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vq->vdev);
	struct reset_vqueue *payload = virtio_msg_payload(vmdev->request);

	static_assert(sizeof(*vmdev->request) + sizeof(*payload) <
		      VIRTIO_MSG_MIN_SIZE);

	transport_msg_prepare(vmdev, VIRTIO_MSG_RESET_VQUEUE, sizeof(*payload));
	payload->index = cpu_to_le32(vq->index);

	if (virtio_msg_xfer(vmdev))
		dev_warn(&vmdev->vdev.dev, "Virtqueue reset failed\n");
}

static void virtio_msg_reset(struct virtio_device *vdev)
{
	/* Setting status to 0 initiates a device reset */
	virtio_msg_set_status(vdev, 0);
}

static bool _vmsg_notify(struct virtqueue *vq, u32 index, u32 offset, u32 wrap)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vq->vdev);
	struct event_avail *payload = virtio_msg_payload(vmdev->request);
	u32 val;

	static_assert(sizeof(*vmdev->request) + sizeof(*payload) <
		      VIRTIO_MSG_MIN_SIZE);

	transport_msg_prepare(vmdev, VIRTIO_MSG_EVENT_AVAIL, sizeof(*payload));
	payload->index = cpu_to_le32(index);

	val = offset & ((1U << VIRTIO_MSG_EVENT_AVAIL_WRAP_SHIFT) - 1);
	val |= wrap & (1U << VIRTIO_MSG_EVENT_AVAIL_WRAP_SHIFT);
	payload->next_offset_wrap = cpu_to_le32(val);

	return !virtio_msg_send(vmdev);
}

static bool virtio_msg_notify(struct virtqueue *vq)
{
	return _vmsg_notify(vq, vq->index, 0, 0);
}

static bool virtio_msg_notify_with_data(struct virtqueue *vq)
{
	u32 index, offset, wrap, data = vring_notification_data(vq);

	index = data | 0xFFFF;
	data >>= 16;
	offset = data | 0x7FFF;
	wrap = data >> 15;

	return _vmsg_notify(vq, index, offset, wrap);
}

/**
 * virtio_msg_event - Handle an incoming event message for a virtio message device.
 * @vmdev: Pointer to the virtio message device.
 * @vmsg: Pointer to the incoming virtio message containing the event.
 *
 * Processes an event message received from the bus layer for the given
 * virtio message device. Depending on the message type, this function either:
 *
 *  - Triggers the device configuration change handler if the message is of type
 *    VIRTIO_MSG_EVENT_CONFIG.
 *
 *  - Delivers a used-buffer notification to the corresponding virtqueue if the
 *    message is of type VIRTIO_MSG_EVENT_USED.
 *
 * Return: 0 on success, -EIO if the interrupt handler fails, -EINVAL for
 * unknown message types or invalid virtqueue indices.
 */
int virtio_msg_event(struct virtio_msg_device *vmdev, struct virtio_msg *vmsg)
{
	struct event_used *payload = virtio_msg_payload(vmsg);
	struct device *dev = &vmdev->vdev.dev;
	struct virtqueue *vq;
	unsigned int index;

	if (vmsg->msg_id == VIRTIO_MSG_EVENT_CONFIG) {
		virtio_config_changed(&vmdev->vdev);
		return 0;
	}

	if (vmsg->msg_id == VIRTIO_MSG_EVENT_USED) {
		index = le32_to_cpu(payload->index);

		virtio_device_for_each_vq(&vmdev->vdev, vq) {
			if (index == vq->index) {
				if (vring_interrupt(0, vq) != IRQ_HANDLED)
					return -EIO;

				return 0;
			}
		}

		dev_err(dev, "Failed to find virtqueue (%u)", index);
	} else {
		dev_err(dev, "Unexpected message id: (%u)\n", vmsg->msg_id);
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(virtio_msg_event);

static void virtio_msg_del_vqs(struct virtio_device *vdev)
{
	struct virtqueue *vq, *n;

	list_for_each_entry_safe(vq, n, &vdev->vqs, list) {
		virtio_msg_vq_reset(vq);
		vring_del_virtqueue(vq);
	}
}

static int virtio_msg_vq_get(struct virtio_msg_device *vmdev, unsigned int *num,
			     unsigned int index)
{
	struct get_vqueue *req_payload = virtio_msg_payload(vmdev->request);
	struct get_vqueue_resp *res_payload = virtio_msg_payload(vmdev->response);
	int ret;

	static_assert(sizeof(*vmdev->request) + sizeof(*req_payload) <
		      VIRTIO_MSG_MIN_SIZE);
	static_assert(sizeof(*vmdev->response) + sizeof(*res_payload) <
		      VIRTIO_MSG_MIN_SIZE);

	transport_msg_prepare(vmdev, VIRTIO_MSG_GET_VQUEUE, sizeof(*req_payload));
	req_payload->index = cpu_to_le32(index);

	ret = virtio_msg_xfer(vmdev);
	if (ret)
		return ret;

	*num = le32_to_cpu(res_payload->max_size);
	if (!*num)
		return -ENOENT;

	return 0;
}

static int virtio_msg_vq_set(struct virtio_msg_device *vmdev,
			     struct virtqueue *vq, unsigned int index)
{
	struct set_vqueue *payload = virtio_msg_payload(vmdev->request);

	static_assert(sizeof(*vmdev->request) + sizeof(*payload) <
		      VIRTIO_MSG_MIN_SIZE);

	transport_msg_prepare(vmdev, VIRTIO_MSG_SET_VQUEUE, sizeof(*payload));
	payload->index = cpu_to_le32(index);
	payload->size = cpu_to_le32(virtqueue_get_vring_size(vq));
	payload->descriptor_addr = cpu_to_le64(virtqueue_get_desc_addr(vq));
	payload->driver_addr = cpu_to_le64(virtqueue_get_avail_addr(vq));
	payload->device_addr = cpu_to_le64(virtqueue_get_used_addr(vq));

	return virtio_msg_xfer(vmdev);
}

static struct virtqueue *
virtio_msg_setup_vq(struct virtio_msg_device *vmdev, unsigned int index,
		    void (*callback)(struct virtqueue *vq), const char *name,
		    bool ctx)
{
	bool (*notify)(struct virtqueue *vq);
	struct virtqueue *vq;
	unsigned int num;
	int ret;

	if (__virtio_test_bit(&vmdev->vdev, VIRTIO_F_NOTIFICATION_DATA))
		notify = virtio_msg_notify_with_data;
	else
		notify = virtio_msg_notify;

	ret = virtio_msg_vq_get(vmdev, &num, index);
	if (ret)
		return ERR_PTR(ret);

	vq = vring_create_virtqueue(index, num, PAGE_SIZE, &vmdev->vdev, true,
				    true, ctx, notify, callback, name);
	if (!vq)
		return ERR_PTR(-ENOMEM);

	vq->num_max = num;

	ret = virtio_msg_vq_set(vmdev, vq, index);
	if (ret) {
		vring_del_virtqueue(vq);
		return ERR_PTR(ret);
	}

	return vq;
}

static int virtio_msg_find_vqs(struct virtio_device *vdev, unsigned int nvqs,
			       struct virtqueue *vqs[],
			       struct virtqueue_info vqs_info[],
			       struct irq_affinity *desc)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);
	int i, queue_idx = 0;

	for (i = 0; i < nvqs; ++i) {
		struct virtqueue_info *vqi = &vqs_info[i];

		if (!vqi->name) {
			vqs[i] = NULL;
			continue;
		}

		vqs[i] = virtio_msg_setup_vq(vmdev, queue_idx++, vqi->callback,
					     vqi->name, vqi->ctx);
		if (IS_ERR(vqs[i])) {
			virtio_msg_del_vqs(vdev);
			return PTR_ERR(vqs[i]);
		}
	}

	return 0;
}

static const char *virtio_msg_bus_name(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);

	return vmdev->bus_name;
}

static void virtio_msg_synchronize_cbs(struct virtio_device *vdev)
{
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);

	vmdev->ops->synchronize_cbs(vmdev);
}

static void virtio_msg_release_dev(struct device *_d)
{
	struct virtio_device *vdev =
			container_of(_d, struct virtio_device, dev);
	struct virtio_msg_device *vmdev = to_virtio_msg_device(vdev);

	if (vmdev->ops->release)
		vmdev->ops->release(vmdev);
}

static struct virtio_config_ops virtio_msg_config_ops = {
	.get			= virtio_msg_get,
	.set			= virtio_msg_set,
	.generation		= virtio_msg_generation,
	.get_status		= virtio_msg_get_status,
	.set_status		= virtio_msg_set_status,
	.reset			= virtio_msg_reset,
	.find_vqs		= virtio_msg_find_vqs,
	.del_vqs		= virtio_msg_del_vqs,
	.get_features		= virtio_msg_get_features,
	.finalize_features	= virtio_msg_finalize_features,
	.bus_name		= virtio_msg_bus_name,
};

/**
 * virtio_msg_register - Register a virtio message device with the virtio core
 * @vmdev: Pointer to the virtio message device to register
 *
 * Initializes and registers a virtio message device.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int virtio_msg_register(struct virtio_msg_device *vmdev)
{
	u32 version;
	int ret;

	if (!vmdev || !vmdev->ops || !vmdev->ops->bus_info ||
	    !vmdev->ops->transfer)
		return -EINVAL;

	vmdev->bus_name = vmdev->ops->bus_info(vmdev, &vmdev->msg_size,
					       &version);
	if (version != VIRTIO_MSG_REVISION_1 ||
	    vmdev->msg_size < VIRTIO_MSG_MIN_SIZE)
		return -EINVAL;

	/*
	 * Allocate per-device request and response buffers, each of size
	 * `msg_size`.
	 *
	 * Since requests are serialized per device, one pair of buffers
	 * suffices.
	 */
	vmdev->request = kzalloc(2 * vmdev->msg_size, GFP_KERNEL);
	if (!vmdev->request)
		return -ENOMEM;

	vmdev->response = (void *)vmdev->request + vmdev->msg_size;

	vmdev->vdev.config = &virtio_msg_config_ops;
	vmdev->vdev.dev.release = virtio_msg_release_dev;

	if (vmdev->ops->synchronize_cbs)
		virtio_msg_config_ops.synchronize_cbs = virtio_msg_synchronize_cbs;

	ret = virtio_msg_get_device_info(vmdev);
	if (ret) {
		if (vmdev->ops->release)
			vmdev->ops->release(vmdev);
		goto free;
	}

	ret = register_virtio_device(&vmdev->vdev);
	if (ret) {
		put_device(&vmdev->vdev.dev);
		goto free;
	}

	return 0;

free:
	kfree(vmdev->request);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_register);

/**
 * virtio_msg_unregister - Unregister a virtio message device
 * @vmdev: Pointer to the virtio message device to unregister
 *
 * Unregisters a previously registered virtio message device from the virtio
 * core and releases associated resources.
 */
void virtio_msg_unregister(struct virtio_msg_device *vmdev)
{
	unregister_virtio_device(&vmdev->vdev);
	kfree(vmdev->request);
}
EXPORT_SYMBOL_GPL(virtio_msg_unregister);

MODULE_AUTHOR("Viresh Kumar <viresh.kumar@linaro.org>");
MODULE_DESCRIPTION("Virtio message transport");
MODULE_LICENSE("GPL");
