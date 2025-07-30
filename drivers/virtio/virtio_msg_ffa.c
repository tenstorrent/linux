// SPDX-License-Identifier: GPL-2.0+
/*
 * FF-A bus implementation for Virtio message transport.
 *
 * Copyright (C) 2025 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * This implements the FF-A (Arm Firmware Framework) bus for Virtio msg
 * transport.
 */

#define pr_fmt(fmt) "virtio-msg-ffa: " fmt

#include <linux/arm_ffa.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/virtio.h>
#include <uapi/linux/virtio_msg_ffa.h>

#include "virtio_msg_internal.h"

struct virtio_msg_indirect_data {
	struct completion completion;
	struct virtio_msg *response;
};

struct virtio_msg_device_data {
	struct virtio_msg_device vmdev;
	struct virtio_msg_indirect_data idata;
};

/* Represents FF-A corresponding to a partition */
struct virtio_msg_ffa_device {
	struct ffa_device *ffa_dev;
	struct reserved_mem *rmem;
	struct virtio_msg_indirect_data idata;
	struct virtio_msg_device_data *vmdevs;
	int (*send)(struct virtio_msg_ffa_device *vmfdev,
		    struct virtio_msg *request,
		    struct virtio_msg *response,
		    struct virtio_msg_indirect_data *idata);
	int vmdev_count;
	u16 msg_size;
};

#define to_vmdevdata(_vmdev) \
	container_of(_vmdev, struct virtio_msg_device_data, vmdev)
#define to_vmfdev(_vmdev) ((struct virtio_msg_ffa_device *)(_vmdev)->bus_data)

static int vmsg_ffa_send_direct(struct virtio_msg_ffa_device *vmfdev,
				struct virtio_msg *request,
				struct virtio_msg *response,
				struct virtio_msg_indirect_data *idata_unused)
{
	struct ffa_device *ffa_dev = vmfdev->ffa_dev;
	struct ffa_send_direct_data2 ffa_data;
	int ret;

	memcpy(&ffa_data, request, request->msg_size);

	ret = ffa_dev->ops->msg_ops->sync_send_receive2(ffa_dev, &ffa_data);
	if (ret) {
		dev_dbg(&ffa_dev->dev,
			"Unable to send direct FF-A message: %d\n", ret);
		return ret;
	}

	if (response)
		memcpy(response, &ffa_data, vmfdev->msg_size);

	return 0;
}

static int vmsg_ffa_send_indirect(struct virtio_msg_ffa_device *vmfdev,
				  struct virtio_msg *request,
				  struct virtio_msg *response,
				  struct virtio_msg_indirect_data *idata)
{
	struct ffa_device *ffa_dev = vmfdev->ffa_dev;
	struct device *dev = &ffa_dev->dev;
	int ret, count = 10;

	/*
	 * Store the response pointer in idata structure. This will be updated
	 * by vmsg_ffa_notifier_cb() later.
	 */
	idata->response = response;

try_again:
	ret = ffa_dev->ops->msg_ops->indirect_send(ffa_dev, request,
						   request->msg_size);
	if (ret == -EBUSY && --count) {
		cpu_relax();
		goto try_again;
	}

	if (ret) {
		dev_err(dev, "Failed sending indirect FF-A message: %d\n", ret);
		return ret;
	}

	/*
	 * Always wait for the operation to finish, otherwise we may start
	 * another operation while the previous one is still ongoing.
	 */
	ret = wait_for_completion_interruptible_timeout(&idata->completion, 1000);
	if (ret < 0) {
		dev_err(dev, "Interrupted - waiting for a response: %d\n", ret);
	} else if (!ret) {
		dev_err(dev, "Timed out waiting for a response\n");
		ret = -ETIMEDOUT;
	} else {
		ret = 0;
	}

	return ret;
}

static struct virtio_msg_device *
find_vmdev(struct virtio_msg_ffa_device *vmfdev, u16 dev_id)
{
	int i;

	/* Find the device corresponding to a dev_id */
	for (i = 0; i < vmfdev->vmdev_count; i++) {
		if (vmfdev->vmdevs[i].vmdev.dev_id == dev_id)
			return &vmfdev->vmdevs[i].vmdev;
	}

	dev_err(&vmfdev->ffa_dev->dev, "Couldn't find matching vmdev: %d\n",
		dev_id);
	return NULL;
}

static void vmsg_ffa_notifier_cb(int notify_id, void *cb_data, void *buf)
{
	struct virtio_msg_ffa_device *vmfdev = cb_data;
	struct ffa_device *ffa_dev = vmfdev->ffa_dev;
	struct virtio_msg_indirect_data *idata;
	struct virtio_msg_device *vmdev;
	struct virtio_msg *vmsg = buf;

	/*
	 * We can either receive a response message (to a previously sent
	 * request), or an EVENT_USED request message.
	 */
	if (vmsg->type & VIRTIO_MSG_TYPE_RESPONSE) {
		if (vmsg->type & VIRTIO_MSG_TYPE_BUS) {
			idata = &vmfdev->idata;
		} else {
			vmdev = find_vmdev(vmfdev, le16_to_cpu(vmsg->dev_id));
			if (!vmdev)
				return;

			idata = &to_vmdevdata(vmdev)->idata;
		}

		if (idata->response)
			memcpy(idata->response, vmsg, vmsg->msg_size);

		complete(&idata->completion);

		return;
	}

	/* Only support EVENT_USED virtio request messages */
	if (vmsg->type & VIRTIO_MSG_TYPE_BUS ||
	    vmsg->msg_id != VIRTIO_MSG_EVENT_USED) {
		dev_err(&ffa_dev->dev, "Unsupported message received\n");
		return;
	}

	vmdev = find_vmdev(vmfdev, le16_to_cpu(vmsg->dev_id));
	if (!vmdev)
		return;

	virtio_msg_event(vmdev, vmsg);
}

static int vmsg_ffa_notify_setup(struct virtio_msg_ffa_device *vmfdev)
{
	struct ffa_device *ffa_dev = vmfdev->ffa_dev;
	int ret;

	ret = ffa_dev->ops->notifier_ops->fwk_notify_request(ffa_dev,
			&vmsg_ffa_notifier_cb, vmfdev, 0);
	if (ret)
		dev_err(&ffa_dev->dev, "Unable to request notifier: %d\n", ret);

	return ret;
}

static void vmsg_ffa_notify_cleanup(struct virtio_msg_ffa_device *vmfdev)
{
	struct ffa_device *ffa_dev = vmfdev->ffa_dev;
	int ret;

	ret = ffa_dev->ops->notifier_ops->fwk_notify_relinquish(ffa_dev, 0);
	if (ret)
		dev_err(&ffa_dev->dev, "Unable to relinquish notifier: %d\n", ret);
}

static int vmsg_ffa_bus_get_devices(struct virtio_msg_ffa_device *vmfdev,
				    u16 *map, u16 *count)
{
	u8 req_buf[VIRTIO_MSG_FFA_BUS_MSG_SIZE];
	u8 res_buf[VIRTIO_MSG_FFA_BUS_MSG_SIZE];
	struct virtio_msg *request = (struct virtio_msg *)&req_buf;
	struct virtio_msg *response = (struct virtio_msg *)&res_buf;
	struct bus_get_devices *req_payload = virtio_msg_payload(request);
	struct bus_get_devices_resp *res_payload = virtio_msg_payload(response);
	int ret;

	static_assert(sizeof(*request) + sizeof(*req_payload) <
		      VIRTIO_MSG_FFA_BUS_MSG_SIZE);
	static_assert(sizeof(*response) + sizeof(*res_payload) <
		      VIRTIO_MSG_FFA_BUS_MSG_SIZE);

	virtio_msg_prepare(request, VIRTIO_MSG_BUS_GET_DEVICES,
			   sizeof(*req_payload));
	req_payload->offset = 0;
	req_payload->num = cpu_to_le16(0xFF);

	ret = vmfdev->send(vmfdev, request, response, &vmfdev->idata);
	if (ret < 0)
		return ret;

	*count = le16_to_cpu(res_payload->num);
	if (!*count)
		return -ENODEV;

	if (res_payload->offset != req_payload->offset)
		return -EINVAL;

	/* Support up to 16 devices for now */
	if (res_payload->next_offset)
		return -EINVAL;

	map[0] = res_payload->devices[0];
	map[1] = res_payload->devices[1];

	return 0;
}

static int vmsg_ffa_bus_version(struct virtio_msg_ffa_device *vmfdev)
{
	u8 req_buf[VIRTIO_MSG_FFA_BUS_MSG_SIZE];
	u8 res_buf[VIRTIO_MSG_FFA_BUS_MSG_SIZE];
	struct virtio_msg *request = (struct virtio_msg *)&req_buf;
	struct virtio_msg *response = (struct virtio_msg *)&res_buf;
	struct bus_ffa_version *req_payload = virtio_msg_payload(request);
	struct bus_ffa_version_resp *res_payload = virtio_msg_payload(response);
	u32 features;
	int ret;

	static_assert(sizeof(*request) + sizeof(*req_payload) <
		      VIRTIO_MSG_FFA_BUS_MSG_SIZE);
	static_assert(sizeof(*response) + sizeof(*res_payload) <
		      VIRTIO_MSG_FFA_BUS_MSG_SIZE);

	virtio_msg_prepare(request, VIRTIO_MSG_FFA_BUS_VERSION,
			   sizeof(*req_payload));
	req_payload->driver_version = cpu_to_le32(VIRTIO_MSG_FFA_BUS_VERSION_1_0);
	req_payload->vmsg_revision = cpu_to_le32(VIRTIO_MSG_REVISION_1);
	req_payload->vmsg_features = cpu_to_le32(VIRTIO_MSG_FEATURES);
	req_payload->features = cpu_to_le32(VIRTIO_MSG_FFA_FEATURE_BOTH_SUPP);
	req_payload->area_num = cpu_to_le16(VIRTIO_MSG_FFA_AREA_ID_MAX);

	ret = vmfdev->send(vmfdev, request, response, &vmfdev->idata);
	if (ret < 0)
		return ret;

	if (le32_to_cpu(res_payload->device_version) != VIRTIO_MSG_FFA_BUS_VERSION_1_0)
		return -EINVAL;

	if (le32_to_cpu(res_payload->vmsg_revision) != VIRTIO_MSG_REVISION_1)
		return -EINVAL;

	if (le32_to_cpu(res_payload->vmsg_features) != VIRTIO_MSG_FEATURES)
		return -EINVAL;

	features = le32_to_cpu(res_payload->features);

	/*
	 * - Direct message must be supported if it already worked.
	 * - Indirect message must be supported if it already worked
	 * - And direct message must not be supported since it didn't work.
	 */
	if ((ffa_partition_supports_direct_recv(vmfdev->ffa_dev) &&
	     !(features & VIRTIO_MSG_FFA_FEATURE_DIRECT_MSG_RX_SUPP)) ||
	    (ffa_partition_supports_indirect_msg(vmfdev->ffa_dev) &&
	     !(features & VIRTIO_MSG_FFA_FEATURE_INDIRECT_MSG_SUPP))) {
		dev_err(&vmfdev->ffa_dev->dev, "Invalid features\n");
		return -EINVAL;
	}

	return 0;
}

static int virtio_msg_ffa_transfer(struct virtio_msg_device *vmdev,
				   struct virtio_msg *request,
				   struct virtio_msg *response)
{
	struct virtio_msg_indirect_data *idata = &to_vmdevdata(vmdev)->idata;
	struct virtio_msg_ffa_device *vmfdev = to_vmfdev(vmdev);

	return vmfdev->send(vmfdev, request, response, idata);
}

static const char *virtio_msg_ffa_bus_info(struct virtio_msg_device *vmdev,
					   u16 *msg_size, u32 *rev)
{
	struct virtio_msg_ffa_device *vmfdev = to_vmfdev(vmdev);

	*msg_size = vmfdev->msg_size;
	*rev = VIRTIO_MSG_REVISION_1;

	return dev_name(&vmfdev->ffa_dev->dev);
}

static struct virtio_msg_ops vmf_ops = {
	.transfer = virtio_msg_ffa_transfer,
	.bus_info = virtio_msg_ffa_bus_info,
};

static void remove_vmdevs(struct virtio_msg_ffa_device *vmfdev, int count)
{
	while (count--)
		virtio_msg_unregister(&vmfdev->vmdevs[count].vmdev);
}

static int virtio_msg_ffa_probe(struct ffa_device *ffa_dev)
{
	struct virtio_msg_ffa_device *vmfdev;
	struct device *dev = &ffa_dev->dev;
	struct virtio_msg_device *vmdev;
	unsigned long devices = 0;
	int ret, i = 0, bit;
	u16 count;

	vmfdev = devm_kzalloc(dev, sizeof(*vmfdev), GFP_KERNEL);
	if (!vmfdev)
		return -ENOMEM;

	if (ffa_partition_supports_direct_recv(ffa_dev)) {
		vmfdev->send = vmsg_ffa_send_direct;
	} else if (ffa_partition_supports_indirect_msg(ffa_dev)) {
		vmfdev->send = vmsg_ffa_send_indirect;
	} else {
		dev_err(dev, "Direct or Indirect messages not supported\n");
		return -EINVAL;
	}

	vmfdev->ffa_dev = ffa_dev;
	vmfdev->msg_size = VIRTIO_MSG_FFA_BUS_MSG_SIZE;
	ffa_dev_set_drvdata(ffa_dev, vmfdev);
	init_completion(&vmfdev->idata.completion);

	ret = vmsg_ffa_notify_setup(vmfdev);
	if (ret)
		return ret;

	ret = vmsg_ffa_bus_version(vmfdev);
	if (ret)
		goto notify_cleanup;

	ret = vmsg_ffa_bus_get_devices(vmfdev, (u16 *)&devices, &count);
	if (ret)
		goto notify_cleanup;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		dev_warn(dev, "Failed to enable 64-bit or 32-bit DMA\n");

	vmfdev->rmem = of_reserved_mem_lookup_by_name("vmsgffa");
	if (!IS_ERR(vmfdev->rmem)) {
		ret = reserved_mem_device_init(dev, vmfdev->rmem);
		if (ret)
			goto rmem_free;
	} else {
		dev_info(dev, "Continuing without reserved-memory block\n");
	}

	vmfdev->vmdevs = devm_kcalloc(dev, count, sizeof(*vmfdev->vmdevs),
				      GFP_KERNEL);
	if (!vmfdev->vmdevs) {
		ret = -ENOMEM;
		goto rmem_free;
	}
	vmfdev->vmdev_count = count;

	for_each_set_bit(bit, &devices, sizeof(devices)) {
		init_completion(&vmfdev->vmdevs[i].idata.completion);
		vmdev = &vmfdev->vmdevs[i].vmdev;
		vmdev->dev_id = bit;
		vmdev->ops = &vmf_ops;
		vmdev->vdev.dev.parent = dev;
		vmdev->bus_data = vmfdev;

		ret = virtio_msg_register(vmdev);
		if (ret) {
			dev_err(dev, "Failed to register virtio-msg device (%d)\n", ret);
			goto unregister;
		}

		i++;
	}

	return 0;

unregister:
	remove_vmdevs(vmfdev, i);
rmem_free:
	if (!IS_ERR(vmfdev->rmem))
		of_reserved_mem_device_release(dev);
notify_cleanup:
	vmsg_ffa_notify_cleanup(vmfdev);
	return ret;
}

static void virtio_msg_ffa_remove(struct ffa_device *ffa_dev)
{
	struct virtio_msg_ffa_device *vmfdev = ffa_dev->dev.driver_data;

	remove_vmdevs(vmfdev, vmfdev->vmdev_count);

	if (!IS_ERR(vmfdev->rmem))
		of_reserved_mem_device_release(&ffa_dev->dev);

	vmsg_ffa_notify_cleanup(vmfdev);
}

static const struct ffa_device_id virtio_msg_ffa_device_ids[] = {
	/* c66028b5-2498-4aa1-9de7-77da6122abf0 */
	{ UUID_INIT(0xc66028b5, 0x2498, 0x4aa1,
		    0x9d, 0xe7, 0x77, 0xda, 0x61, 0x22, 0xab, 0xf0) },
	{}
};

static int __maybe_unused virtio_msg_ffa_suspend(struct device *dev)
{
	struct virtio_msg_ffa_device *vmfdev = dev_get_drvdata(dev);
	int ret, i, index;

	for (i = 0; i < vmfdev->vmdev_count; i++) {
		index = vmfdev->vmdev_count - i - 1;
		ret = virtio_device_freeze(&vmfdev->vmdevs[index].vmdev.vdev);
		if (ret)
			return ret;
	}

	return 0;
}

static int __maybe_unused virtio_msg_ffa_resume(struct device *dev)
{
	struct virtio_msg_ffa_device *vmfdev = dev_get_drvdata(dev);
	int ret, i;

	for (i = 0; i < vmfdev->vmdev_count; i++) {
		ret = virtio_device_restore(&vmfdev->vmdevs[i].vmdev.vdev);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct dev_pm_ops virtio_msg_ffa_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(virtio_msg_ffa_suspend, virtio_msg_ffa_resume)
};

static struct ffa_driver virtio_msg_ffa_driver = {
	.name = "virtio-msg-ffa",
	.probe = virtio_msg_ffa_probe,
	.remove = virtio_msg_ffa_remove,
	.id_table = virtio_msg_ffa_device_ids,
	.driver = {
		.pm = &virtio_msg_ffa_pm_ops,
	},
};

static int virtio_msg_ffa_init(void)
{
	if (IS_REACHABLE(CONFIG_ARM_FFA_TRANSPORT))
		return ffa_register(&virtio_msg_ffa_driver);
	else
		return -EOPNOTSUPP;
}
module_init(virtio_msg_ffa_init);

static void virtio_msg_ffa_exit(void)
{
	if (IS_REACHABLE(CONFIG_ARM_FFA_TRANSPORT))
		ffa_unregister(&virtio_msg_ffa_driver);
}
module_exit(virtio_msg_ffa_exit);

MODULE_AUTHOR("Viresh Kumar <viresh.kumar@linaro.org>");
MODULE_DESCRIPTION("Virtio message FF-A bus driver");
MODULE_LICENSE("GPL");
