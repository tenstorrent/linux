/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Virtio message transport header.
 *
 * Copyright (C) 2025 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 */

#ifndef _DRIVERS_VIRTIO_MSG_INTERNAL_H
#define _DRIVERS_VIRTIO_MSG_INTERNAL_H

#include <linux/virtio.h>
#include <uapi/linux/virtio_msg.h>

struct virtio_msg_device;

/*
 * struct virtio_msg_ops - Virtio message bus operations.
 * @bus_info: Return bus information.
 * @transfer: Transfer a message.
 * @synchronize_cbs: Synchronize with the virtqueue callbacks (optional).
 * @release: Release the resources corresponding to the device (optional).
 */
struct virtio_msg_ops {
	const char *(*bus_info)(struct virtio_msg_device *vmdev, u16 *msg_size, u32 *rev);
	int (*transfer)(struct virtio_msg_device *vmdev, struct virtio_msg *request,
			struct virtio_msg *response);
	void (*synchronize_cbs)(struct virtio_msg_device *vmdev);
	void (*release)(struct virtio_msg_device *vmdev);
};

/*
 * Representation of a device using virtio message
 * transport.
 */
struct virtio_msg_device {
	struct virtio_device vdev;
	struct virtio_msg_ops *ops;
	const char *bus_name;
	void *bus_data;
	u32 generation_count;
	u32 config_size;
	u16 msg_size;
	u16 dev_id;

	struct virtio_msg *request;
	struct virtio_msg *response;
};

int virtio_msg_register(struct virtio_msg_device *vmdev);
void virtio_msg_unregister(struct virtio_msg_device *vmdev);

void virtio_msg_prepare(struct virtio_msg *vmsg, u8 msg_id, u16 payload_size);
int virtio_msg_event(struct virtio_msg_device *vmdev, struct virtio_msg *vmsg);

#endif /* _DRIVERS_VIRTIO_MSG_INTERNAL_H */
