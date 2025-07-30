// SPDX-License-Identifier: GPL-2.0+
/*
 * Virtio message transport user API.
 *
 * Copyright (C) 2025 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 */

#define pr_fmt(fmt) "virtio-msg: " fmt

#include <linux/err.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "virtio_msg_internal.h"

#define to_virtio_msg_user_device(_misc) \
	container_of(_misc, struct virtio_msg_user_device, misc)

static ssize_t vmsg_miscdev_read(struct file *file, char __user *buf,
				 size_t count, loff_t *pos)
{
	struct miscdevice *misc = file->private_data;
	struct virtio_msg_user_device *vmudev = to_virtio_msg_user_device(misc);
	struct device *dev = vmudev->parent;
	int ret;

	if (count < VIRTIO_MSG_MIN_SIZE) {
		dev_err(dev, "Trying to read message of incorrect size: %zu\n",
			count);
		return 0;
	}

	/* Wait for the message */
	ret = wait_for_completion_interruptible(&vmudev->r_completion);
	if (ret < 0) {
		dev_err(dev, "Interrupted while waiting for response: %d\n", ret);
		return 0;
	}

	WARN_ON(!vmudev->vmsg);

	/* The "vmsg" pointer is filled by the bus driver before waking up */
	if (copy_to_user(buf, vmudev->vmsg, count) != 0)
		return 0;

	vmudev->vmsg = NULL;

	return count;
}

static ssize_t vmsg_miscdev_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *pos)
{
	struct miscdevice *misc = file->private_data;
	struct virtio_msg_user_device *vmudev = to_virtio_msg_user_device(misc);
	struct virtio_msg *vmsg __free(kfree) = NULL;

	if (count < VIRTIO_MSG_MIN_SIZE) {
		dev_err(vmudev->parent, "Trying to write message of incorrect size: %zu\n",
			count);
		return 0;
	}

	vmsg = kzalloc(count, GFP_KERNEL);
	if (!vmsg)
		return 0;

	if (copy_from_user(vmsg, buf, count) != 0)
		return 0;

	vmudev->ops->handle(vmudev, vmsg);

	/* Wake up the handler only for responses */
	if (vmsg->type & VIRTIO_MSG_TYPE_RESPONSE)
		complete(&vmudev->w_completion);

	return count;
}

static const struct file_operations vmsg_miscdev_fops = {
	.owner = THIS_MODULE,
	.read = vmsg_miscdev_read,
	.write = vmsg_miscdev_write,
};

/**
 * virtio_msg_user_register - Register a user-space accessible virtio message device
 * @vmudev: Pointer to the virtio message user device
 *
 * Initializes and registers a user-accessible virtio message device as a `misc`
 * character device. Upon successful registration, the device appears in
 * userspace as `/dev/virtio-msg-N` where `N` is a unique identifier assigned at
 * runtime.
 *
 * The resulting device node allows user-space interaction with the virtio
 * message transport.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int virtio_msg_user_register(struct virtio_msg_user_device *vmudev)
{
	static u8 vmsg_user_device_count;
	int ret;

	if (!vmudev || !vmudev->ops)
		return -EINVAL;

	init_completion(&vmudev->r_completion);
	init_completion(&vmudev->w_completion);

	vmudev->misc.parent = vmudev->parent;
	vmudev->misc.minor = MISC_DYNAMIC_MINOR;
	vmudev->misc.fops = &vmsg_miscdev_fops;
	vmudev->misc.name = vmudev->name;
	sprintf(vmudev->name, "virtio-msg-%d", vmsg_user_device_count);

	ret = misc_register(&vmudev->misc);
	if (ret)
		return ret;

	vmsg_user_device_count++;
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_user_register);

/**
 * virtio_msg_user_unregister - Unregister a user-space virtio message device
 * @vmudev: Pointer to the virtio message user device
 *
 * Unregisters a previously registered virtio message device from the misc
 * subsystem. This removes its user-space interface (e.g., /dev/virtio-msg-N).
 */
void virtio_msg_user_unregister(struct virtio_msg_user_device *vmudev)
{
	misc_deregister(&vmudev->misc);
}
EXPORT_SYMBOL_GPL(virtio_msg_user_unregister);
