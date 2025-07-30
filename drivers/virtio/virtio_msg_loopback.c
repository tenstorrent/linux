// SPDX-License-Identifier: GPL-2.0+
/*
 * Loopback bus implementation for Virtio message transport.
 *
 * Copyright (C) 2025 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * This implements the Loopback bus for Virtio msg transport.
 */

#define pr_fmt(fmt) "virtio-msg-loopback: " fmt

#include <linux/err.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_reserved_mem.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/virtio.h>
#include <uapi/linux/virtio_msg.h>
#include <uapi/linux/virtio_msg_lb.h>

#include "virtio_msg_internal.h"

struct vmlb_device {
	struct virtio_msg_device vmdev;
	struct list_head list;
};

struct virtio_msg_lb {
	/* Serializes transfers and protects list */
	struct mutex lock;
	struct list_head list;
	struct miscdevice misc;
	struct virtio_msg_user_device vmudev;
	struct virtio_msg *response;
	struct reserved_mem *rmem;
	struct device *dev;
};

static struct virtio_msg_lb *vmlb;

#define to_vmlbdev(_vmdev)	((struct vmlb_device *)(_vmdev)->bus_data)

static struct vmlb_device *find_vmlbdev(u16 dev_id)
{
	struct vmlb_device *vmlbdev;

	list_for_each_entry(vmlbdev, &vmlb->list, list) {
		if (vmlbdev->vmdev.dev_id == dev_id)
			return vmlbdev;
	}

	return NULL;
}

static const char *virtio_msg_lb_bus_info(struct virtio_msg_device *vmdev,
					  u16 *msg_size, u32 *rev)
{
	*msg_size = VIRTIO_MSG_MIN_SIZE;
	*rev = VIRTIO_MSG_REVISION_1;

	return dev_name(vmlb->dev);
}

static int virtio_msg_lb_transfer(struct virtio_msg_device *vmdev,
				  struct virtio_msg *request,
				  struct virtio_msg *response)
{
	struct virtio_msg_user_device *vmudev = &vmlb->vmudev;
	int ret;

	/*
	 * Allow only one transaction to progress at once.
	 */
	guard(mutex)(&vmlb->lock);

	/*
	 * Set `vmsg` to `request` and finish the completion to wake up the
	 * `read()` thread.
	 */
	vmudev->vmsg = request;
	vmlb->response = response;
	complete(&vmudev->r_completion);

	/*
	 * Wait here for the `write()` thread to finish and not return before
	 * the operation is finished to avoid any potential races.
	 */
	ret = wait_for_completion_interruptible_timeout(&vmudev->w_completion, 1000);
	if (ret < 0) {
		dev_err(vmlb->dev, "Interrupted while waiting for response: %d\n", ret);
	} else if (!ret) {
		dev_err(vmlb->dev, "Timed out waiting for response\n");
		ret = -ETIMEDOUT;
	} else {
		ret = 0;
	}

	/* Clear the pointers, just to be safe */
	vmudev->vmsg = NULL;
	vmlb->response = NULL;

	return ret;
}

static struct virtio_msg_ops virtio_msg_lb_ops = {
	.transfer = virtio_msg_lb_transfer,
	.bus_info = virtio_msg_lb_bus_info,
};

static int virtio_msg_lb_user_handle(struct virtio_msg_user_device *vmudev,
				     struct virtio_msg *vmsg)
{
	struct vmlb_device *vmlbdev;

	/* Response message */
	if (vmsg->type & VIRTIO_MSG_TYPE_RESPONSE) {
		if (vmlb->response)
			memcpy(vmlb->response, vmsg, VIRTIO_MSG_MIN_SIZE);

		return 0;
	}

	/* Only support EVENT_USED virtio request messages */
	if (vmsg->type & VIRTIO_MSG_TYPE_BUS || vmsg->msg_id != VIRTIO_MSG_EVENT_USED) {
		dev_err(vmlb->dev, "Unsupported message received\n");
		return 0;
	}

	vmlbdev = find_vmlbdev(le16_to_cpu(vmsg->dev_id));
	if (!vmlbdev)
		return 0;

	virtio_msg_event(&vmlbdev->vmdev, vmsg);
	return 0;
}

static struct virtio_msg_user_ops vmlb_user_ops = {
	.handle = virtio_msg_lb_user_handle,
};

static int vmlbdev_add(struct file *file, struct vmsg_lb_dev_info *info)
{
	struct vmlb_device *vmlbdev;
	int ret;

	scoped_guard(mutex, &vmlb->lock) {
		if (find_vmlbdev(info->dev_id))
			return -EEXIST;

		vmlbdev = kzalloc(sizeof(*vmlbdev), GFP_KERNEL);
		if (!vmlbdev)
			return -ENOMEM;

		vmlbdev->vmdev.dev_id = info->dev_id;
		vmlbdev->vmdev.ops = &virtio_msg_lb_ops;
		vmlbdev->vmdev.vdev.dev.parent = vmlb->dev;
		vmlbdev->vmdev.bus_data = vmlbdev;

		list_add(&vmlbdev->list, &vmlb->list);
	}

	ret = virtio_msg_register(&vmlbdev->vmdev);
	if (ret) {
		dev_err(vmlb->dev, "Failed to register virtio msg lb device (%d)\n", ret);
		goto out;
	}

	return 0;

out:
	scoped_guard(mutex, &vmlb->lock)
		list_del(&vmlbdev->list);

	kfree(vmlbdev);
	return ret;
}

static int vmlbdev_remove(struct file *file, struct vmsg_lb_dev_info *info)
{
	struct vmlb_device *vmlbdev;

	scoped_guard(mutex, &vmlb->lock) {
		vmlbdev = find_vmlbdev(info->dev_id);
		if (vmlbdev) {
			list_del(&vmlbdev->list);
			virtio_msg_unregister(&vmlbdev->vmdev);
			return 0;
		}
	}

	dev_err(vmlb->dev, "Failed to find virtio msg lb device.\n");
	return -ENODEV;
}

static void vmlbdev_remove_all(void)
{
	struct vmlb_device *vmlbdev, *tvmlbdev;

	guard(mutex)(&vmlb->lock);

	list_for_each_entry_safe(vmlbdev, tvmlbdev, &vmlb->list, list) {
		virtio_msg_unregister(&vmlbdev->vmdev);
		list_del(&vmlbdev->list);
	}
}

static long vmlb_ioctl(struct file *file, unsigned int cmd, unsigned long data)
{
	struct vmsg_lb_dev_info info;

	if (copy_from_user(&info, (void __user *)data, sizeof(info)))
		return -EFAULT;

	switch (cmd) {
	case IOCTL_VMSG_LB_ADD:
		return vmlbdev_add(file, &info);

	case IOCTL_VMSG_LB_REMOVE:
		return vmlbdev_remove(file, &info);

	default:
		return -ENOTTY;
	}
}

static int vmlb_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset > vmlb->rmem->size - size)
		return -EINVAL;

	return remap_pfn_range(vma, vma->vm_start,
			(vmlb->rmem->base + offset) >> PAGE_SHIFT,
			size,
			vma->vm_page_prot);
}

static loff_t vmlb_llseek(struct file *file, loff_t offset, int whence)
{
	return fixed_size_llseek(file, offset, whence, vmlb->rmem->size);
}

static const struct file_operations vmlb_miscdev_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vmlb_ioctl,
	.mmap = vmlb_mmap,
	.llseek = vmlb_llseek,
};

static int virtio_msg_lb_init(void)
{
	int ret;

	vmlb = kzalloc(sizeof(*vmlb), GFP_KERNEL);
	if (!vmlb)
		return -ENOMEM;

	INIT_LIST_HEAD(&vmlb->list);
	mutex_init(&vmlb->lock);
	vmlb->vmudev.ops = &vmlb_user_ops;

	vmlb->misc.name = "virtio-msg-lb";
	vmlb->misc.minor = MISC_DYNAMIC_MINOR;
	vmlb->misc.fops = &vmlb_miscdev_fops;

	ret = misc_register(&vmlb->misc);
	if (ret)
		goto vmlb_free;

	vmlb->dev = vmlb->misc.this_device;
	vmlb->vmudev.parent = vmlb->dev;

	vmlb->rmem = of_reserved_mem_lookup_by_name("vmsglb");
	if (IS_ERR(vmlb->rmem)) {
		ret = PTR_ERR(vmlb->rmem);
		goto unregister;
	}
	ret = reserved_mem_device_init(vmlb->dev, vmlb->rmem);
	if (ret)
		goto mem_release;

	/* Register with virtio-msg UAPI */
	ret = virtio_msg_user_register(&vmlb->vmudev);
	if (ret) {
		dev_err(vmlb->dev, "Could not register virtio-msg user API\n");
		goto mem_release;
	}

	ret = dma_coerce_mask_and_coherent(vmlb->dev, DMA_BIT_MASK(64));
	if (ret)
		dev_warn(vmlb->dev, "Failed to enable 64-bit or 32-bit DMA\n");

	return 0;

mem_release:
	of_reserved_mem_device_release(vmlb->dev);
unregister:
	misc_deregister(&vmlb->misc);
vmlb_free:
	kfree(vmlb);
	return ret;
}
module_init(virtio_msg_lb_init);

static void virtio_msg_lb_exit(void)
{
	virtio_msg_user_unregister(&vmlb->vmudev);
	of_reserved_mem_device_release(vmlb->dev);
	vmlbdev_remove_all();
	misc_deregister(&vmlb->misc);
	kfree(vmlb);
}
module_exit(virtio_msg_lb_exit);

MODULE_AUTHOR("Viresh Kumar <viresh.kumar@linaro.org>");
MODULE_DESCRIPTION("Virtio message loopback bus driver");
MODULE_LICENSE("GPL");
