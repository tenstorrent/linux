// SPDX-License-Identifier: GPL-2.0
/*
 * Virtio-msg-amp common code
 *
 * Copyright (c) Linaro Ltd, 2024
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>

#include "virtio_msg_amp.h"

#define VIRTIO_MSG_AMP_SIZE 64

#define to_virtio_msg_amp_device(_vmdev) \
	container_of(_vmdev, struct virtio_msg_amp_device, this_dev)

#define MK_RESP(type, msg_id) (((u16) type) << 8 | (u16) (msg_id))

static void tx_msg(struct virtio_msg_amp *amp_dev, void* msg_buf, size_t size);

/* wait for completion with timeout */
static bool wait_for_it(struct completion* p_it, u32 msec)
{
	long consumed;

	consumed = wait_for_completion_timeout(p_it, msecs_to_jiffies(msec));
	return (consumed > 0);
}

static int virtio_msg_amp_transfer(struct virtio_msg_device *vmdev,
				struct virtio_msg *request,
				struct virtio_msg *response)
{
	struct virtio_msg_amp_device *vmadev = to_virtio_msg_amp_device(vmdev);
	struct virtio_msg_amp *amp_dev = vmadev->amp_dev;
	struct device *pdev = amp_dev->ops->get_device(amp_dev);
	int len = VIRTIO_MSG_AMP_SIZE;
	int rc = 0;
	u16 match = MK_RESP(request->type | VIRTIO_MSG_TYPE_RESPONSE, request->msg_id);

	if (response) {
		/* init a bad response in case we fail or timeout */
		response->type = 0;
		response->msg_id = 0;

		dev_dbg(pdev, "about to grab mutex dev_id=%d type/id=%04x\n",
			vmadev->dev_id, match);

		/* we need to serialize messages that need a response */
		mutex_lock(&vmadev->response_lock);

		/* does the device still exist? */
		if (!vmadev->in_use) {
			mutex_unlock(&vmadev->response_lock);
			return -2;
		}

		dev_dbg(pdev, "send w/ resp dev_id=%d type/id=%04x\n",
			vmadev->dev_id, match);
		vmadev->response = response;
		vmadev->expected_response = match;
		reinit_completion(&vmadev->response_done);
	} else {
		dev_dbg(pdev, "send only dev_id=%d type/id=%04x\n",
			vmadev->dev_id, match);
	}

	tx_msg(amp_dev, request, len);

	if (response) {
		if (!wait_for_it(&vmadev->response_done, 5000 * 20)) {
			dev_err(pdev,
			  "response wait timeout dev_id=%d, type/id=%04x\n",
			  vmadev->dev_id, match);
			rc = -2;
		} else {
			dev_dbg(pdev,
			  "send response complete dev_id=%d, type/id=%04x\n",
			  vmadev->dev_id, match);
		}

		/* in either case we need to release the lock */
		mutex_unlock(&vmadev->response_lock);
	}

	return rc;
}

static const char *virtio_msg_amp_bus_info(struct virtio_msg_device *vmdev,
                                           u16 *msg_size, u32 *rev)
{
	struct virtio_msg_amp_device *vmadev = to_virtio_msg_amp_device(vmdev);
	struct virtio_msg_amp *amp_dev = vmadev->amp_dev;
	struct device *pdev = amp_dev->ops->get_device(amp_dev);

	*msg_size = VIRTIO_MSG_AMP_SIZE;
	*rev = VIRTIO_MSG_REVISION_1;

	dev_info(pdev, "get bus name for dev_id=%d\n",  vmadev->dev_id);

	return dev_name(pdev);
}

static void virtio_msg_amp_synchronize_cbs(struct virtio_msg_device *vmdev)
{
	struct virtio_msg_amp_device *vmadev = to_virtio_msg_amp_device(vmdev);
	struct virtio_msg_amp *amp_dev = vmadev->amp_dev;
	struct device *pdev = amp_dev->ops->get_device(amp_dev);

	dev_info(pdev, "sync cbs for dev_id=%d\n",  vmadev->dev_id);

	/* hope for the best */
}

static void virtio_msg_amp_release(struct virtio_msg_device *vmdev)
{
	struct virtio_msg_amp_device *vmadev = to_virtio_msg_amp_device(vmdev);
	struct virtio_msg_amp *amp_dev = vmadev->amp_dev;
	struct device *pdev = amp_dev->ops->get_device(amp_dev);

	dev_info(pdev, "starting release for dev_id=%d\n", vmadev->dev_id);

	/* let any waiting thread know we are going away */
	vmadev->in_use = false;
	vmadev->expected_response = 0;
	vmadev->response = NULL;
	complete_all(&vmadev->response_done);
	mutex_unlock(&vmadev->response_lock);

	dev_info(pdev, "release done for dev_id=%d\n", vmadev->dev_id);
}

static struct virtio_msg_ops amp_msg_device_ops = {
	.transfer = virtio_msg_amp_transfer,
	.bus_info = virtio_msg_amp_bus_info,
	.synchronize_cbs = virtio_msg_amp_synchronize_cbs,
	.release = virtio_msg_amp_release,
};

static void init_vmadev(struct virtio_msg_amp_device* vmadev,
	struct virtio_msg_amp* amp_dev, u16 dev_id)
{
	struct device* parent_dev = amp_dev->ops->get_device(amp_dev);
	vmadev->this_dev.ops = &amp_msg_device_ops;
	vmadev->this_dev.priv = NULL;
	vmadev->this_dev.dev_id = dev_id;
	vmadev->this_dev.vdev.dev.parent = parent_dev;

	vmadev->amp_dev = amp_dev;
	vmadev->in_use = true;
	vmadev->dev_id = dev_id;
	mutex_init(&vmadev->response_lock);
	vmadev->expected_response = 0;
	vmadev->response = NULL;
	init_completion(&vmadev->response_done);
}

/* this one is temporary as the v0 layout is not self describing */
int virtio_msg_amp_register_v0(struct virtio_msg_amp *amp_dev) {
	return 0;
}

static struct virtio_msg_amp_device *amp_find_dev(
	struct virtio_msg_amp 	*amp_dev,
	u16			dev_id)
{
	//printk(KERN_ERR "find device %d to %d\n",
	//	dev_id, amp_dev->one_dev.dev_id);

	if (amp_dev->one_dev.dev_id == dev_id)
		return &amp_dev->one_dev;

	return NULL;
}

static bool vmadev_check_rx_match(
	struct virtio_msg_amp_device	*vmadev,
	struct virtio_msg 		*msg)
{
	u16 match;

	match = MK_RESP(msg->type, msg->msg_id);
	//printk(KERN_ERR "try to match %04x to %04x\n",
	//	match, vmadev->expected_response);

	if (vmadev->expected_response == match ) {
		memcpy(vmadev->response, msg, VIRTIO_MSG_AMP_SIZE);
		vmadev->expected_response = 0;
		complete(&vmadev->response_done);
		return true;
	}
	return false;
}

static void rx_proc_all(struct virtio_msg_amp *amp_dev) {
	struct device *pdev = amp_dev->ops->get_device(amp_dev);
	struct virtio_msg_amp_device *vmadev;
	struct virtio_msg *msg;
	bool expected = false;
	u16 dev_id;
	u8 *buf = amp_dev->rx_temp_buf;
	int err;

	while (spsc_recv(&amp_dev->dev2drv, buf, VIRTIO_MSG_AMP_SIZE)) {
		dev_dbg(pdev, "RX MSG: %40ph \n", buf);
		msg = (struct virtio_msg*) buf;
		dev_id =  le16_to_cpu(msg->dev_id);
		if ((vmadev = amp_find_dev(amp_dev, dev_id))) {
			if (vmadev_check_rx_match(vmadev, msg)) {
				expected = true;
			} else {
				err = virtio_msg_event(&vmadev->this_dev, msg);
				if (!err)
					expected = true;
				else
					dev_err(pdev, "vm rx err=%d", err);
			}
		}
		if (!expected) {
			dev_err(pdev,
				"Unexpected msg dev_id=%d, type/id=%02x/%02x\n",
				msg->dev_id, msg->type, msg->msg_id);
		}
	}
}

static void tx_msg(struct virtio_msg_amp *amp_dev, void* msg_buf,
	size_t msg_len) {
	struct device *pdev = amp_dev->ops->get_device(amp_dev);

	dev_dbg(pdev, "TX MSG: %40ph \n", msg_buf);

	/* queue a message */
	while ( ! spsc_send(&amp_dev->drv2dev, msg_buf, msg_len) ) {
		dev_info(pdev, "out of tx space, sleep");
		mdelay(10);
	}

	/* Notify the peer */
	amp_dev->ops->tx_notify(amp_dev, 0);
}

#if 0
u8 test_msg[64] = {
	0x00, 0xFA, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77
};
#endif

/* normal API */
int  virtio_msg_amp_register(struct virtio_msg_amp *amp_dev) {
	size_t page_size = 4096;
	char* mem = amp_dev->shmem;
	void* page0 = &mem[0 * page_size];
	void* page1 = &mem[1 * page_size];
	int err;

	/* create the first (and only) device */
	init_vmadev(&amp_dev->one_dev, amp_dev, 0);

	/* create the structures that point to the message FIFOs in memory */
	spsc_init(&amp_dev->drv2dev, "drv2dev", spsc_capacity(page_size), page0);
	spsc_init(&amp_dev->dev2drv, "dev2drv", spsc_capacity(page_size), page1);

	/* empty the rx queue */
	rx_proc_all(amp_dev);

	//tx_msg(amp_dev, test_msg, 64);
	//pr_err("drv2dev [0]: %64ph \n", page0);
	//pr_err("drv2dev [1]: %64ph \n", ((u8*) page0) + 64);
	//pr_err("drv2dev [1]: %64ph \n", ((u8*) page0) + 128);

	/* register with the virtio-msg common code */
	err = virtio_msg_register(&amp_dev->one_dev.this_dev);

	return err;
}

static void virtio_msg_amp_device_unregister(
	struct virtio_msg_amp_device *vmadev) {
	if (vmadev->in_use)
		virtio_msg_unregister(&vmadev->this_dev);
}

void virtio_msg_amp_unregister(struct virtio_msg_amp *amp_dev) {
	/* destroy all devices */
	virtio_msg_amp_device_unregister(&amp_dev->one_dev);
}

int  virtio_msg_amp_notify_rx(struct virtio_msg_amp *amp_dev, u32 notify_idx) {
	rx_proc_all(amp_dev);
	return 0;
}

static int __init virtio_msg_amp_init(void)
{
	return 0;
}
module_init(virtio_msg_amp_init);

static void __exit virtio_msg_amp_exit(void) {}
module_exit(virtio_msg_amp_exit);

EXPORT_SYMBOL_GPL(virtio_msg_amp_register_v0);
EXPORT_SYMBOL_GPL(virtio_msg_amp_register);
EXPORT_SYMBOL_GPL(virtio_msg_amp_unregister);
EXPORT_SYMBOL_GPL(virtio_msg_amp_notify_rx);

MODULE_DESCRIPTION("Virtio-msg for AMP systems");
MODULE_AUTHOR("Bill Mills <bill.mills@linaro.org>");
MODULE_LICENSE("GPL v2");
