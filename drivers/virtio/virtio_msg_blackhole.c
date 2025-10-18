// SPDX-License-Identifier: GPL-2.0
/*
 * Virtio-msg-blackhole driver
 *
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "virtio_msg_amp.h"

#define DRV_NAME "virtio_msg_blackhole"

// First 4K bytes reserved for registers for this device
// Registers cannot be directly accessed, instead we update a "generation" register to inform
// the device side that we want to perform a read and wait for the device side to ack this request
// by incrementing the generation register again and filling the required value into the register
// Right now, we just have one set of registers that are used to determine if the 2nd direction (x280->host) of interrupts
// is supported. This direction of interrupts works by writing to the msi address on the pcie tile.
// The program running on the host is responsible for determining this address and setting noc mappings up so that the x280
// can write to this register, all at boot time.
struct blackhole_regs {
	__le32 msi_reg_generation;
	__le32 msi_reg_supported;
	__le64 msi_reg_address;
	__le32 msi_reg_write_value;
};

struct blackhole_dev {
	struct virtio_msg_amp amp_dev;
	struct platform_device *pdev;
	struct blackhole_regs __iomem *regs;
	__le32 __iomem *msi_reg;
};

/**
 *  blackhole_irq_handler: IRQ from on the PLIC
 */
static irqreturn_t blackhole_irq_handler(int irq, void *dev_id)
{
	struct blackhole_dev *blackhole_dev = dev_id;
	int err;
	/* we always use notify index 0 */
	err = virtio_msg_amp_notify_rx(&blackhole_dev->amp_dev, 0);
	if (err)
		dev_err(&blackhole_dev->pdev->dev, "blackhole IRQ error %d", err);
	//else
	//	dev_info(&blackhole_dev->pdev->dev, "blackhole IRQ fired");

	return IRQ_HANDLED;
}

/**
 *  blackhole_tx_notify: request from AMP layer to notify our peer
 */
static int blackhole_tx_notify(struct virtio_msg_amp *_amp_dev, u32 notify_idx) {
	struct blackhole_dev *blackhole_dev =
		container_of(_amp_dev, struct blackhole_dev, amp_dev);

	if (notify_idx != 0) {
		dev_warn(&blackhole_dev->pdev->dev, "blackhole tx_notify_idx not 0");
		notify_idx = 0;
	}

	if (le32_to_cpu(readl(&blackhole_dev->regs->msi_reg_supported))){
		writel(readl(&blackhole_dev->regs->msi_reg_write_value), blackhole_dev->msi_reg);
	}

	return 0;
}

static struct device *blackhole_get_device(struct virtio_msg_amp *_amp_dev) {
	struct blackhole_dev *blackhole_dev =
		container_of(_amp_dev, struct blackhole_dev, amp_dev);

	return &blackhole_dev->pdev->dev;
}

/**
 *  blackhole_release: release from virtio-msg-amp layer
 *  disable notifications but leave free to the PCI layer callback
 */
static void blackhole_release(struct virtio_msg_amp *_amp_dev) {
	// struct blackhole_dev *blackhole_dev =
	// 	container_of(_amp_dev, struct blackhole_dev, amp_dev);

}

static struct virtio_msg_amp_ops blackhole_amp_ops = {
	.tx_notify = blackhole_tx_notify,
	.get_device  = blackhole_get_device,
	.release   = blackhole_release
};

static int blackhole_probe(struct platform_device *pdev)
{
	struct blackhole_dev *blackhole_dev;
	int err, irq;
	void __iomem *regs; 

	blackhole_dev = devm_kzalloc(&pdev->dev, sizeof(struct blackhole_dev),
				 GFP_KERNEL);
	if (!blackhole_dev) {
		err = -ENOMEM;
		goto error;
	}

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs)) {
		err = PTR_ERR(regs);
		goto error;
	}

	// regs occupy the first 4k bytes of the register space
	blackhole_dev->regs = regs;

	// shmem region occupies the bottom 8k bytes of the register space
	blackhole_dev->amp_dev.shmem = regs + 4096;
	blackhole_dev->amp_dev.shmem_size = 4096 * 2;
	memset_io(blackhole_dev->amp_dev.shmem, 0, 4096 *2);

	dev_info(&pdev->dev, "SHMEM @ 0: %32ph \n", blackhole_dev->amp_dev.shmem);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		goto error;
	err = devm_request_irq(&pdev->dev, irq, blackhole_irq_handler, IRQF_SHARED, dev_name(&pdev->dev), blackhole_dev);
	if (err)
		goto error;

	// Check with device if 2nd direction of interrupts (a.k.a msi register) is supported
	u32 prev_generation_value = le32_to_cpu(readl(&blackhole_dev->regs->msi_reg_generation)), curr_generation_value;
	writel(cpu_to_le32(prev_generation_value) + 1, &blackhole_dev->regs->msi_reg_generation);
	if (readl_poll_timeout(&blackhole_dev->regs->msi_reg_generation, curr_generation_value, le32_to_cpu(curr_generation_value) == prev_generation_value + 2, 1, 5000)){
		pr_err("Timeout waiting for generation value to update\n");
		goto error_irq;
	}
	dev_info(&pdev->dev, "msi supported %d address %llx", le32_to_cpu(readl(&blackhole_dev->regs->msi_reg_supported)), le64_to_cpu(readq(&blackhole_dev->regs->msi_reg_address)));
	if(le32_to_cpu(readl(&blackhole_dev->regs->msi_reg_supported))){
		blackhole_dev->msi_reg = devm_ioremap(&pdev->dev, le64_to_cpu(readq(&blackhole_dev->regs->msi_reg_address)), 4);
		if (IS_ERR(blackhole_dev->msi_reg)) {
			err = PTR_ERR(blackhole_dev->msi_reg);
			goto error_irq;
		}
	}

	platform_set_drvdata(pdev, blackhole_dev);
	blackhole_dev->pdev = pdev;

	blackhole_dev->amp_dev.ops = &blackhole_amp_ops;
	err = virtio_msg_amp_register(&blackhole_dev->amp_dev);
	if (err)
		goto error_irq;

	dev_info(&pdev->dev, "probe successful\n");

	return 0;

error_irq:
	free_irq(irq, blackhole_dev);

error:
	dev_info(&pdev->dev, "probe failed!\n");

	return err;
}

static void blackhole_remove(struct platform_device *pdev)
{
	struct blackhole_dev *blackhole_dev = platform_get_drvdata(pdev);

	virtio_msg_amp_unregister(&blackhole_dev->amp_dev);

	free_irq(platform_get_irq(blackhole_dev->pdev, 0), blackhole_dev);
	dev_info(&pdev->dev, "device removed\n");
}

static const struct of_device_id blackhole_device_id_table[] = {
	{ .compatible = "virtio,blackhole", },
	{ }
};
MODULE_DEVICE_TABLE(of, blackhole_device_id_table);

static struct platform_driver virtio_msg_blackhole_driver = {
	.driver		= {
		.name = DRV_NAME,
		.of_match_table = blackhole_device_id_table,
	},
	.probe = blackhole_probe,
	.remove = blackhole_remove,
};
module_platform_driver(virtio_msg_blackhole_driver);

MODULE_AUTHOR("Anirudh Srinivasan <asrinivasan@tenstorrent.com>");
MODULE_LICENSE("GPL v2");
