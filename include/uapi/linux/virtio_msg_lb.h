/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Virtio message Loopback bus header.
 *
 * Copyright (C) 2025 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 */

#ifndef _LINUX_VIRTIO_MSG_LB_H
#define _LINUX_VIRTIO_MSG_LB_H

struct vmsg_lb_dev_info {
	unsigned int dev_id;
};

#define IOCTL_VMSG_LB_ADD					\
	_IOC(_IOC_NONE, 'P', 0, sizeof(struct vmsg_lb_dev_info))

#define IOCTL_VMSG_LB_REMOVE					\
	_IOC(_IOC_NONE, 'P', 1, sizeof(struct vmsg_lb_dev_info))

#endif /* _LINUX_VIRTIO_MSG_LB_H */
