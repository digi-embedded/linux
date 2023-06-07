/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause*/
/*
 * Copyright 2019-2022 NXP
 */

#ifndef ELE_MU_IOCTL_H
#define ELE_MU_IOCTL_H

/* IOCTL definitions. */

struct ele_mu_ioctl_setup_iobuf {
	u8 *user_buf;
	u32 length;
	u32 flags;
	u64 ele_addr;
};

struct ele_mu_ioctl_shared_mem_cfg {
	u32 base_offset;
	u32 size;
};

struct ele_mu_ioctl_get_mu_info {
	u8 ele_mu_id;
	u8 interrupt_idx;
	u8 tz;
	u8 did;
};

struct ele_mu_ioctl_signed_message {
	u8 *message;
	u32 msg_size;
	u32 error_code;
};

#define ELE_MU_IO_FLAGS_IS_INTPUT		(0x01u)
#define ELE_MU_IO_FLAGS_USE_SEC_MEM		(0x02u)
#define ELE_MU_IO_FLAGS_USE_SHORT_ADDR	(0x04u)

#define ELE_MU_IOCTL			0x0A /* like MISC_MAJOR. */
#define ELE_MU_IOCTL_ENABLE_CMD_RCV	_IO(ELE_MU_IOCTL, 0x01)
#define ELE_MU_IOCTL_SHARED_BUF_CFG	_IOW(ELE_MU_IOCTL, 0x02, \
					struct ele_mu_ioctl_shared_mem_cfg)
#define ELE_MU_IOCTL_SETUP_IOBUF	_IOWR(ELE_MU_IOCTL, 0x03, \
					struct ele_mu_ioctl_setup_iobuf)
#define ELE_MU_IOCTL_GET_MU_INFO	_IOR(ELE_MU_IOCTL, 0x04, \
					struct ele_mu_ioctl_get_mu_info)
#define ELE_MU_IOCTL_SIGNED_MESSAGE	_IOWR(ELE_MU_IOCTL, 0x05, \
					struct ele_mu_ioctl_signed_message)
#endif
