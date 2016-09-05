/*
 * Copyright © 2016 Digi International Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MTD_CRYPT_H__
#define __MTD_CRYPT_H__

int mtdcrypt_crypt(struct mtd_crypt_info *crypt_info, const u_char *buf,
		u_char *dstbuf, size_t len, loff_t block_offset, int op);

extern int fsl_otp_get_hwid(unsigned int *hwid);
#endif /* __MTD_CRYPT_H__ */

