/**
 * MTD encryption layer
 *
 * Copyright (C) 2016 Digi International Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/scatterlist.h>
#include <linux/random.h>
#include <linux/crypto.h>
#include <linux/mtd/mtd.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/dma-mapping.h>
#include "mtdcrypt.h"
#include "../crypto/caam/sm.h"
#include "../crypto/caam/jr.h"
#include "../crypto/caam/caam_keyblob.h"

/* Debug dump */
static int debug;
module_param(debug, int, 0);

/* Encrypted key blob */
static char keyblob[KEY_ASCIIBLOB_BYTES + 1] = "";
module_param_string(keyblob, keyblob, sizeof(keyblob), S_IRUGO);
MODULE_PARM_DESC(key, "CAAM encrypted crypto key blob");
static int keyblob_size;
module_param(keyblob_size, int, S_IRUGO);
MODULE_PARM_DESC(keyblob_size, "CAAM encrypted crypto key size");
static int keyblob_offset = -1;
module_param(keyblob_offset, int, S_IRUGO);
MODULE_PARM_DESC(keyblob_offset, "Offset in NAND for the CAAM encrypted crypto key");
static int keyblob_part = -1;
module_param(keyblob_part, int, S_IRUGO);
MODULE_PARM_DESC(keyblob_part, "Partition number where the CAAM encrypted crypto key is stored");

#define DEFAULT_CIPHER		"aes"
#define DEFAULT_KEY_BYTES	32
#define MD5_DIGEST_SIZE		16

struct mtdcrypt_result {
	struct completion completion;
	int rc;
};

struct sm_key_job_result {
	int error;
	struct completion completion;
};

static void dump_hex(unsigned char *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if ((i % 16) == 0)
			printk("%s%08x: ", i ? "\n" : "",
					(unsigned int)&buf[i]);
		printk("%02x ", buf[i]);
	}
	printk("\n");
}

/**
 * mtdcrypt_calculate_md5 - calculates the md5 of @src
 * @dst: Pointer to 16 bytes of allocated memory
 * @crypt_info: Pointer to crypt_info struct
 * @src: Data to be md5'd
 * @len: Length of @src
 *
 * Uses the allocated crypto context that crypt_info references to
 * generate the MD5 sum of the contents of src.
 */
static int mtdcrypt_calculate_md5(char *dst, struct mtd_crypt_info *crypt_info,
		char *src, int len)
{
	struct scatterlist sg;
	struct hash_desc desc = {
		.tfm = crypt_info->hash_tfm,
		.flags = CRYPTO_TFM_REQ_MAY_SLEEP
	};
	int rc = 0;

	mutex_lock(&crypt_info->cs_hash_tfm_mutex);
	sg_init_one(&sg, (u8 *)src, len);
	if (!desc.tfm) {
		desc.tfm = crypto_alloc_hash("md5", 0, CRYPTO_ALG_ASYNC);
		if (IS_ERR(desc.tfm)) {
			rc = PTR_ERR(desc.tfm);
			pr_err("mtdcrypt: Error attempting to allocate crypto context; rc = [%d]\n",
					rc);
			goto out;
		}
		crypt_info->hash_tfm = desc.tfm;
	}
	rc = crypto_hash_init(&desc);
	if (rc) {
		pr_err("mtdcrypt: Error initializing crypto hash; rc = [%d]\n",
				rc);
		goto out;
	}
	rc = crypto_hash_update(&desc, &sg, len);
	if (rc) {
		pr_err("mtdcrypt:Error updating crypto hash; rc = [%d]\n", rc);
		goto out;
	}
	rc = crypto_hash_final(&desc, dst);
	if (rc) {
		pr_err("mtdcrypt: Error finalizing crypto hash; rc = [%d]\n",
				rc);
		goto out;
	}
out:
	mutex_unlock(&crypt_info->cs_hash_tfm_mutex);
	return rc;
}

/**
 * mtdcrypt_compute_root_iv
 *
 * @crypt_info: The cryptographic context
 *
 * Calculates the md5 hash of the encryption key in the provided crypto
 * context and uses it as root initizalization vector.
 *
 * On error, sets the root IV to all 0's.
 */
static int mtdcrypt_compute_root_iv(struct mtd_crypt_info *crypt_info)
{
	int rc = 0;

	BUG_ON(MAX_IV_BYTES > MD5_DIGEST_SIZE);
	if (!(crypt_info->flags & KEY_VALID)) {
		rc = -EINVAL;
		pr_warn("mtdcrypt: Session key not valid; "
				"cannot generate root IV\n");
		goto out;
	}
	rc = mtdcrypt_calculate_md5(crypt_info->root_iv, crypt_info,
			crypt_info->key, crypt_info->key_size);
	if (rc) {
		pr_warn("mtdcrypt: Error attempting to compute MD5 while generating root IV\n");
		memset(crypt_info->root_iv, 0, MAX_IV_BYTES);
		goto out;
	}

	if (debug) {
		printk("Generated new root_iv:\n");
		dump_hex(crypt_info->root_iv, MAX_IV_BYTES);
	}
out:
	return rc;
}

/**
 * mtdcrypt_derive_iv
 * @iv: destination for the derived iv value
 * @mtd_crypt_info: Pointer to mtd_crypt_info struct
 * @offset: Offset of the block whose IV we are to derive
 *
 * Generate the initialization vector from the given root IV and block
 * offset.
 *
 * Returns zero on success; non-zero on error.
 */
static int mtdcrypt_derive_iv(char *iv, struct mtd_crypt_info *crypt_info,
		loff_t offset)
{
	int rc = 0;
	char dst[MD5_DIGEST_SIZE];
	char src[MAX_IV_BYTES + BLOCK_ID_BYTES];

	if (debug) {
		printk("root iv:\n");
		dump_hex(crypt_info->root_iv, MAX_IV_BYTES);
	}
	memcpy(src, crypt_info->root_iv, MAX_IV_BYTES);
	memset((src + MAX_IV_BYTES), 0, BLOCK_ID_BYTES);
	snprintf((src + MAX_IV_BYTES), BLOCK_ID_BYTES, "%lld", offset);
	if (debug) {
		printk("source:\n");
		dump_hex(src, (MAX_IV_BYTES + BLOCK_ID_BYTES));
	}
	rc = mtdcrypt_calculate_md5(dst, crypt_info, src,
			(MAX_IV_BYTES + BLOCK_ID_BYTES));
	if (rc) {
		pr_warn("Error attempting to compute MD5 while generating IV\n");
		goto out;
	}
	memcpy(iv, dst, MAX_IV_BYTES);
	if (debug) {
		printk("derived iv:\n");
		dump_hex(iv, MAX_IV_BYTES);
	}
out:
	return rc;
}

static void mtdcrypt_complete(struct crypto_async_request *req, int rc)
{
	struct mtdcrypt_result *res = req->data;

	if (rc == -EINPROGRESS)
		return;

	res->rc = rc;
	complete(&res->completion);
}

static int mtdcrypt_is_all_bits_set(const u_char *buf, unsigned int len)
{
	while (len) {
		if (*buf != 0xff)
			break;
		buf++;
		len--;
	}
	return !len;
}

/**
 * mtdcrypt_scatterlist
 * @crypt_info: Pointer to the mtd_crypt_info struct (crypto context).
 * @dst_sg: Destination of the data after performing the crypto operation
 * @src_sg: Data to be encrypted or decrypted
 * @size: Length of data
 * @iv: Initialization vector to use
 * @op: MTD_ENCRYPT or MTD_DECRYPT to indicate the desired operation
 *
 * Encrypts/Decrypts the provided scattergather list with the provided
 * cryptographic context.
 *
 * Returns the number of bytes encrypted or decrypted; negative value on error
 */
static int mtdcrypt_scatterlist(struct mtd_crypt_info *crypt_info,
			     struct scatterlist *dst_sg,
			     struct scatterlist *src_sg, int size,
			     unsigned char *iv, int op)
{
	struct ablkcipher_request *req;
	struct mtdcrypt_result ecr;
	int rc = 0;

	BUG_ON(!crypt_info || !crypt_info->tfm ||
		!(crypt_info->flags & STRUCT_INITIALIZED));

	init_completion(&ecr.completion);

	mutex_lock(&crypt_info->cs_tfm_mutex);
	req = ablkcipher_request_alloc(crypt_info->tfm, GFP_NOFS);
	if (!req) {
		mutex_unlock(&crypt_info->cs_tfm_mutex);
		rc = -ENOMEM;
		goto out;
	}

	ablkcipher_request_set_callback(req,
			CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
			mtdcrypt_complete, &ecr);

	mutex_unlock(&crypt_info->cs_tfm_mutex);
	ablkcipher_request_set_crypt(req, src_sg, dst_sg, size, iv);
	rc = op == MTD_ENCRYPT ? crypto_ablkcipher_encrypt(req) :
			     crypto_ablkcipher_decrypt(req);
	if (rc == -EINPROGRESS || rc == -EBUSY) {
		struct mtdcrypt_result *ecr = req->base.data;

		wait_for_completion(&ecr->completion);
		rc = ecr->rc;
		reinit_completion(&ecr->completion);
	}
out:
	ablkcipher_request_free(req);
	return rc;
}

/**
 * mtdcrypt_op
 * @crypt_info: crypt_info containing cryptographic context for the
 *              encryption operation
 * @buf: The buffer to read from
 * @dstbuf: The buffer to write to
 * @len: Length of the src and dst buffers
 * @block_offset: Physical block offset for use in generating IV
 * @op: MTD_ENCRYPT or MTD_DECRYPT to indicate the desired operation
 *
 * Encrypts or decrypts the provided data.
 *
 * Return zero on success; non-zero otherwise
 */
static int mtdcrypt_op(struct mtd_crypt_info *crypt_info,
			const u_char *buf, u_char *dstbuf, int len,
			unsigned long block_offset, int op)
{
	char iv[MAX_IV_BYTES];
	struct sg_table sg_tbl;
	struct scatterlist *sgl;
	int rc, rest, i, tmp_len;
	int nents = 0;
	gfp_t gfp_mask = GFP_KERNEL | GFP_NOIO | GFP_DMA;
	u_char *mp_entry = NULL;

	if (len > crypt_info->block_size) {
		pr_err("mtdcrypt: Operation on block sized chunks only\n");
		return -EINVAL;
	}

	/* Divide the buffer in PAGE_SIZE fragments. */
	nents = len >> PAGE_SHIFT;
	rest = len % PAGE_SIZE;
	if (rest)
		nents++;

	if (nents > MAX_POOL_SIZE) {
		pr_err("mtdcrypt: Maximum number of pool entries reached.\n");
		rc = -ENOMEM;
		goto out;
	}

	rc = sg_alloc_table(&sg_tbl, nents, gfp_mask);
	if (rc) {
		rc = -ENOMEM;
		goto sgtbl_out;
	}
	for_each_sg(sg_tbl.sgl, sgl, nents, i) {
		tmp_len = (i == nents - 1 && rest) ? rest : PAGE_SIZE;
		mp_entry = mempool_alloc(crypt_info->mem_pool, gfp_mask);
		if (!mp_entry) {
			rc = -ENOMEM;
			goto mempool_out;
		}
		sg_set_buf(sgl, mp_entry, tmp_len);
	}
	rc = sg_copy_from_buffer(sg_tbl.sgl, nents, (void *)buf, len);
	if (rc != len) {
		pr_err("mtdcrypt: Error copying data to sgl, %d copied, expected %d\n",
				rc, len);
		rc = -EIO;
		goto mempool_out;
	}
	rc = mtdcrypt_derive_iv(iv, crypt_info, block_offset);
	if (rc) {
		pr_err("mtdcrypt: Error attempting to derive IV for block_offset [0x%.16llx]; rc = [%d]\n",
				(unsigned long long)(block_offset), rc);
		goto mempool_out;
	}
	rc = mtdcrypt_scatterlist(crypt_info, sg_tbl.sgl, sg_tbl.sgl,
			len, iv, op);
	if (rc < 0) {
		pr_err("mtdcrypt: Error attempting to crypt page with block_offset = [%ld]; rc = [%d]\n",
				block_offset, rc);
		goto mempool_out;
	}

	rc = sg_copy_to_buffer(sg_tbl.sgl, nents, dstbuf, len);
	if (rc != len) {
		pr_err("Error copying data from sgl, %d copied, expected %d\n",
				rc, len);
		rc = -EIO;
		goto mempool_out;
	}
	rc = 0;

mempool_out:
	for_each_sg(sg_tbl.sgl, sgl, nents, i)
		mempool_free(mp_entry, crypt_info->mem_pool);
sgtbl_out:
	sg_free_table(&sg_tbl);
out:
	return rc;
}

/* mtdcrypt_crypt
 *
 * @crypt_info: Cryptographic context
 * @buf: Block aligned buffer to read from
 * @dstbuf: Block aligned buffer to write to
 * @block_offset: Physical erase block offset
 * @op: MTD_ENCRYPT for encryption, MTD_DECRYPT for decryption
 *
 * Encrypts/Decrypts the provided data using the provided crypto context
 *
 * Returns 0 on success and a negative error number on failure.
 */
int mtdcrypt_crypt(struct mtd_crypt_info *crypt_info, const u_char *buf,
		u_char *dstbuf, size_t size, loff_t block_offset, int op)
{
	int rc = 0;
	int len;

	if (!(crypt_info->flags & KEY_SET) ||
	    !(crypt_info->flags & STRUCT_INITIALIZED))
			return -EINVAL;

	if (ALIGN(size, crypt_info->block_size) != size) {
		pr_err("mtdcrypt: Length should be block aligned\n");
		return -EINVAL;
	}

	/* Divide in block sized chunks */
	len = size > crypt_info->block_size ? crypt_info->block_size : size;
	while (size) {
		if (!mtdcrypt_is_all_bits_set(buf, len))
			rc = mtdcrypt_op(crypt_info, buf, dstbuf, len,
					block_offset, op);
		else {
			if (buf != dstbuf)
				memcpy(dstbuf, buf, len);
		}
		/* Processed len, adjust */
		size -= len;
		block_offset++;
		buf += len;
		dstbuf += len;
		len = size > crypt_info->block_size ?
			crypt_info->block_size : size;
	}
	return rc;
}
EXPORT_SYMBOL_GPL(mtdcrypt_crypt);

static int mtdcrypt_api_algify_cipher_name(char **algified_name,
					 char *cipher_name,
					 char *chaining_modifier)
{
	int cipher_name_len = strlen(cipher_name);
	int chaining_modifier_len = strlen(chaining_modifier);
	int algified_name_len;
	int rc;

	algified_name_len = (chaining_modifier_len + cipher_name_len + 3);
	(*algified_name) = kmalloc(algified_name_len, GFP_KERNEL);
	if (!(*algified_name)) {
		rc = -ENOMEM;
		goto out;
	}
	snprintf((*algified_name), algified_name_len, "%s(%s)",
		 chaining_modifier, cipher_name);
	rc = 0;
out:
	return rc;
}

#if defined(CONFIG_CRYPTO_DEV_FSL_CAAM)
static int mtdcrypt_compute_keymod(struct mtd_crypt_info *crypt_info,
		char *outbuf)
{
	unsigned int hwid[2] = {0};
	int ret = 0;

	ret = fsl_otp_get_hwid(hwid);
	if (!ret) {
		if (debug) {
			printk("mtdcrypt: hwid dump: ");
			dump_hex((unsigned char *)hwid, 8);
		}
		ret = mtdcrypt_calculate_md5(outbuf, crypt_info,
				(char *)hwid, sizeof(hwid));
	}
	return ret;
}

static void sm_key_job_done(struct device *dev, u32 *desc,
		u32 err, void *context)
{
	struct sm_key_job_result *res = context;

	res->error = err;	/* save off the error for postprocessing */
	complete(&res->completion);	/* mark us complete */
}

/* mtdcrypt_decrypt_key
 *
 * @jr_dev: Job ring device
 * @keyblobbuf: DMAble buffer containing the encrypted key
 * @bloblen: Length of the buffer containing the encrypted key
 * @kmodbuf: DMAble buffer containing the key modifier
 * @outbuf: DMAable buffer to copy the decrypted result to
 *
 * Decrypts the provided key blob using the CAAM unique and secure key.
 *
 * Return 0 on success or non-zero on failure.
 *
 */
static int mtdcrypt_decrypt_key(struct device *jr_dev, void *keyblobbuf,
		int bloblen, void *kmodbuf, void *outbuf)
{
	int retval = -EINVAL;
	int keylen = bloblen - BLOB_OVERHEAD;
	u32 dsize;
	dma_addr_t keyblob_dma = 0, keymod_dma = 0, outbuf_dma = 0;
	struct sm_key_job_result testres;
	u32 __iomem *decapdesc = NULL;

	if (!jr_dev)
		return -EINVAL;

	keyblob_dma = dma_map_single(jr_dev, keyblobbuf, bloblen,
				    DMA_TO_DEVICE);

	keymod_dma = dma_map_single(jr_dev, kmodbuf, GENMEM_KEYMOD_LEN,
				    DMA_TO_DEVICE);

	outbuf_dma = dma_map_single(jr_dev, outbuf, keylen,
				    DMA_FROM_DEVICE);

	/* Build the encapsulation job descriptor */
	dsize = blob_decap_jobdesc(&decapdesc, keymod_dma, keyblob_dma,
			(u8 *)outbuf_dma,  keylen, RED_KEY, SM_GENMEM,
			KEY_COVER_ECB);
	if (!dsize) {
		dev_err(jr_dev, "mtdcrypt: can't alloc a decapsulation descriptor\n");
		retval = -ENOMEM;
		goto out;
	}

	init_completion(&testres.completion);

	retval = caam_jr_enqueue(jr_dev, decapdesc, sm_key_job_done,
			      &testres);
	if (!retval) {
		wait_for_completion_interruptible(&testres.completion);
		dev_dbg(jr_dev, "job ring return %d\n", testres.error);
		if (!testres.error)
			dma_sync_single_for_cpu(jr_dev, outbuf_dma, keylen,
				DMA_FROM_DEVICE);
		retval = testres.error;
	}

out:
	if (outbuf_dma)
		dma_unmap_single(jr_dev, outbuf_dma, keylen,
				DMA_FROM_DEVICE);
	if (keymod_dma)
		dma_unmap_single(jr_dev, keymod_dma, GENMEM_KEYMOD_LEN,
				DMA_TO_DEVICE);
	if (keyblob_dma)
		dma_unmap_single(jr_dev, keyblob_dma, bloblen,
				DMA_TO_DEVICE);
	kfree(decapdesc);
	return retval;
}
#endif

/*
 * Convert string with hexadecimal characters into a hex number
 * @in: Pointer to input string
 * @out Pointer to output number array
 * @len Number of elements in the output array
*/
#define STR_HEX_CHUNK			8
void strtohex(char *in, unsigned long *out, int len)
{
	char tmp[] = "ffffffff";
	int i, j;

	for (i = 0, j = 0; j < len; i += STR_HEX_CHUNK, j++) {
		strncpy(tmp, &in[i], STR_HEX_CHUNK);
		out[j] = cpu_to_be32(simple_strtol(tmp, NULL, 16));
	}
}

static int mtdcrypt_get_key_from_part(int keyblob_part, int keyblob_size,
				      int keyblob_offset, char *keyblob_str)
{
	struct mtd_info *mtd_envpart;
	int retlen;

	if (keyblob_size > MAX_KEYBLOB_BYTES) {
		pr_err("mtdcrypt: Missing valid key blob size\n");
		return -EINVAL;
	}

	mtd_envpart = get_mtd_device(NULL, keyblob_part);
	if (mtd_read(mtd_envpart, keyblob_offset, keyblob_size,
				&retlen, keyblob_str)) {
		pr_err("mtdcrypt: Failed to read keyblob from media\n");
		return -EINVAL;
	}

	if (retlen != keyblob_size) {
		pr_err("mtdcrypt: Failed to read keyblob from media\n");
		return -EINVAL;
	}
	return 0;
}

static int mtdcrypt_set_key(struct mtd_info *mtd)
{
	struct mtd_crypt_info *crypt_info = mtd->crypt_info;
	char *keyblob_str = NULL;
#if defined(CONFIG_CRYPTO_DEV_FSL_CAAM)
	char *keymod = NULL;
#endif
	int ret = 0;

	if (!crypt_info)
		return -EINVAL;

	keyblob_str = kzalloc(KEY_ASCIIBLOB_BYTES+1, GFP_KERNEL | GFP_DMA);
	if (!keyblob_str)
		return -ENOMEM;

	/* Kernel command line arguments have priority */
	if (keyblob[0] != '\0' && keyblob_size) {
		if (keyblob_size > MAX_KEYBLOB_BYTES) {
			pr_err("mtdcrypt: Missing valid key blob size\n");
			ret = -EINVAL;
			goto out_err1;
		}
		strtohex(keyblob, (unsigned long *)keyblob_str,
				keyblob_size/sizeof(unsigned long));
	} else if (keyblob_offset && keyblob_part && keyblob_size) {
		ret = mtdcrypt_get_key_from_part(keyblob_part, keyblob_size,
				keyblob_offset, keyblob_str);
		if (ret)
			goto out_err1;

	} else if (crypt_info->np) {
		ret = of_property_read_u32(crypt_info->np,
				"encryption-keyblob-size",
				&keyblob_size);
		if (ret) {
			pr_err("mtdcrypt: Missing key blob size\n");
			ret = -EINVAL;
			goto out_err1;
		}
		ret = of_property_read_u32(crypt_info->np,
				"encryption-keyblob-part",
				&keyblob_part);
		if (ret) {
			pr_err("mtdcrypt: Missing key blob part\n");
			ret = -EINVAL;
			goto out_err1;
		}
		ret = of_property_read_u32(crypt_info->np,
				"encryption-keyblob-offset",
				&keyblob_offset);
		if (ret) {
			pr_err("mtdcrypt: Missing key blob offset\n");
			ret = -EINVAL;
			goto out_err1;
		}
		ret = mtdcrypt_get_key_from_part(keyblob_part, keyblob_size,
				keyblob_offset, keyblob_str);
		if (ret)
			goto out_err1;
	} else {
		/* Default values */
		keyblob_size = DEFAULT_KEY_BYTES + BLOB_OVERHEAD;
		/* environment partition in default mtdparts */
		keyblob_part = 1;
		/* On the third erase block, after two erase blocks used
		 * for the redundant U-Boot environments */
		keyblob_offset = mtd->crypt_info->erase_size * 2;
		ret = mtdcrypt_get_key_from_part(keyblob_part, keyblob_size,
				keyblob_offset, keyblob_str);
		if (ret)
			goto out_err1;
	}

	crypt_info->key_size = keyblob_size - BLOB_OVERHEAD;

	if (debug) {
		printk("mtdcrypt: keyblob (%d bytes) dump: ", keyblob_size);
		dump_hex(keyblob_str,  keyblob_size);
	}

#if defined(CONFIG_CRYPTO_DEV_FSL_CAAM)
	keymod = kmalloc(MD5_DIGEST_SIZE, GFP_KERNEL | GFP_DMA);
	if (!keymod) {
		ret = -ENOMEM;
		goto out_err1;
	}
	ret = mtdcrypt_compute_keymod(crypt_info, keymod);
	if (ret) {
		pr_err("mtdcrypt: Error calculating key modifier\n");
		goto out_err2;
	}

	if (debug) {
		printk("mtdcrypt: keymod dump: ");
		dump_hex(keymod, MD5_DIGEST_SIZE);
	}
	ret = mtdcrypt_decrypt_key(crypt_info->jr_dev, keyblob_str,
				keyblob_size, keymod,
				crypt_info->key);
	if (ret) {
		pr_err("mtdcrypt: Key decryption error.\n");
		ret = -EINVAL;
		goto out_err2;
	}
#else
	if (debug) {
		pr_warn("mtdcrypt:  Security warning: Using keyblob as cipher key\n");
		memcpy(crypt_info->key, keyblob, crypt_info->key_size);
	} else {
		pr_err("mtdcrypt: CRYPTO_DEV_FSL_CAAM needs to be enabled\n");
		ret = -EINVAL;
		goto out_err2;
	}
#endif
	if (debug) {
		printk("Session key (size %d)n",
				crypt_info->key_size);
		dump_hex(crypt_info->key, crypt_info->key_size);
	}
	crypt_info->flags |= KEY_VALID;
out_err2:
#if defined(CONFIG_CRYPTO_DEV_FSL_CAAM)
	kfree(keymod);
#endif
out_err1:
	kfree(keyblob_str);
	return ret;
}

/* mtdcrypt_init_crypt_info
 *
 * @mtd: Pointer to mtd_info struct for the MTD partition.
 *
 * Initializes cryptographic context of mtd partition. It will use a AES
 * symmetric 512 bit key cipher and the Cipher Block Chaining (CBC) mode of
 * operation. The block size is set to the minimal MTD I/O unit size.
 *
 * Return 0 on success or non-zero on failure.
 *
 */
int mtdcrypt_init_crypt_info(struct mtd_info *mtd)
{
	char *full_alg_name;
	int rc = -EINVAL;
	int i;

	if (!mtd->crypt_info) {
		mtd->crypt_info = kzalloc(sizeof(struct mtd_crypt_info),
					GFP_KERNEL);
		if (!mtd->crypt_info)
			return -ENOMEM;
	}

	if (mtd->crypt_info->flags & STRUCT_INITIALIZED) {
		pr_err("mtdcrypt: Already initialized\n");
		return -EINVAL;
	}

	mutex_init(&mtd->crypt_info->cs_tfm_mutex);
	mutex_init(&mtd->crypt_info->cs_hash_tfm_mutex);

	mutex_lock(&mtd->crypt_info->cs_tfm_mutex);

	mtd->crypt_info->mem_pool = mempool_create_kmalloc_pool(MAX_POOL_SIZE,
			PAGE_SIZE);
	if (!mtd->crypt_info->mem_pool) {
		pr_err("mtdcrypt: Error allocating memory pool\n");
		return -ENOMEM;
	}

	mtd->crypt_info->block_size = mtd->writesize >> mtd->subpage_sft;
	mtd->crypt_info->erase_size = mtd->erasesize;
	i = mtd->crypt_info->block_size;
	while (i >>= 1)
		++mtd->crypt_info->block_shift;
	pr_debug("mtdcrypt: Block size %d , shift %d\n",
			mtd->crypt_info->block_size,
			mtd->crypt_info->block_shift);
	strcpy(mtd->crypt_info->cipher, DEFAULT_CIPHER);
	mtd->crypt_info->key_size = DEFAULT_KEY_BYTES;

#if defined(CONFIG_CRYPTO_DEV_FSL_CAAM)
	mtd->crypt_info->jr_dev = caam_jr_alloc();
#endif
	mtd->crypt_info->key = kzalloc(MAX_KEY_BYTES, GFP_KERNEL | GFP_DMA);
	rc = mtdcrypt_set_key(mtd);
	if (rc) {
		pr_err("mtdcrypt: Unable to set crypto key.\n");
		rc = -EINVAL;
		goto out_unlock;
	}

#if defined(CONFIG_CRYPTO_DEV_FSL_CAAM)
	caam_jr_free(mtd->crypt_info->jr_dev);
#endif
	mtdcrypt_compute_root_iv(mtd->crypt_info);
	if (mtd->crypt_info->tfm) {
		rc = 0;
		goto out_unlock;
	}
	rc = mtdcrypt_api_algify_cipher_name(&full_alg_name,
					mtd->crypt_info->cipher, "cbc");
	if (rc)
		goto out_unlock;
	mtd->crypt_info->tfm = crypto_alloc_ablkcipher(full_alg_name, 0, 0);
	if (IS_ERR(mtd->crypt_info->tfm)) {
		rc = PTR_ERR(mtd->crypt_info->tfm);
		mtd->crypt_info->tfm = NULL;
		pr_err("mtdcrypt: Error initializing cipher [%s]\n",
				full_alg_name);
		goto out_free;
	}
	crypto_ablkcipher_set_flags(mtd->crypt_info->tfm,
			CRYPTO_TFM_REQ_WEAK_KEY);
	rc = crypto_ablkcipher_setkey(mtd->crypt_info->tfm,
			mtd->crypt_info->key, mtd->crypt_info->key_size);
	if (rc) {
		pr_err("mtdcrypt: Error setting key; rc = [%d]\n", rc);
		rc = -EINVAL;
		goto out_free;
	}
	mtd->crypt_info->flags |=  (KEY_SET | STRUCT_INITIALIZED);
	rc = 0;
	pr_debug("mtdcrypt: Initializing cipher [%s]; strlen = [%d]; "
		"key_size_bits = [%zd]\n",
		mtd->crypt_info->cipher, (int)strlen(mtd->crypt_info->cipher),
		mtd->crypt_info->key_size << 3);

out_free:
	kfree(full_alg_name);
out_unlock:
	mutex_unlock(&mtd->crypt_info->cs_tfm_mutex);
	return rc;
}
EXPORT_SYMBOL_GPL(mtdcrypt_init_crypt_info);

/* mtdcrypt_destroy_crypt_info
 *
 * @mtd: Pointer to mtd_info struct for the MTD partition.
 *
 * Destroys the previously initialized cryptographic context.
 *
 */
void mtdcrypt_destroy_crypt_info(struct mtd_info *mtd)
{
	mutex_destroy(&mtd->crypt_info->cs_tfm_mutex);
	mutex_destroy(&mtd->crypt_info->cs_hash_tfm_mutex);
	kfree(mtd->crypt_info->key);
	mempool_destroy(mtd->crypt_info->mem_pool);
	kfree(mtd->crypt_info);
}
EXPORT_SYMBOL_GPL(mtdcrypt_destroy_crypt_info);
