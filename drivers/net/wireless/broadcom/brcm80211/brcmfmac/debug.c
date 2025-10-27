// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2012 Broadcom Corporation
 */
#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/devcoredump.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/dcache.h>
#include <linux/user_namespace.h>
#include <linux/statfs.h>

#include <brcmu_wifi.h>
#include <brcmu_utils.h>
#include "core.h"
#include "bus.h"
#include "fweh.h"
#include "debug.h"
#include "common.h"


static int
brcmf_debug_msgtrace_seqchk(u32 *prev, u32 cur)
{
	if ((cur == 0 && *prev == 0xFFFFFFFF) || ((cur - *prev) == 1)) {
		goto done;
	} else if (cur == *prev) {
		brcmf_dbg(FWCON, "duplicate trace\n");
		return -1;
	} else if (cur > *prev) {
		brcmf_dbg(FWCON, "lost %d packets\n", cur - *prev);
	} else {
		brcmf_dbg(FWCON, "seq out of order, host %d, dongle %d\n",
			  *prev, cur);
	}
done:
	*prev = cur;
	return 0;
}

static int
brcmf_debug_msg_parser(void *event_data)
{
	int err = 0;
	struct msgtrace_hdr *hdr;
	char *data, *s;
	static u32 seqnum_prev;

	hdr = (struct msgtrace_hdr *)event_data;
	data = (char *)event_data + MSGTRACE_HDRLEN;

	/* There are 2 bytes available at the end of data */
	data[ntohs(hdr->len)] = '\0';

	if (ntohl(hdr->discarded_bytes) || ntohl(hdr->discarded_printf)) {
		brcmf_dbg(FWCON, "Discarded_bytes %d discarded_printf %d\n",
			  ntohl(hdr->discarded_bytes),
				ntohl(hdr->discarded_printf));
	}

	err = brcmf_debug_msgtrace_seqchk(&seqnum_prev, ntohl(hdr->seqnum));
	if (err)
		return err;

	while (*data != '\0' && (s = strstr(data, "\n")) != NULL) {
		*s = '\0';
		brcmf_dbg(FWCON, "CONSOLE: %s\n", data);
		data = s + 1;
	}
	if (*data)
		brcmf_dbg(FWCON, "CONSOLE: %s", data);

	return err;
}

static int
brcmf_debug_trace_parser(struct brcmf_if *ifp,
			 const struct brcmf_event_msg *evtmsg,
			 void *event_data)
{
	int err = 0;
	struct msgtrace_hdr *hdr;

	hdr = (struct msgtrace_hdr *)event_data;
	if (hdr->version != MSGTRACE_VERSION) {
		brcmf_dbg(FWCON, "trace version mismatch host %d dngl %d\n",
			  MSGTRACE_VERSION, hdr->version);
		err = -EPROTO;
		return err;
	}

	if (hdr->trace_type == MSGTRACE_HDR_TYPE_MSG)
		err = brcmf_debug_msg_parser(event_data);

	return err;
}

static int
brcmf_debug_del_dir_file(struct brcmf_bus *bus)
{
	struct path dir_path;
	struct dentry *parent, *child;
	struct inode *dir_inode;
	char *del_prefix = "coredump_";
	int ret;

	ret = kern_path(bus->drvr->settings->coredump_path, 0, &dir_path);

	if (ret) {
		brcmf_err("Can't find path in kernel\n");
		return ret;
	}

	parent = dir_path.dentry;
	dir_inode = parent->d_inode;

	/* lock dir for searching */
	inode_lock(dir_inode);
	spin_lock(&parent->d_lock);

	/* search all files under storing path and store file point on chlid */
#if (KERNEL_VERSION(6, 8, 0) <= LINUX_VERSION_CODE)
	hlist_for_each_entry(child, &parent->d_children, d_sib) {
#else
	list_for_each_entry(child, &parent->d_subdirs, d_child) {
#endif
		if (d_is_negative(child))
			continue;

		brcmf_dbg(INFO, "Find file %s\n", child->d_name.name);
		if (strncmp(child->d_name.name, del_prefix, strlen(del_prefix)) == 0) {
			dget(child);
			spin_unlock(&parent->d_lock);

			brcmf_dbg(INFO, "del %s\n", child->d_name.name);
			/* del the file */
#if (KERNEL_VERSION(5, 12, 0) > LINUX_VERSION_CODE)
			ret = vfs_unlink(dir_inode, child, NULL);
#elif (KERNEL_VERSION(6, 3, 0) > LINUX_VERSION_CODE)
			ret = vfs_unlink(&init_user_ns, dir_inode, child, NULL);
#else
			ret = vfs_unlink(&nop_mnt_idmap, dir_inode, child, NULL);
#endif
			if (ret)
				brcmf_err("vfs_unlink failed: %d\n", ret);
			dput(child);
			spin_lock(&parent->d_lock);
		}
	}
	spin_unlock(&parent->d_lock);

	inode_unlock(dir_inode);
	path_put(&dir_path);
	return 0;
}

static unsigned long long
brcmf_debug_cal_avail_space(struct brcmf_bus *bus, const char *cal_path)
{
	struct path dir_path;
	unsigned long long avail_bytes = 0;
	int err = 0;
	struct kstatfs stat;

	err = kern_path(cal_path, 0, &dir_path);
	if (err) {
		brcmf_err("Can't find path in kernel\n");
		return avail_bytes;
	}

	err = vfs_statfs(&dir_path, &stat);
	if (err) {
		brcmf_err("Fail to stat dir %s. error: %d\n", cal_path, err);
		goto exit;
	}
	avail_bytes = (unsigned long long)stat.f_bavail * stat.f_bsize;
	brcmf_dbg(INFO, "space available bytes:%llu\n", avail_bytes);
exit:
	path_put(&dir_path);
	return avail_bytes;
}

int brcmf_debug_write_file(struct brcmf_bus *bus, const char *file_name,
			   u32 flags, void *buf, size_t size)
{
	int ret = 0;
	char *coredump_path;
	struct file *fp = NULL;
	loff_t pos = 0;
	unsigned long long avail_bytes;

	/* If set up driver parameter file size,
	 * it would check on writing space whether enough or not.
	 */
	if (bus->drvr->settings->coredump_file_size) {
		if (size > bus->drvr->settings->coredump_file_size) {
			brcmf_err("coredump %s is over setting storing size.\n", file_name);
			goto exit;
		}

		/* Check on current available space.
		 * Delete old file coredump_* if space is insufficient.
		 */
		coredump_path = bus->drvr->settings->coredump_path;
		avail_bytes = brcmf_debug_cal_avail_space(bus, coredump_path);
		if (size > avail_bytes) {
			brcmf_dbg(INFO, "Not enough space. Try to del old coredump files.\n");
			ret = brcmf_debug_del_dir_file(bus);
			if (ret) {
				brcmf_err("del file error\n");
				goto exit;
			} else {
				avail_bytes = brcmf_debug_cal_avail_space(bus, coredump_path);
				if (size > avail_bytes) {
					brcmf_err("After del, space isn't enough to store coredump.\n");
					goto exit;
				}
			}
		}
	}

	/* open file to write */
	fp = filp_open(file_name, flags, 0664);
	if (IS_ERR(fp)) {
		brcmf_err("open file error:%s, err = %ld\n", file_name, PTR_ERR(fp));
		goto exit;
	}

	/* write to file */
#if (KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE)
	ret = vfs_write(fp, buf, size, &pos);
#else
	ret = vfs_write(fp, buf, size, pos);
#endif
	if (ret < 0) {
		brcmf_err("write file error, err = %d\n", ret);
		goto exit;
	}

	/* Sync file from filesystem to physical media */
	ret = vfs_fsync(fp, 0);
	if (ret < 0) {
		brcmf_err("sync file error, error = %d\n", ret);
		goto exit;
	}
	ret = BRCMF_OK;

exit:
	/* close file before return */
	if (!IS_ERR(fp))
		filp_close(fp, current->files);

	return ret;
}

void brcmf_debug_get_dump_time(char *str)
{
	struct timespec64 curtime;

	if (!strlen(str)) {
		ktime_get_boottime_ts64(&curtime);
		snprintf(str, BRCMF_DEBUG_DUMP_TIME_BUF_LEN, BRCMF_LOG_DUMP_BOOTTIME,
			 curtime.tv_sec, curtime.tv_nsec / NSEC_PER_USEC);
	}
}

int brcmf_debug_ramdump_to_file(struct brcmf_bus *bus, void *dump, size_t size, char *fname)
{
	int ret = 0;
	char memdump_path[256] = {0};
	char debug_dump_time_str[BRCMF_DEBUG_DUMP_TIME_BUF_LEN] = {0};
	u32 file_mode;

	brcmf_debug_get_dump_time(debug_dump_time_str);

	snprintf(memdump_path, sizeof(memdump_path), "%s%s_%s.bin",
		 bus->drvr->settings->coredump_path, fname, debug_dump_time_str);
	file_mode = O_CREAT | O_WRONLY;

	/* print SOCRAM dump file path */
	brcmf_err("%s: file_path = %s\n", __func__, memdump_path);

	/* Write file */
	ret = brcmf_debug_write_file(bus, memdump_path, file_mode, dump, size);

	return ret;
}

int brcmf_debug_create_memdump(struct brcmf_bus *bus, const void *data,
			       size_t len)
{
	void *dump;
	size_t ramsize;
	int err;
	char *fname = "coredump";

	ramsize = brcmf_bus_get_ramsize(bus);
	if (!ramsize)
		return -ENOTSUPP;

	dump = vzalloc(len + ramsize);
	if (!dump)
		return -ENOMEM;

	if (data && len > 0)
		memcpy(dump, data, len);
	err = brcmf_bus_get_memdump(bus, dump + len, ramsize);
	if (err) {
		vfree(dump);
		return err;
	}

	brcmf_debug_ramdump_to_file(bus, dump, len + ramsize, fname);

	dev_coredumpv(bus->dev, dump, len + ramsize, GFP_KERNEL);

	return 0;
}


int brcmf_debug_fwlog_init(struct brcmf_pub *drvr)
{
	return brcmf_fweh_register(drvr, BRCMF_E_TRACE,
				brcmf_debug_trace_parser);
}

struct dentry *brcmf_debugfs_get_devdir(struct brcmf_pub *drvr)
{
	return drvr->wiphy->debugfsdir;
}

void brcmf_debugfs_add_entry(struct brcmf_pub *drvr, const char *fn,
			    int (*read_fn)(struct seq_file *seq, void *data))
{
	WARN(!drvr->wiphy->debugfsdir, "wiphy not (yet) registered\n");
	debugfs_create_devm_seqfile(drvr->bus_if->dev, fn,
				    drvr->wiphy->debugfsdir, read_fn);
}
