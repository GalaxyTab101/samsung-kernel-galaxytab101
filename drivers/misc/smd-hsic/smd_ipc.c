/*
 * driver/misc/smd-hsic/smd_ipc.c
 *
 * driver supporting miscellaneous functions for Samsung P3 device
 *
 * COPYRIGHT(C) Samsung Electronics Co., Ltd. 2006-2011 All Right Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/mutex.h>
#include <linux/gfp.h>

#include "smd_ipc.h"
#include "smd_core.h"
#include "../smdctl/smd_ctl.h"

#define IPC_COMP_REQ		1
#define WRITE_TIME_OUT		(HZ*5)

#define IPC_TEMP_BUFFER_SIZE	4096

#define IPC_TX_URB_CNT		1
#define IPC_RX_URB_CNT		1

#undef DEBUG_LOG

static void ipc_rx_comp(struct urb *urb);
static void ipc_tx_comp(struct urb *urb);

static int smdipc_rx_usb(struct str_smdipc *smdipc)
{
	int ret = 0;
	char *buf = smdipc->tpbuf;
	struct str_hsic *hsic = &smdipc->hsic;

	if (smdipc->pm_stat == IPC_PM_STATUS_SUSPEND ||
		smdipc->pm_stat == IPC_PM_STATUS_DISCONNECT)
		return -ENODEV;

	if (!hsic || !hsic->usb || !hsic->rx_urb[0].urb)
		return -EFAULT;

	tegra_ehci_txfilltuning();

	usb_fill_bulk_urb(hsic->rx_urb[0].urb, hsic->usb, hsic->rx_pipe,
			  buf, IPC_TEMP_BUFFER_SIZE, ipc_rx_comp,
			  (void *)smdipc);

	hsic->rx_urb[0].urb->transfer_flags = 0;

	ret = usb_submit_urb(hsic->rx_urb[0].urb, GFP_ATOMIC);
	if (ret < 0)
		pr_err("%s:usb_submit_urb() failed:(%d)\n", __func__, ret);

	return ret;
}

static void ipc_rx_comp(struct urb *urb)
{
	int status;
	int ret;
	struct str_smdipc *smdipc;

	pr_debug("%s: Enter read size:%d\n", __func__, urb->actual_length);

	smdipc = urb->context;
	status = urb->status;

	if (!atomic_read(&smdipc->opened)) {
		pr_err("%s Not Initialised\n", __func__);
		return;
	}

	if (smdipc->pm_stat == IPC_PM_STATUS_SUSPEND)
		return;

	usb_mark_last_busy(smdipc->hsic.usb);

	switch (status) {
	case 0:
		if (urb->actual_length) {
			ret = memcpy_to_ringbuf(&smdipc->read_buf,
						smdipc->tpbuf,
						urb->actual_length);
			if (ret < 0)
				pr_err("%s:memcpy_to_ringbuf failed :%d\n",
				       __func__, ret);

			wake_up(&smdipc->poll_wait);
			wake_lock_timeout(&smdipc->rxguide_lock, HZ/2);
#ifdef DEBUG_LOG
			pr_info("=============== RECEIVE IPC ==============\n");
			dump_buffer(smdipc->tpbuf, urb->actual_length);
			pr_info("==========================================\n");
#endif
		}
		goto resubmit;

	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		pr_err("%s:LINK ERROR:%d\n", __func__, status);
		return;

	case -EOVERFLOW:
		smdipc->stat.rx_over_err++;
		break;

	case -EILSEQ:
		smdipc->stat.rx_crc_err++;
		break;

	case -ETIMEDOUT:
	case -EPROTO:
		smdipc->stat.rx_pro_err++;
		break;

	}
	smdipc->stat.rx_err++;

resubmit:
	smdipc_rx_usb(smdipc);
	return;
}

static void ipc_tx_comp(struct urb *urb)
{
	pr_debug("%s: Enter write size:%d\n", __func__, urb->actual_length);

	if (urb->status != 0)
		pr_warn("%s:Wrong Status:%d\n", __func__, urb->status);

	kfree(urb->transfer_buffer);
	usb_free_urb(urb);

	return;
}

static int smdipc_start(struct str_smdipc *smdipc)
{
	char *buf = NULL;
	struct str_hsic *hsic;
	int r;
	struct urb *urb;
	char data[1] = { 'a' };

	pr_info("%s:Start\n", __func__);

	hsic = &smdipc->hsic;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return -ENODEV;

	buf = kmalloc(16, GFP_KERNEL);
	if (!buf) {
		pr_err("%s:malloc for buffer failed\n", __func__);
		usb_free_urb(urb);
		return -ENOMEM;
	}
	memcpy(buf, data, 1);

	r = smdhsic_pm_resume_AP();
	if (r < 0) {
		pr_err("%s: HSIC Resume Failed %d\n", __func__, r);
		usb_free_urb(urb);
		kfree(buf);
		return r;
	}

	usb_fill_bulk_urb(urb, hsic->usb, hsic->tx_pipe,
			  buf, 1, ipc_tx_comp, smdipc);
	urb->transfer_flags = URB_ZERO_PACKET;

	r = usb_submit_urb(urb, GFP_KERNEL);
	if (r) {
		pr_err("%s:usb_submit_urb() failed (%d)\n", __func__, r);
		usb_free_urb(urb);
		kfree(buf);
		return r;
	}

	/* now RiL can handle reset event, check if unhandled event is remained */
	smdctl_request_connection_recover(false);

	return 0;

}

static int smdipc_open(struct inode *inode, struct file *file)
{
	int r;
	struct cdev *cdev = inode->i_cdev;
	struct str_hsic *hsic;
	struct str_smdipc *smdipc;

	pr_info("%s: Enter\n", __func__);
	smdipc = container_of(cdev, struct str_smdipc, cdev);
	hsic = &smdipc->hsic;

	if (atomic_cmpxchg(&smdipc->opened, 0, 1)) {
		pr_err("%s : already opened..\n", __func__);
		r = -EBUSY;
		goto err_opened;
	}

	r = smdipc_rx_usb(smdipc);
	if (r) {
		pr_err("%s:smdipc_rx_usb() failed\n (%d)\n", __func__, r);
		goto err_smdipc_rx_usb;
	}
	init_err_stat(&smdipc->stat);

	file->private_data = smdipc;

	r = smdipc_start(smdipc);
	if (r) {
		pr_err("smdipc_start() failed\n");
		goto err_smdipc_start;
	}

	smdipc->open_fail_cnt = 0;
	return 0;

err_smdipc_start:
	smd_kill_urb(&smdipc->hsic);
err_smdipc_rx_usb:
	atomic_set(&smdipc->opened, 0);
err_opened:
	if (smdipc->open_fail_cnt++ > 5)
		smdctl_request_connection_recover(true);
	return r;
}

static int smdipc_release(struct inode *inode, struct file *file)
{
	struct str_smdipc *smdipc = file->private_data;
	pr_info("%s: Enter\n", __func__);

	smd_kill_urb(&smdipc->hsic);
	atomic_set(&smdipc->opened, 0);

	return 0;
}

static int smdipc_fmt_hdlc(const char __user *buf, char *dest_buf, ssize_t len)
{
	int r;
	struct fmt_hdr hd;
	unsigned int offset;
	unsigned char hdlcst = HDLC_START;
	unsigned char hdlcen = HDLC_END;

	hd.len = len + sizeof(struct fmt_hdr);
	hd.control = 0;
	offset = 0;
	memcpy(dest_buf + offset, &hdlcst, 1);
	offset += 1;
	memcpy(dest_buf + offset, &hd, sizeof(struct fmt_hdr));
	offset += sizeof(struct fmt_hdr);
	r = copy_from_user(dest_buf + offset, buf, len);
	if (r < 0) {
		pr_err("%s:copy_form_user failed(ret : %d)..\n", __func__,
		       r);
		return -EFAULT;
	}
	offset += len;
	memcpy(dest_buf + offset, &hdlcen, 1);
	offset += 1;

	return offset;
}

static ssize_t smdipc_write(struct file *file, const char __user * buf,
			    size_t count, loff_t *ppos)
{
	int r;
	char *drvbuf;
	int pktsz;
	struct str_smdipc *smdipc;
	struct str_hsic *hsic;
	struct urb *urb;

	pr_debug("%s: Enter write size:%d\n", __func__, count);

	smdipc = file->private_data;
	hsic = &smdipc->hsic;

	if (!hsic)
		return -ENODEV;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		pr_err("%s: tx urb is NULL\n", __func__);
		return -ENODEV;
	}

	drvbuf = kzalloc(count + sizeof(struct fmt_hdr) + 2, GFP_KERNEL);
	if (!drvbuf) {
		pr_err("%s:kmalloc for drvbuf failed\n", __func__);
		r = -EAGAIN;
		goto exit;
	}

	pktsz = smdipc_fmt_hdlc(buf, drvbuf, count);
	if (pktsz < 0) {
		pr_err("%s Total Pkt : %d, Req : %d\n", __func__, pktsz,
		       count);
		r = pktsz;
		goto exit;
	}
	if (pktsz != count + sizeof(struct fmt_hdr) + 2) {
		pr_warn("%s Size Mismatch Pkt: %d, Req: %d\n", __func__,
		       pktsz, count);
	}
#if 0
	pr_info("================= TRANSMIT IPC =================\n");
	dump_buffer(drvbuf, pktsz);
	pr_info("================================================\n");
#endif

	r = smdhsic_pm_resume_AP();
	if (r < 0) {
		pr_err("%s: HSIC Resume Failed %d\n", __func__, r);
		r = -EAGAIN;
		goto exit;
	}

	usb_fill_bulk_urb(urb, hsic->usb, hsic->tx_pipe,
			  drvbuf, pktsz, ipc_tx_comp, smdipc);

	urb->transfer_flags = URB_ZERO_PACKET;

	usb_mark_last_busy(hsic->usb);
	r = usb_submit_urb(urb, GFP_KERNEL);
	if (r) {
		pr_err("%s:usb_submit_urb() failed (%d)\n", __func__, r);
		goto exit;
	}

	return count;
exit:
	if (urb)
		usb_free_urb(urb);
	if (drvbuf)
		kfree(drvbuf);
	return r;
}

static ssize_t smdipc_read(struct file *file, char *buf, size_t count,
			   loff_t *f_pos)
{
	int ret;
	int pktsize = 0;
	struct str_smdipc *smdipc;
	struct str_ring_buf *readbuf;
	char temp;
	struct fmt_hdr hdr;

	pr_debug("%s: Enter\n", __func__);

	smdipc = (struct str_smdipc *)file->private_data;
	readbuf = &smdipc->read_buf;

	if (!readbuf->buf)
		return -EFAULT;

	if (!get_remained_size(readbuf))
		goto done;

find_hdlc_start:
	ret = memcpy_from_ringbuf(readbuf, &temp, 1);
	if ((!ret) && (temp != HDLC_START)) {
		pr_warn("%s:HDLC START Fail: %d, ret: %d\n",
		       __func__, temp, ret);
		goto find_hdlc_start;
	}
	ret = memcpy_from_ringbuf(readbuf, (char *)&hdr, sizeof(hdr));
	if (!ret) {
		pr_warn("%s:read ipc hdr failed: %d\n", __func__, ret);
		ret = -1;
	}
	pktsize = hdr.len - sizeof(struct fmt_hdr);
	ret = memcpy_from_ringbuf_user(readbuf, buf, pktsize);
	if (!ret) {
		pr_warn("%s:read ipc data failed: %d\n", __func__, ret);
		ret = -1;
	}
	ret = memcpy_from_ringbuf(readbuf, &temp, 1);
	if ((!ret) && (temp != HDLC_END)) {
		pr_warn("%s:HDLC END: %d, ret: %d\n", __func__, temp, ret);
		ret = -1;
	}
#ifdef DEBUG_LOG
	pr_info("=================== READ IPC ===================\n");
	dump_buffer(buf, pktsize);
	pr_info("================================================\n");
#endif

	if (ret < 0)
		return ret;

done:
	pr_debug("%s##: pktsize: %d\n", __func__, pktsize);

	return pktsize;
}

static unsigned int smdipc_poll(struct file *file,
				struct poll_table_struct *wait)
{
	struct str_smdipc *smdipc = file->private_data;

	if (get_remained_size(&smdipc->read_buf))
		return POLLIN | POLLRDNORM;

	if (wait)
		poll_wait(file, &smdipc->poll_wait, wait);
	else
		usleep_range(2000, 10000);

	if (get_remained_size(&smdipc->read_buf))
		return POLLIN | POLLRDNORM;
	else
		return 0;
}

static const struct file_operations smdipc_fops = {
	.owner = THIS_MODULE,
	.open = smdipc_open,
	.release = smdipc_release,
	.write = smdipc_write,
	.read = smdipc_read,
	.poll = smdipc_poll,
};

static int register_smdipc_dev(struct str_smdipc *smdipc)
{
	int r;
	dev_t devid;

	if (!smdipc) {
		pr_err("%s: smdipc is NULL\n", __func__);
		return -EINVAL;
	}

	smdipc->class = class_create(THIS_MODULE, SMDIPC_DEVNAME);
	if (IS_ERR_OR_NULL(smdipc->class)) {
		r = PTR_ERR(smdipc->class);
		smdipc->class = NULL;
		pr_err("%s: class_create() failed, r = %d\n", __func__, r);
		return r;
	}

	r = alloc_chrdev_region(&devid, SMDIPC_MINOR, 1, SMDIPC_DEVNAME);
	if (r) {
		pr_err("%s: alloc_chrdev_region() failed, r = %d\n",
			__func__, r);
		goto err_alloc_chrdev_region;
	}

	cdev_init(&smdipc->cdev, &smdipc_fops);

	r = cdev_add(&smdipc->cdev, devid, 1);
	if (r) {
		pr_err("%s: cdev_add() failed, r = %d\n", __func__, r);
		goto err_cdev_add;
	}
	smdipc->devid = devid;

	smdipc->dev = device_create(smdipc->class, NULL, smdipc->devid,
				NULL, SMDIPC_DEVNAME);
	if (IS_ERR_OR_NULL(smdipc->dev)) {
		r = PTR_ERR(smdipc->dev);
		smdipc->dev = NULL;
		pr_err("%s: device_create() failed, r = %d\n", __func__, r);
		goto err_device_create;
	}

	atomic_set(&smdipc->opened, 0);
	init_waitqueue_head(&smdipc->poll_wait);
	dev_set_drvdata(smdipc->dev, smdipc);

	return 0;

err_device_create:
	cdev_del(&smdipc->cdev);
err_cdev_add:
	unregister_chrdev_region(devid, 1);
err_alloc_chrdev_region:
	class_destroy(smdipc->class);
	return r;
}

static void dereg_smdipc_dev(struct str_smdipc *smdipc)
{
	dev_set_drvdata(smdipc->dev, NULL);
	device_destroy(smdipc->class, smdipc->devid);
	cdev_del(&smdipc->cdev);
	unregister_chrdev_region(smdipc->devid, 1);
	class_destroy(smdipc->class);
}

void *init_smdipc(void)
{
	int r;
	struct str_smdipc *smdipc;

	pr_info("%s: Enter\n", __func__);
	smdipc = kzalloc(sizeof(*smdipc), GFP_KERNEL);
	if (!smdipc) {
		pr_err("%s:malloc for smdipc failed\n", __func__);
		return NULL;
	}

	r = register_smdipc_dev(smdipc);
	if (r) {
		pr_err("%s:register_smdipc_dev() failed (%d)\n", __func__, r);
		goto err_register_smdipc_dev;
	}

	smdipc->tpbuf = kzalloc(IPC_TEMP_BUFFER_SIZE, GFP_DMA);
	if (!smdipc->tpbuf) {
		pr_err("%s:malloc ipc tp buf failed\n", __func__);
		r = -ENOMEM;
		goto err_kzalloc_tpbuf;
	}

	r = alloc_buf(&smdipc->read_buf, IPC_READ_BUF_SIZE);
	if (r) {
		pr_err("%s:alloc_buf() failed\n", __func__);
		goto err_alloc_buf;
	}

	smdipc->hsic.rx_urbcnt = IPC_RX_URB_CNT;

	r = smd_alloc_urb(&smdipc->hsic);
	if (r) {
		pr_err("%s:smd_alloc_urb() failed\n", __func__);
		goto err_smd_alloc_urb;
	}

	smdipc->pm_stat = IPC_PM_STATUS_SUSPEND;

	wake_lock_init(&smdipc->wakelock, WAKE_LOCK_SUSPEND, "smdipc");
	wake_lock_init(&smdipc->rxguide_lock, WAKE_LOCK_SUSPEND, "rxipc");

	pr_info("%s: End\n", __func__);
	return smdipc;

err_smd_alloc_urb:
	free_buf(&smdipc->read_buf);
err_alloc_buf:
	kfree(smdipc->tpbuf);
err_kzalloc_tpbuf:
	dereg_smdipc_dev(smdipc);
err_register_smdipc_dev:
	kfree(smdipc);
	return NULL;
}

void *connect_smdipc(void *smd_device, struct str_hsic *hsic)
{
	struct str_smdipc *smdipc = smd_device;

	smdipc->hsic.intf = hsic->intf;
	smdipc->hsic.usb = hsic->usb;
	smdipc->hsic.rx_pipe = hsic->rx_pipe;
	smdipc->hsic.tx_pipe = hsic->tx_pipe;

	smdipc->pm_stat = IPC_PM_STATUS_RESUME;

	init_err_stat(&smdipc->stat);

	smd_kill_urb(&smdipc->hsic);

	return smd_device;
}

void disconnect_smdipc(void *smd_device)
{
	struct str_smdipc *smdipc = smd_device;

	pr_info("%s: Enter\n", __func__);

	smdipc->pm_stat = IPC_PM_STATUS_DISCONNECT;
	smdipc->suspended = 0;

	flush_buf(&smdipc->read_buf);
	smd_kill_urb(&smdipc->hsic);
	wake_unlock(&smdipc->wakelock);
	wake_unlock(&smdipc->rxguide_lock);
}

void exit_smdipc(void *param)
{
	struct str_smdipc *smdipc = param;

	pr_info("%s: Enter\n", __func__);

	wake_lock_destroy(&smdipc->wakelock);
	wake_lock_destroy(&smdipc->rxguide_lock);

	smd_free_urb(&smdipc->hsic);
	free_buf(&smdipc->read_buf);
	kfree(smdipc->tpbuf);
	dereg_smdipc_dev(smdipc);
	kfree(smdipc);
}

int smdipc_suspend(struct str_smdipc *smdipc)
{
	pr_debug("%s: Enter\n", __func__);
	if (smdipc->pm_stat == IPC_PM_STATUS_RESUME) {
		smdipc->pm_stat = IPC_PM_STATUS_SUSPEND;
		smd_kill_urb(&smdipc->hsic);
	}
	return 0;
}

int smdipc_resume(struct str_smdipc *smdipc)
{
	int r = 0;
	int retrycnt;
	pr_debug("%s: Enter\n", __func__);

	if (smdipc->pm_stat != IPC_PM_STATUS_SUSPEND)
		return 0;

	for (retrycnt = 0; retrycnt < 50; retrycnt++) {
		smdipc->pm_stat = IPC_PM_STATUS_RESUME;
		r = smdipc_rx_usb(smdipc);
		if (!r)
			return 0;
		usleep_range(1000, 10000);
	}
	pr_err("%s:smdrfs_rx_usb() failed : %d\n", __func__, r);
	return r;
}
