// SPDX-License-Identifier: GPL
/*
 * Copyright (C) STMicroelectronics 2023 - All Rights Reserved
 * Author(s): Patrice Chotard <patrice.chotard@foss.st.com> for STMicroelectronics.
 */

#include <memory/stm32-omi.h>

#define STM32_AUTOSUSPEND_DELAY -1

struct stm32_ospi_flash {
	struct spi_device *spi;
	u32 cs;
	u32 presc;
	u32 dlyb_cr;
	u32 dcr_reg;
	u64 idcode;
	bool calibrated;
	bool sample_later;
};

struct stm32_ospi {
	struct device *dev;
	struct spi_controller *ctrl;
	struct stm32_omi *omi;
	struct stm32_ospi_flash flash[STM32_OMI_MAX_NORCHIP];

	u32 cr_reg;
	u64 id_buf;
	int last_cs;
	unsigned long status_timeout;

	/*
	 * to protect device configuration, could be different between
	 * 2 flash access
	 */
	struct mutex lock;
};

static int stm32_ospi_tx_mm(struct stm32_ospi *ospi,
			    const struct spi_mem_op *op)
{
	struct stm32_omi *omi = ospi->omi;

	memcpy_fromio(op->data.buf.in, omi->mm_base + op->addr.val,
		      op->data.nbytes);
	return 0;
}

static int stm32_ospi_tx_dma(struct stm32_ospi *ospi,
			     const struct spi_mem_op *op)
{
	struct dma_async_tx_descriptor *desc;
	struct stm32_omi *omi = ospi->omi;
	void __iomem *regs_base = omi->regs_base;
	enum dma_transfer_direction dma_dir;
	struct dma_chan *dma_ch;
	struct sg_table sgt;
	dma_cookie_t cookie;
	u32 cr, t_out;
	int err;

	if (op->data.dir == SPI_MEM_DATA_IN) {
		dma_dir = DMA_DEV_TO_MEM;
		dma_ch = omi->dma_chrx;
	} else {
		dma_dir = DMA_MEM_TO_DEV;
		dma_ch = omi->dma_chtx;
	}

	/*
	 * spi_map_buf return -EINVAL if the buffer is not DMA-able
	 * (DMA-able: in vmalloc | kmap | virt_addr_valid)
	 */
	err = spi_controller_dma_map_mem_op_data(ospi->ctrl, op, &sgt);
	if (err)
		return err;

	desc = dmaengine_prep_slave_sg(dma_ch, sgt.sgl, sgt.nents,
				       dma_dir, DMA_PREP_INTERRUPT);
	if (!desc) {
		err = -ENOMEM;
		goto out_unmap;
	}

	cr = readl_relaxed(regs_base + OSPI_CR);

	reinit_completion(&omi->dma_completion);
	desc->callback = stm32_omi_dma_callback;
	desc->callback_param = &omi->dma_completion;
	cookie = dmaengine_submit(desc);
	err = dma_submit_error(cookie);
	if (err)
		goto out;

	dma_async_issue_pending(dma_ch);

	writel_relaxed(cr | CR_DMAEN, regs_base + OSPI_CR);

	t_out = sgt.nents * STM32_COMP_TIMEOUT_MS;
	if (!wait_for_completion_timeout(&omi->dma_completion,
					 msecs_to_jiffies(t_out)))
		err = -ETIMEDOUT;

	if (err)
		dmaengine_terminate_all(dma_ch);

out:
	writel_relaxed(cr & ~CR_DMAEN, regs_base + OSPI_CR);
out_unmap:
	spi_controller_dma_unmap_mem_op_data(ospi->ctrl, op, &sgt);

	return err;
}

static int stm32_ospi_tx(struct stm32_ospi *ospi, const struct spi_mem_op *op)
{
	struct stm32_omi *omi = ospi->omi;
	u8 *buf;

	if (!op->data.nbytes)
		return 0;

	if (omi->fmode == CR_FMODE_MM)
		return stm32_ospi_tx_mm(ospi, op);
	else if (((op->data.dir == SPI_MEM_DATA_IN && omi->dma_chrx) ||
		 (op->data.dir == SPI_MEM_DATA_OUT && omi->dma_chtx)) &&
		  op->data.nbytes > 4)
		if (!stm32_ospi_tx_dma(ospi, op))
			return 0;

	if (op->data.dir == SPI_MEM_DATA_IN)
		buf = op->data.buf.in;
	else
		buf = (u8 *)op->data.buf.out;

	return stm32_omi_tx_poll(omi, buf, op->data.nbytes,
				 op->data.dir == SPI_MEM_DATA_IN);
}

static int stm32_ospi_wait_poll_status(struct stm32_ospi *ospi,
				       const struct spi_mem_op *op)
{
	struct stm32_omi *omi = ospi->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 cr;

	reinit_completion(&omi->match_completion);
	cr = readl_relaxed(regs_base + OSPI_CR);
	writel_relaxed(cr | CR_SMIE, regs_base + OSPI_CR);

	if (!wait_for_completion_timeout(&omi->match_completion,
					 msecs_to_jiffies(ospi->status_timeout)))
		return -ETIMEDOUT;

	writel_relaxed(FCR_CSMF, regs_base + OSPI_FCR);

	return 0;
}

static int stm32_ospi_get_mode(u8 buswidth)
{
	switch (buswidth) {
	case 8:
		return CCR_BUSWIDTH_8;
	case 4:
		return CCR_BUSWIDTH_4;
	default:
		return buswidth;
	}
}

static void stm32_ospi_set_prescaler(struct stm32_ospi *ospi, u32 presc)
{
	struct stm32_omi *omi = ospi->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 dcr2;

	/* set prescaler */
	dcr2 = readl_relaxed(regs_base + OSPI_DCR2);
	dcr2 &= ~DCR2_PRESC_MASK;
	dcr2 |= FIELD_PREP(DCR2_PRESC_MASK, presc);
	writel_relaxed(dcr2, regs_base + OSPI_DCR2);
}

static int stm32_ospi_send(struct spi_device *spi, const struct spi_mem_op *op)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(spi->master);
	struct stm32_ospi_flash *flash = &ospi->flash[spi->chip_select];
	struct stm32_omi *omi = ospi->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 ccr, cr, tcr = 0;
	int timeout, err = 0, err_poll_status = 0;
	int ret;

	dev_dbg(ospi->dev, "cmd:%#x mode:%d.%d.%d.%d addr:%#llx len:%#x\n",
		op->cmd.opcode, op->cmd.buswidth, op->addr.buswidth,
		op->dummy.buswidth, op->data.buswidth,
		op->addr.val, op->data.nbytes);

	if (ospi->last_cs != spi->chip_select) {
		ospi->last_cs = spi->chip_select;

		stm32_omi_dlyb_stop(omi);
		writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);

		stm32_ospi_set_prescaler(ospi, flash->presc);

		if (flash->calibrated) {
			ret = stm32_omi_dlyb_set_cr(omi, flash->dlyb_cr);
			if (ret)
				return ret;
		}
	}

	cr = readl_relaxed(regs_base + OSPI_CR);
	cr &= ~CR_CSSEL;
	cr |= FIELD_PREP(CR_CSSEL, flash->cs);
	cr &= ~CR_FMODE_MASK;
	cr |= FIELD_PREP(CR_FMODE_MASK, omi->fmode);
	writel_relaxed(cr, regs_base + OSPI_CR);

	if (op->data.nbytes)
		writel_relaxed(op->data.nbytes - 1, regs_base + OSPI_DLR);

	ccr = FIELD_PREP(CCR_IMODE_MASK, stm32_ospi_get_mode(op->cmd.buswidth));

	if (op->addr.nbytes) {
		ccr |= FIELD_PREP(CCR_ADMODE_MASK,
				  stm32_ospi_get_mode(op->addr.buswidth));
		ccr |= FIELD_PREP(CCR_ADSIZE_MASK, op->addr.nbytes - 1);
	}

	if (flash->sample_later)
		tcr |= TCR_SSHIFT;

	if (op->dummy.buswidth && op->dummy.nbytes) {
		tcr |= FIELD_PREP(TCR_DCYC_MASK,
				  op->dummy.nbytes * 8 / op->dummy.buswidth);
	}
	writel_relaxed(tcr, regs_base + OSPI_TCR);

	if (op->data.nbytes) {
		ccr |= FIELD_PREP(CCR_DMODE_MASK,
				  stm32_ospi_get_mode(op->data.buswidth));
	}

	writel_relaxed(ccr, regs_base + OSPI_CCR);

	/* set instruction, must be set after ccr register update */
	writel_relaxed(op->cmd.opcode, regs_base + OSPI_IR);

	if (op->addr.nbytes && omi->fmode != CR_FMODE_MM)
		writel_relaxed(op->addr.val, regs_base + OSPI_AR);

	if (omi->fmode == CR_FMODE_APM)
		err_poll_status = stm32_ospi_wait_poll_status(ospi, op);

	err = stm32_ospi_tx(ospi, op);

	/*
	 * Abort in:
	 * -error case
	 * -read memory map: prefetching must be stopped if we read the last
	 *  byte of device (device size - fifo size). like device size is not
	 *  knows, the prefetching is always stop.
	 */
	if (err || err_poll_status || omi->fmode == CR_FMODE_MM)
		goto abort;

	/* wait end of tx in indirect mode */
	err = stm32_omi_wait_cmd(omi);
	if (err)
		goto abort;

	return 0;

abort:
	timeout = stm32_omi_abort(omi);
	writel_relaxed(FCR_CTCF | FCR_CSMF, regs_base + OSPI_FCR);

	if (err || err_poll_status || timeout)
		dev_err(ospi->dev, "%s err:%d err_poll_status:%d abort timeout:%d\n",
			__func__, err, err_poll_status, timeout);

	return err;
}

static int stm32_ospi_poll_status(struct spi_mem *mem,
				  const struct spi_mem_op *op,
				  u16 mask, u16 match,
				  unsigned long initial_delay_us,
				  unsigned long polling_rate_us,
				  unsigned long timeout_ms)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(mem->spi->master);
	struct stm32_omi *omi = ospi->omi;
	void __iomem *regs_base = omi->regs_base;
	int ret;

	ret = pm_runtime_get_sync(ospi->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(ospi->dev);
		return ret;
	}

	mutex_lock(&ospi->lock);

	writel_relaxed(mask, regs_base + OSPI_PSMKR);
	writel_relaxed(match, regs_base + OSPI_PSMAR);
	omi->fmode = CR_FMODE_APM;
	ospi->status_timeout = timeout_ms;

	ret = stm32_ospi_send(mem->spi, op);
	mutex_unlock(&ospi->lock);

	pm_runtime_mark_last_busy(ospi->dev);
	pm_runtime_put_autosuspend(ospi->dev);

	return ret;
}

static int stm32_ospi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(mem->spi->master);
	struct stm32_omi *omi = ospi->omi;
	int ret;

	ret = pm_runtime_get_sync(ospi->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(ospi->dev);
		return ret;
	}

	mutex_lock(&ospi->lock);
	if (op->data.dir == SPI_MEM_DATA_IN && op->data.nbytes)
		omi->fmode = CR_FMODE_INDR;
	else
		omi->fmode = CR_FMODE_INDW;

	ret = stm32_ospi_send(mem->spi, op);
	mutex_unlock(&ospi->lock);

	pm_runtime_mark_last_busy(ospi->dev);
	pm_runtime_put_autosuspend(ospi->dev);

	return ret;
}

static int stm32_ospi_dirmap_create(struct spi_mem_dirmap_desc *desc)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(desc->mem->spi->master);
	struct stm32_omi *omi = ospi->omi;

	if (desc->info.op_tmpl.data.dir == SPI_MEM_DATA_OUT)
		return -EOPNOTSUPP;

	/* should never happen, as mm_base == null is an error probe exit condition */
	if (!omi->mm_base && desc->info.op_tmpl.data.dir == SPI_MEM_DATA_IN)
		return -EOPNOTSUPP;

	if (!omi->mm_size)
		return -EOPNOTSUPP;

	return 0;
}

static ssize_t stm32_ospi_dirmap_read(struct spi_mem_dirmap_desc *desc,
				      u64 offs, size_t len, void *buf)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(desc->mem->spi->master);
	struct stm32_omi *omi = ospi->omi;
	struct spi_mem_op op;
	u32 addr_max;
	int ret;

	ret = pm_runtime_get_sync(ospi->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(ospi->dev);
		return ret;
	}

	mutex_lock(&ospi->lock);
	/* make a local copy of desc op_tmpl and complete dirmap rdesc
	 * spi_mem_op template with offs, len and *buf in  order to get
	 * all needed transfer information into struct spi_mem_op
	 */
	memcpy(&op, &desc->info.op_tmpl, sizeof(struct spi_mem_op));
	dev_dbg(ospi->dev, "%s len = 0x%zx offs = 0x%llx buf = 0x%p\n", __func__, len, offs, buf);

	op.data.nbytes = len;
	op.addr.val = desc->info.offset + offs;
	op.data.buf.in = buf;

	addr_max = op.addr.val + op.data.nbytes + 1;
	if (addr_max < omi->mm_size && op.addr.buswidth)
		omi->fmode = CR_FMODE_MM;
	else
		omi->fmode = CR_FMODE_INDR;

	ret = stm32_ospi_send(desc->mem->spi, &op);
	mutex_unlock(&ospi->lock);

	pm_runtime_mark_last_busy(ospi->dev);
	pm_runtime_put_autosuspend(ospi->dev);

	return ret ?: len;
}

static int stm32_ospi_transfer_one_message(struct spi_controller *ctrl,
					   struct spi_message *msg)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(ctrl);
	struct stm32_omi *omi = ospi->omi;
	struct spi_transfer *transfer;
	struct spi_device *spi = msg->spi;
	struct spi_mem_op op;
	int ret = 0;

	if (!spi->cs_gpiod)
		return -EOPNOTSUPP;

	ret = pm_runtime_resume_and_get(ospi->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&ospi->lock);

	gpiod_set_value_cansleep(spi->cs_gpiod, true);

	list_for_each_entry(transfer, &msg->transfers, transfer_list) {
		u8 dummy_bytes = 0;

		memset(&op, 0, sizeof(op));

		dev_dbg(ospi->dev, "tx_buf:%p tx_nbits:%d rx_buf:%p rx_nbits:%d len:%d dummy_data:%d\n",
			transfer->tx_buf, transfer->tx_nbits,
			transfer->rx_buf, transfer->rx_nbits,
			transfer->len, transfer->dummy_data);

		/*
		 * OSPI hardware supports dummy bytes transfer.
		 * If current transfer is dummy byte, merge it with the next
		 * transfer in order to take into account OSPI block constraint
		 */
		if (transfer->dummy_data) {
			op.dummy.buswidth = transfer->tx_nbits;
			op.dummy.nbytes = transfer->len;
			dummy_bytes = transfer->len;

			/* if happens, means that message is not correctly built */
			if (list_is_last(&transfer->transfer_list, &msg->transfers)) {
				ret = -EINVAL;
				goto end_of_transfer;
			}

			transfer = list_next_entry(transfer, transfer_list);
		}

		op.data.nbytes = transfer->len;

		if (transfer->rx_buf) {
			omi->fmode = CR_FMODE_INDR;
			op.data.buswidth = transfer->rx_nbits;
			op.data.dir = SPI_MEM_DATA_IN;
			op.data.buf.in = transfer->rx_buf;
		} else {
			omi->fmode = CR_FMODE_INDW;
			op.data.buswidth = transfer->tx_nbits;
			op.data.dir = SPI_MEM_DATA_OUT;
			op.data.buf.out = transfer->tx_buf;
		}

		ret = stm32_ospi_send(spi, &op);
		if (ret)
			goto end_of_transfer;

		msg->actual_length += transfer->len + dummy_bytes;
	}

end_of_transfer:
	gpiod_set_value_cansleep(spi->cs_gpiod, false);

	mutex_unlock(&ospi->lock);

	msg->status = ret;
	spi_finalize_current_message(ctrl);

	pm_runtime_mark_last_busy(ospi->dev);
	pm_runtime_put_autosuspend(ospi->dev);

	return ret;
}

static int stm32_ospi_readid(struct stm32_omi *omi)
{
	struct stm32_ospi *ospi = platform_get_drvdata(omi->vdev);
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	struct spi_device *spi = flash->spi;
	u64 *rx_buf = &ospi->id_buf;
	const struct spi_mem_op readid_op =
		SPI_MEM_OP(SPI_MEM_OP_CMD(0x9F, 1),
			   SPI_MEM_OP_NO_ADDR,
			   SPI_MEM_OP_NO_DUMMY,
			   SPI_MEM_OP_DATA_IN(sizeof(*rx_buf), (u8 *)rx_buf, 1));
	int ret;

	omi->fmode = CR_FMODE_INDR;

	if (spi->cs_gpiod)
		gpiod_set_value_cansleep(spi->cs_gpiod, true);

	ret = stm32_ospi_send(spi, &readid_op);

	if (spi->cs_gpiod)
		gpiod_set_value_cansleep(spi->cs_gpiod, false);

	if (ret)
		return ret;

	dev_dbg(ospi->dev, "Flash ID 0x%08llx\n", *rx_buf);

	/*
	 * In case of SNAND, the first byte is a dummy byte. Depending of
	 * memory device, its value can be different in function of frequency.
	 * Ignore this byte and force its value to 0.
	 */
	*rx_buf &= 0xffffffffffffff00;

	/* On stm32_ospi_readid() first execution, save the golden READID command's answer */
	if (!flash->idcode)
		flash->idcode = *rx_buf;

	if (*rx_buf == flash->idcode)
		return 0;

	return -EIO;
}

static int stm32_ospi_calibration(struct stm32_ospi *ospi)
{
	struct stm32_omi *omi = ospi->omi;
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	struct spi_device *spi = flash->spi;
	void __iomem *regs_base = omi->regs_base;
	u8 window_len_tcr0 = 0, window_len_tcr1 = 0;
	int ret, ret_tcr0, ret_tcr1;

	/*
	 * set memory device at low frequency (50MHz) and sent
	 * READID (0x9F) command, save the answer as golden answer
	 */
	flash->presc = DIV_ROUND_UP(omi->clk_rate,
				    STM32_DLYB_FREQ_THRESHOLD) - 1;
	stm32_ospi_set_prescaler(ospi, flash->presc);

	ret = stm32_ospi_readid(omi);
	if (ret)
		return ret;

	/*
	 * set memory device at expected frequency read from DT
	 * prescaler should be set befor locking the DLL
	 */
	flash->presc = DIV_ROUND_UP(omi->clk_rate, spi->max_speed_hz) - 1;
	stm32_ospi_set_prescaler(ospi, flash->presc);

	flash->dcr_reg &= ~DCR1_DLYBYP;
	writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);

	ret = stm32_omi_dlyb_init(omi, false, 0);
	if (ret)
		return ret;

	/*
	 * perform only Rx TAP selection
	 * when DTR support will be added, Rx/Tx TAP selection will have to
	 * be performed
	 */
	ret_tcr0 = stm32_omi_dlyb_find_tap(omi, true, &window_len_tcr0);
	if (!ret_tcr0)
		/*
		 * save flash delay block configuration, will be restored
		 * each time this flash is addressed
		 */
		stm32_omi_dlyb_get_cr(omi, &flash->dlyb_cr);

	stm32_omi_dlyb_stop(omi);
	ret = stm32_omi_dlyb_init(omi, false, 0);
	if (ret)
		return ret;

	flash->sample_later = true;
	ret_tcr1 = stm32_omi_dlyb_find_tap(omi, true, &window_len_tcr1);
	if (ret_tcr0 && ret_tcr1) {
		dev_info(ospi->dev, "Calibration phase failed\n");
		return ret_tcr0;
	}

	if (window_len_tcr0 >= window_len_tcr1) {
		flash->sample_later = false;
		stm32_omi_dlyb_stop(omi);

		ret = stm32_omi_dlyb_set_cr(omi, flash->dlyb_cr);
		if (ret)
			return ret;
	} else {
		stm32_omi_dlyb_get_cr(omi, &flash->dlyb_cr);
	}

	flash->calibrated = true;

	return 0;
}

static int stm32_ospi_setup(struct spi_device *spi)
{
	struct spi_controller *ctrl = spi->master;
	struct stm32_ospi *ospi = spi_controller_get_devdata(ctrl);
	struct stm32_omi *omi = ospi->omi;
	struct stm32_ospi_flash *flash;
	void __iomem *regs_base = omi->regs_base;
	u32 bus_freq;
	int ret;

	if (ctrl->busy)
		return -EBUSY;

	if (!spi->max_speed_hz)
		return -EINVAL;

	ret = pm_runtime_get_sync(ospi->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(ospi->dev);
		return ret;
	}

	flash = &ospi->flash[spi->chip_select];
	flash->cs = spi->chip_select;
	flash->presc = DIV_ROUND_UP(omi->clk_rate, spi->max_speed_hz) - 1;
	flash->spi = spi;

	mutex_lock(&ospi->lock);

	ospi->cr_reg = CR_APMS | 3 << CR_FTHRES_SHIFT | CR_EN;
	writel_relaxed(ospi->cr_reg, regs_base + OSPI_CR);

	/* stop the DLL */
	stm32_omi_dlyb_stop(omi);

	/* set dcr fsize to max address */
	flash->dcr_reg = DCR1_DEVSIZE_MASK | DCR1_DLYBYP;
	writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);

	stm32_ospi_set_prescaler(ospi, flash->presc);

	ospi->last_cs = spi->chip_select;

	bus_freq = omi->clk_rate / (flash->presc + 1);
	/* calibration needed above 50MHz*/
	if (bus_freq > STM32_DLYB_FREQ_THRESHOLD) {
		ret = stm32_ospi_calibration(ospi);
		if (ret) {
			dev_info(ospi->dev, "Set flash frequency to a safe value (%d Hz)\n",
				 STM32_DLYB_FREQ_THRESHOLD);

			/* stop the DLL */
			stm32_omi_dlyb_stop(omi);
			flash->sample_later = false;

			flash->presc = DIV_ROUND_UP(omi->clk_rate,
						    STM32_DLYB_FREQ_THRESHOLD) - 1;
			stm32_ospi_set_prescaler(ospi, flash->presc);

			flash->dcr_reg |= DCR1_DLYBYP;
			writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);
		}
	}
	mutex_unlock(&ospi->lock);

	pm_runtime_mark_last_busy(ospi->dev);
	pm_runtime_put_autosuspend(ospi->dev);

	return 0;
}

/*
 * no special host constraint, so use default spi_mem_default_supports_op
 * to check supported mode.
 */
static const struct spi_controller_mem_ops stm32_ospi_mem_ops = {
	.exec_op	= stm32_ospi_exec_op,
	.dirmap_create	= stm32_ospi_dirmap_create,
	.dirmap_read	= stm32_ospi_dirmap_read,
	.poll_status	= stm32_ospi_poll_status,
};

static int stm32_ospi_probe(struct platform_device *pdev)
{
	struct device *parent = pdev->dev.parent;
	struct device *dev = &pdev->dev;
	struct spi_controller *ctrl;
	struct stm32_ospi *ospi;
	struct stm32_omi *omi = dev_get_drvdata(parent);
	struct dma_slave_config dma_cfg;
	int ret;

	ctrl = devm_spi_alloc_master(dev, sizeof(*ospi));
	if (!ctrl)
		return -ENOMEM;

	ospi = spi_controller_get_devdata(ctrl);
	ospi->ctrl = ctrl;
	ospi->omi = omi;
	omi->check_transfer = stm32_ospi_readid;

	ret = clk_prepare_enable(omi->clk);
	if (ret) {
		dev_err(dev, "Can not enable clock\n");
		return ret;
	}

	if (omi->rstc) {
		reset_control_assert(omi->rstc);
		udelay(2);
		reset_control_deassert(omi->rstc);
	}

	ospi->dev = dev;
	platform_set_drvdata(pdev, ospi);

	memset(&dma_cfg, 0, sizeof(dma_cfg));
	dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_cfg.src_addr = omi->regs_phys_base + OSPI_DR;
	dma_cfg.dst_addr = omi->regs_phys_base + OSPI_DR;
	dma_cfg.src_maxburst = 4;
	dma_cfg.dst_maxburst = 4;
	stm32_omi_dma_setup(omi, &dma_cfg);

	mutex_init(&ospi->lock);

	ctrl->mode_bits = SPI_RX_DUAL | SPI_RX_QUAD |
			  SPI_TX_DUAL | SPI_TX_QUAD |
			  SPI_TX_OCTAL | SPI_RX_OCTAL;
	ctrl->setup = stm32_ospi_setup;
	ctrl->bus_num = -1;
	ctrl->mem_ops = &stm32_ospi_mem_ops;
	ctrl->use_gpio_descriptors = true;
	ctrl->transfer_one_message = stm32_ospi_transfer_one_message;
	ctrl->num_chipselect = STM32_OMI_MAX_NORCHIP;
	ctrl->dev.of_node = parent->of_node;

	pm_runtime_set_autosuspend_delay(dev, STM32_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_noresume(dev);

	ret = spi_register_master(ctrl);
	if (ret)
		goto err_pm_runtime_free;

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_pm_runtime_free:
	pm_runtime_get_sync(dev);
	/* disable ospi */
	writel_relaxed(0, omi->regs_base + OSPI_CR);
	mutex_destroy(&ospi->lock);
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_dont_use_autosuspend(dev);
	clk_disable_unprepare(omi->clk);

	return ret;
}

static int stm32_ospi_remove(struct platform_device *pdev)
{
	struct stm32_ospi *ospi = platform_get_drvdata(pdev);
	struct stm32_omi *omi = ospi->omi;

	pm_runtime_get_sync(ospi->dev);
	spi_unregister_master(ospi->ctrl);
	/* disable ospi */
	writel_relaxed(0, omi->regs_base + OSPI_CR);
	stm32_omi_dlyb_stop(omi);
	mutex_destroy(&ospi->lock);
	pm_runtime_put_noidle(ospi->dev);
	pm_runtime_disable(ospi->dev);
	pm_runtime_set_suspended(ospi->dev);
	pm_runtime_dont_use_autosuspend(ospi->dev);
	clk_disable_unprepare(omi->clk);

	return 0;
}

static int __maybe_unused stm32_ospi_runtime_suspend(struct device *dev)
{
	struct stm32_ospi *ospi = dev_get_drvdata(dev);
	struct stm32_omi *omi = ospi->omi;

	clk_disable_unprepare(omi->clk);

	return 0;
}

static int __maybe_unused stm32_ospi_runtime_resume(struct device *dev)
{
	struct stm32_ospi *ospi = dev_get_drvdata(dev);
	struct stm32_omi *omi = ospi->omi;

	return clk_prepare_enable(omi->clk);
}

static int __maybe_unused stm32_ospi_suspend(struct device *dev)
{
	pinctrl_pm_select_sleep_state(dev);

	return pm_runtime_force_suspend(dev);
}

static int __maybe_unused stm32_ospi_resume(struct device *dev)
{
	struct stm32_ospi *ospi = dev_get_drvdata(dev);
	struct stm32_omi *omi = ospi->omi;
	void __iomem *regs_base = omi->regs_base;
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret < 0)
		return ret;

	pinctrl_pm_select_default_state(dev);

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	writel_relaxed(ospi->cr_reg, regs_base + OSPI_CR);
	ospi->last_cs = -1;
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;
}

static const struct dev_pm_ops stm32_ospi_pm_ops = {
	SET_RUNTIME_PM_OPS(stm32_ospi_runtime_suspend,
			   stm32_ospi_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(stm32_ospi_suspend, stm32_ospi_resume)
};

static struct platform_driver stm32_ospi_driver = {
	.probe	= stm32_ospi_probe,
	.remove	= stm32_ospi_remove,
	.driver	= {
		.name = "stm32-ospi",
		.pm = &stm32_ospi_pm_ops,
	},
};
module_platform_driver(stm32_ospi_driver);

MODULE_DESCRIPTION("STMicroelectronics STM32 OCTO SPI driver");
MODULE_LICENSE("GPL");
