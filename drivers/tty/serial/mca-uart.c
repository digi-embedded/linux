/* mca-uart.c - UART driver for MCA devices.
 * Based on sc16is7xx.c, by Jon Ringle <jringle@gridpoint.com>
 *
 * Copyright (C) 2017-2019  Digi International Inc
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library.
 */

#include <linux/device.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>
#include <linux/mfd/mca-common/registers.h>
#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-cc6ul/core.h>
#include <linux/mfd/mca-cc8x/core.h>
#include <linux/delay.h>

#define MCA_DRVNAME_UART		"mca-uart"
#define MCA_UART_DEV_NAME		"ttyMCA"
#define MCA_UART_DEFAULT_BRATE		9600
#define MCA_UART_DEFAULT_BAUD_REG	MCA_REG_UART_BAUD_9600
#define MCA_UART_MIN_BAUD		1200
#define MCA_UART_MAX_BAUD		230400
#define MCA_UART_RX_FIFO_SIZE		128
#define MCA_UART_TX_FIFO_SIZE		128
#define MCA_UART_CLK			24000000
#define MCA_UART_MAX_TX_FRAME_LEN	32
#define MCA_UART_MAX_RS485_DELAY_MS	250

#define MCA_REG_UART_FLUSH_BUF_MSK	(MCA_REG_UART_CFG0_CTX | \
					 MCA_REG_UART_CFG0_CRX)

#define MCA_UART_HAS_RTS		BIT(0)
#define MCA_UART_HAS_CTS		BIT(1)

#define MCA_UART_IOS			4
#define MCA_UART_MAX_NUM_UARTS		4
const char *iopins_names[] = {"rx", "tx", "cts", "rts"};
bool required[] = {1, 1, 0, 0};

#define MCA_REG_UART_PIN(p)		(MCA_REG_UART_RXPIN + (p))

enum mca_uart_type {
	CC6UL_MCA_UART,
	CC8X_MCA_UART,
};

enum {
	WORK_FLUSH_BUF	= BIT(0),
	WORK_RS485_CFG	= BIT(1),
};

struct mca_uart_data {
	enum mca_uart_type	devtype;
	u16			since;
	int			nuarts;
};

static struct mca_uart_data mca_uart_devdata[];

struct mca_uart {
	u32			line;
	struct mca_drv		*mca;
	struct uart_port	port;
	struct mutex		mutex;
	unsigned int		baddr;
	struct work_struct	tx_work;
	struct work_struct	dc_work;
	unsigned int		pending_dc_work;
	unsigned int		has_rtscts;
	u8			pins[MCA_UART_IOS];
	u8			npins;
	bool			enable_power_on;
};

struct mca_uart_drv {
	int			num_uarts;
	struct device		*dev;
	struct mca_drv		*mca;
	struct device_node	*np;
	struct uart_driver	uart;
	struct mca_uart		*ports;
	struct mca_uart_data	*uart_data;
};

#define to_mca_uart(p,e)	(container_of((p), struct mca_uart, e))

static void mca_uart_stop_tx(struct uart_port *port)
{
	struct mca_uart *mca_uart = to_mca_uart(port, port);
	struct regmap *regmap = mca_uart->mca->regmap;
	int ret;

	ret = regmap_update_bits(regmap, mca_uart->baddr + MCA_REG_UART_CFG0,
				 MCA_REG_UART_CFG0_CTX,  MCA_REG_UART_CFG0_CTX);
	if (ret)
		dev_err(mca_uart->port.dev, "Failed to write MCA_REG_UART_CFG0\n");
}

static void mca_uart_stop_rx(struct uart_port *port)
{
	/* Nothing to do here. Shutdown will do the work */
}

static void mca_uart_start_tx(struct uart_port *port)
{
	struct mca_uart *mca_uart = to_mca_uart(port, port);

	if (!work_pending(&mca_uart->tx_work))
		schedule_work(&mca_uart->tx_work);
}

static unsigned int mca_uart_tx_empty(struct uart_port *port)
{
	struct mca_uart *mca_uart = to_mca_uart(port, port);
	struct regmap *regmap = mca_uart->mca->regmap;
	unsigned int txlvl;
	int ret;

	ret = regmap_read(regmap, mca_uart->baddr + MCA_REG_UART_TXLVL, &txlvl);
	if (ret) {
		dev_err(port->dev, "Failed to read MCA_REG_UART_TXLVL\n");
		/* This is the behavior if not implemented */
		return TIOCSER_TEMT;
	}

	return (txlvl == MCA_UART_TX_FIFO_SIZE) ? TIOCSER_TEMT : 0;
}

static unsigned int mca_uart_get_mctrl(struct uart_port *port)
{
	/*
	 * DCD and DSR are not wired and CTS/RTS is handled automatically
	 * so just indicate DSR and CAR asserted. Also regmap cannot be called
	 * from atomic context, so reading the status of the lines here is not
	 * possible.
	 */
	return TIOCM_DSR | TIOCM_CAR;
}

static void mca_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/*
	 * Regmap cannot be called from atomic context, so this would require a
	 * work queue to set/clear RTS. However, that line is handled
	 * automatically by the hardware when using flow control, and the
	 * get_mctrl for reading CTS and RTS cannot be implemented for the same
	 * reason. If RTS/CTS are used for something different that hardware
	 * flow control, perhaps they should be declared as GPIOs.
	 */
}

static void mca_uart_break_ctl(struct uart_port *port, int break_state)
{
	dev_warn(port->dev, "BREAK condition not supported\n");
}

static void mca_uart_set_termios(struct uart_port *port,
				  struct ktermios *termios,
				  struct ktermios *old)
{
	struct mca_uart *mca_uart = to_mca_uart(port, port);
	struct regmap *regmap = mca_uart->mca->regmap;
	unsigned int cfg1 = 0;
	unsigned int baudrate;
	unsigned int baud_reg_val;
	int ret;

	/* Mask unsupported termios capabilities */
	if (!mca_uart->has_rtscts)
		termios->c_cflag &= ~CRTSCTS;

	termios->c_iflag &= ~(IXON | IXOFF | IXANY | CMSPAR | CSIZE);

	/* Only 8-bit size supported */
	termios->c_cflag |= CS8;

	if (termios->c_cflag & CSTOPB)
		cfg1 |= MCA_REG_UART_CFG1_TWO_STOPBITS;
	if (termios->c_cflag & PARENB)
		cfg1 |= MCA_REG_UART_CFG1_PARITY_EN;
	if (termios->c_cflag & PARODD)
		cfg1 |= MCA_REG_UART_CFG1_PARITY_ODD;
	if (termios->c_cflag & CRTSCTS) {
		if (mca_uart->has_rtscts & MCA_UART_HAS_CTS) {
			cfg1 |= MCA_REG_UART_CFG1_CTS_EN;
			port->status |= UPSTAT_AUTOCTS;
		}
		if (mca_uart->has_rtscts & MCA_UART_HAS_RTS) {
			cfg1 |= MCA_REG_UART_CFG1_RTS_EN;
			port->status |= UPSTAT_AUTORTS;
		}
	} else {
		port->status &= ~(UPSTAT_AUTOCTS | UPSTAT_AUTORTS);
	}

	/* Set status ignore mask */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= MCA_REG_UART_LSR_PARITY_ERROR;
	if (termios->c_iflag & IGNBRK)
		port->ignore_status_mask |= MCA_REG_UART_LSR_BREAK;
	if (!(termios->c_cflag & CREAD))
		port->ignore_status_mask |= MCA_REG_UART_LSR_FRAMING_ERROR |
					    MCA_REG_UART_LSR_PARITY_ERROR |
					    MCA_REG_UART_LSR_FIFO_OR_ERROR |
					    MCA_REG_UART_LSR_HW_OR_ERROR |
					    MCA_REG_UART_LSR_BREAK;

	ret = regmap_write(regmap, mca_uart->baddr + MCA_REG_UART_CFG1, cfg1);
	if (ret) {
		dev_err(port->dev, "Failed to write MCA_REG_UART_CFG1\n");
		return;
	}

	baudrate = uart_get_baud_rate(port, termios, old, MCA_UART_MIN_BAUD,
				      MCA_UART_MAX_BAUD);
	uart_update_timeout(port, termios->c_cflag, baudrate);

	switch (baudrate) {
	case 1200:
		baud_reg_val = MCA_REG_UART_BAUD_1200;
		break;
	case 2400:
		baud_reg_val = MCA_REG_UART_BAUD_2400;
		break;
	case 4800:
		baud_reg_val = MCA_REG_UART_BAUD_4800;
		break;
	case 9600:
		baud_reg_val = MCA_REG_UART_BAUD_9600;
		break;
	case 19200:
		baud_reg_val = MCA_REG_UART_BAUD_19200;
		break;
	case 38400:
		baud_reg_val = MCA_REG_UART_BAUD_38400;
		break;
	case 57600:
		baud_reg_val = MCA_REG_UART_BAUD_57600;
		break;
	case 115200:
		baud_reg_val = MCA_REG_UART_BAUD_115200;
		break;
	case 230400:
		baud_reg_val = MCA_REG_UART_BAUD_230400;
		break;
	default:
		dev_warn(port->dev,
			 "Baud rate %d not supported, using default %d\n",
			 baudrate, MCA_UART_DEFAULT_BRATE);
		baud_reg_val = MCA_UART_DEFAULT_BAUD_REG;
		break;
	}

	ret = regmap_write(regmap, mca_uart->baddr + MCA_REG_UART_BAUD,
			   baud_reg_val);
	if (ret) {
		dev_err(port->dev, "Failed to write MCA_REG_UART_BAUD\n");
		return;
	}

	/* Wait a bit until the uart is reconfigued with the new settings */
	msleep(10);
}

static int mca_uart_startup(struct uart_port *port)
{
	struct mca_uart *mca_uart = to_mca_uart(port, port);
	struct regmap *regmap = mca_uart->mca->regmap;
	int ret;
	unsigned int cfg_mask;
	unsigned int ier_mask;

	/* Reset RX and TX FIFOs and enable TX and RX */
	cfg_mask = MCA_REG_UART_CFG0_CTX | MCA_REG_UART_CFG0_CRX |
		   MCA_REG_UART_CFG0_TXEN | MCA_REG_UART_CFG0_RXEN;

	ret = regmap_update_bits(regmap, mca_uart->baddr + MCA_REG_UART_CFG0,
				 cfg_mask, cfg_mask);
	if (ret) {
		dev_err(port->dev, "Failed to read MCA_REG_UART_CFG0\n");
		return ret;
	}

	ier_mask = MCA_REG_UART_IER_THR | MCA_REG_UART_IER_RHR |
		   MCA_REG_UART_IER_RLSE;
	ret = regmap_update_bits(regmap, mca_uart->baddr + MCA_REG_UART_IER,
				 ier_mask, ier_mask);
	if (ret)
		dev_err(port->dev, "Failed to read MCA_REG_UART_IER\n");

	return ret;
}

static void mca_uart_shutdown(struct uart_port *port)
{
	struct mca_uart *mca_uart = to_mca_uart(port, port);
	struct regmap *regmap = mca_uart->mca->regmap;
	int ret;
	unsigned int cfg_mask;

	/* Reset RX and TX FIFOs and disable TX and RX */
	cfg_mask = MCA_REG_UART_CFG0_CTX | MCA_REG_UART_CFG0_CRX |
		   MCA_REG_UART_CFG0_TXEN | MCA_REG_UART_CFG0_RXEN;

	ret = regmap_update_bits(regmap, mca_uart->baddr + MCA_REG_UART_CFG0,
				 cfg_mask,
				 MCA_REG_UART_CFG0_CTX | MCA_REG_UART_CFG0_CRX);
	if (ret)
		dev_err(port->dev, "Failed to read MCA_REG_UART_CFG0\n");

	/* Disable all IRQs */
	ret = regmap_write(regmap, mca_uart->baddr + MCA_REG_UART_IER, 0);
	if (ret)
		dev_err(port->dev, "Failed to write MCA_REG_UART_IER\n");
}

static const char *mca_uart_type(struct uart_port *port)
{
	return "MCA UART";
}

static int mca_uart_request_port(struct uart_port *port)
{
	/* Do nothing */
	return 0;
}

static void mca_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_LPUART;
}

static int mca_uart_verify_port(struct uart_port *port,
				 struct serial_struct *s)
{
	if ((s->type != PORT_UNKNOWN) && (s->type != PORT_LPUART))
		return -EINVAL;
	if (s->irq != port->irq)
		return -EINVAL;

	return 0;
}

static void mca_uart_release_port(struct uart_port *port)
{
	/* Do nothing */
}

static void mca_uart_throttle(struct uart_port *port)
{
	struct mca_uart *mca_uart = to_mca_uart(port, port);
	struct regmap *regmap = mca_uart->mca->regmap;
	int ret;

	ret = regmap_update_bits(regmap, mca_uart->baddr + MCA_REG_UART_CFG1,
				 MCA_REG_UART_CFG1_THROTTLE,
				 MCA_REG_UART_CFG1_THROTTLE);
	if (ret)
		dev_err(port->dev, "Failed to write MCA_REG_UART_CFG1\n");
}

static void mca_uart_unthrottle(struct uart_port *port)
{
	struct mca_uart *mca_uart = to_mca_uart(port, port);
	struct regmap *regmap = mca_uart->mca->regmap;
	int ret;

	ret = regmap_update_bits(regmap, mca_uart->baddr + MCA_REG_UART_CFG1,
				 MCA_REG_UART_CFG1_THROTTLE, 0);
	if (ret)
		dev_err(port->dev, "Failed to write MCA_REG_UART_CFG1\n");
}

static void mca_uart_flush_buffer(struct uart_port *port)
{
	struct mca_uart *mca_uart = to_mca_uart(port, port);

	mutex_lock(&mca_uart->mutex);
	mca_uart->pending_dc_work |= WORK_FLUSH_BUF;
	mutex_unlock(&mca_uart->mutex);

	schedule_work(&mca_uart->dc_work);
}

static int mca_uart_rs485_config(struct uart_port *port,
				 struct serial_rs485 *rs485conf)
{
	struct mca_uart *mca_uart = to_mca_uart(port, port);

	if ((rs485conf->delay_rts_before_send > MCA_UART_MAX_RS485_DELAY_MS) ||
	    (rs485conf->delay_rts_after_send > MCA_UART_MAX_RS485_DELAY_MS))
		return -ERANGE;

	port->rs485 = *rs485conf;

	mutex_lock(&mca_uart->mutex);
	mca_uart->pending_dc_work |= WORK_RS485_CFG;
	mutex_unlock(&mca_uart->mutex);

	schedule_work(&mca_uart->dc_work);

	return 0;
}

static const struct uart_ops mca_uart_ops = {
	.tx_empty	= mca_uart_tx_empty,
	.set_mctrl	= mca_uart_set_mctrl,
	.get_mctrl	= mca_uart_get_mctrl,
	.stop_tx	= mca_uart_stop_tx,
	.start_tx	= mca_uart_start_tx,
	.stop_rx	= mca_uart_stop_rx,
	.break_ctl	= mca_uart_break_ctl,
	.startup	= mca_uart_startup,
	.shutdown	= mca_uart_shutdown,
	.set_termios	= mca_uart_set_termios,
	.type		= mca_uart_type,
	.request_port	= mca_uart_request_port,
	.release_port	= mca_uart_release_port,
	.config_port	= mca_uart_config_port,
	.verify_port	= mca_uart_verify_port,
	.throttle	= mca_uart_throttle,
	.unthrottle	= mca_uart_unthrottle,
	.flush_buffer	= mca_uart_flush_buffer,
	.pm		= NULL,
};

static void mca_uart_handle_tx(struct uart_port *port)
{
	struct mca_uart *mca_uart = to_mca_uart(port, port);
	struct regmap *regmap = mca_uart->mca->regmap;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int to_send;
	uint8_t tx_buf[MCA_UART_TX_FIFO_SIZE];

	/*
	 * There is a corner case in which the job is scheduled after the port
	 * has been shut down and port->state->port.tty is NULL. If not checked,
	 * uart_tx_stopped() would crash.
	 */
	if (!port->state->port.tty || uart_circ_empty(xmit) ||
	    uart_tx_stopped(port))
		return;

	/* Get length of data pending in circular buffer */
	to_send = uart_circ_chars_pending(xmit);
	if (likely(to_send)) {
		unsigned int txlen;
		unsigned int i;
		int ret;

		/* Limit to size of TX FIFO */
		ret = regmap_read(regmap, mca_uart->baddr + MCA_REG_UART_TXLVL,
				  &txlen);
		if (ret) {
			dev_err(port->dev,
				"Failed to read MCA_REG_UART_TXLVL\n");
			txlen = 0;
		}

		if (unlikely(!txlen)) {
			dev_dbg(port->dev, "TX FIFO is full\n");
			if (!work_pending(&mca_uart->tx_work))
				schedule_work(&mca_uart->tx_work);
			return;
		}

		if (unlikely(txlen > sizeof(tx_buf))) {
			dev_err(port->dev,
				"Invalid MCA_REG_UART_TXLVL value %d\n", txlen);
			if (!work_pending(&mca_uart->tx_work))
				schedule_work(&mca_uart->tx_work);
			return;
		}

		if (to_send > txlen)
			to_send = txlen;

		/*
		 * Limit the amount of data sent to avoid blocking the bus for
		 * too long
		 */
		if (to_send > MCA_UART_MAX_TX_FRAME_LEN)
			to_send = MCA_UART_MAX_TX_FRAME_LEN;

		port->icount.tx += to_send;
		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			tx_buf[i] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		}

		ret = regmap_bulk_write(regmap,
					mca_uart->baddr + MCA_REG_UART_THR,
					tx_buf, to_send);
		if (ret)
			dev_err(port->dev,
				"Failed to write MCA_REG_UART_THR\n");
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static void mca_uart_handle_rx(struct mca_uart *mca_uart, u8 iir)
{
	struct uart_port *port = &mca_uart->port;
	struct regmap *regmap = mca_uart->mca->regmap;
	unsigned int flag = TTY_NORMAL;
	uint8_t rx_buf[MCA_UART_RX_FIFO_SIZE];
	uint8_t error_buf[MCA_UART_RX_FIFO_SIZE];
	uint8_t has_errors = iir & MCA_REG_UART_IIR_RLSE;
	unsigned int lsr;
	unsigned int rxlen;
	unsigned int i;
	int ret;

	ret = regmap_read(regmap, mca_uart->baddr + MCA_REG_UART_RXLVL, &rxlen);
	if (ret) {
		dev_err(port->dev, "Failed to read MCA_REG_UART_RXLVL\n");
		return;
	}

	if (unlikely(!rxlen))
		return;

	ret = regmap_bulk_read(regmap, mca_uart->baddr + MCA_REG_UART_RHR,
			       rx_buf, rxlen);
	if (ret) {
		dev_warn(port->dev,
			"Failed to read MCA_REG_UART_RHR %d, retrying\n", ret);
		ret = regmap_bulk_read(regmap,
				       mca_uart->baddr + MCA_REG_UART_RHR,
				       rx_buf, rxlen);
		if (ret) {
			dev_err(port->dev,
				"Failed to read MCA_REG_UART_RHR %d\n", ret);
			goto exit;
		}
	}

	if (unlikely(has_errors)) {
		ret = regmap_read(regmap, mca_uart->baddr + MCA_REG_UART_LSR,
				  &lsr);
		if (ret) {
			dev_err(port->dev,
				"Failed to read MCA_REG_UART_LSR\n");
			return;
		}

		if (lsr & MCA_REG_UART_LSR_FIFO_OR_ERROR)
			dev_warn(port->dev, "fifo overrun\n");

		ret = regmap_bulk_read(regmap,
				       mca_uart->baddr + MCA_REG_UART_RX_ERRORS,
				       error_buf, rxlen);
		if (ret) {
			dev_err(port->dev,
				"Failed to read MCA_REG_UART_RX_ERRORS\n");
			return;
		}
	}

	port->icount.rx += rxlen;
	for (i = 0; i < rxlen; i++) {
		uint8_t const ch = rx_buf[i];
		uint8_t status;

		if (unlikely(has_errors)) {
			status = error_buf[i];

			if (status & MCA_REG_UART_LSR_BREAK) {
				port->icount.brk++;
				uart_handle_break(port);
			} else if (status & MCA_REG_UART_LSR_PARITY_ERROR)
				port->icount.parity++;
			else if (status & MCA_REG_UART_LSR_FRAMING_ERROR)
				port->icount.frame++;
			else if (status & MCA_REG_UART_LSR_HW_OR_ERROR)
				port->icount.overrun++;

			status &= port->read_status_mask;
			if (status & MCA_REG_UART_LSR_BREAK)
				flag = TTY_BREAK;
			else if (status & MCA_REG_UART_LSR_PARITY_ERROR)
				flag = TTY_PARITY;
			else if (status & MCA_REG_UART_LSR_FRAMING_ERROR)
				flag = TTY_FRAME;
			else if (status & MCA_REG_UART_LSR_HW_OR_ERROR)
				flag = TTY_OVERRUN;
		} else {
			status = 0;
		}

		if (uart_handle_sysrq_char(port, ch))
			continue;

		if (status & port->ignore_status_mask)
			continue;

		uart_insert_char(port, status, MCA_REG_UART_LSR_HW_OR_ERROR, ch,
				 flag);
	}
exit:
	tty_flip_buffer_push(&port->state->port);
}

static irqreturn_t mca_uart_irq_handler(int irq, void *private)
{
	struct mca_uart *mca_uart = private;
	struct regmap *regmap = mca_uart->mca->regmap;
	unsigned int iir;
	int ret;

	do {
		ret = regmap_read(regmap, mca_uart->baddr + MCA_REG_UART_IIR,
				  &iir);
		if (ret) {
			dev_err(mca_uart->port.dev,
				"Failed to read MCA_REG_UART_IIR\n");
			goto ret;
		}

		if (!iir)
			break;

		if (iir & MCA_REG_UART_IIR_RHR)
			mca_uart_handle_rx(mca_uart, iir);

		if (iir & MCA_REG_UART_IIR_THR) {
			mutex_lock(&mca_uart->mutex);
			mca_uart_handle_tx(&mca_uart->port);
			mutex_unlock(&mca_uart->mutex);
		}
	} while (1);

ret:
	return IRQ_HANDLED;
}

static int mca_uart_reconf_rs485(struct mca_uart *mca_uart)
{
	struct regmap *regmap = mca_uart->mca->regmap;
	struct serial_rs485 *rs485conf = &mca_uart->port.rs485;
	unsigned int cfg2_reg = 0;
	int ret;

	if (rs485conf->flags & SER_RS485_ENABLED)
		cfg2_reg |= MCA_REG_UART_CFG2_RS485_EN;

	if (!(rs485conf->flags & SER_RS485_RTS_ON_SEND) &&
	    (rs485conf->flags & SER_RS485_RTS_AFTER_SEND))
		cfg2_reg |= MCA_REG_UART_CFG2_RTS_INV;

	ret = regmap_update_bits(regmap, mca_uart->baddr + MCA_REG_UART_CFG2,
				 MCA_REG_UART_CFG2_RS485_EN |
				 MCA_REG_UART_CFG2_RTS_INV,
				 cfg2_reg);
	if (ret < 0) {
		dev_err(mca_uart->port.dev,
			"Failed writing MCA_REG_UART_CFG2 (%d)\n", ret);
		return ret;
	}

	ret = regmap_write(regmap,
			   mca_uart->baddr + MCA_REG_UART_RS485_PRE_DEL,
			   rs485conf->delay_rts_before_send);
	if (ret) {
		dev_err(mca_uart->port.dev,
			"Failed writing MCA_REG_UART_RS485_PRE_DEL (%d)\n",
			ret);
		return ret;
	}

	ret = regmap_write(regmap,
			   mca_uart->baddr + MCA_REG_UART_RS485_POST_DEL,
			   rs485conf->delay_rts_after_send);
	if (ret)
		dev_err(mca_uart->port.dev,
			"Failed writing MCA_UART_RS485_POST_DEL reg (%d)\n",
			ret);

	return ret;
}

static void mca_uart_dc_work_proc(struct work_struct *ws)
{
	struct mca_uart *mca_uart = to_mca_uart(ws, dc_work);
	struct regmap *regmap = mca_uart->mca->regmap;
	int ret;

	mutex_lock(&mca_uart->mutex);

	if (mca_uart->pending_dc_work & WORK_FLUSH_BUF) {
		ret = regmap_update_bits(regmap,
					 mca_uart->baddr + MCA_REG_UART_CFG0,
					 MCA_REG_UART_FLUSH_BUF_MSK,
					 MCA_REG_UART_FLUSH_BUF_MSK);
		if (ret)
			dev_err(mca_uart->port.dev,
				"Failed to read MCA_REG_UART_CFG0\n");
	}

	if (mca_uart->pending_dc_work & WORK_RS485_CFG)
		(void)mca_uart_reconf_rs485(mca_uart);

	mca_uart->pending_dc_work = 0;

	mutex_unlock(&mca_uart->mutex);
}

static void mca_uart_tx_work_proc(struct work_struct *ws)
{
	struct mca_uart *mca_uart = to_mca_uart(ws, tx_work);

	mutex_lock(&mca_uart->mutex);
	mca_uart_handle_tx(&mca_uart->port);
	mutex_unlock(&mca_uart->mutex);
}

static ssize_t power_on_rx_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct tty_port *port = dev_get_drvdata(dev);
	struct uart_state *state = container_of(port, struct uart_state, port);
	struct uart_port *uart_port = state->uart_port;
	struct mca_uart *mca_uart = container_of(uart_port, struct mca_uart,
						 port);

	return sprintf(buf, "%s\n", mca_uart->enable_power_on ?
							"enabled" : "disabled");
}

static ssize_t power_on_rx_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct tty_port *port = dev_get_drvdata(dev);
	struct uart_state *state = container_of(port, struct uart_state, port);
	struct uart_port *uart_port = state->uart_port;
	struct mca_uart *mca_uart = container_of(uart_port, struct mca_uart,
						 port);
	struct regmap *regmap = mca_uart->mca->regmap;
	int ret;

	if (!strncmp(buf, "enabled", sizeof("enabled") - 1))
		mca_uart->enable_power_on = true;
	else if (!strncmp(buf, "disabled", sizeof("disabled") - 1))
		mca_uart->enable_power_on = false;
	else
		return -EINVAL;

	ret = regmap_update_bits(regmap, mca_uart->baddr + MCA_REG_UART_CFG0,
				 MCA_REG_UART_CFG0_PWR_ON,
				 mca_uart->enable_power_on ?
						MCA_REG_UART_CFG0_PWR_ON : 0);
	if (ret < 0)
		dev_err(dev, "Failed to write MCA_REG_UART_CFG0\n");

	return count;
}
static DEVICE_ATTR(power_on_rx, 0600, power_on_rx_show, power_on_rx_store);

static struct attribute *uart_sysfs_entries[] = {
	&dev_attr_power_on_rx.attr,
	NULL,
};

static struct attribute_group uart_port_extra_attr = {
	.name	= "power_extra_opts",
	.attrs	= uart_sysfs_entries,
};

static const struct of_device_id mca_uart_ids[];
static inline struct mca_uart_data *mca_uart_get_driver_data(struct platform_device *pdev)
{
#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_node(mca_uart_ids, pdev->dev.of_node);
		return (struct mca_uart_data *)match->data;
	}
#endif
	return (struct mca_uart_data *)platform_get_device_id(pdev)->driver_data;
}

static int mca_uart_get_rs485_config_of(struct mca_uart *mca_uart, struct device_node *np)
{
	struct uart_port *port = &mca_uart->port;
	struct serial_rs485 *rs485conf = &port->rs485;
	u32 rs485_delay[2];
	int i;

	rs485conf->flags = SER_RS485_RTS_ON_SEND;

	if (!of_property_read_u32_array(np, "rs485-rts-delay", rs485_delay, 2)) {
		for (i = 0; i < 2; i++) {
			if (rs485_delay[i] > MCA_UART_MAX_RS485_DELAY_MS) {
				dev_warn(port->dev,
					"RS485 rts-%s-send (%u) limit exceeded,"
					" set to %d ms\n",
					i ? "after" : "before", rs485_delay[i],
					MCA_UART_MAX_RS485_DELAY_MS);
				rs485_delay[i] = MCA_UART_MAX_RS485_DELAY_MS;
			}
		}

		rs485conf->delay_rts_before_send = rs485_delay[0];
		rs485conf->delay_rts_after_send = rs485_delay[1];
	}

	if (of_property_read_bool(np, "rs485-rts-active-low")) {
		rs485conf->flags &= ~SER_RS485_RTS_ON_SEND;
		rs485conf->flags |= SER_RS485_RTS_AFTER_SEND;
	}

	if (of_property_read_bool(np, "linux,rs485-enabled-at-boot-time"))
		rs485conf->flags |= SER_RS485_ENABLED;

	if (of_property_read_bool(np, "rs485-rx-during-tx"))
		rs485conf->flags |= SER_RS485_RX_DURING_TX;

	return 0;
}

static int mca_uart_get_pins(struct mca_uart_drv *uart_drv,
			     struct mca_uart *mca_uart, struct device_node *np)
{
	u32 mca_io;
	int ret, i, index;

	mca_uart->npins = 0;
	for (i = 0; i < ARRAY_SIZE(iopins_names); i++) {
		index = of_property_match_string(np, "iopins-names",
						 iopins_names[i]);
		if (index < 0) {
			if (!required[i])
				continue;
			dev_err(uart_drv->dev, "Missing '%s' I/O pin name\n",
				iopins_names[i]);
			return -EINVAL;
		}
		ret = of_property_read_u32_index(np, "iopins", index, &mca_io);
		if (ret < 0 || mca_io >= MCA_MAX_IOS) {
			dev_err(uart_drv->dev,
				"Missing or invalid I/O pin for '%s'\n",
				iopins_names[i]);
			return -EINVAL;
		}

		if (!strcmp(iopins_names[i], "rts"))
			mca_uart->has_rtscts |= MCA_UART_HAS_RTS;
		if (!strcmp(iopins_names[i], "cts"))
			mca_uart->has_rtscts |= MCA_UART_HAS_CTS;

		mca_uart->pins[i] = mca_io;
		mca_uart->npins++;
	}

	return 0;
}

#ifdef CONFIG_OF
static int mca_uart_get_config_of(struct mca_uart_drv *uart_drv, int num_uarts)
{
	struct device_node *node;

	uart_drv->num_uarts = 0;

	for_each_child_of_node(uart_drv->np, node) {
		struct mca_uart *mca_uart =
			&uart_drv->ports[uart_drv->num_uarts];
		u32 val;

		if (of_property_read_u32(node, "reg", &val)) {
			dev_err(uart_drv->dev,
				"invalid/missing reg entry in devicetree\n");
			continue;
		}
		mca_uart->baddr = val;

		if (of_property_read_u32(node, "index", &val)) {
			dev_err(uart_drv->dev,
				"invalid/missing index entry in devicetree\n");
			continue;
		}
		mca_uart->line = val;

		if (mca_uart_get_pins(uart_drv, mca_uart, node))
			continue;

		if (mca_uart_get_rs485_config_of(mca_uart, node))
			continue;

		uart_drv->num_uarts++;
	}

	if (uart_drv->num_uarts == 0)
		return -EINVAL;

	return 0;
}
#endif

static int mca_uart_allocate_port_resources(struct mca_uart_drv *uart_drv,
					    struct mca_uart *mca_uart)
{
	struct regmap *regmap = mca_uart->mca->regmap;
	int i, ret;

	/* Request GPIOs */
	for (i = 0; i < mca_uart->npins; i++) {
		int mca_io = mca_uart->pins[i];
		int gpio = mca_uart->mca->gpio_base + mca_io;

		ret = devm_gpio_request(uart_drv->dev, gpio, MCA_DRVNAME_UART);
		if (ret) {
			dev_err(uart_drv->dev,
				"Failed to allocate MCA IO%d (gpio %d) (%d)\n",
				mca_io, gpio, ret);
			return ret;
		}

		ret = regmap_write(regmap,
				   mca_uart->baddr + MCA_REG_UART_PIN(i),
				   mca_io);
		if (ret) {
			dev_err(uart_drv->dev,
				"Failed to write MCA UART IO %d (%d)\n",
				mca_io, ret);
			return ret;
		}
	}

	/* Request interrupt */
	ret = devm_request_threaded_irq(uart_drv->dev,
					mca_uart->port.irq,
					NULL, mca_uart_irq_handler,
					IRQF_ONESHOT, MCA_DRVNAME_UART,
					mca_uart);
	if (ret) {
		dev_err(uart_drv->dev, "Failed to register IRQ\n");
		goto error_ios;
	}

	return 0;

error_ios:
	for (i = 0; i < mca_uart->npins; i++) {
		int io = mca_uart->pins[i];

		devm_gpio_free(uart_drv->dev, uart_drv->mca->gpio_base + io);
	}
	return ret;
}

static int mca_uart_release_port_resources(struct mca_uart *mca_uart)
{
	struct mca_uart_drv *uart_drv =
		dev_get_drvdata(mca_uart->port.dev->parent);
	int i;

	cancel_work_sync(&mca_uart->tx_work);
	cancel_work_sync(&mca_uart->dc_work);

	mutex_destroy(&mca_uart->mutex);
	uart_remove_one_port(&uart_drv->uart, &mca_uart->port);

	if (mca_uart->port.irq)
		devm_free_irq(uart_drv->dev, mca_uart->port.irq, mca_uart);

	for (i = 0; i < mca_uart->npins; i++) {
		int io = mca_uart->pins[i];

		devm_gpio_free(uart_drv->dev, uart_drv->mca->gpio_base + io);
	}

	return 0;
}

static int mca_uart_probe(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct regmap *regmap = mca->regmap;
	struct mca_uart_drv *uart_drv;
	u32 num_uarts;
	int ret, i, pin;
	char msg[256];

	if (IS_ERR(mca))
		return PTR_ERR(mca);

	/* Find entry in device-tree */
	if (!mca->dev->of_node)
		return -ENODEV;

	uart_drv = devm_kzalloc(&pdev->dev, sizeof(*uart_drv), GFP_KERNEL);
	if (!uart_drv) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, uart_drv);
	uart_drv->np = pdev->dev.of_node;
	uart_drv->mca = mca;
	uart_drv->dev = &pdev->dev;

	if (!of_get_next_child(uart_drv->np, NULL)) {
		dev_err(&pdev->dev, "no child nodes defined for MCA UART\n");
		ret = -EINVAL;
		goto err_free1;
	}

	num_uarts = of_get_child_count(uart_drv->np);
	if (!num_uarts || num_uarts > MCA_UART_MAX_NUM_UARTS) {
		dev_err(uart_drv->dev,
			"exceeded number of MCA UART child nodes (max is %d)",
			MCA_UART_MAX_NUM_UARTS);
		ret = -ENODEV;
		goto err_free1;
	}

	uart_drv->ports = devm_kzalloc(uart_drv->dev,
				       num_uarts * sizeof(struct mca_uart),
				       GFP_KERNEL);
	if (!uart_drv->ports) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_free1;
	}

	/* Check if the firmware has uart support */
	uart_drv->uart_data = mca_uart_get_driver_data(pdev);
	if (!uart_drv->uart_data) {
		dev_err(&pdev->dev, "failed to get MCA UART data\n");
		ret = -ENODEV;
		goto err_free2;
	}
	if (mca->fw_version < uart_drv->uart_data->since) {
		dev_err(&pdev->dev,
			"UART is not supported in MCA firmware v%d.%02d.\n",
			MCA_FW_VER_MAJOR(mca->fw_version),
			MCA_FW_VER_MINOR(mca->fw_version));
		ret = -ENODEV;
		goto err_free2;
	}

	ret = mca_uart_get_config_of(uart_drv, num_uarts);
	if (ret < 0)
		goto err_free2;


	/* Register UART driver */
	uart_drv->uart.owner = THIS_MODULE;
	uart_drv->uart.dev_name = MCA_UART_DEV_NAME;
	uart_drv->uart.nr = num_uarts;

	ret = uart_register_driver(&uart_drv->uart);
	if (ret) {
		dev_err(&pdev->dev, "Registering UART driver failed\n");
		goto err_free2;
	}

	for (i = 0; i < uart_drv->num_uarts; i++) {
		struct mca_uart *mca_uart = &uart_drv->ports[i];

		/* Initialize port data */
		mca_uart->mca = mca;
		mca_uart->port.line = i;
		mca_uart->port.dev = &pdev->dev;
		mca_uart->port.irq = platform_get_irq(pdev, mca_uart->line);
		mca_uart->port.type = PORT_LPUART;
		mca_uart->port.fifosize = max(MCA_UART_TX_FIFO_SIZE,
					      MCA_UART_RX_FIFO_SIZE);
		mca_uart->port.flags = UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		mca_uart->port.iotype = UPIO_PORT;
		mca_uart->port.uartclk = MCA_UART_CLK;
		mca_uart->port.ops = &mca_uart_ops;
		mca_uart->port.rs485_config = mca_uart_rs485_config;
		mca_uart->port.attr_group = &uart_port_extra_attr;
		mutex_init(&mca_uart->mutex);

		/* Initialize queue for start TX */
		INIT_WORK(&mca_uart->tx_work, mca_uart_tx_work_proc);

		/* Initialize queue for delayed configurations */
		INIT_WORK(&mca_uart->dc_work, mca_uart_dc_work_proc);

		/* Register port */
		ret = uart_add_one_port(&uart_drv->uart, &mca_uart->port);
		if (ret) {
			dev_err(&pdev->dev, "Failed adding a port (%d)\n", ret);
			goto error_drv;
		}

		/* Allocate resources */
		ret = mca_uart_allocate_port_resources(uart_drv, mca_uart);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed allocating resources for port (%d)\n",
				ret);
			uart_remove_one_port(&uart_drv->uart, &mca_uart->port);
			i--;
			goto error_port;
		}

		/* Apply initial settings for rs485 */
		ret = mca_uart_reconf_rs485(mca_uart);
		if (ret)
			goto error_port;

		/* Enable UART */
		ret = regmap_write(regmap, mca_uart->baddr + MCA_REG_UART_CFG0,
				   MCA_REG_UART_CFG0_ENABLE);
		if (ret) {
			dev_err(&pdev->dev, "Failed to write MCA_REG_UART_CFG0\n");
			goto error_port;
		}

		/* Report registered UART data to user */
		sprintf(msg, "%s%d@0x%x IRQ=%d, IOs:",
			uart_drv->uart.dev_name, i, mca_uart->baddr,
			mca_uart->port.irq);
		for (pin = 0; pin < mca_uart->npins; pin++)
			sprintf(msg, "%s %s=%d,", msg, iopins_names[pin],
				mca_uart->pins[pin]);
		dev_info(&pdev->dev, "%s\n", msg);
	}

	return 0;

error_port:
	while (i-- >= 0) {
		struct mca_uart *mca_uart = &uart_drv->ports[i];

		mca_uart_release_port_resources(mca_uart);
	}
error_drv:
	uart_unregister_driver(&uart_drv->uart);
err_free2:
	devm_kfree(&pdev->dev, uart_drv->ports);
err_free1:
	devm_kfree(&pdev->dev, uart_drv);

	return ret;
}

static int mca_uart_remove(struct platform_device *pdev)
{
	struct mca_uart_drv *uart_drv = dev_get_drvdata(pdev->dev.parent);
	int i;

	for (i = 0; i < uart_drv->num_uarts; i++) {
		mca_uart_release_port_resources(&uart_drv->ports[i]);
	}

	uart_unregister_driver(&uart_drv->uart);
	devm_kfree(&pdev->dev, uart_drv->ports);
	devm_kfree(&pdev->dev, uart_drv);

	return 0;
}

#ifdef CONFIG_PM
/*
 * The code snippet below was grabbed from drivers/tty/serial/serial_core.c
 * It is used for retrieving the TTY layer struct device. This struct is used to
 * check the value of /sys/class/tty/ttyMCAx/power/wakeup which is more standard
 * than the one at /sys/bus/i2c/devices/0-0063/mca-cc8x-uart/power/wakeup.
 */
struct uart_match {
	struct uart_port *port;
	struct uart_driver *driver;
};

static int serial_match_port(struct device *dev, void *data)
{
	struct uart_match *match = data;
	struct tty_driver *tty_drv = match->driver->tty_driver;
	dev_t devt = MKDEV(tty_drv->major, tty_drv->minor_start) +
		     match->port->line;

	return dev->devt == devt;
}

static int mca_uart_suspend(struct device *dev)
{
	struct mca_uart_drv *uart_drv = dev_get_drvdata(dev);
	struct regmap *regmap = uart_drv->mca->regmap;
	unsigned int mask = MCA_REG_UART_CFG0_WAKEUP;
	unsigned int new_value = 0;
	int i, ret;

	for (i = 0; i < uart_drv->num_uarts; i++) {
		struct uart_match match = {&uart_drv->ports[i].port,
					   &uart_drv->uart};
		struct device *tty_dev =
			device_find_child(uart_drv->ports[i].port.dev,
					  &match,
					  serial_match_port);
		struct mca_uart *mca_uart = &uart_drv->ports[i];

		if (!tty_dev)
			continue;

		if (device_may_wakeup(tty_dev))
			new_value |= MCA_REG_UART_CFG0_WAKEUP;

		ret = regmap_update_bits(regmap, mca_uart->baddr +
					 MCA_REG_UART_CFG0, mask, new_value);
		if (ret < 0)
			dev_err(dev, "Failed to write MCA_REG_UART_CFG0\n");
	}

	return 0;
}

static const struct dev_pm_ops mca_uart_pm_ops = {
	.suspend	= mca_uart_suspend,
	.resume		= NULL,
	.poweroff	= NULL,
};
#endif

static struct mca_uart_data mca_uart_devdata[] = {
	[CC6UL_MCA_UART] = {
		.devtype	= CC6UL_MCA_UART,
		.since		= MCA_CC6UL_UART_MIN_FW,
		.nuarts		= 1,
	},
	[CC8X_MCA_UART] = {
		.devtype	= CC8X_MCA_UART,
		.since		= MCA_CC8X_UART_MIN_FW,
		.nuarts		= 3,
	},
};

static const struct platform_device_id mca_uart_devtype[] = {
	{
		.name = "mca-cc6ul-uart",
		.driver_data = (kernel_ulong_t)&mca_uart_devdata[CC6UL_MCA_UART],
	}, {
		.name = "mca-cc8x-uart",
		.driver_data = (kernel_ulong_t)&mca_uart_devdata[CC8X_MCA_UART],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, mca_uart_devtype);

#ifdef CONFIG_OF
static const struct of_device_id mca_uart_ids[] = {
	{
		.compatible = "digi,mca-cc6ul-uart",
		.data = &mca_uart_devdata[CC6UL_MCA_UART]
	}, {
		.compatible = "digi,mca-cc8x-uart",
		.data = &mca_uart_devdata[CC8X_MCA_UART]
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, mca_uart_ids);
#endif

static struct platform_driver mca_uart_driver = {
	.probe	= mca_uart_probe,
	.remove	= mca_uart_remove,
	.id_table = mca_uart_devtype,
	.driver	= {
		.name	= MCA_DRVNAME_UART,
		.of_match_table = of_match_ptr(mca_uart_ids),
#ifdef CONFIG_PM
		.pm	= &mca_uart_pm_ops,
#endif
	},
};

static int __init mca_uart_init(void)
{
	return platform_driver_register(&mca_uart_driver);
}
module_init(mca_uart_init);

static void __exit mca_uart_exit(void)
{
	platform_driver_unregister(&mca_uart_driver);
}
module_exit(mca_uart_exit);

MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("UART driver for MCA of ConnectCore modules");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_DRVNAME_UART);
