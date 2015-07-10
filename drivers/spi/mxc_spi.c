/*
 * Copyright (C) 2008, Guennadi Liakhovetski <lg@denx.de>
 *
 * Copyright (C) 2013-2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <spi_flash.h>
#include <mtd/spi-nor.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>

#ifdef CONFIG_MX27
/* i.MX27 has a completely wrong register layout and register definitions in the
 * datasheet, the correct one is in the Freescale's Linux driver */

#error "i.MX27 CSPI not supported due to drastic differences in register definitions" \
"See linux mxc_spi driver from Freescale for details."
#endif

static unsigned long spi_bases[] = {
	MXC_SPI_BASE_ADDRESSES
};

#define OUT	MXC_GPIO_DIRECTION_OUT

#define reg_read readl
#define reg_write(a, v) writel(v, a)

struct mxc_spi_slave {
	struct spi_slave slave;
	unsigned long	base;
	u32		ctrl_reg;
#if defined(MXC_ECSPI)
	u32		cfg_reg;
#endif
	int		gpio;
	int		ss_pol;
};

static inline struct mxc_spi_slave *to_mxc_spi_slave(struct spi_slave *slave)
{
	return container_of(slave, struct mxc_spi_slave, slave);
}

void spi_cs_activate(struct spi_slave *slave)
{
	struct mxc_spi_slave *mxcs = to_mxc_spi_slave(slave);
	if (mxcs->gpio > 0)
		gpio_set_value(mxcs->gpio, mxcs->ss_pol);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	struct mxc_spi_slave *mxcs = to_mxc_spi_slave(slave);
	if (mxcs->gpio > 0)
		gpio_set_value(mxcs->gpio,
			      !(mxcs->ss_pol));
}

u32 get_cspi_div(u32 div)
{
	int i;

	for (i = 0; i < 8; i++) {
		if (div <= (4 << i))
			return i;
	}
	return i;
}

#ifdef MXC_CSPI
static s32 spi_cfg_mxc(struct mxc_spi_slave *mxcs, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	unsigned int ctrl_reg;
	u32 clk_src;
	u32 div;

	clk_src = mxc_get_clock(MXC_CSPI_CLK);

	div = DIV_ROUND_UP(clk_src, max_hz);
	div = get_cspi_div(div);

	debug("clk %d Hz, div %d, real clk %d Hz\n",
		max_hz, div, clk_src / (4 << div));

	ctrl_reg = MXC_CSPICTRL_CHIPSELECT(cs) |
		MXC_CSPICTRL_BITCOUNT(MXC_CSPICTRL_MAXBITS) |
		MXC_CSPICTRL_DATARATE(div) |
		MXC_CSPICTRL_EN |
#ifdef CONFIG_MX35
		MXC_CSPICTRL_SSCTL |
#endif
		MXC_CSPICTRL_MODE;

	if (mode & SPI_CPHA)
		ctrl_reg |= MXC_CSPICTRL_PHA;
	if (mode & SPI_CPOL)
		ctrl_reg |= MXC_CSPICTRL_POL;
	if (mode & SPI_CS_HIGH)
		ctrl_reg |= MXC_CSPICTRL_SSPOL;
	mxcs->ctrl_reg = ctrl_reg;

	return 0;
}
#endif

#ifdef MXC_ECSPI
static s32 spi_cfg_mxc(struct mxc_spi_slave *mxcs, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
#ifdef MXC_ECSPI_SEPARATE_CLKS
	u32 clk_src = mxc_get_clock_bus(MXC_CSPI_CLK, mxcs->slave.bus);
#else
	u32 clk_src = mxc_get_clock(MXC_CSPI_CLK);
#endif
	s32 pre_div = 0, post_div = 0, i, reg_ctrl, reg_config;
	u32 ss_pol = 0, sclkpol = 0, sclkpha = 0;
	struct cspi_regs *regs = (struct cspi_regs *)mxcs->base;

	if (max_hz == 0) {
		printf("Error: desired clock is 0\n");
		return -1;
	}

	/*
	 * Reset SPI and set all CSs to master mode, if toggling
	 * between slave and master mode we might see a glitch
	 * on the clock line
	 */
	reg_ctrl = MXC_CSPICTRL_MODE_MASK;
	reg_write(&regs->ctrl, reg_ctrl);
	reg_ctrl |=  MXC_CSPICTRL_EN;
	reg_write(&regs->ctrl, reg_ctrl);

	/*
	 * The following computation is taken directly from Freescale's code.
	 */
	if (clk_src > max_hz) {
		pre_div = DIV_ROUND_UP(clk_src, max_hz);
		if (pre_div > 16) {
			post_div = pre_div / 16;
			pre_div = 15;
		}
		if (post_div != 0) {
			for (i = 0; i < 16; i++) {
				if ((1 << i) >= post_div)
					break;
			}
			if (i == 16) {
				printf("Error: no divider for the freq: %d\n",
					max_hz);
				return -1;
			}
			post_div = i;
		}
	}

	debug("pre_div = %d, post_div=%d\n", pre_div, post_div);
	reg_ctrl = (reg_ctrl & ~MXC_CSPICTRL_SELCHAN(3)) |
		MXC_CSPICTRL_SELCHAN(cs);
	reg_ctrl = (reg_ctrl & ~MXC_CSPICTRL_PREDIV(0x0F)) |
		MXC_CSPICTRL_PREDIV(pre_div);
	reg_ctrl = (reg_ctrl & ~MXC_CSPICTRL_POSTDIV(0x0F)) |
		MXC_CSPICTRL_POSTDIV(post_div);

	/* We need to disable SPI before changing registers */
	reg_ctrl &= ~MXC_CSPICTRL_EN;

	if (mode & SPI_CS_HIGH)
		ss_pol = 1;

	if (mode & SPI_CPOL)
		sclkpol = 1;

	if (mode & SPI_CPHA)
		sclkpha = 1;

	reg_config = reg_read(&regs->cfg);

	/*
	 * Configuration register setup
	 * The MX51 supports different setup for each SS
	 */
	reg_config = (reg_config & ~(1 << (cs + MXC_CSPICON_SSPOL))) |
		(ss_pol << (cs + MXC_CSPICON_SSPOL));
	reg_config = (reg_config & ~(1 << (cs + MXC_CSPICON_POL))) |
		(sclkpol << (cs + MXC_CSPICON_POL));
	reg_config = (reg_config & ~(1 << (cs + MXC_CSPICON_PHA))) |
		(sclkpha << (cs + MXC_CSPICON_PHA));

	debug("reg_ctrl = 0x%x\n", reg_ctrl);
	reg_write(&regs->ctrl, reg_ctrl);
	debug("reg_config = 0x%x\n", reg_config);
	reg_write(&regs->cfg, reg_config);

	/* save config register and control register */
	mxcs->ctrl_reg = reg_ctrl;
	mxcs->cfg_reg = reg_config;

	/* clear interrupt reg */
	reg_write(&regs->intr, 0);
	reg_write(&regs->stat, MXC_CSPICTRL_TC | MXC_CSPICTRL_RXOVF);

	return 0;
}
#endif

static int decode_cs(struct mxc_spi_slave *mxcs, unsigned int cs)
{
	int ret;

	/*
	 * Some SPI devices require active chip-select over multiple
	 * transactions, we achieve this using a GPIO. Still, the SPI
	 * controller has to be configured to use one of its own chipselects.
	 * To use this feature you have to call spi_setup_slave() with
	 * cs = internal_cs | (gpio << 8), and you have to use some unused
	 * on this SPI controller cs between 0 and 3.
	 */
	if (cs > 3) {
		mxcs->gpio = cs >> 8;
		cs &= 3;
		ret = gpio_direction_output(mxcs->gpio, !(mxcs->ss_pol));
		if (ret) {
			printf("mxc_spi: cannot setup gpio %d\n", mxcs->gpio);
			return -EINVAL;
		}
	} else {
		mxcs->gpio = -1;
	}

	return cs;
}

int spi_xchg_single(struct spi_slave *slave, unsigned int bitlen,
	const u8 *dout, u8 *din, unsigned long flags)
{
	struct mxc_spi_slave *mxcs = to_mxc_spi_slave(slave);
	int nbytes = (bitlen + 7) / 8;
	u32 data, cnt, i;
	struct cspi_regs *regs = (struct cspi_regs *)mxcs->base;

	debug("%s: bitlen %d dout 0x%x din 0x%x\n",
		__func__, bitlen, (u32)dout, (u32)din);

	mxcs->ctrl_reg = (mxcs->ctrl_reg &
		~MXC_CSPICTRL_BITCOUNT(MXC_CSPICTRL_MAXBITS)) |
		MXC_CSPICTRL_BITCOUNT(bitlen - 1);

	reg_write(&regs->ctrl, mxcs->ctrl_reg | MXC_CSPICTRL_EN);
#ifdef MXC_ECSPI
	reg_write(&regs->cfg, mxcs->cfg_reg);
#endif

	/* Clear interrupt register */
	reg_write(&regs->stat, MXC_CSPICTRL_TC | MXC_CSPICTRL_RXOVF);

	/*
	 * The SPI controller works only with words,
	 * check if less than a word is sent.
	 * Access to the FIFO is only 32 bit
	 */
	if (bitlen % 32) {
		data = 0;
		cnt = (bitlen % 32) / 8;
		if (dout) {
			for (i = 0; i < cnt; i++) {
				data = (data << 8) | (*dout++ & 0xFF);
			}
		}
		debug("Sending SPI 0x%x\n", data);

		reg_write(&regs->txdata, data);
		nbytes -= cnt;
	}

	data = 0;

	while (nbytes > 0) {
		data = 0;
		if (dout) {
			/* Buffer is not 32-bit aligned */
			if ((unsigned long)dout & 0x03) {
				data = 0;
				for (i = 0; i < 4; i++)
					data = (data << 8) | (*dout++ & 0xFF);
			} else {
				data = *(u32 *)dout;
				data = cpu_to_be32(data);
			}
			dout += 4;
		}
		debug("Sending SPI 0x%x\n", data);
		reg_write(&regs->txdata, data);
		nbytes -= 4;
	}

	/* FIFO is written, now starts the transfer setting the XCH bit */
	reg_write(&regs->ctrl, mxcs->ctrl_reg |
		MXC_CSPICTRL_EN | MXC_CSPICTRL_XCH);

	/* Wait until the TC (Transfer completed) bit is set */
	while ((reg_read(&regs->stat) & MXC_CSPICTRL_TC) == 0)
		;

	/* Transfer completed, clear any pending request */
	reg_write(&regs->stat, MXC_CSPICTRL_TC | MXC_CSPICTRL_RXOVF);

	nbytes = (bitlen + 7) / 8;

	cnt = nbytes % 32;

	if (bitlen % 32) {
		data = reg_read(&regs->rxdata);
		cnt = (bitlen % 32) / 8;
		data = cpu_to_be32(data) >> ((sizeof(data) - cnt) * 8);
		debug("SPI Rx unaligned: 0x%x\n", data);
		if (din) {
			memcpy(din, &data, cnt);
			din += cnt;
		}
		nbytes -= cnt;
	}

	while (nbytes > 0) {
		u32 tmp;
		tmp = reg_read(&regs->rxdata);
		data = cpu_to_be32(tmp);
		debug("SPI Rx: 0x%x 0x%x\n", tmp, data);
		cnt = min(nbytes, sizeof(data));
		if (din) {
			memcpy(din, &data, cnt);
			din += cnt;
		}
		nbytes -= cnt;
	}

	return 0;
}

void _spi_init(void)
{
}

int _spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags)
{
	int n_bytes = (bitlen + 7) / 8;
	int n_bits;
	int ret;
	u32 blk_size;
	u8 *p_outbuf = (u8 *)dout;
	u8 *p_inbuf = (u8 *)din;

	if (!slave)
		return -1;

	if (flags & SPI_XFER_BEGIN)
		spi_cs_activate(slave);

	while (n_bytes > 0) {
		if (n_bytes < MAX_SPI_BYTES)
			blk_size = n_bytes;
		else
			blk_size = MAX_SPI_BYTES;

		n_bits = blk_size * 8;

		ret = spi_xchg_single(slave, n_bits, p_outbuf, p_inbuf, 0);

		if (ret)
			return ret;
		if (dout)
			p_outbuf += blk_size;
		if (din)
			p_inbuf += blk_size;
		n_bytes -= blk_size;
	}

	if (flags & SPI_XFER_END) {
		spi_cs_deactivate(slave);
	}

	return 0;
}

struct spi_slave *_spi_setup_slave(unsigned int bus, unsigned int cs,
			unsigned int max_hz, unsigned int mode)
{
	struct mxc_spi_slave *mxcs;
	int ret;

	if (bus >= ARRAY_SIZE(spi_bases))
		return NULL;

	mxcs = spi_alloc_slave(struct mxc_spi_slave, bus, cs);
	if (!mxcs) {
		puts("mxc_spi: SPI Slave not allocated !\n");
		return NULL;
	}

	mxcs->ss_pol = (mode & SPI_CS_HIGH) ? 1 : 0;

	ret = decode_cs(mxcs, cs);
	if (ret < 0) {
		free(mxcs);
		return NULL;
	}

	cs = ret;

	mxcs->base = spi_bases[bus];

	ret = spi_cfg_mxc(mxcs, cs, max_hz, mode);
	if (ret) {
		printf("mxc_spi: cannot setup SPI controller\n");
		free(mxcs);
		return NULL;
	}
	return &mxcs->slave;
}

void _spi_free_slave(struct spi_slave *slave)
{
	struct mxc_spi_slave *mxcs = to_mxc_spi_slave(slave);

	free(mxcs);
}

int _spi_claim_bus(struct spi_slave *slave)
{
	struct mxc_spi_slave *mxcs = to_mxc_spi_slave(slave);
	struct cspi_regs *regs = (struct cspi_regs *)mxcs->base;

	reg_write(&regs->rxdata, 1);
	udelay(1);
	reg_write(&regs->ctrl, mxcs->ctrl_reg);
	reg_write(&regs->period, MXC_CSPIPERIOD_32KHZ);
	reg_write(&regs->intr, 0);

	return 0;
}

void _spi_release_bus(struct spi_slave *slave)
{
	/* TODO: Shut the controller down */
}

/* -----------------------------------------------------------------------*/
#ifdef CONFIG_FSL_D4400_QSPI

#define SWAP32(val32) \
			(\
			((val32 & 0xff000000) >> 24) |\
			((val32 & 0x00ff0000) >>  8) |\
			((val32 & 0x0000ff00) <<  8) |\
			((val32 & 0x000000ff) << 24) \
			)

struct qspi_data {
	struct spi_slave slave;
	u32 iomode;
};

struct spi_slave *qspi_slave = NULL;
volatile struct qspi_regs *qspi_reg =
	(volatile struct qspi_regs *)QSPI_BASE_ADDR;

static inline struct qspi_data *to_qspi_data(struct spi_slave *slave)
{
	if (slave)
		return container_of(slave, struct qspi_data, slave);
	else
		return NULL;
}

void _qspi_print_buffer(u8 *data, u32 num_bytes, u8 num_per_line)
{
	u32 i;
	u8 *p8 = data;

	for (i = 0; i < num_bytes; ++i) {
		if ((i % num_per_line) == 0) {
			printf("\n");
			printf("%08x: ", (u32)p8);
		}

		printf(" %02x", *p8++);
	}
	printf("\n\n");
}

static u32 get_qspi_clk_freq(u32 clk_src_hz, u32 div)
{
	div += 2;
	if (!div)
		/* Just in case, avoid div by zero */
		return 0;
	return (clk_src_hz << 1) / div;
}

static u32 get_qspi_clk_div(u32 clk_src_hz, u32 max_hz)
{
	u32 div = 0;
	u32 div_shf;

	if (max_hz < clk_src_hz) {
		/* Qspi can divide in 1/2 increments
			0000	0.0	0
			0000	0.5	0
			0000	1.0	0
			0001	1.5	1
			0010	2.0	2
			0011	2.5	3
			0100	3.0	4
			0101	3.5	5
			0110	4.0	6
			0111	4.5	7
			1000	5.0	8
			1001	5.5	9
			1010	6.0	10
			1011	6.5	11
			1100	7.0	12
			1101	7.5	13
			1110	8.0	14
			1111	8.5	15
		*/
		div_shf = (clk_src_hz  << 1) / max_hz;
		div = div_shf - 2;

		if (div > 0x0F) /* 0x0F is max, div by 8.5 */
		  div = 0x0F;

		debug("div_shf = %i\n", div_shf);
	}
	return div;
}

static u32 qspi_cfg(struct qspi_data *qspi, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	u32 mcr_reg;
	u32 clk_src_hz;
	u32 div;

	/* Override requested speed as that is for regular spi */
	if (max_hz < CONFIG_SPI_FLASH_SPEED_MIN_HZ)
		max_hz = CONFIG_SPI_FLASH_SPEED_MIN_HZ;
	else if (max_hz > CONFIG_SPI_FLASH_SPEED_MAX_HZ)
		max_hz = CONFIG_SPI_FLASH_SPEED_MAX_HZ;

	clk_src_hz = mxc_get_clock(MXC_QSPI_CLK);
	div = get_qspi_clk_div(clk_src_hz, max_hz);
	debug("clk %d Hz, div %d, real clk %d Hz\n",
		max_hz, div, clk_src_hz / (4 << div));

	/* Allow certain reg to be modified */
	qspi_reg->mcr |= QSPI_MCR_MDIS_MASK;

	mcr_reg = qspi_reg->mcr;
	mcr_reg &= ~QSPI_MCR_SCLKCFG_MASK;
	mcr_reg |= (div << QSPI_MCR_SCLKCFG_OFFSET);

	qspi_reg->mcr = mcr_reg;
	qspi_reg->mcr &= ~QSPI_MCR_MDIS_MASK; /* Lock */

	/* Return actual clock speed */
	return get_qspi_clk_freq(clk_src_hz, div);
}

/* Read a small number of bytes from qspi rx fifo.
 * This function can be used to read status, ids, etc.
 */
static int _qspi_read_rx_fifo(u8 *dest8, int num_bytes)
{
	int ret = 0;
	u32 *qspi_rxbuf = (u32 *)&qspi_reg->rbdr_base;
	u32 val32;
	int i, j, k;
	u8 *src8;

	if (num_bytes > QSPI_RX_FIFO_SIZE)
		return -EINVAL;

	j = 0;
	/* Copy from qspi rx fifo to memory */
	for (i = 0; i < ((num_bytes+3)/4); ++i) {
		val32 = qspi_rxbuf[i];
		val32 = SWAP32(val32);
		src8 = (u8 *)&val32;
		/* Copy up to 32-bit/4 bytes */
		k = 0;
		while ((j < num_bytes) && (k < 4))
			dest8[j++] = src8[k++];
	}
	return ret;
}

/* Write a small number of bytes from qspi tx fifo.
 * This function can be used to write config reg, etc.
 */
static int _qspi_write_tx_fifo(u8 *src8, int num_bytes)
{
	int ret = 0;
	u32 *qspi_txbuf = (u32 *)&qspi_reg->tbdr;
	u32 val32;
	int i, j, k;

	if (num_bytes > QSPI_TX_FIFO_SIZE)
		return -EINVAL;

	j = 0;
	/* Write from memory to qspi tx fifo */
	for (i = 0; i < ((num_bytes+3)/4); ++i) {
		val32 = 0;
		for (k = 0; (j < num_bytes) & (k < 4); ++k)
			/* Note that msb/lsb swapping is done here */
			val32 |= (src8[j++] << ((3 - k) * 8));
			val32 |= 0;
		*qspi_txbuf = val32;
	}
	return ret;
}

/* Write up to one tx fifo size */
int _qspi_xfer_write(u32 *src_buf, u32 *dest_buf, u32 num_bytes, u8 opcode,
	struct spi_flash *flash)
{
	/* Address of system memory */
	u8 *src = (u8 *)src_buf;
	/* Address of qspi flash device */
	u8 *dest = (u8 *)dest_buf;

	/* Length must be less than tx fifo size */
	if (num_bytes > QSPI_TX_FIFO_SIZE) {
		printf("qspi: Number of bytes exceeds TX fifo size, %i\n",
			num_bytes);
		return -EINVAL;
	}

	/* Fill the tx fifo */
	_qspi_write_tx_fifo(src, num_bytes);

	/* Set flash address to erase */
	QSPI_SET_ADDR((u32)dest);
	/* Write */
	QSPI_SEND_CMD(num_bytes, opcode, CLEAR_BUFPTR_NONE);

	return 0;
}

int _qspi_xfer_read(u32 *src_buf, u32 *dest_buf, u32 num_bytes, u8 opcode,
	struct spi_flash *flash)
{
	int ret = 0;
	u8 *src;			/* Address of qspi flash device */
	u8 *dest = (u8 *)dest_buf;	/* Address of system memory */

	/* Set read opcode */
	QSPI_SET_AMBA_READ_OPCODE(opcode);

	src = (u8 *)((u32)src_buf + QSPI_FLASH_BASE_ADDR);
	memcpy(dest, src, num_bytes);

	return ret;
}

/* Erase one sector of flash */
int _qspi_erase(u32 *start_addr, u32 num_bytes, u8 opcode,
	struct spi_flash *flash)
{
	int ret = 0;

	/* Get address of start of sector */
	u32 addr = (u32)start_addr &
		~(u32)(flash->sector_size-1);

	/* Set flash address to erase */
	QSPI_SET_ADDR(addr);
	/* Do erase */
	QSPI_SEND_CMD(0, opcode, CLEAR_BUFPTR_NONE);

	return ret;
}

/* _qspi_xfer implements read/write with the following rule:
 *
 * - For read operations:
 *	dout = addr in qspi flash, data source
 *	din  = addr in system memory, data destination
 *
 * - For write operations:
 *	dout = addr in system memory, data source
 *	din  = addr in qspi flash, data destination
 *
 * - Read operation: flags with QSPI_MODE_RW bit cleared
 * - Write operation: flags with QSPI_MODE_RW bit set
 */
static int _qspi_xfer(struct spi_slave *slave, unsigned int bitlen,
	const void *dout, void *din, unsigned long flags)
{
	int ret = 0;
	int i;
	u8 *src8;
	u32 num_bytes =  (bitlen + 7) / 8;
	static u8 cmd[8];
	struct spi_flash *flash;

	/* Note that din/dout must be 32-bit aligned. */
	u32 *p_outbuf = (u32 *)dout;
	u32 *p_inbuf = (u32 *)din;

	if (!slave)
		return -EINVAL;
	flash = spi_get_flash(slave->bus, slave->cs);

	/* For typical spi devices, read/write operation are done in
	 * two stages: 1) command sent 2) read or write operation.
	 * The xfer() func is first invoked with flag set to SPI_XFER_BEGIN
	 * to send the command.  It is called again with SPI_XFER_END
	 * to do the actual read/write operation.  Sometimes it is called
	 * with both flags set, but the order of operation is the same.
	 *
	 * For qspi flash devices connected to an qspi module in an ASIC,
	 * the read/write operation is typically done in one operation.
	 * The address and number of bytes to transfer is programmed in
	 * the qspi module of the ASIC.  When the command is sent, the
	 * qspi module reads/write data from/to a buffer in the ASIC.
	 * For this type of use model, when this function is invoked with
	 * the SPI_XFER_BEGIN flag, it queues up the command and parameters.
	 * When invoked with the SPI_XFER_END flag, it programs everything
	 * in the qspi module which sends the command and performs read/write
	 * operation in hardware.  If invoked with both flags set, this func
	 * performs the operation as listed above.
	 */
	if (flags & SPI_XFER_BEGIN) {
		/* Reset command queue */
		memset(cmd, 0, 8);

		if (num_bytes < 8) {
			/* Queue up the command */
			src8 = (u8 *)p_outbuf;
			for (i = 0; i < num_bytes; ++i)
				cmd[i] = src8[i];
		} else {
			printf("qspi: Too many command bytes (%i)\n",
				num_bytes);
			ret = -EINVAL;
		}

		if (flags == SPI_XFER_BEGIN)
			return ret;
	}

	/* Check current quad mode setting and change if needed */

	switch (cmd[0]) {
	case SPINOR_OP_BE:
	case SPINOR_OP_DIE_ERASE:
		/* Bulk erase */
		QSPI_SEND_CMD(0, cmd[0],	CLEAR_BUFPTR_NONE);
		/* Caller needs to wait for erase to be done. */
		break;
	case SPINOR_OP_WREN:
		/* Write enable */
		QSPI_SEND_CMD(0, cmd[0], CLEAR_BUFPTR_NONE);
		break;
	case SPINOR_OP_WRSR:
		/* Write to status / config reg */
		 _qspi_write_tx_fifo((u8 *)p_inbuf, num_bytes);
		QSPI_SEND_CMD(num_bytes, cmd[0],	CLEAR_BUFPTR_NONE);
		break;
	case SPINOR_OP_RDSR: /* Read status reg */
	case SPINOR_OP_RDCR: /* Read config reg */
	case SPINOR_OP_BRRD: /* Read bank reg */
	case SPINOR_OP_RDID: /* JEDEC id */
		QSPI_SET_ADDR(0);
		QSPI_SEND_CMD(num_bytes, cmd[0],
			CLEAR_BUFPTR_RXTX);
		_qspi_read_rx_fifo((u8 *)p_inbuf, num_bytes);
		break;
	case SPINOR_OP_READ:
	case SPINOR_OP_READ4:
	case SPINOR_OP_READ4_1_1_2:
	case SPINOR_OP_READ4_1_1_4:
		/* Assemble the flash offset address */
		p_outbuf = (u32 *)((cmd[1] << 24) | (cmd[2] << 16) |
			(cmd[3] << 8) | cmd[4]);
		ret = _qspi_xfer_read(p_outbuf, p_inbuf, num_bytes, cmd[0],
			flash);
		break;
	case SPINOR_OP_PP:
	case SPINOR_OP_PP_4B:
	case SPINOR_OP_PP_4B_1_1_4:
		/* Assemble the flash offset address */
		p_inbuf = (u32 *)((cmd[1] << 24) | (cmd[2] << 16) |
			(cmd[3] << 8) | cmd[4]);
		ret = _qspi_xfer_write(p_outbuf, p_inbuf, num_bytes, cmd[0],
			flash);
		break;
	case SPINOR_OP_SE:
	case SPINOR_OP_SE_4B:
		/* Convert sector number in cmd[2:1] to address */
		p_outbuf = (u32 *)(((cmd[1] << 8) | cmd[2]) *
			flash->sector_size);
		/* 16 is a dummy value.  Any sector that the p_outbuf
		 * falls in is erased.
		 */
		ret = _qspi_erase(p_outbuf, 16, cmd[0], flash);
		break;
	default:
		printf("%s: Unrecognized command cmd[0]:x%02x\n",
			__func__, cmd[0]);
		ret = -1;
		break;
	};

	if (flags & SPI_XFER_END) {
		/* Reset command queue again in case a SPI_XFER_BEGIN is
		 * is not sent.
		 */
		memset(cmd, 0, 8);
	}
	return ret;
}

void _qspi_free_slave(struct spi_slave *slave)
{
	struct qspi_data *qspi = to_qspi_data(slave);
	free(qspi);
}

struct spi_slave *_qspi_setup_slave(unsigned int bus, unsigned int cs,
			unsigned int max_hz, unsigned int mode)
{
	struct qspi_data *qspi = NULL;

	qspi = spi_alloc_slave(struct qspi_data, bus, cs);
	if (!qspi) {
		puts("qspi: QSPI Slave not allocated !\n");
		return NULL;
	}

	/* Actual clock speed is returned */
	qspi->slave.speed_hz = qspi_cfg(qspi, cs, max_hz, mode);

	if (!qspi->slave.speed_hz) {
		printf("qspi: cannot setup QSPI controller\n");
		free(qspi);
		return NULL;
	}

	/* Always operate in 4-byte address mode */
	QSPI_SET_ADDR_MODE(QSPI_ADDR_MODE_32BIT);

	/* Save quad io mode */
	qspi->slave.mode = mode;

	return &qspi->slave;
}

int _qspi_claim_bus(struct spi_slave *slave)
{
	if (!slave)
		return -EINVAL;

	/* Clear RX and TX buffer status */
	QSPI_CLEAR_RX_TX_BUFPTR();
	return 0;
}

void _qspi_release_bus(struct spi_slave *slave)
{
	/* TODO: Shut the controller down */
}

#endif /* CONFIG_FSL_D4400_QSPI */
/* -----------------------------------------------------------------------*/

/* -----------------------------------------------------------------------*/
/* SPI interface functions. */

void spi_init(void)
{
	_spi_init();
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
			unsigned int max_hz, unsigned int mode)
{
#ifdef CONFIG_FSL_D4400_QSPI
	int num_spi_devices = ARRAY_SIZE(spi_bases);

	/* Use last device+1 as the qspi bus */
	if (bus == (num_spi_devices + 1)) {
		/* Quad io spi */
		qspi_slave = _qspi_setup_slave(bus, cs, max_hz, mode);
		return qspi_slave;
	} else
#endif
		/* Two wire spi */
		return _spi_setup_slave(bus, cs, max_hz, mode);
}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags)
{
#ifdef CONFIG_FSL_D4400_QSPI
	if ((qspi_slave->bus == slave->bus) &&
		(qspi_slave->cs == slave->cs))
		/* bitlen is really number of bytes for qspi */
		return _qspi_xfer(slave, bitlen, dout, din, flags);
	else
#endif
		return _spi_xfer(slave, bitlen, dout, din, flags);
}

void spi_free_slave(struct spi_slave *slave)
{
#ifdef CONFIG_FSL_D4400_QSPI
	if ((qspi_slave->bus == slave->bus) &&
		(qspi_slave->cs == slave->cs))
		_qspi_free_slave(slave);
	else
#endif
		_spi_free_slave(slave);
}

int spi_claim_bus(struct spi_slave *slave)
{
#ifdef CONFIG_FSL_D4400_QSPI
	if ((qspi_slave->bus == slave->bus) &&
		(qspi_slave->cs == slave->cs))
		return _qspi_claim_bus(slave);
	else
#endif
		return _spi_claim_bus(slave);
}

void spi_release_bus(struct spi_slave *slave)
{
	/* TODO: Shut the controller down */
#ifdef CONFIG_FSL_D4400_QSPI
	if ((qspi_slave->bus == slave->bus) &&
		(qspi_slave->cs == slave->cs))
		_qspi_release_bus(slave);
	else
#endif
		_spi_release_bus(slave);
}
