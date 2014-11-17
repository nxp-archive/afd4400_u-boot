/*
 * Copyright (C) 2008, Guennadi Liakhovetski <lg@denx.de>
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
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
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#ifdef CONFIG_QSPI_FLASH_SPANSION
#include <asm/arch/qspi_spansion_s25l.h>
#endif

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

struct qspi_slave {
	struct spi_slave slave;
	u32		clk_actual;
	u32		addr_mode;
	u32		quad_enabled;
};


struct spi_slave *qspi_slave = NULL;
volatile struct qspi_regs *qspi_reg = (volatile struct qspi_regs *)QSPI_BASE_ADDR;
static inline struct qspi_slave *to_qspi_slave(struct spi_slave *slave)
{
	return container_of(slave, struct qspi_slave, slave);
}

void _qspi_print_buffer(u8 *pData, u32 num_bytes, u8 num_per_line)
{
	
	u32 i;
	u8 *p8 = pData;
	
	for(i = 0; i < num_bytes; ++i)
	{
		if ( (i % num_per_line) == 0)
		{
			printf("\n");
			printf("%08x: ", (u32)p8);
		}

		printf(" %02x", *p8++);
	}
	printf("\n\n");
}

static u32 get_qspi_clk_div(u32 clk_src_hz, u32 max_hz)
{
	u32 div = 0;
	u32 div_shf;
	  
	if (max_hz < clk_src_hz)
	{
		/* Qspi can divide in 1/2 increments

			0000 	0.0	0
			0000 	0.5 	0
			0000 	1.0 	0
			0001 	1.5 	1
			0010 	2.0 	2
			0011 	2.5 	3
			0100 	3.0 	4
			0101 	3.5 	5
			0110 	4.0 	6
			0111 	4.5 	7
			1000 	5.0 	8
			1001 	5.5 	9
			1010 	6.0 	10
			1011 	6.5 	11
			1100 	7.0 	12
			1101 	7.5 	13
			1110 	8.0 	14
			1111    8.5	15
		*/

		div_shf = (clk_src_hz  << 1) / max_hz;
    
		div = div_shf - 2;
		
		if (div > 0x0F) /* 0x0F is max, div by 8.5 */
		  div = 0x0F;
		  
		debug("div_shf = %i\n", div_shf);		  
	}

	return div;
}

static s32 qspi_cfg(struct qspi_slave *qspi, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	u32 mcr_reg;
	u32 clk_src_hz;
	u32 div;

	/* Override requested speed as that is for regular spi */
	max_hz = CONFIG_QSPI_FLASH_SPEED_HZ;

	clk_src_hz = mxc_get_clock(MXC_QSPI_CLK);
	div = get_qspi_clk_div(clk_src_hz, max_hz);
	debug("clk %d Hz, div %d, real clk %d Hz\n",
		max_hz, div, clk_src_hz / (4 << div));

 	qspi_reg->mcr |= QSPI_MCR_MDIS_MASK;		/* Allow certain reg to be modified */

	mcr_reg = qspi_reg->mcr;
	mcr_reg &= ~QSPI_MCR_SCLKCFG_MASK;
 	mcr_reg |= (div << QSPI_MCR_SCLKCFG_OFFSET);

	if (mode & SPI_CPHA)
		qspi_reg->smpr |= QSPI_SMPR_FSPHS_MASK; /* Can be modified only when MCR MDIS=1) */

	qspi_reg->mcr = mcr_reg;
	qspi_reg->mcr &= ~QSPI_MCR_MDIS_MASK;		/* Lock */

	return 0;
}

int _qspi_xfer_write(u8 *pSrc_buf, u8 *pDest_buf, u32 num_bytes)
{
	int ret = 0;
	volatile u8 *pSrc = (volatile u8 *)pSrc_buf;   /* Address of system memory */
	volatile u8 *pDest = (volatile u8 *)pDest_buf; /* Address of qspi flash device */
	u32 blk_size;

	/* If source starting or ending address is greater than 16MB, 
	   then switch to 32bit addressing. 
	*/
	if (  ((u32)pSrc_buf & 0xff000000) ||  (((u32)pSrc_buf + num_bytes) & 0xff000000) )
	{
#ifdef CONFIG_FSL_D4400_QSPI
		QSPI_SET_ADDR_MODE(QSPI_ADDR_MODE_32BIT);
#endif
#ifdef CONFIG_QSPI_FLASH_SPANSION 
		QSPI_SPANSION_FLASH_SET_ADDR_MODE(SPANSION_ADDR_MODE_32BIT);
#endif
	}

	while (num_bytes > 0) {

		if (num_bytes < QSPI_MAX_TX_BYTES)
			blk_size = num_bytes;
		else
			blk_size = QSPI_MAX_TX_BYTES;

		/* Page programming requires that we do not cross page
		 * boundaries.  Limit the number of bytes so that we
		 * do not exceed the boundary.
		 */
		u32 next_page_start_addr = ((u32)pDest + SPANSION_PAGE_SIZE_BYTES) & ~(u32)(SPANSION_PAGE_SIZE_BYTES-1);
		u32 end_addr = (u32)pDest + (u32)blk_size;

		if  (end_addr > next_page_start_addr)
		{
			blk_size -= (end_addr - next_page_start_addr);
		}

		/* Do the write */
		{
			u32 num_words = blk_size / 4;
			volatile u32 *p32 = (volatile u32 *)pSrc;

			/* Left over bytes is 3 or less */
			u32 leftover_bytes = (blk_size - (num_words * 4));

			/* Bytes to write to flash must be 32bit aligned */
			u32 align32_size = blk_size;

			if (leftover_bytes)
				align32_size = (num_words * 4) + 4; /* Add one more word */


			/* Load whole words */
			QSPI_CLEAR_TX_BUFPTR();
			while(num_words--)
			{
#ifdef CONFIG_FSL_D4400_QSPI
				/* When booting from qspi flash devices, the D4400 boot rom reverses 
				 * the order of the bytes when reading data in 32-bit words (bit endian).
				 * Since the data is naturally written in little endian format to the
				 * qspi flash device, the bytes must be swapped to big endian format
				 * before written.
				 */
				u32 tmp = SWAP32(*p32);
				qspi_reg->tbdr = tmp;
				++p32;
#else
				qspi_reg->tbdr = *p32++;
#endif
			} 

			/* Load left over bytes if any */
			if (leftover_bytes)
			{
				/* Residue bytes (number of bytes left less than 4)
				 * must be "pushed" up towards the high end of the 32-bit
				 * TX register.
				 */

				int shift = 4 - leftover_bytes;
				u32 tmp = *p32;

				/* For unwritten bytes part of the 32-bit word, put 0xffs */
				tmp |= ((1 << (shift * 8))-1);
#ifdef CONFIG_FSL_D4400_QSPI
				tmp = SWAP32(tmp);
#endif
				qspi_reg->tbdr = tmp;
			}

#ifdef CONFIG_QSPI_FLASH_SPANSION 
			/* Blocking call */
	#ifdef CONFIG_QSPI_QUAD_ENABLE
			QSPI_SPANSION_FLASH_WRITE_QUAD((u32)pDest, align32_size);
	#else
			QSPI_SPANSION_FLASH_WRITE((u32)pDest, align32_size);
	#endif
#else
	#error Qspi flash write op not implemented!
#endif
		}

		if (ret)
			return ret;

		pSrc += blk_size;
		pDest += blk_size;
		num_bytes -= blk_size;
	}

	/* Switch back to 24bit mode in case cpu is reset. */
	if (  ((u32)pSrc_buf & 0xff000000) ||  (((u32)pSrc_buf + num_bytes) & 0xff000000) )
	{
#ifdef CONFIG_FSL_D4400_QSPI
		QSPI_SET_ADDR_MODE(QSPI_ADDR_MODE_24BIT);
#endif
#ifdef CONFIG_QSPI_FLASH_SPANSION 
		QSPI_SPANSION_FLASH_SET_ADDR_MODE(SPANSION_ADDR_MODE_24BIT);
#endif	
	}
	return 0;
}

int _qspi_xfer_read(u8 *pSrc_buf, u8 *pDest_buf, u32 num_bytes)
{
	int ret = 0;
	u8 *pSrc = (u8 *)pSrc_buf;   /* Address of qspi flash device */
	u8 *pDest = (u8 *)pDest_buf; /* Address of system memory */
	u32 blk_size;
	u32 aligned_size;

	/* If source starting or ending address is greater than 16MB, 
	   then switch to 32bit addressing. 
	*/
	if (  ((u32)pSrc_buf & 0xff000000) ||  (((u32)pSrc_buf + num_bytes) & 0xff000000) )
	{
#ifdef CONFIG_FSL_D4400_QSPI
		QSPI_SET_ADDR_MODE(QSPI_ADDR_MODE_32BIT);
#endif
#ifdef CONFIG_QSPI_FLASH_SPANSION 
		QSPI_SPANSION_FLASH_SET_ADDR_MODE(SPANSION_ADDR_MODE_32BIT);
#endif
	}

	while (num_bytes > 0) {

		if (num_bytes < QSPI_MAX_RX_BYTES)
			blk_size = num_bytes;
		else
			blk_size = QSPI_MAX_RX_BYTES;

		aligned_size = blk_size;
		while (aligned_size & 0x03)
		{ ++aligned_size; }

		/* Do the read */
		{
			u32 num_words = aligned_size / 4;
			volatile u32 *p32 = (u32 *)pDest;
			volatile u32 *pQspi_buf = (volatile u32*)&qspi_reg->rbdr_base;
			
			/* Left over bytes is 3 or less */
			u32 leftover_bytes = (aligned_size - blk_size);

#ifdef CONFIG_QSPI_FLASH_SPANSION 
			/* Blocking call */
	#ifdef CONFIG_QSPI_QUAD_ENABLE
			QSPI_SPANSION_FLASH_READ_QUAD((u32)pSrc, aligned_size);
	#else
			QSPI_SPANSION_FLASH_READ((u32)pSrc, aligned_size);
	#endif
#else
	#error Qspi flash read op not implemented!
#endif
			/* Read whole words */
			while(num_words--)
			{
#ifdef CONFIG_FSL_D4400_QSPI
				/* When booting from qspi flash devices, the D4400 boot rom reverses 
				 * the order of the bytes when reading data in 32-bit words (bit endian).
				 * Since the data is naturally written in little endian format to the
				 * qspi flash device, the bytes must be swapped to big endian format
				 * before written.
				 */
				*p32 = *pQspi_buf++;
				*p32 = SWAP32(*p32);
				p32++;
#else
				*p32++ = *pQspi_buf++;
#endif
			} 

			/* Read left over bytes if any */
			if (leftover_bytes)
			{
				/* Residue bytes (number of bytes left less than 4)
				 * must be "pushed" up towards the high end of the 32-bit
				 * TX register.
				 */
				int shift = 4 - leftover_bytes;
				volatile u32 tmp = *pQspi_buf;

				/* For bytes that are not read but part of the 32-bit word, put zeros */
				tmp &= ~((1 << (shift * 8))-1);

#ifdef CONFIG_FSL_D4400_QSPI
				tmp = SWAP32(tmp);
#endif
				*p32 = tmp;
			}
		}

		if (ret)
			return ret;

		pSrc += blk_size;
		pDest += blk_size;
		num_bytes -= blk_size;
	}

	/* Switch back to 24bit mode in case cpu is reset. */
	if (  ((u32)pSrc_buf & 0xff000000) ||  (((u32)pSrc_buf + num_bytes) & 0xff000000) )
	{
#ifdef CONFIG_FSL_D4400_QSPI
		QSPI_SET_ADDR_MODE(QSPI_ADDR_MODE_24BIT);
#endif
#ifdef CONFIG_QSPI_FLASH_SPANSION 
		QSPI_SPANSION_FLASH_SET_ADDR_MODE(SPANSION_ADDR_MODE_24BIT);
#endif
	}
	return 0;
}

int _qspi_erase(u8 *pStart_addr, u32 num_bytes)
{
	int ret = 0;

#ifdef CONFIG_FSL_D4400_QSPI
	/* If starting or ending address is greater than 16MB, 
	   then switch to 32bit addressing. 
	*/
	if (  ((u32)pStart_addr & 0x00ffffff) ||  (((u32)pStart_addr + num_bytes) & 0x00ffffff) )
	{
		QSPI_SET_ADDR_MODE(QSPI_ADDR_MODE_32BIT);
#ifdef CONFIG_QSPI_FLASH_SPANSION 
		QSPI_SPANSION_FLASH_SET_ADDR_MODE(SPANSION_ADDR_MODE_32BIT);
#endif
	}
	else
	{
		QSPI_SET_ADDR_MODE(QSPI_ADDR_MODE_24BIT);
#ifdef CONFIG_QSPI_FLASH_SPANSION 
		QSPI_SPANSION_FLASH_SET_ADDR_MODE(SPANSION_ADDR_MODE_24BIT);
#endif
	}
#endif

	if ( (pStart_addr == 0) && (num_bytes == 0) )
	{
		printf("Bulk erase started...");

#ifdef CONFIG_QSPI_FLASH_SPANSION 
		/* Erase device */
		QSPI_SPANSION_FLASH_BULK_ERASE();
#else
	#error Bulk erase not implemented!
#endif
		printf("done!\n");
	}
	else if (num_bytes > 0)
	{
		/* Erase sector */

		/* Get address of start of sector */
		u32 addr = (u32)pStart_addr & ~(u32)(SPANSION_SECTOR_SIZE_BYTES-1);
		u32 end_addr = (u32)pStart_addr + num_bytes;
		u32 sectors_erased = 0;

		printf("Sector erase started...\n");

		do
		{
			printf("Erasing sector at address = 0x%08x\n", addr);
#ifdef CONFIG_QSPI_FLASH_SPANSION 
			QSPI_SPANSION_FLASH_SECTOR_ERASE((u32)addr);
			addr += SPANSION_SECTOR_SIZE_BYTES;
#else
	#error Sector erase not implemented!
#endif
			++sectors_erased;
		} while(addr <= end_addr);

		printf("Total sectors erased: %i\n", sectors_erased);
	}
	else
	{
		/* Error */
		ret = 1;
	}

	/* Switch back to 24bit mode in case cpu is reset. */
#ifdef CONFIG_FSL_D4400_QSPI
	QSPI_SET_ADDR_MODE(QSPI_ADDR_MODE_24BIT);
#endif
#ifdef CONFIG_QSPI_FLASH_SPANSION 
	QSPI_SPANSION_FLASH_SET_ADDR_MODE(SPANSION_ADDR_MODE_24BIT);
#endif
	return ret;
}

/* _qspi_xfer implements read/write with the following rule: 
 *
 * - For read operations:
 *	dout 	= addr in qspi flash, data source
 *	din 	= addr in system memory, data destination
 *
 * - For write operations:
 *	dout 	= addr in system memory, data source
 *	din 	= addr in qspi flash, data destination
 *
 * - Read operation: flags with QSPI_MODE_RW bit cleared
 * - Write operation: flags with QSPI_MODE_RW bit set
 */
int _qspi_xfer(struct spi_slave *slave, unsigned int num_bytes, const void *dout,
		void *din, unsigned long flags)
{
	int ret = 0;

	if (!slave)
		return -1;

	switch (flags & QSPI_MODE_MASK)
	{
	case QSPI_MODE_READ:
		ret = _qspi_xfer_read((u8 *)dout, (u8 *)din, num_bytes);

		printf("\n\n---------------------------------------------------------\n");
		printf(" Src: 0x%08x  Dest: 0x%08x  Bytes: %i / x%x\n", (u32)dout, (u32)din, num_bytes, num_bytes);
		_qspi_print_buffer(din, num_bytes, 16);

		break;
	case QSPI_MODE_WRITE:
		ret = _qspi_xfer_write((u8 *)dout, (u8 *)din, num_bytes);
		break;
	case QSPI_MODE_ERASE_SECTOR:
		ret = _qspi_erase((u8 *)dout, num_bytes);
		break;
	case QSPI_MODE_ERASE_ALL:
		ret = _qspi_erase((u8 *)NULL, 0);
		break;
	}

	return ret;
}

struct spi_slave *_qspi_setup_slave(unsigned int bus, unsigned int cs,
			unsigned int max_hz, unsigned int mode)
{
	struct qspi_slave *qspi;
	int ret;
	static u8 qspi_cfg_done = 0;

	qspi = spi_alloc_slave(struct qspi_slave, bus, cs);
	if (!qspi) {
		puts("qspi: QSPI Slave not allocated !\n");
		return NULL;
	}

	/* Don't need to do config every single time.  Just do it once.
	   If done for every r/w operation, re-configuring the clocks could
	   cause problems with the single i/o read. 
	*/
	if (qspi_cfg_done == 0) {
		ret = qspi_cfg(qspi, cs, max_hz, mode);
		qspi_cfg_done = 1;
	}

	if (ret) {
		printf("qspi: cannot setup QSPI controller\n");
		free(qspi);
		return NULL;
	}
	return &qspi->slave;
}

void _qspi_free_slave(struct spi_slave *slave)
{
	struct qspi_slave *qspi = to_qspi_slave(slave);
	free(qspi);
}

int _qspi_claim_bus(struct spi_slave *slave)
{
	if (!slave)
		return -1;

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
	int num_spi_devices = ARRAY_SIZE(spi_bases);

#ifdef CONFIG_FSL_D4400_QSPI
	if (bus == (num_spi_devices + 1))
	{
		/* Quad io spi */
		qspi_slave = _qspi_setup_slave(bus, cs, max_hz, mode);
		return qspi_slave;
	}
	else /* (bus <= num_spi_devices) */
#endif
	{
		/* Two wire spi */
		return _spi_setup_slave(bus, cs, max_hz, mode);
	}

}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags)
{
#ifdef CONFIG_FSL_D4400_QSPI
	if (qspi_slave == slave)
		/* bitlen is really number of bytes for qspi */
		return _qspi_xfer(slave, bitlen, dout, din, flags);
	else
#endif
		return _spi_xfer(slave, bitlen, dout, din, flags);
}


void spi_free_slave(struct spi_slave *slave)
{
#ifdef CONFIG_FSL_D4400_QSPI
	if (qspi_slave == slave)
		_qspi_free_slave(slave);
	else
#endif
		_spi_free_slave(slave);
}

int spi_claim_bus(struct spi_slave *slave)
{
#ifdef CONFIG_FSL_D4400_QSPI
	if (qspi_slave == slave)
		return _qspi_claim_bus(slave);
	else
#endif
		return _spi_claim_bus(slave);
}

void spi_release_bus(struct spi_slave *slave)
{
	/* TODO: Shut the controller down */
#ifdef CONFIG_FSL_D4400_QSPI
	if (qspi_slave == slave)
		_qspi_release_bus(slave);
	else
#endif
		_spi_release_bus(slave);
}
