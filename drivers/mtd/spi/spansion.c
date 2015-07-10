/*
 * Copyright (C) 2009-2015 Freescale Semiconductor, Inc.
 *
 * Author: Mingkai Hu (Mingkai.hu@freescale.com)
 * Based on stmicro.c by Wolfgang Denk (wd@denx.de),
 * TsiChung Liew (Tsi-Chung.Liew@freescale.com),
 * and  Jason McMullan (mcmullan@netapp.com)
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <malloc.h>
#include <asm/errno.h>
#include <spi_flash.h>
#include <mtd/spi-nor.h>
#include "spi_flash_internal.h"
#ifdef CONFIG_FSL_D4400_QSPI
#include <asm/arch/imx-regs.h>
#endif


#define SPSN_ID_S25FL008A 0x0213
#define SPSN_ID_S25FL016A 0x0214
#define SPSN_ID_S25FL032A 0x0215
#define SPSN_ID_S25FL064A 0x0216
#define SPSN_ID_S25FL128P 0x2018
#define SPSN_ID_S25FL256S 0x0219
#define SPSN_ID_S25FL512S 0x0220

#define SPSN_EXT_ID_S25FL128P_256KB	0x0300
#define SPSN_EXT_ID_S25FL128P_64KB	0x0301
#define SPSN_EXT_ID_S25FL032A		0x4d00
#define SPSN_EXT_ID_S25FL128P_S		0x4d01
#define SPSN_EXT_ID_S25FL256S		0x4d00

/* x4d is place holder, xx in datasheet */
#define SPSN_EXT_ID_S25FL512S		0x4d00

#define SPANSION_SR_WIP (1 << 0)	/* Write-in-Progress */

struct spansion_spi_flash {
	struct spi_flash flash;
	const struct spansion_spi_flash_params *params;
};

static inline struct spansion_spi_flash
	*to_spansion_spi_flash(struct spi_flash *flash)
{
	return container_of(flash, struct spansion_spi_flash, flash);
}

struct spansion_spi_flash_params {
	u16 idcode1;
	u16 idcode2;
	u16 page_size;
	u16 pages_per_sector;
	u16 nr_sectors;
	const char *name;
};

static const struct spansion_spi_flash_params spansion_spi_flash_table[] = {
	{
		.idcode1 = SPSN_ID_S25FL008A,
		.idcode2 = 0,
		.page_size = 256,
		.pages_per_sector = 256,
		.nr_sectors = 16,
		.name = "S25FL008A",
	},
	{
		.idcode1 = SPSN_ID_S25FL016A,
		.idcode2 = 0,
		.page_size = 256,
		.pages_per_sector = 256,
		.nr_sectors = 32,
		.name = "S25FL016A",
	},
	{
		.idcode1 = SPSN_ID_S25FL032A,
		.idcode2 = 0,
		.page_size = 256,
		.pages_per_sector = 256,
		.nr_sectors = 64,
		.name = "S25FL032A",
	},
	{
		.idcode1 = SPSN_ID_S25FL064A,
		.idcode2 = 0,
		.page_size = 256,
		.pages_per_sector = 256,
		.nr_sectors = 128,
		.name = "S25FL064A",
	},
	{
		.idcode1 = SPSN_ID_S25FL128P,
		.idcode2 = SPSN_EXT_ID_S25FL128P_64KB,
		.page_size = 256,
		.pages_per_sector = 256,
		.nr_sectors = 256,
		.name = "S25FL128P_64K",
	},
	{
		.idcode1 = SPSN_ID_S25FL128P,
		.idcode2 = SPSN_EXT_ID_S25FL128P_256KB,
		.page_size = 256,
		.pages_per_sector = 1024,
		.nr_sectors = 64,
		.name = "S25FL128P_256K",
	},
	{
		.idcode1 = SPSN_ID_S25FL032A,
		.idcode2 = SPSN_EXT_ID_S25FL032A,
		.page_size = 256,
		.pages_per_sector = 256,
		.nr_sectors = 64,
		.name = "S25FL032A",
	},
	{
		.idcode1 = SPSN_ID_S25FL128P,
		.idcode2 = SPSN_EXT_ID_S25FL128P_S,
		.page_size = 256,
		.pages_per_sector = 256,
		.nr_sectors = 256,
		.name = "S25FL128P_64K/S25FL128S",
	},
	{
		.idcode1 = SPSN_ID_S25FL256S,
		.idcode2 = SPSN_EXT_ID_S25FL256S,
		.page_size = 256,
		.pages_per_sector = 1024,
		.nr_sectors = 128,
		.name = "S25FL256S",
	},
	{
		.idcode1 = SPSN_ID_S25FL512S,
		.idcode2 = SPSN_EXT_ID_S25FL512S,
		.page_size = 256,
		.pages_per_sector = 1024,
		.nr_sectors = 256,
		.name = "S25FL512S",
	},
};

static int spansion_wait_ready(struct spi_flash *flash, unsigned long timeout)
{
	struct spi_slave *spi = flash->spi;
	unsigned long timebase;
	int ret;
	u8 status;

	timebase = get_timer(0);
	do {

		ret = spi_flash_cmd(spi, SPINOR_OP_RDSR, &status,
			sizeof(status));
		if (ret)
			return -1;

		if ((status & SPANSION_SR_WIP) == 0)
			break;
	} while (get_timer(timebase) < timeout);

	if ((status & SPANSION_SR_WIP) == 0)
		return 0;

	/* Timed out */
	return -1;
}

static int spansion_set_mode(struct spi_flash *flash)
{
	int ret;
	struct spi_slave *spi = flash->spi;

	/* Writing to CR1 reg requires writing to the SR reg
	 * followed by the CR1 reg.
	 */
	u8 buf[2] = {0, 0}; /* [0]-status reg, [1]-CR1 reg */

	/* Enable write first */
	ret = spi_flash_cmd(spi, SPINOR_OP_WREN, NULL, 0);
	ret = spansion_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
	if (ret)
		return ret;

	if (spi->mode & SPI_QUAD_IO)
		/* Quad I/O mode */
		buf[1] = CR_QUAD_EN_SPAN; /* Quad bit in CR1 reg */

	ret = spi_flash_cmd(spi, SPINOR_OP_WRSR, &buf, 2);
	ret = spansion_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
	if (ret)
		return ret;

	/* Read back and check */
	buf[0] = 0;
	ret = spi_flash_cmd(spi, SPINOR_OP_RDCR, buf, 1);
	if (ret)
		return ret;

	if ((buf[0] & CR_QUAD_EN_SPAN) != buf[1])
		ret = -1;

	return ret;
}

static int spansion_read_fast(struct spi_flash *flash, u32 offset,
	size_t len, void *buf)
{
	struct spansion_spi_flash *spsn = to_spansion_spi_flash(flash);
	struct spi_slave *spi = flash->spi;
	unsigned long page_addr;
	unsigned long page_size;
	u8 cmd[5];
	int ret;

#ifdef CONFIG_FSL_D4400_QSPI
	/* There are multiple sources accessing the flash, cmd_sf, and env_sf.
	 * Each could have set quad io mode differently.  Need to sync with
	 * current settings.
	 */
	ret = spansion_set_mode(flash);
	if (ret) {
		printf("SF: Error in setting I/O mode of Spansion flash\n");
		return ret;
	}
#endif

	page_size = spsn->params->page_size;
	page_addr = offset / page_size;

	/* 4-byte address commands */
	if (spi->mode & SPI_QUAD_IO)
		cmd[0] = SPINOR_OP_READ4_1_1_4;
	else
		cmd[0] = SPINOR_OP_READ4;
	cmd[1] = page_addr >> 16;
	cmd[2] = page_addr >> 8;
	cmd[3] = page_addr;
	cmd[4] = offset % page_size;
	debug
		("READ: 0x%x => cmd = { 0x%02x 0x%02x%02x%02x%02x } len = 0x%x\n",
		 offset, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], len);
	return spi_flash_read_common(flash, cmd, sizeof(cmd), buf, len);
}

static int spansion_write(struct spi_flash *flash, u32 offset,
	size_t len, const void *buf)
{
	struct spi_slave *spi = flash->spi;
	unsigned long page_size;
	size_t chunk_len;
	size_t actual;
	int ret;
	u8 cmd[5];
	u32 addr;
	u32 next_page_start_addr;
	u32 end_addr;

#ifdef CONFIG_FSL_D4400_QSPI
	/* There are multiple sources accessing the flash, cmd_sf, and env_sf.
	 * Each could have set quad io mode differently.  Need to sync with
	 * current settings.
	 */
	ret = spansion_set_mode(flash);
	if (ret) {
		printf("SF: Error in setting I/O mode of Spansion flash\n");
		return ret;
	}
#endif

	page_size = flash->page_size;
	addr = offset;
	ret = spi_claim_bus(flash->spi);

	if (ret) {
		debug("SF: Unable to claim SPI bus\n");
		return ret;
	}

	ret = 0;
	for (actual = 0; actual < len; actual += chunk_len) {

		chunk_len = min(len - actual, page_size);

		/* Page programming requires that we do not cross page
		 * boundaries.  Limit the number of bytes so that we
		 * do not exceed the boundary.
		 */
		next_page_start_addr = ((u32)addr + page_size)
			& ~(u32)(page_size-1);
		end_addr = (u32)addr + (u32)chunk_len;

		if  (end_addr > next_page_start_addr)
			chunk_len -= (end_addr - next_page_start_addr);

		/* 4-byte address commands */
		if (spi->mode & SPI_QUAD_IO)
			cmd[0] = SPINOR_OP_PP_4B_1_1_4;
		else
			cmd[0] = SPINOR_OP_PP_4B;

		cmd[1] = (addr >> 24) & 0xff;
		cmd[2] = (addr >> 16) & 0xff;
		cmd[3] = (addr >> 8) & 0xff;
		cmd[4] = addr & 0xff;

		debug
			("PP: 0x%p => cmd = { 0x%02x 0x%02x%02x%02x } chunk_len = %d\n",
				buf + actual, cmd[0], cmd[1], cmd[2], cmd[3],
				chunk_len);

		ret = spi_flash_cmd(spi, SPINOR_OP_WREN, NULL, 0);
		ret = spansion_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
		if (ret < 0) {
			debug("SF: Enabling Write failed\n");
			break;
		}

		ret = spi_flash_cmd_write(flash->spi, cmd, sizeof(cmd),
			buf + actual, chunk_len);
		if (ret < 0) {
			debug("SF: SPANSION Page Program failed\n");
			break;
		}

		ret = spansion_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
		if (ret < 0) {
			debug("SF: SPANSION page programming timed out\n");
			break;
		}
		addr += chunk_len;
	}

	debug("SF: SPANSION: Successfully programmed %u bytes @ 0x%x\n",
		len, offset);

	spi_release_bus(flash->spi);

	return ret;
}

int spansion_erase(struct spi_flash *flash, u32 offset, size_t len)
{
	struct spansion_spi_flash *spsn = to_spansion_spi_flash(flash);
	unsigned long sector_size;
	size_t actual;
	int ret;
	u8 cmd[5];
	int numsec, sec;

#ifdef CONFIG_FSL_D4400_QSPI
	/* There are multiple sources accessing the flash, cmd_sf, and env_sf.
	 * Each could have set quad io mode differently.  Need to sync with
	 * current settings.
	 */
	ret = spansion_set_mode(flash);
	if (ret) {
		printf("SF: Error in setting I/O mode of Spansion flash\n");
		return ret;
	}
#endif

	/*
	 * This function currently uses sector erase only.
	 * probably speed things up by using bulk erase
	 * when possible.
	*/
	sector_size =
		spsn->params->page_size * spsn->params->pages_per_sector;

	if ((offset % sector_size) || (len % sector_size)) {
		printf("SF: Erase offset/len not multiple of sector size\n");
		return -1;
	}

	numsec = len / sector_size;

	/* 4-byte address commands */
	cmd[0] = SPINOR_OP_SE_4B;
	cmd[3] = 0x00;
	cmd[4] = 0x00;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		debug("SF: Unable to claim SPI bus\n");
		return ret;
	}

	ret = 0;
	for (actual = 0; actual < numsec; actual++) {
		sec = (offset / sector_size) + actual;
		/* Sector number to erase, 16-bit */
		cmd[1] = sec >> 8;
		cmd[2] = sec & 0xff;

		ret = spi_flash_cmd(flash->spi, SPINOR_OP_WREN, NULL, 0);
		ret = spansion_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
		if (ret < 0) {
			debug("SF: Enabling Write failed\n");
			break;
		}

		ret = spi_flash_cmd_write(flash->spi, cmd, 4, NULL, 0);
		if (ret < 0) {
			debug("SF: SPANSION page erase failed\n");
			break;
		}

		/* Up to 2 seconds */
		ret = spansion_wait_ready(flash, SPI_FLASH_PAGE_ERASE_TIMEOUT);
		if (ret < 0) {
			debug("SF: SPANSION page erase timed out\n");
			break;
		}
	}

	debug("SF: SPANSION: Successfully erased %u bytes @ 0x%x\n",
		(unsigned int)(numsec * sector_size), offset);

	spi_release_bus(flash->spi);
	return ret;
}

struct spi_flash *spi_flash_probe_spansion(struct spi_slave *spi, u8 *idcode)
{
	const struct spansion_spi_flash_params *params;
	struct spansion_spi_flash *spsn;
	unsigned int i;
	unsigned short jedec, ext_jedec;
	int ret;

	jedec = idcode[1] << 8 | idcode[2];
	ext_jedec = idcode[3] << 8 | idcode[4];

	for (i = 0; i < ARRAY_SIZE(spansion_spi_flash_table); i++) {
		params = &spansion_spi_flash_table[i];
		if (params->idcode1 == jedec) {
			if (params->idcode2 == ext_jedec)
				break;
		}
	}

	if (i == ARRAY_SIZE(spansion_spi_flash_table)) {
		debug("SF: Unsupported SPANSION ID %04x %04x\n",
			jedec, ext_jedec);
		return NULL;
	}

	spsn = malloc(sizeof(struct spansion_spi_flash));
	if (!spsn) {
		debug("SF: Failed to allocate memory\n");
		return NULL;
	}

	spsn->params = params;
	spsn->flash.spi = spi;
	spsn->flash.name = params->name;
	spsn->flash.write = spansion_write;
	spsn->flash.erase = spansion_erase;
	spsn->flash.read = spansion_read_fast;
	spsn->flash.sector_size = params->page_size * params->pages_per_sector;
	spsn->flash.size = params->page_size * params->pages_per_sector
	    * params->nr_sectors;
	spsn->flash.page_size = params->page_size; /* Write size */
#ifdef CONFIG_FSL_D4400_QSPI
	/* Cap program size to tx fifo or sector size, whichever is less. */
	if (spsn->flash.page_size > QSPI_TX_FIFO_SIZE)
		spsn->flash.page_size = QSPI_TX_FIFO_SIZE;
#endif
	spsn->flash.memory_map = 0; /* Not a memory mapped device */

	debug("SF: Detected %s with page size %u, total %u bytes, speed %i Hz\n",
		params->name, params->page_size, spsn->flash.size,
		spi->speed_hz);

	/* Set IO mode */
	ret = spansion_set_mode(&spsn->flash);
	if (ret) {
		printf("SF: Error in setting I/O mode of Spansion flash\n");
		goto out_err;
	}
	return &spsn->flash;

out_err:
	if (!spsn)
		free(spsn);
	return NULL;
}
