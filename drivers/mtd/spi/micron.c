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

/* Id */
#define MICRON_ID_MT25L01G_64KB		0xba21

/* Extended id */
#define MICRON_EXT_ID_MT25L01G		0x1044

#define MICRON_SR_WIP		(1 << 0) /* Write-in-Progress */
#define MICRON_FSR_EXTADDR	(1 << 0) /* Extended address mode */

struct micron_spi_flash {
	struct spi_flash flash;
	const struct micron_spi_flash_params *params;
};

static inline struct micron_spi_flash
	*to_micron_spi_flash(struct spi_flash *flash)
{
	return container_of(flash, struct micron_spi_flash, flash);
}

struct micron_spi_flash_params {
	u16 idcode1;
	u16 idcode2;
	u16 page_size;
	u16 pages_per_sector;
	u16 nr_sectors;
	const char *name;
};

static const struct micron_spi_flash_params micron_spi_flash_table[] = {
	{
		.idcode1 = MICRON_ID_MT25L01G_64KB,
		.idcode2 = MICRON_EXT_ID_MT25L01G,
		.page_size = 256,
		.pages_per_sector = 256,
		.nr_sectors = 2048,
		.name = "MT25QL01G",
	},
};

static int micron_wait_ready(struct spi_flash *flash, unsigned long timeout)
{
	struct spi_slave *spi = flash->spi;
	unsigned long timebase;
	int ret;
	u8 status;

	timebase = get_timer(0);
	do {

		ret = spi_flash_cmd(spi, SPINOR_OP_RDSR, &status,
			sizeof(status));
		if (ret) {
			return -1;
		}

		if ((status & MICRON_SR_WIP) == 0)
			break;
	} while (get_timer(timebase) < timeout);

	if ((status & MICRON_SR_WIP) == 0)
		return 0;

	/* Timed out */
	return -1;
}


#if (defined D4400_QSPI_NUMONYX_BUG_WORKAROUND)
static int micron_set_extaddr(struct spi_flash *flash, int addrmode)
{
	int ret;
	struct spi_slave *spi = flash->spi;
	u8 buf[2] = {0, 0};

	if (addrmode)
		/* Enter extended 4-byte addr mode */
		ret = spi_flash_cmd(spi, SPINOR_OP_EN4B, NULL, 0);
	else
		/* Exit extended addr mode, 3-byte addr */
		ret = spi_flash_cmd(spi, SPINOR_OP_EX4B, NULL, 0);
	ret = micron_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
	if (ret)
		return ret;

	/* Read flag status to verify address mode */
	buf[0] = buf[1] = 0;
	ret = spi_flash_cmd(spi, SPINOR_OP_RDFSR, buf, 1);
	if (ret)
		return ret;

	/* Bit set for ext/4-byte  addr, bit cleared for 3-byte addr mode */
	buf[0] &= MICRON_FSR_EXTADDR;

	if ((addrmode) && (!buf[0]))
		ret = -1; /* Bit is cleared, error */
	else if ((!addrmode) && (buf[0]))
		ret = -1; /* Bit is set, error */

	return ret;
}
#endif

static int micron_set_mode(struct spi_flash *flash)
{
	int ret;
	struct spi_slave *spi = flash->spi;

	if (spi->mode & SPI_QUAD_IO) {

		// TODO: Quad mode is not yet functional.
		printf("WARNING: Qspi quad mode not implemented, default to single I/O.\n");
		spi->mode = 0;
		return 0;
	}
	return ret;
}

static int micron_read_fast(struct spi_flash *flash, u32 offset,
	size_t len, void *buf)
{
	struct micron_spi_flash *micron = to_micron_spi_flash(flash);
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
	ret = micron_set_mode(flash);
	if (ret) {
		printf("SF: Error in setting I/O mode of Spansion flash\n");
		return ret;
	}
#endif

	page_size = micron->params->page_size;
	page_addr = offset / page_size;

	/* 4-byte address commands */
	if (spi->mode & SPI_QUAD_IO) {
#ifdef D4400_QSPI_QUAD_NOT_READY
		printf("ERROR: Qspi READ quad I/O not implemented.\n");
		return -1;
#else
		cmd[0] = SPINOR_OP_READ4_1_1_4;  /* ? TBD */
#endif
	} else {
#if (defined D4400_QSPI_NUMONYX_BUG_USE_SPANSION)
		/* This command sends 4-byte address regardless. */
		cmd[0] = SPINOR_OP_READ4;
#else
		/* Standard page program command.  Number of address bytes
		 * is dependent on qspi module and flash device setup.
		 */
		cmd[0] = SPINOR_OP_READ;
#endif
	}

	cmd[1] = page_addr >> 16;
	cmd[2] = page_addr >> 8;
	cmd[3] = page_addr;
	cmd[4] = offset % page_size;
	debug
		("READ: 0x%x => cmd = { 0x%02x 0x%02x%02x%02x%02x } len = 0x%x\n",
		 offset, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], len);
	return spi_flash_read_common(flash, cmd, sizeof(cmd), buf, len);
}

static int micron_write(struct spi_flash *flash, u32 offset,
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
	ret = micron_set_mode(flash);
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

		/* Page program commands */
		if (spi->mode & SPI_QUAD_IO) {
#ifdef D4400_QSPI_QUAD_NOT_READY
			printf("ERROR: Qspi WRITE quad I/O not implemented.\n");
			return -1;
#else
			cmd[0] = SPINOR_OP_PP_4B_1_2_4; /* ? TBD */
#endif
		} else {
#if (defined D4400_QSPI_NUMONYX_BUG_USE_SPANSION)
			/* This command sends 4-byte address regardless. */
			cmd[0] = SPINOR_OP_PP_4B;
#else
			/* Standard page program command.  Number of address
			 * bytes is dependent on qspi module and flash device
			 * setup.
			 */
			cmd[0] = SPINOR_OP_PP;
#endif
		}

		cmd[1] = (addr >> 24) & 0xff;
		cmd[2] = (addr >> 16) & 0xff;
		cmd[3] = (addr >> 8) & 0xff;
		cmd[4] = addr & 0xff;

		debug
			("PP: 0x%p => cmd = { 0x%02x 0x%02x%02x%02x } chunk_len = %d\n",
				buf + actual, cmd[0], cmd[1], cmd[2], cmd[3],
				chunk_len);

		ret = spi_flash_cmd(spi, SPINOR_OP_WREN, NULL, 0);
		ret = micron_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
		if (ret < 0) {
			debug("SF: Enabling Write failed\n");
			break;
		}

		ret = spi_flash_cmd_write(flash->spi, cmd, sizeof(cmd),
			buf + actual, chunk_len);
		if (ret < 0) {
			debug("SF: MICRON Page Program failed\n");
			break;
		}

		ret = micron_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
		if (ret < 0) {
			debug("SF: MICRON page programming timed out\n");
			break;
		}
		addr += chunk_len;
	}

	debug("SF: MICRON: Successfully programmed %u bytes @ 0x%x\n",
		len, offset);

	spi_release_bus(flash->spi);

	return ret;
}

int micron_erase(struct spi_flash *flash, u32 offset, size_t len)
{
	struct micron_spi_flash *micron = to_micron_spi_flash(flash);
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
	ret = micron_set_mode(flash);
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
		micron->params->page_size * micron->params->pages_per_sector;

	if ((offset % sector_size) || (len % sector_size)) {
		printf("SF: Erase offset/len not multiple of sector size\n");
		return -1;
	}

	numsec = len / sector_size;

	/* Sector/block erase commands. */
#if (defined D4400_QSPI_NUMONYX_BUG_USE_SPANSION)
	/* This command sends 4-byte address regardless */
	cmd[0] = SPINOR_OP_SE_4B;
#else
	/* Standard sector erase command.  Number of address bytes is
	 * dependent on how qspi module and flash device is setup.
	 */
	cmd[0] = SPINOR_OP_SE;
#endif

	/* Lower 16-bit address is zero because sector is 64KB or larger */
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

		/* Sector number to erase, upper 16-bit address */
		cmd[1] = sec >> 8;
		cmd[2] = sec & 0xff;

		ret = spi_flash_cmd(flash->spi, SPINOR_OP_WREN, NULL, 0);
		ret = micron_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
		if (ret < 0) {
			debug("SF: Enabling Write failed\n");
			break;
		}

		ret = spi_flash_cmd_write(flash->spi, cmd, 4, NULL, 0);
		if (ret < 0) {
			debug("SF: MICRON page erase failed\n");
			break;
		}

		/* Up to 2 seconds */
		ret = micron_wait_ready(flash, SPI_FLASH_PAGE_ERASE_TIMEOUT);
		if (ret < 0) {
			debug("SF: MICRON page erase timed out\n");
			break;
		}
	}

	debug("SF: MICRON: Successfully erased %u bytes @ 0x%x\n",
		(unsigned int)(numsec * sector_size), offset);

	spi_release_bus(flash->spi);
	return ret;
}

struct spi_flash *spi_flash_probe_micron(struct spi_slave *spi, u8 *idcode)
{
	const struct micron_spi_flash_params *params;
	struct micron_spi_flash *micron;
	unsigned int i;
	unsigned short jedec, ext_jedec;
	int ret;

	jedec = idcode[1] << 8 | idcode[2];
	ext_jedec = idcode[3] << 8 | idcode[4];

	for (i = 0; i < ARRAY_SIZE(micron_spi_flash_table); i++) {
		params = &micron_spi_flash_table[i];
		if (params->idcode1 == jedec) {
			if (params->idcode2 == ext_jedec)
				break;
		}
	}

	if (i == ARRAY_SIZE(micron_spi_flash_table)) {
		debug("SF: Unsupported MICRON ID %04x %04x\n",
			jedec, ext_jedec);
		return NULL;
	}
	micron = malloc(sizeof(struct micron_spi_flash));
	if (!micron) {
		debug("SF: Failed to allocate memory\n");
		return NULL;
	}

	micron->params = params;
	micron->flash.spi = spi;
	micron->flash.name = params->name;
	micron->flash.write = micron_write;
	micron->flash.erase = micron_erase;
	micron->flash.read = micron_read_fast;
	micron->flash.sector_size = params->page_size *
		params->pages_per_sector;
	micron->flash.size = params->page_size * params->pages_per_sector
	    * params->nr_sectors;
	micron->flash.page_size = params->page_size; /* Write size */
#ifdef CONFIG_FSL_D4400_QSPI
	/* Cap program size to tx fifo or sector size, whichever is less. */
	if (micron->flash.page_size > QSPI_TX_FIFO_SIZE)
		micron->flash.page_size = QSPI_TX_FIFO_SIZE;
#endif
	micron->flash.memory_map = 0; /* Not a memory mapped device */

	debug("SF: Detected %s with page size %u, total %u bytes, speed %i Hz\n",
		params->name, params->page_size, micron->flash.size,
		spi->speed_hz);

#if (defined D4400_QSPI_NUMONYX_BUG_WORKAROUND)
	/* Set address mode if qspi is in Numonyx mode. If using Spansion mode
	 * as workaround, then DON'T need to set address mode as we will use
	 * 4-byte address specific commands.
	 */

	/* Set extended 4-byte address mode */
	micron_set_extaddr(&micron->flash, 1);
#endif

	/* Set IO mode */
	ret = micron_set_mode(&micron->flash);
	if (ret) {
		printf("SF: Error in setting I/O mode of Micron flash\n");
		goto out_err;
	}
	return &micron->flash;

out_err:
	if (!micron)
		free(micron);
	return NULL;
}
