/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
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

#ifndef _D4400IMAGE_H_
#define _D4400IMAGE_H_

#include <configs/d4400_evb.h>
/* Number of DCD entries is limited to 200 */
#define MAX_DCD_ENTRY 200

#define DCD_BARKER	0x45832901
#define APP_CODE_BARKER	0x01020304

#define CMD_DATA_STR	"DATA"
#define IVT_OFFSET_UNDEFINED	0xFFFFFFFF
#define IVT_OFFSET_NOR	0x1000
#define IVT_OFFSET_IPC	0x0
#define IVT_OFFSET_SDP	0x0

enum d4400_image_cmd {
	CMD_INVALID,
	CMD_IMAGE_VERSION,
	CMD_BOOT_FROM,
	CMD_DATA
};

enum d4400_image_fld_types {
	CFG_INVALID = -1,
	CFG_COMMAND,
	CFG_REG_SIZE,
	CFG_REG_ADDRESS,
	CFG_REG_VALUE
};

enum d4400_image_version {
	D4400_IMAGE_VER_INVALID = -1,
	D4400_IMAGE = 1
};

struct dcd_addr_data {
	uint32_t type; /* Type of pointer (byte, halfword, word, wait/read) */
	uint32_t addr; /* Address to write to */
	uint32_t value; /* Data to write */
};

struct dcd_preamble {
	uint32_t barker; /* Barker for sanity check */
	uint32_t length; /* Device configuration length (without preamble) */
};

struct dcd_block {
	struct dcd_preamble preamble;
	struct dcd_addr_data addr_data[MAX_DCD_ENTRY];
};

struct ivt_table {
	uint32_t app_code_barker;
	uint32_t entry;
	uint32_t reserved1;
	uint32_t dcd_ptr;
	uint32_t boot_data_ptr;
	uint32_t self;
	uint32_t unified_image;
	uint32_t secondary_image;
};

struct boot_data {
	uint32_t start;
	uint32_t size;
};

struct boot_header {
	struct ivt_table ivt;
	struct boot_data boot_data;
	struct dcd_block dcd_table;
};

struct d4400_header {
	struct boot_header hdr;
	uint32_t flash_offset;
};

#endif /* _D4400IMAGE_H_ */
