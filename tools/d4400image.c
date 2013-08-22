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

/* Required to obtain the getline prototype from stdio.h */
#define _GNU_SOURCE

#include "mkimage.h"
#include <image.h>
#include "d4400image.h"

/*
 * Supported commands for configuration file
 */
static table_entry_t d4400_image_cmds[] = {
	{CMD_BOOT_FROM,         "BOOT_FROM",            "boot command",	  },
	{CMD_DATA,              "DATA",                 "Reg Write Data", },
	{-1,                    "",                     "",	          },
};

/*
 * Supported Boot options for configuration file
 * this is needed to set the correct flash offset
 */
static table_entry_t d4400_image_bootops[] = {
	{IVT_OFFSET_NOR,	"nor",	"Parallel NOR Flash via weim", },
	{IVT_OFFSET_IPC,	"eth",	"IPC via ethernet", },
	{IVT_OFFSET_SDP,	"sdp",	"SDP via uart", },
	{-1,			"",	"Invalid", },
};
/*
 * D4400IMAGE version definition for D4400 chips
 */
static table_entry_t d4400_image_versions[] = {
	{D4400_IMAGE,   "",     " (d4400 compatible)", },
	{-1,           "",     " (Invalid)",	},
};

static struct d4400_header d4400_image_hdr;
static uint32_t d4400_image_version;
static void set_dcd_val(struct d4400_header *d4400_hdr,
			char *name, int lineno,
			int fld, uint32_t value,
			uint32_t off);

static void set_dcd_rst(struct d4400_header *d4400_hdr,
			uint32_t dcd_len,
			char *name, int lineno);

static void set_d4400_hdr(struct d4400_header *d4400_hdr,
				uint32_t dcd_len,
				struct stat *sbuf,
				struct mkimage_params *params);

static uint32_t get_cfg_value(char *token, char *name,  int linenr)
{
	char *endptr;
	uint32_t value;

	errno = 0;
	value = strtoul(token, &endptr, 16);
	if (errno || (token == endptr)) {
		fprintf(stderr, "Error: %s[%d] - Invalid hex data(%s)\n",
			name,  linenr, token);
		exit(EXIT_FAILURE);
	}
	return value;
}

static uint32_t detect_d4400_image_version(struct d4400_header *d4400_hdr)
{
	struct boot_header *bhdr = &d4400_hdr->hdr;
	struct ivt_table *ivthdr = &bhdr->ivt;

	if ((ivthdr->app_code_barker == APP_CODE_BARKER) &&
	    (bhdr->dcd_table.preamble.barker == DCD_BARKER))
		return D4400_IMAGE;

	return D4400_IMAGE_VER_INVALID;
}

static void err_d4400_image_version(int version)
{
	fprintf(stderr,
		"Error: Unsupported d4400 image version:%d\n", version);

	exit(EXIT_FAILURE);
}

static void set_dcd_val(struct d4400_header *d4400_hdr, char *name, int lineno,
					int fld, uint32_t value, uint32_t off)
{
	struct dcd_block *dcd = &d4400_hdr->hdr.dcd_table;

	switch (fld) {
	case CFG_REG_SIZE:
		/* Byte, halfword, word */
		if ((value != 1) && (value != 2) && (value != 4)) {
			fprintf(stderr,
				"Error: %s[%d] - Invalid register size " "(%d)\n",
				name, lineno, value);
			exit(EXIT_FAILURE);
		}
		dcd->addr_data[off].type = value;
		break;
	case CFG_REG_ADDRESS:
		dcd->addr_data[off].addr = value;
		break;
	case CFG_REG_VALUE:
		dcd->addr_data[off].value = value;
		break;
	default:
		break;
	}
}

/*
 * Complete setting up the rest field of DCD
 * such as barker code and DCD data length.
 */
static void set_dcd_rst(struct d4400_header *d4400_hdr, uint32_t dcd_len,
						char *name, int lineno)
{
	struct dcd_block *dcd = &d4400_hdr->hdr.dcd_table;

	if (dcd_len > MAX_DCD_ENTRY) {
		fprintf(stderr,
			"Error: %s[%d] -DCD table exceeds maximum size(%d)\n",
			name, lineno, MAX_DCD_ENTRY);
		exit(EXIT_FAILURE);
	}

	dcd->preamble.barker = DCD_BARKER;
	dcd->preamble.length = dcd_len *
				sizeof(struct dcd_addr_data);
}

static void set_d4400_hdr(struct d4400_header *d4400_hdr, uint32_t dcd_len,
					struct stat *sbuf,
					struct mkimage_params *params)
{
	struct boot_header *bhdr = &d4400_hdr->hdr;
	struct ivt_table *ivthdr = &bhdr->ivt;

	/* Exit if there is no BOOT_FROM field specifying the ivt_offset */
	if (d4400_hdr->flash_offset == IVT_OFFSET_UNDEFINED) {
		fprintf(stderr, "Error: Header: No BOOT_FROM tag in %s\n",
			params->imagename);
		exit(EXIT_FAILURE);
	}

	/* Set magic number */
	ivthdr->app_code_barker = APP_CODE_BARKER;

	ivthdr->entry = params->ep;
	ivthdr->reserved1 = 0;
	ivthdr->self = params->ep - sizeof(struct d4400_header);

	ivthdr->dcd_ptr = ivthdr->self +
		offsetof(struct boot_header, dcd_table);

	ivthdr->boot_data_ptr = ivthdr->self +
		offsetof(struct boot_header, boot_data);

	bhdr->boot_data.start = ivthdr->self - d4400_hdr->flash_offset;
	bhdr->boot_data.size = sbuf->st_size +
		d4400_hdr->flash_offset +
		sizeof(struct d4400_header);
#ifdef CONFIG_D4400_UBOOT_VALIDATION_SHA256
	ivthdr->unified_image = ivthdr->self + sbuf->st_size;
#else
	ivthdr->unified_image = 0;
#endif
	ivthdr->secondary_image = 0;
}

static void print_hdr(struct d4400_header *d4400_hdr)
{
	struct boot_header *bhdr = &d4400_hdr->hdr;
	struct ivt_table *ivthdr = &bhdr->ivt;
	struct dcd_block *dcd = &bhdr->dcd_table;
	uint32_t size, ver;

	size = dcd->preamble.length;
	if (size > (MAX_DCD_ENTRY * sizeof(struct dcd_addr_data))) {
		fprintf(stderr,
			"Error: Image corrupt DCD size %d exceed maximum %d\n",
			(uint32_t)(size / sizeof(struct dcd_addr_data)),
			MAX_DCD_ENTRY);
		exit(EXIT_FAILURE);
	}
	ver = detect_d4400_image_version(d4400_hdr);

	printf("Image Type:   Freescale D4400 Boot Image\n");
	printf("Image Ver:    %x", ver);
	printf("%s\n", get_table_entry_name(d4400_image_versions, NULL, ver));
	printf("Data Size:    ");
	genimg_print_size(bhdr->boot_data.size);
	printf("Load Address: %08x\n", (uint32_t)ivthdr->boot_data_ptr);
	printf("Entry Point:  %08x\n", (uint32_t)ivthdr->entry);
}

static void parse_cfg_cmd(struct d4400_header *d4400_hdr, int32_t cmd,
		char *token, char *name, int lineno, int fld, int dcd_len)
{
	int value;
	static int cmd_ver_first = ~0;

	switch (cmd) {
	case CMD_IMAGE_VERSION:
		d4400_image_version = get_cfg_value(token, name, lineno);
		if (cmd_ver_first == 0) {
			fprintf(stderr, "Error: %s[%d] - IMAGE_VERSION command need be the first before other valid command in the file\n",
				name, lineno);
			exit(EXIT_FAILURE);
		}
		cmd_ver_first = 1;
		break;
	case CMD_BOOT_FROM:
		d4400_hdr->flash_offset = get_table_entry_id
			(d4400_image_bootops, "d4400image boot option", token);
		if (d4400_hdr->flash_offset == -1) {
			fprintf(stderr,
				"Error: %s[%d] -Invalid boot device (%s)\n",
				name, lineno, token);
				exit(EXIT_FAILURE);
		}
		if (unlikely(cmd_ver_first != 1))
			cmd_ver_first = 0;
		break;
	case CMD_DATA:
		value = get_cfg_value(token, name, lineno);
		set_dcd_val(d4400_hdr, name, lineno, fld, value, dcd_len);
		if (unlikely(cmd_ver_first != 1))
			cmd_ver_first = 0;
		break;
	}
}

static void parse_cfg_fld(struct d4400_header *d4400_hdr, int32_t *cmd,
		char *token, char *name, int lineno, int fld, int *dcd_len)
{
	int value;

	switch (fld) {
	case CFG_COMMAND:
		*cmd = get_table_entry_id(d4400_image_cmds,
				"d4400image commands", token);
		if (*cmd < 0) {
			fprintf(stderr,
				"Error: %s[%d] - Invalid command(%s)\n",
				name, lineno, token);
			exit(EXIT_FAILURE);
		}
		break;
	case CFG_REG_SIZE:
		parse_cfg_cmd(d4400_hdr, *cmd, token, name, lineno, fld,
			      *dcd_len);
		break;
	case CFG_REG_ADDRESS:
	case CFG_REG_VALUE:
		if (*cmd != CMD_DATA)
			return;

		value = get_cfg_value(token, name, lineno);
		set_dcd_val(d4400_hdr, name, lineno, fld, value, *dcd_len);

		if (fld == CFG_REG_VALUE)
			(*dcd_len)++;
		break;
	default:
		break;
	}
}
static uint32_t parse_cfg_file(struct d4400_header *d4400_hdr, char *name)
{
	FILE *fd = NULL;
	char *line = NULL;
	char *token, *saveptr1, *saveptr2;
	int lineno = 0;
	int fld;
	size_t len;
	int dcd_len = 0;
	int32_t cmd;

	fd = fopen(name, "r");
	if (fd == 0) {
		fprintf(stderr, "Error: %s - Can't open DCD file\n", name);
		exit(EXIT_FAILURE);
	}

	/* Very simple parsing, line starting with # are comments
	 * and are dropped
	 */
	while ((getline(&line, &len, fd)) > 0) {
		lineno++;

		token = strtok_r(line, "\r\n", &saveptr1);
		if (token == NULL)
			continue;

		/* Check inside the single line */
		for (fld = CFG_COMMAND, cmd = CMD_INVALID,
				line = token;; line = NULL, fld++) {
			token = strtok_r(line, " \t", &saveptr2);
			if (token == NULL)
				break;

			/* Drop all text starting with '#' as comments */
			if (token[0] == '#')
				break;

			parse_cfg_fld(d4400_hdr, &cmd, token, name,
				      lineno, fld, &dcd_len);
		}
	}

	set_dcd_rst(d4400_hdr, dcd_len, name, lineno);
	fclose(fd);

	return dcd_len;
}

static int d4400_image_check_image_types(uint8_t type)
{
	if (type == IH_TYPE_D4400IMAGE)
		return EXIT_SUCCESS;
	else
		return EXIT_FAILURE;
}

static int d4400_image_verify_header(unsigned char *ptr, int image_size,
			struct mkimage_params *params)
{
	struct d4400_header *d4400_hdr = (struct d4400_header *)ptr;

	if (detect_d4400_image_version(d4400_hdr) == D4400_IMAGE_VER_INVALID)
		return -FDT_ERR_BADSTRUCTURE;

	return 0;
}

static void d4400_image_print_header(const void *ptr)
{
	struct d4400_header *d4400_hdr = (struct d4400_header *)ptr;
	uint32_t version = detect_d4400_image_version(d4400_hdr);

	switch (version) {
	case D4400_IMAGE:
		print_hdr(d4400_hdr);
		break;
	default:
		err_d4400_image_version(version);
		break;
	}
}

static void d4400_image_set_header(void *ptr, struct stat *sbuf, int ifd,
				struct mkimage_params *params)
{
	struct d4400_header *d4400_hdr = (struct d4400_header *)ptr;
	uint32_t dcd_len;

	d4400_image_version = D4400_IMAGE;
	d4400_hdr->flash_offset = IVT_OFFSET_UNDEFINED;

	/* Parse dcd configuration file */
	dcd_len = parse_cfg_file(d4400_hdr, params->imagename);

	/* Set the d4400 header */
	set_d4400_hdr(d4400_hdr, dcd_len, sbuf, params);
}

int d4400_image_check_params(struct mkimage_params *params)
{
	if (!params)
		return CFG_INVALID;
	if (!strlen(params->imagename)) {
		fprintf(stderr, "Error: %s - Configuration file not specified, it is needed for d4400 image generation\n",
			params->cmdname);
		return CFG_INVALID;
	}
	/*
	 * Check parameters:
	 * XIP is not allowed and verify that incompatible
	 * parameters are not sent at the same time
	 * For example, if list is required a data image must not be provided
	 */
	return	(params->dflag && (params->fflag || params->lflag)) ||
		(params->fflag && (params->dflag || params->lflag)) ||
		(params->lflag && (params->dflag || params->fflag)) ||
		(params->xflag) || !(strlen(params->imagename));
}

/*
 * d4400image parameters
 */
static struct image_type_params d4400_image_params = {
	.name		= "Freescale D4400 Boot Image support",
	.header_size	= sizeof(struct d4400_header),
	.hdr		= (void *)&d4400_image_hdr,
	.check_image_type = d4400_image_check_image_types,
	.verify_header	= d4400_image_verify_header,
	.print_header	= d4400_image_print_header,
	.set_header	= d4400_image_set_header,
	.check_params	= d4400_image_check_params,
};

void init_d4400_image_type(void)
{
	mkimage_register(&d4400_image_params);
}
