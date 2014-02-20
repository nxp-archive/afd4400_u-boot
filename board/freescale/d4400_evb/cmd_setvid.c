/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <configs/d4400_evb.h>

#if defined(CONFIG_ZL6105_VID)

#include <common.h>
#include <command.h>

int setup_vid(unsigned short);
static int atoi (const char *s);
void dump_zl6105_reg(void);

static int do_setvid(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc < 2)
		return CMD_RET_USAGE;

	int val = atoi(argv[1]);
	return setup_vid(val);
}

static int atoi (const char *s)
{
	bool negative;
	int value;

	/* Parse sign. */
	negative = false;
	if (*s == '+')
		s++;
	else if (*s == '-')
	{
		negative = true;
		s++;
	}

	/* Parse digits.  We always initially parse the value as
	   negative, and then make it positive later, because the
	   negative range of an int is bigger than the positive range
	   on a 2's complement system. */
	for (value = 0; (*s >= '0' && *s <= '9'); s++)
		value = value * 10 - (*s - '0');
	if (!negative)
		value = -value;

	return value;
}

static int do_getvidreg(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc > 1)
		return CMD_RET_USAGE;

	dump_zl6105_reg();

	return 0;
}

U_BOOT_CMD(setvid,    2,      1,  do_setvid,
	"Set VID for D4400 EVB",
	"[vid value 0..31]");

U_BOOT_CMD(getvidreg,    1,   1,  do_getvidreg,
	"Get VID for D4400 EVB",
	"");

#endif
