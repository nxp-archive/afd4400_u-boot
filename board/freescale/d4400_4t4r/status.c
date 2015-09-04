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

#include <common.h>
#include <command.h>
#include <i2c.h>
#include <configs/d4400_4t4r.h>

#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/d4400_pins.h>
#include <asm/arch/ccm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/arch/iomux.h>
#include <miiphy.h>
#include <netdev.h>

#include "d4400_4t4r_priv.h"

//#define DEBUG

// Zilker PMBUS defines
#define LINEAR_MODE           (0x00)
#define VOUT_MODE             (0x20)
#define READ_VOUT             (0x8B)
#define USER_CONFIG           (0xD1)

enum i2c_bus {
	I2C1 = 0, /* Bus 0: SFP1 */
/*	I2C2 = 0, - Not used */
	I2C3 = 1, /* Bus 1: SFP 3 */
/*	I2C4 = 0, - Not used */
/*	I2C5 = 0, - Not used */
	I2C6 = 2, /* Bus 2: PA path 1/2 i2c */
	I2C7 = 3, /* Bus 3: PA path 3/4 i2c */
/*	I2C8 = 0, - Used for POR sampling */
	I2C9 = 4,  /* Bus4: eeprom, ad7461, ir36 */
/*	I2C10 = 0, - Used for gpio uart3 rts/cts */
	I2C11 = 0   /* Not used */
};

union voltages_t {
	uint v[12];
	struct {
		uint vsec_1v8;
		uint vsec_1v5;
		uint vsec_jfvdd;
		uint vsec_2v5;
		uint vpri_3v3;
		uint vpri_12v0;
		uint vpri_5v0;
		uint gnd;
		uint vsec_gvdda;
		uint vsec_gvddb;
		uint vsec_gvddc;
		uint vsec_3v3;
	} s;
};

union currents_t {
	int i[9];
	struct {
		int  ddr;
		int  gvdda;
		int  gvddb;
		int  gvddc;
		int  gvdd5;
		int  jfvdd;
		int  lvdd;
		int  dvdd;
		int  temp;
	} s;
};

static int read_adt7461(uchar chip, int *local_temp, int *ext_temp_int,
			int *ext_temp_frac)
{
	uchar rbuf[3];

	if (i2c_read(chip, 0x00, 1, &rbuf[0], 1) != 0) /* Local temp */
		return -1;
	if (i2c_read(chip, 0x01, 1, &rbuf[1], 1) != 0) /* External temp hi */
		return -1;
	if (i2c_read(chip, 0x10, 1, &rbuf[2], 1) != 0) /* External temp lo */
		return -1;
	*local_temp = rbuf[0]; /* Degrees C */
	*ext_temp_int = rbuf[1]; /* Degrees C */
	*ext_temp_frac = (rbuf[2] >> 6) * 25; /* 1/100 Degrees C */
	debug("adt7461 = 0x%02X 0x%02X 0x%02X\n", rbuf[0], rbuf[1], rbuf[2]);
	return 0;
}

/*
 * Display 4T4R voltages and temperatures
 */
static void do_bdstatus_4t4r(void)
{
	int old_i2c_dev;
	int old_i2c_speed;
	int local_temp, die_temp_int, die_temp_frac;

	local_temp = 0;
	die_temp_int = 0;
	die_temp_frac = 0;
	old_i2c_dev = i2c_get_bus_num();
	old_i2c_speed = i2c_get_bus_speed();

	i2c_set_bus_num(I2C9);
	i2c_set_bus_speed(100000);

	if (read_adt7461(0x4C, &local_temp, &die_temp_int, &die_temp_frac) != 0)
		printf("adt7461 reading failed\n");

	i2c_set_bus_num(old_i2c_dev);
	i2c_set_bus_speed(old_i2c_speed);

	printf("ADT7461 temp    :   %3d C\n", local_temp);
	printf("AFD4400 temp    :   %3d.%02d C\n", die_temp_int, die_temp_frac);
}

/*
 * Display board voltages, currents and temperatures
 */
int do_bdstatus(cmd_tbl_t *cmdtp, int flag, int argc,
		char * const argv[])
{
	switch (get_board_type()) {
	case BOARD_TYPE_D4400_4T4R:
		do_bdstatus_4t4r();
		break;
	default:
		puts("Error - unknown board type\n");
		break;
	}
	return 0;
}

/***************************************************/

U_BOOT_CMD(
	bdstatus, CONFIG_SYS_MAXARGS, 1, do_bdstatus,
	"display board voltages, currents and temperatures",
	""
);
