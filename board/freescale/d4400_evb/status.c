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
#include <configs/d4400_evb.h>

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

//#define DEBUG

// Zilker PMBUS defines
#define LINEAR_MODE           (0x00)
#define VOUT_MODE             (0x20)
#define READ_VOUT             (0x8B)
#define USER_CONFIG           (0xD1)

enum i2c_bus {
	I2C1 = 0,
	I2C2 = 1,
	I2C3 = 2,
/*	I2C4 = 0, - Used for UART4 */
	I2C5 = 3,
	I2C6 = 4,
	I2C7 = 5,
	I2C8 = 6,
	I2C9 = 7,
	I2C10 = 8,
	I2C11 = 9
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

static int board_rev(void)
{
	int rev = 0;
#if defined(CONFIG_CMD_WEIM_NOR) && defined(CONFIG_QIXIS)
	rev = readb(CONFIG_QIXIS_BASE_ADDR + 1);
#endif
	return rev;
}

static int read_adt7461(uchar chip, int *local_temp, int
			*ext_temp_int, int *ext_temp_frac)
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

static int read_ltc2499(uchar chip, int results[])
{
	int i;
	uint wval;
	uchar wbuf[2];
	uchar rbuf[4];

	/* Start first measurement */
	wbuf[0] = 0xA0;
	wbuf[1] = 0x80;
	i2c_write(chip, 0, 0, wbuf, 2);
	mdelay(170);
	/* Read first measurment and start next measurement */
	for (i = 0; i <= 8; i++) {
		putc('.');
		wval = 0xA080 | (((i+1) & 7) << 8);
		if (i > 6)
			wval |= 0x40;
		if(i2c_read(chip, wval, 2, rbuf, 4) != 0)
			return -1;
		mdelay(170);
		results[i] = (int)(((rbuf[0] << 25) |
				    (rbuf[1] << 17) |
				    (rbuf[2] << 9) |
				    (rbuf[3] << 1)) >> 7);
		debug(" ch%d = %02X %02X %02X %02X\n", i, rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
		if ((rbuf[0] & 0xC0) == 0x00) /* -FS */
			results[i] = -0x01000001;
		else if ((rbuf[0] & 0xC0) == 0xC0) /* +FS */
			results[i] =  0x01000000;

		debug(" ch%d = 0x%08X\n", i, results[i]);
	}
	printf("\n");
	return 0;
}

static int read_zilker(int *result)
{
	u16 val = 0x0000;
#if defined(CONFIG_ZL6105_VID)
	u16 val1 = 0x0000;
	u32 addr;
	u8 dataformat;
	u8 exponent;

	i2c_set_bus_num(CONFIG_ZL6105_VID_I2C_BUS_NUM);
	addr = CONFIG_ZL6105_VID_I2C_ADDR;

	i2c_read(addr, USER_CONFIG, 1, (u8*)&val, 2);
	if (val & 3) {
		/* get voltage mode and format */
		i2c_read(addr, VOUT_MODE, 1, (u8*)&val, 1);
		exponent = -(((s8)(val << 3)) >> 3);
		dataformat = (u8)(val & 0x00E0);
		if (dataformat == LINEAR_MODE) {
			/* get regular VOUT voltage */
			int ctr = 10;
			int diff;
			do {
				mdelay(2);
				val = 0;
				i2c_read(addr, READ_VOUT, 1, (u8*)&val, 2);
				val = (1000ul * val) >> exponent;
				val1 = 0;
				mdelay(2);
				i2c_read(addr, READ_VOUT, 1, (u8*)&val1, 2);
				val1 = (1000ul * val1) >> exponent;
				diff = val - val1;
				if (diff < 0) diff = -diff;
			} while (ctr-- > 0 && diff > 100);
		}
	}

#endif
	*result = val;
	return 0;
}

static uchar ad7993_wbuf[4] = { 0x00, 0x00, 0xFF, 0xFF };

static int read_ad7993(uchar chip, uint results[])
{
	uchar rbuf[2];
	int i;

	/* Read the configuration register */
	if (i2c_read(chip, 2, 1, rbuf, 1) != 0) {
		printf("Error reading device");
		return -1;
	}
	/* If any channel is not selected then configure to monitor */
	if ((rbuf[0] & 0xF0) != 0xF0) {
		rbuf[1] = rbuf[0];
		for (i = 0; i < 3; i++, rbuf[0] >>= 1) {
			if (rbuf[0] & 0x10)
				continue;
			i2c_write(chip, (i*3)+4, 1, &ad7993_wbuf[0], 2);
			i2c_write(chip, (i*3)+5, 1, &ad7993_wbuf[2], 2);
			i2c_write(chip, (i*3)+6, 1, &ad7993_wbuf[2], 2);
		}
		rbuf[1] |= 0xF0; /* enable all channels */
		i2c_write(chip, 2, 1, &rbuf[1], 1);
	}

	for (i = 0; i < 4; i++) {
		do {
			if(i2c_read(chip, 0, 1, rbuf, 2) != 0)
				return -1;
		} while (((rbuf[0] >> 4)&0x3) != i);
		/* reduce to 10 bit result */
		results[i] = (((rbuf[0] << 8) | rbuf[1]) & 0xFFF) >> 2;
		debug(" ch%d = 0x%02X%02X : %4d\n",
		      i, rbuf[0], rbuf[1], results[i]);
	}
	return 0;
}

#ifdef DEBUG
static int dump_ad7993(uchar chip)
{
	uchar buf[2];
	int i;

	printf("Reading AD7993 at address 0x%02X\n", chip);
	for (i = 0; i < 16; i++) {
		if (i2c_read(chip, i, 1, buf, 2) != 0) {
			printf("Error reading device");
			return -1;
		}
		printf(" %2d = %02X %02X\n", i, buf[0], buf[1]);
	}
	return 0;
}
#endif

/*
 * Display board voltages, currents and temperatures
 */
int do_bdstatus(cmd_tbl_t *cmdtp, int flag, int argc,
		char * const argv[])
{
	int i;
	int old_i2c_dev;
	int old_i2c_speed;
	int local_temp, die_temp_int, die_temp_frac;
	union voltages_t voltages;
	union currents_t currents;
	int *v_scale;
	int ltc_v_ref;
	int core_volts;

	static int v_scale_rev_a[] = {
		3300, 3300, 3300, 3300,
		3300, 18649, 6600, 3300,
		3300, 3300, 3300, 3300
	};
	static int v_scale_rev_b[] = {
		2500, 2500, 2500, 5000,
		5000, 14128, 7819, 2500,
		5000, 5000, 5000, 5000
	};
	static int i_resistor[] = {
		100, 10, 10, 10,
		10, 10, 10, 1000
	};

	ltc_v_ref = 3300;
	v_scale = v_scale_rev_a;
	if (board_rev() == EVB_REV_B) {
		ltc_v_ref = 2500;
		v_scale = v_scale_rev_b;
	}

	local_temp = 0;
	die_temp_int = 0;
	die_temp_frac = 0;
	old_i2c_dev = i2c_get_bus_num();
	old_i2c_speed = i2c_get_bus_speed();

	i2c_set_bus_num(I2C11);
	i2c_set_bus_speed(400000);

#ifdef DEBUG
	dump_ad7993(0x20);
#endif
	if(read_ad7993(0x20, &voltages.v[0]) != 0)
	printf("ad7993 device reading failed at chip addr 0x20\n");

#ifdef DEBUG
	dump_ad7993(0x20);
	dump_ad7993(0x21);
#endif
	if(read_ad7993(0x21, &voltages.v[4]) != 0)
	printf("ad7993 device reading failed at chip addr 0x21\n");
#ifdef DEBUG
	dump_ad7993(0x21);
	dump_ad7993(0x22);
#endif
	if(read_ad7993(0x22, &voltages.v[8]) != 0)
	printf("ad7993 device reading failed at chip addr 0x22\n");

#ifdef DEBUG
	dump_ad7993(0x22);
#endif

	if(read_ltc2499(0x45, &currents.i[0]) != 0)
	printf("ltc2499 reading failed\n");
	if(read_adt7461(0x4C, &local_temp, &die_temp_int, &die_temp_frac) != 0)
	printf("adt7461 reading failed\n");

	core_volts = 0;
	if (read_zilker(&core_volts) != 0)
	printf("Zilker device reading failed at chip addr 0x22\n");

	i2c_set_bus_num(old_i2c_dev);
	i2c_set_bus_speed(old_i2c_speed);

	for (i = 0; i < 12; i++)
		voltages.v[i] = (voltages.v[i] * v_scale[i]) >> 10;

	for (i = 0; i < 8; i++)
		currents.i[i] = ((((currents.i[i] >> 5) * ltc_v_ref) >> 10) *
					i_resistor[i]) >> 10;

	currents.s.temp = (((currents.s.temp >> 1) * ltc_v_ref + 785000) / 1570000) -
					273;

	printf("Pri 12V         : %5d mV\n", voltages.s.vpri_12v0);
	printf("Pri 5V0         : %5d mV\n", voltages.s.vpri_5v0);
	printf("Pri 3V3         : %5d mV\n", voltages.s.vpri_3v3);
	printf("Sec 3V3 / GVDD5 : %5d mV, %5d mA\n",
	       voltages.s.vsec_3v3, currents.s.gvdd5);
	printf("Sec 2V5         : %5d mV\n", voltages.s.vsec_2v5);
	printf("Sec 1V8 / LVDD  : %5d mV, %5d mA\n",
	       voltages.s.vsec_1v8, currents.s.lvdd);
	printf("Sec 1V5 / DDR   : %5d mV, %5d mA\n",
	       voltages.s.vsec_1v5, currents.s.ddr);
	printf("JFVDD           : %5d mV, %5d mA\n",
	       voltages.s.vsec_jfvdd, currents.s.jfvdd);
	printf("GVDDA           : %5d mV, %5d mA\n",
	       voltages.s.vsec_gvdda, currents.s.gvdda);
	printf("GVDDB           : %5d mV, %5d mA\n",
	       voltages.s.vsec_gvddb, currents.s.gvddb);
	printf("GVDDC           : %5d mV, %5d mA\n",
	       voltages.s.vsec_gvddc, currents.s.gvddc);
	if (core_volts > 0)
		printf("Core 1V / DVDD  : %5d mV,%6d mA\n",
			core_volts, currents.s.dvdd);
	else
		printf("Core 1V / DVDD  :          %6d mA\n", currents.s.dvdd);
	printf("LTC2499 temp    :   %3d C\n", currents.s.temp);
	printf("ADT7461 temp    :   %3d C\n", local_temp);
	printf("AFD4400 temp    :   %3d.%02d C\n", die_temp_int, die_temp_frac);

	return 0;
}

/***************************************************/

U_BOOT_CMD(
	bdstatus, CONFIG_SYS_MAXARGS, 1, do_bdstatus,
	"display board voltages, currents and temperatures",
	""
);
