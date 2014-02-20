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
#include <i2c.h>
#include <configs/d4400_evb.h>
#include <asm/arch/imx-regs.h>

#if defined(CONFIG_ZL6105_VID)

#define INVALID_VALUE	0.0000
#define	MAX_VID_INDEX	32

/* map for VID to volt */
float vid2voltmap[MAX_VID_INDEX] = {

	/*  VID 4 LSB bit will be use as ARRAY INDEX
	 *  for map to volt.MSB will be act as switch
	 *  between two part of arrays.
	 *
	 *  0.0000 is INVALID value.
	 */

	/***** PART 1, VID MSB = 0 *****/

	/* volt     // VID */
	1.050,  // 0x00000
	0.9875, // 0x00001
	0.9750, // 0x00010
	0.9625, // 0x00011
	0.9500, // 0x00100
	0.9375, // 0x00101
	0.9250, // 0x00110
	0.9125, // 0x00111
	0.9000, // 0x01000
	INVALID_VALUE, // 0x01001
	INVALID_VALUE, // 0x01010
	INVALID_VALUE, // 0x01011
	INVALID_VALUE, // 0x01100
	INVALID_VALUE, // 0x01101
	INVALID_VALUE, // 0x01110
	INVALID_VALUE, // 0x01111

	/***** PART 2, VID MSB = 1 *****/
	1.0000, // 0x10000
	1.0125, // 0x10001
	1.0250, // 0x10010
	1.0375, // 0x10011
	1.0500, // 0x10100
	INVALID_VALUE, // 0x10101
	INVALID_VALUE, // 0x10110
	INVALID_VALUE, // 0x10111
	INVALID_VALUE, // 0x11000
	INVALID_VALUE, // 0x11001
	INVALID_VALUE, // 0x11010
	INVALID_VALUE, // 0x11011
	INVALID_VALUE, // 0x11100
	INVALID_VALUE, // 0x11101
	INVALID_VALUE, // 0x11110
	INVALID_VALUE, // 0x11111
};

//#define DEBUG

#define JUMP2_PART2	16
#define LINEAR_MODE 0x00

#define VOUT_MODE   0x20
#define VOUT_MAX	0x24
#define POWER_GOOD_ON 0x5E
#define VOUT_OV_FAULT_LIMIT 0x40
#define VOUT_MARGIN_HIGH 0x25
#define OPERATION 0x01
#define VOUT_UV_FAULT_LIMIT 0x44
#define VOUT_MARGIN_LOW 0x26

void dump_zl6105_reg(void);
s32 setup_vid(u16 vid);
s32 configure_vid(void);

static u16 convert2linearformat(float volts, u8 exponent) {
	return (u16)(volts * (1 << exponent));
}

static u16 get_efuse_vid(void)
{
	struct iim_fuse_t *iim_f = (struct iim_fuse_t *)(IIM_BASE_ADDR +
			IIM_FUSE_BITS_OFFSET);

	u32 lsb = iim_f->vid1;
	u32 msb = iim_f->vid2;

	u16 vid = (((u16)(msb & 0x00FF)) << 8) | (u16)(lsb & 0x00FF);

	return vid;
}

s32 configure_vid(void)
{
	u16 vid = get_efuse_vid();
	return setup_vid(vid);
}

s32 setup_vid(u16 vid)
{
	u32 old_bus, addr;
	u16 val=0x0000;
	u8 dataformat;
	u8 exponent;

	if(vid >= MAX_VID_INDEX) {
		printf("\nError:VID %02x out of map", vid);
		return -1;
	}

	float volts = vid2voltmap[vid];

	if (volts == INVALID_VALUE) {
		printf("\nError:VID %02x has map to volts = 0.0000", vid);
		return -1;
	}

	old_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_ZL6105_VID_I2C_BUS_NUM);
	addr = CONFIG_ZL6105_VID_I2C_ADDR;

	i2c_read(addr, VOUT_MODE, 1, (u8*)&val, 1);

	dataformat = (u8)(val & 0x00E0);
	if (dataformat != LINEAR_MODE) {
		printf("\nData format %02x is not LINEAR", dataformat);
		/* can we set it to LINEAR MODE? */
		i2c_set_bus_num(old_bus);
		return -1;
	}

	exponent = -(((s8)(val << 3)) >> 3);

	float volt_10percent = (volts * 0.10);
	float volt_15percent = (volts * 0.15);

	val = convert2linearformat((volts + volt_10percent), exponent);
	i2c_write(addr, VOUT_MAX, 1, (u8*)&val, 2);

	val = convert2linearformat((volts - volt_10percent), exponent);
	i2c_write(addr, POWER_GOOD_ON, 1, (u8*)&val, 2);

	if (volts > 1.0000) {

		val = convert2linearformat((volts + volt_15percent), exponent);
		i2c_write(addr, VOUT_OV_FAULT_LIMIT, 1, (u8*)&val, 2);

		val = convert2linearformat(volts, exponent);
		i2c_write(addr, VOUT_MARGIN_HIGH, 1, (u8*)&val, 2);

		val = 0xa8;
		i2c_write(addr, OPERATION, 1, (u8*)&val, 1);

	} else if (volts < 1.0000) {

		val = convert2linearformat((volts - volt_15percent), exponent);
		i2c_write(addr, VOUT_UV_FAULT_LIMIT, 1, (u8*)&val, 2);

		val = convert2linearformat(volts, exponent);
		i2c_write(addr, VOUT_MARGIN_LOW, 1, (u8*)&val, 2);

		val = 0x98;
		i2c_write(addr, OPERATION, 1, (u8*)&val, 1);
	}

	printf("VID :%02x\n", vid);
	i2c_set_bus_num(old_bus);

#if defined DEBUG
	dump_zl6105_reg();
#endif
	return 0;
}

void dump_zl6105_reg(void)
{
	int old_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_ZL6105_VID_I2C_BUS_NUM);

	int addr = CONFIG_ZL6105_VID_I2C_ADDR;
	u16 val = 0x0000;

	i2c_read(addr, VOUT_MAX, 1, (u8*)&val, 2);
	printf("VOUT_MAX		:%04x\n", val);

	i2c_read(addr, POWER_GOOD_ON, 1, (u8*)&val, 2);
	printf("POWER_GOOD_ON	:%04x\n", val);

	i2c_read(addr, VOUT_OV_FAULT_LIMIT, 1,(u8*)&val, 2);
	printf("VOUT_OV_FAULT_LIMIT	:%04x\n", val);

	i2c_read(addr, VOUT_MARGIN_HIGH, 1, (u8*)&val, 2);
	printf("VOUT_MARGIN_HIGH	:%04x\n", val);

	i2c_read(addr, VOUT_UV_FAULT_LIMIT, 1, (u8*)&val, 2);
	printf("VOUT_UV_FAULT_LIMIT	:%04x\n", val);

	i2c_read(addr, VOUT_MARGIN_LOW, 1, (u8*)&val, 2);
	printf("VOUT_MARGIN_LOW	:%04x\n", val);

	val = 0x0000;
	i2c_read(addr, OPERATION, 1, (u8*)&val, 1);
	printf("OPERATION	:%04x\n", val);

	i2c_set_bus_num(old_bus);
}

#endif
