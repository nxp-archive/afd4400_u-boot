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
#include <command.h>
#include <configs/d4400_evb.h>
#include <asm/arch/imx-regs.h>

#if defined(CONFIG_ZL6105_VID)

#define INVALID_VALUE   (0.0000)
#define	MAX_VID_INDEX   (32)

/* map for VID to volt */
static float vid2voltmap[MAX_VID_INDEX] = {

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

#define LINEAR_MODE           (0x00)

#define OPERATION             (0x01)
#define VOUT_MODE             (0x20)
#define VOUT_COMMAND          (0x21)
#define VOUT_MAX              (0x24)
#define VOUT_MARGIN_HIGH      (0x25)
#define VOUT_MARGIN_LOW       (0x26)
#define VOUT_OV_FAULT_LIMIT   (0x40)
#define VOUT_UV_FAULT_LIMIT   (0x44)
#define POWER_GOOD_ON         (0x5E)
#define USER_CONFIG           (0xD1)

#define OPERATION_MARGIN      (0xF0)
#define OPERATION_MARGIN_OFF  (0x80)
#define OPERATION_MARGIN_LOW  (0x98)
#define OPERATION_MARGIN_HIGH (0xA8)

#define MINIMUM_VOLTAGE       (0.89999)
#define MAXIMUM_VOLTAGE       (1.10001)

/* Global functions */
s32 configure_vid(void);


static u16 get_efuse_vid(void)
{
	struct iim_fuse_t *iim_f = (struct iim_fuse_t *)(IIM_BASE_ADDR +
			IIM_FUSE_BITS_OFFSET);

	u32 lsb = iim_f->vid1;
	u32 msb = iim_f->vid2;

	u16 vid = (((u16)(msb & 0x00FF)) << 8) | (u16)(lsb & 0x00FF);

	return vid;
}

static u16 convert2linearformat(float volts, u8 exponent) {
	return (u16)(volts * (1 << exponent));
}

static void change_voltage(u32 addr, u8 exponent, float from_volts,
			   float to_volts, u8 operation)
{
	u16   val = 0x0000;
	float hi_volts = from_volts > to_volts ? from_volts : to_volts;
	float lo_volts = from_volts < to_volts ? from_volts : to_volts;

	float to_volt_10percent = (to_volts * 0.10);
	float to_volt_15percent = (to_volts * 0.15);
	float hi_volt_10percent = (hi_volts * 0.10);
	float hi_volt_15percent = (hi_volts * 0.15);
	float lo_volt_10percent = (lo_volts * 0.10);
	float lo_volt_15percent = (lo_volts * 0.15);

	/* Make sure limits are wide enough for both to and from voltages */
	val = convert2linearformat((hi_volts + hi_volt_10percent), exponent);
	i2c_write(addr, VOUT_MAX, 1, (u8*)&val, 2);

	val = convert2linearformat((lo_volts - lo_volt_10percent), exponent);
	i2c_write(addr, POWER_GOOD_ON, 1, (u8*)&val, 2);

	val = convert2linearformat((hi_volts + hi_volt_15percent), exponent);
	i2c_write(addr, VOUT_OV_FAULT_LIMIT, 1, (u8*)&val, 2);

	val = convert2linearformat((lo_volts - lo_volt_15percent), exponent);
	i2c_write(addr, VOUT_UV_FAULT_LIMIT, 1, (u8*)&val, 2);

	mdelay(10);
	/* switch voltage */
	val = operation;
	i2c_write(addr, OPERATION, 1, (u8*)&val, 1);
	mdelay(10);

	/* set correct limits around the new voltage */
	val = convert2linearformat((to_volts + to_volt_10percent), exponent);
	i2c_write(addr, VOUT_MAX, 1, (u8*)&val, 2);

	val = convert2linearformat((to_volts - to_volt_10percent), exponent);
	i2c_write(addr, POWER_GOOD_ON, 1, (u8*)&val, 2);

	val = convert2linearformat((to_volts + to_volt_15percent), exponent);
	i2c_write(addr, VOUT_OV_FAULT_LIMIT, 1, (u8*)&val, 2);

	val = convert2linearformat((to_volts - to_volt_15percent), exponent);
	i2c_write(addr, VOUT_UV_FAULT_LIMIT, 1, (u8*)&val, 2);
}

static void print_zl6105_reg_volts(u16 reg, u8 exponent)
{
	u32 millivolts = (1000 * reg) >> exponent;
	printf(" => %4d mV\n", millivolts);
}



static s32 setup_vid_volts(u16 vid, float volts)
{
	u32 old_bus, addr;
	u16 val=0x0000;
	u8 dataformat;
	u8 exponent;

	if (volts > MAXIMUM_VOLTAGE || volts < MINIMUM_VOLTAGE) {
		volts += 0.00004; // help with rounding errors
		int volts_int = volts;
		int volts_frac = (volts - volts_int) * 10000;
		printf("\nVID: ERROR - requested voltage %d.%04d V is outside supported range\n",
			volts_int, volts_frac);
		return -1;
	}

	old_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_ZL6105_VID_I2C_BUS_NUM);
	addr = CONFIG_ZL6105_VID_I2C_ADDR;

	/* get voltage mode and format */
	i2c_read(addr, VOUT_MODE, 1, (u8*)&val, 1);
	exponent = -(((s8)(val << 3)) >> 3);
	dataformat = (u8)(val & 0x00E0);
	if (dataformat != LINEAR_MODE) {
		printf("\niError: VID data format 0x%02x is not LINEAR\n",
			dataformat);
		/* can we set it to LINEAR MODE? */
		i2c_set_bus_num(old_bus);
		return -1;
	}

	/* get regular VOUT voltage */
	i2c_read(addr, VOUT_COMMAND, 1, (u8*)&val, 2);
	float vout_volts = (1.0 * val) / (1 << exponent);

	i2c_read(addr, OPERATION, 1, (u8*)&val, 1);
	/* If supply is margined then return to normal operation first */
	if ((val & OPERATION_MARGIN) > OPERATION_MARGIN_OFF) {
		if (val & 0x20) {
			i2c_read(addr, VOUT_MARGIN_HIGH, 1, (u8*)&val, 2);
		} else {
			i2c_read(addr, VOUT_MARGIN_LOW, 1, (u8*)&val, 2);
		}
		float current_volts = (1.0 * val) / (1 << exponent);
		change_voltage(addr, exponent, current_volts, vout_volts,
			OPERATION_MARGIN_OFF);
	}

	/* Margin supply voltage up or down from nominal vout if neeeded */
	if (volts != vout_volts) {
		val = convert2linearformat(volts, exponent);
		if (volts > vout_volts) {
			i2c_write(addr, VOUT_MARGIN_HIGH, 1, (u8*)&val, 2);
			change_voltage(addr, exponent, vout_volts, volts,
			               OPERATION_MARGIN_HIGH);
		} else {
			i2c_write(addr, VOUT_MARGIN_LOW, 1, (u8*)&val, 2);
			change_voltage(addr, exponent, vout_volts, volts,
				       OPERATION_MARGIN_LOW);
		}
	}

	volts += 0.00004; // help with rounding errors
	int volts_int = volts;
	int volts_frac = (volts - volts_int) * 10000;
	printf(" %d.%04d Volts\n", volts_int, volts_frac);
	i2c_set_bus_num(old_bus);

#if defined DEBUG
	dump_zl6105_regs();
#endif
	return 0;
}

static s32 setup_vid(u16 vid)
{
	if (vid >= MAX_VID_INDEX) {
		printf("\nError: VID 0x%02x out of map\n", vid);
		return -1;
	}

	float volts = vid2voltmap[vid];

	if (volts == INVALID_VALUE) {
		printf("\nError: VID 0x%02x maps to an invalid value\n", vid);
		return -1;
	}

#if defined DEBUG
	int volts_int = volts;
	int volts_frac = (volts - volts_int) * 10000;
	printf(" %d.%04d Volts\n", volts_int, volts_frac);
#endif
	return setup_vid_volts(vid, volts);
}

static void dump_zl6105_regs(void)
{
	int old_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_ZL6105_VID_I2C_BUS_NUM);
	u8 exponent;

	int addr = CONFIG_ZL6105_VID_I2C_ADDR;
	u16 val = 0x0000;

	i2c_read(addr, VOUT_MODE, 1, (u8*)&val, 1);
	printf("VOUT_MODE:           %04x\n", val);
	exponent = -(((s8)(val << 3)) >> 3);

	i2c_read(addr, VOUT_MAX, 1, (u8*)&val, 2);
	printf("VOUT_MAX:            %04x", val);
	print_zl6105_reg_volts(val, exponent);

	i2c_read(addr, POWER_GOOD_ON, 1, (u8*)&val, 2);
	printf("POWER_GOOD_ON:       %04x", val);
	print_zl6105_reg_volts(val, exponent);

	i2c_read(addr, VOUT_OV_FAULT_LIMIT, 1,(u8*)&val, 2);
	printf("VOUT_OV_FAULT_LIMIT: %04x", val);
	print_zl6105_reg_volts(val, exponent);

	i2c_read(addr, VOUT_MARGIN_HIGH, 1, (u8*)&val, 2);
	printf("VOUT_MARGIN_HIGH:    %04x", val);
	print_zl6105_reg_volts(val, exponent);

	i2c_read(addr, VOUT_UV_FAULT_LIMIT, 1, (u8*)&val, 2);
	printf("VOUT_UV_FAULT_LIMIT: %04x", val);
	print_zl6105_reg_volts(val, exponent);

	i2c_read(addr, VOUT_MARGIN_LOW, 1, (u8*)&val, 2);
	printf("VOUT_MARGIN_LOW:     %04x", val);
	print_zl6105_reg_volts(val, exponent);

	val = 0x0000;
	i2c_read(addr, OPERATION, 1, (u8*)&val, 1);
	printf("OPERATION:           %04x\n", val);

	i2c_read(addr, USER_CONFIG, 1, (u8*)&val, 2);
	printf("USER_CONFIG:         %04x\n", val);

	i2c_set_bus_num(old_bus);
}

s32 configure_vid(void)
{
#if defined DEBUG
	dump_zl6105_regs();
#endif
	u16 vid = get_efuse_vid();
	printf("VID:   %02X ->", vid);
	return setup_vid(vid);
}

/*************************** VID commands *************************/

static int atoi (const char *s)
{
	int value;

	for (value = 0; (*s >= '0' && *s <= '9'); s++)
		value = value * 10 + (*s - '0');

	return value;
}

static float atof (const char *s)
{
	float value, div;

	for (value = 0.0; (*s >= '0' && *s <= '9'); s++)
		value = value * 10 + (*s - '0');
	if (*s++ == '.') {
		for (div = 0.1; (*s >= '0' && *s <= '9'); s++, div /= 10)
			value += (*s - '0') * div;
	}
	return value;
}

static int do_set_vid(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc < 2)
		return CMD_RET_USAGE;

	int val = atoi(argv[1]);
	return setup_vid(val);
}

static int do_set_volts(cmd_tbl_t *cmdtp, int flag, int argc,
			char * const argv[])
{
	if (argc < 2)
		return CMD_RET_USAGE;

	float val = atof(argv[1]);
	return setup_vid_volts(0, val);
}

static int do_show_vid_regs(cmd_tbl_t *cmdtp, int flag, int argc,
			    char * const argv[])
{
	if (argc > 1)
		return CMD_RET_USAGE;

	dump_zl6105_regs();
	return 0;
}

static int do_vid(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[]);

U_BOOT_CMD(vid,    3,      1,  do_vid,
	"VID sub-system for D4400 EVB",
	"regs - show vid registers\n"
	"vid set <vid> - set vid value [0..31]\n"
	"vid setv <volts> - set voltage directly");

static cmd_tbl_t cmd_vid_sub[] = {
	U_BOOT_CMD_MKENT(regs, 1, 1, do_show_vid_regs, "", ""),
	U_BOOT_CMD_MKENT(set,  2, 1, do_set_vid, "", ""),
	U_BOOT_CMD_MKENT(setv, 2, 1, do_set_volts, "", ""),
};

#ifdef CONFIG_NEEDS_MANUAL_RELOC
void vid_reloc(void) {
	fixup_cmdtable(cmd_vid_sub, ARRAY_SIZE(cmd_vid_sub));
}
#endif

/* do_vid() - Handle the "vid" command-line command
 * @cmdtp:      Command data struct pointer
 * @flag:       Command flag
 * @argc:       Command-line argument count
 * @argv:       Array of command-line arguments
 *
 * Returns zero on success, CMD_RET_USAGE in case of misuse and negative
 * on error.
 */
static int do_vid(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
	cmd_tbl_t *c;

	if (argc < 2)
		return CMD_RET_USAGE;

	/* Strip off leading 'vid' command argument */
	argc--;
	argv++;

	c = find_cmd_tbl(argv[0], &cmd_vid_sub[0], ARRAY_SIZE(cmd_vid_sub));

	if (c)
		return c->cmd(cmdtp, flag, argc, argv);
	else
		return CMD_RET_USAGE;
}

#endif
