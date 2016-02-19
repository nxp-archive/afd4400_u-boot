/*
 * Copyright (C) 2013-2015 Freescale Semiconductor, Inc.
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
#include <configs/d4400_4t4r.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/d4400_boards.h>

#if defined(CONFIG_VID)

#define INVALID_VALUE   (0.0000)
#define	MAX_VID_INDEX   (32)

extern enum board_type get_board_type(void);
extern enum board_rev get_board_rev(void);

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
	1.050,  // 0b00000
	0.9875, // 0b00001
	0.9750, // 0b00010
	0.9625, // 0b00011
	0.9500, // 0b00100
	0.9375, // 0b00101
	0.9250, // 0b00110
	0.9125, // 0b00111
	0.9000, // 0b01000
	INVALID_VALUE, // 0b01001
	INVALID_VALUE, // 0b01010
	INVALID_VALUE, // 0b01011
	INVALID_VALUE, // 0b01100
	INVALID_VALUE, // 0b01101
	INVALID_VALUE, // 0b01110
	INVALID_VALUE, // 0b01111

	/***** PART 2, VID MSB = 1 *****/
	1.0000, // 0b10000
	1.0125, // 0b10001
	1.0250, // 0b10010
	1.0375, // 0b10011
	1.0500, // 0b10100
	INVALID_VALUE, // 0b10101
	INVALID_VALUE, // 0b10110
	INVALID_VALUE, // 0b10111
	INVALID_VALUE, // 0b11000
	INVALID_VALUE, // 0b11001
	INVALID_VALUE, // 0b11010
	INVALID_VALUE, // 0b11011
	INVALID_VALUE, // 0b11100
	INVALID_VALUE, // 0b11101
	INVALID_VALUE, // 0b11110
	INVALID_VALUE, // 0b11111
};

//#define DEBUG

#define MINIMUM_VOLTAGE       (0.89999)
#define MAXIMUM_VOLTAGE       (1.10001)

#define LINEAR_MODE           (0x00)

#define OPERATION             (0x01)
#define VOUT_MODE             (0x20)
#define VOUT_COMMAND          (0x21)
#define VOUT_MAX              (0x24)
#define VOUT_MARGIN_HIGH      (0x25)
#define VOUT_MARGIN_LOW       (0x26)
#define VOUT_TRANSITION_RATE  (0x27)
#define VOUT_OV_FAULT_LIMIT   (0x40)
#define VOUT_UV_FAULT_LIMIT   (0x44)
#define POWER_GOOD_ON         (0x5E)
#define READ_VOUT             (0x8B)
#define USER_CONFIG           (0xD1)

#define OPERATION_MARGIN      (0xF0)
#define OPERATION_MARGIN_OFF  (0x80)
#define OPERATION_MARGIN_LOW  (0x98)
#define OPERATION_MARGIN_HIGH (0xA8)

#define IR36021_SET_BOOT_VOUT_LOOP1 (0x17)
#define IR36021_SET_BOOT_VOUT_LOOP2 (0x18)
#define IR36021_SET_VOUT_LOOP1      (0x6A)
#define IR36021_SET_VOUT_LOOP2      (0x6C)
#define IR36021_READ_VOUT_LOOP1     (0x9A)
#define IR36021_READ_VOUT_LOOP2     (0x9B)
#define IR36021_READ_CURRENT_LOOP1  (0x9C)
#define IR36021_READ_CURRENT_LOOP2  (0x9D)
#define IR36021_READ_TEMP1          (0x9E)
#define IR36021_READ_TEMP2          (0x9F)
#define IR36021_READ_STATUS_LOOP1   (0xA1)
#define IR36021_READ_STATUS_LOOP2   (0xA2)

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

static void dump_ir36021_regs(void)
{
	u8 val;
	int addr;
	int old_i2c_num;
	int old_i2c_speed;

	old_i2c_num = i2c_get_bus_num();
	old_i2c_speed = i2c_get_bus_speed();

	i2c_set_bus_num(CONFIG_IR36021_VID_I2C_BUS_NUM);
	i2c_set_bus_speed(CONFIG_IR36021_VID_I2C_SPEED);

	addr = CONFIG_IR36021_VID_I2C_ADDR;

	i2c_read(addr, IR36021_SET_BOOT_VOUT_LOOP1, 1, &val, 1);
	printf("SET_BOOT_VOLTAGE: %02x\n", val);

	i2c_read(addr, IR36021_SET_VOUT_LOOP1, 1, &val, 1);
	printf("SET_VOLTAGE:      %02x\n", val);

	i2c_read(addr, IR36021_READ_VOUT_LOOP1, 1, &val, 1);
	printf("READ_VOLTAGE:     %02x\n", val);

	i2c_read(addr, IR36021_READ_CURRENT_LOOP1, 1, &val, 1);
	printf("READ_CURRENT:     %02x\n", val);

	i2c_read(addr, IR36021_READ_TEMP1, 1, &val, 1);
	printf("READ_TEMP1:       %02x\n", val);

	i2c_read(addr, IR36021_READ_TEMP2, 1, &val, 1);
	printf("READ_TEMP2:       %02x\n", val);

	i2c_read(addr, IR36021_READ_STATUS_LOOP1, 1, &val, 1);
	printf("READ_STATUS:      %02x\n", val);

	i2c_set_bus_num(old_i2c_num);
	i2c_set_bus_speed(old_i2c_speed);
}

static s32 setup_ir36021_vid_volts(float to_volts)
{
	u32 addr;
	int old_i2c_num;
	int old_i2c_speed;
	int ret = 0;

	u8 vid = (u8)((to_volts - 0.2425) * 200);

	old_i2c_num = i2c_get_bus_num();
	old_i2c_speed = i2c_get_bus_speed();

	i2c_set_bus_num(CONFIG_IR36021_VID_I2C_BUS_NUM);
	i2c_set_bus_speed(CONFIG_IR36021_VID_I2C_SPEED);

	addr = CONFIG_IR36021_VID_I2C_ADDR;

	/* switch voltage */
	ret = i2c_write(addr, IR36021_SET_VOUT_LOOP1, 1, &vid, 1);

	/* Wait for voltage to transition */
	mdelay(10);

	to_volts = (vid/200.0) + 0.245 + 0.00004; /* avoid rounding errors */
	int volts_int = to_volts;
	int volts_frac = (to_volts - volts_int) * 10000;
	printf(" %d.%04d Volts\n", volts_int, volts_frac);

	i2c_set_bus_num(old_i2c_num);
	i2c_set_bus_speed(old_i2c_speed);

#if defined DEBUG
	dump_ir36021_regs();
#endif
	return ret;
}

static s32 setup_vid_volts(float to_volts)
{
	s32 ret;

	if (0 == to_volts) {
		u16 vid = get_efuse_vid();
		float volts = vid2voltmap[vid];
		int volts_int = volts;
		int volts_frac = (volts - volts_int) * 10000;
		printf("vid_volts = 0.  Using fuse value %d.%04d Volts\n", volts_int, volts_frac);
		return 0;
	} else if (to_volts > MAXIMUM_VOLTAGE || to_volts < MINIMUM_VOLTAGE) {
		to_volts += 0.00004; /* avoid rounding errors */
		int volts_int = to_volts;
		int volts_frac = (to_volts - volts_int) * 10000;
		printf("\nVID: ERROR - requested voltage %d.%04d V is outside supported range\n",
		       volts_int, volts_frac);
		return -1;
	}
	switch (get_board_type()) {
	case BOARD_TYPE_D4400_4T4R:
	case BOARD_TYPE_D4400_21RRH:
		ret = setup_ir36021_vid_volts(to_volts);
		break;
	default:
		puts("Error - no VID controller identified for this board\n");
		ret = -1;
		break;
	}
	return ret;
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
	return setup_vid_volts(volts);
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

static int atoi(const char *s)
{
	int value;

	for (value = 0; (*s >= '0' && *s <= '9'); s++)
		value = value * 10 + (*s - '0');

	return value;
}

static float atof(const char *s)
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
	return setup_vid_volts(val);
}

static int do_show_vid_regs(cmd_tbl_t *cmdtp, int flag, int argc,
			    char * const argv[])
{
	if (argc > 1)
		return CMD_RET_USAGE;

	switch (get_board_type()) {
	case BOARD_TYPE_D4400_4T4R:
	case BOARD_TYPE_D4400_21RRH:
		dump_ir36021_regs();
		break;
	default:
		puts("Error - no VID controller identified for this board\n");
		break;
	}
	return 0;
}

static int do_vid(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);

U_BOOT_CMD(vid,    3,      1,  do_vid,
	"VID sub-system for D4400 4T4R",
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
