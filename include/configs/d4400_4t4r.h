/*
 * Copyright (C) 2010-2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale D4400 4T4R board.
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

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_D4400
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO
#include <asm/arch/imx-regs.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 2 * 1024 * 1024)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MXC_GPIO

#define CONFIG_FIT

#define CONFIG_D4400_UBOOT_VALIDATION_SHA256
#define CONFIG_D4400_UBOOT_SECONDARY_IMAGE

/*
 * Boot from configuration.  These settings affect the mkimage and ubsha256
 * post build tools.
 */
#define BOOT_FROM_ETHERNET	2
#define BOOT_FROM_SERIAL	3
#define BOOT_FROM_QSPI_FLASH	4
#define CONFIG_BOOT_FROM	BOOT_FROM_QSPI_FLASH

#if defined(CONFIG_D4400_UBOOT_VALIDATION_SHA256) && \
	defined(CONFIG_D4400_UBOOT_SECONDARY_IMAGE)
/* Secondary image is placed in QSPI flash at offset 0x100000
 * For QSPI flash, the secondary header starts at 0x100000.
 */
	#define SECONDARY_IMAGE_OFFSET	0xA0000 /* Micron 128MB */
#endif

/* Board i2c eeprom containing IPMI data */
#define CONFIG_CMD_EEPROM
#define CONFIG_SYS_I2C_EEPROM_ADDR		0x54
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS       4
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS   5
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN          1
#define CONFIG_SYS_I2C_EEPROM_BUS_NUM		4
#define CONFIG_SYS_I2C_EEPROM_SPEED_HZ		50000
/* Note that large eeproms that need more than 16 address bits
 * use bits in the control byte (i2c slave address byte) to extend
 * the address bits.  For example, 128K byte eeprom (17 addr bits
 * needed) uses 2 byte address plus 1 bit in the control reg.
 */
#define CONFIG_SYS_I2C_EEPROM_ADDRLEN_MAX       2

#define CONFIG_VID
/* ZL6105 VID configuration */
#define CONFIG_ZL6105_VID_I2C_BUS_NUM	9
#define CONFIG_ZL6105_VID_I2C_SPEED	50000
#define CONFIG_ZL6105_VID_I2C_ADDR	0x23
/* IR36021 VID configuration */
#define CONFIG_IR36021_VID_I2C_BUS_NUM	4
#define CONFIG_IR36021_VID_I2C_SPEED	50000
#define CONFIG_IR36021_VID_I2C_ADDR	0x38

/* UART configuration */
#define CONFIG_D4400_UART
#define CONFIG_D4400_UART_PORT		3
#define CONFIG_D4400_UART_BASE		UART3_BASE_ADDR

/* Gpio leds */
#define CONFIG_LED

/* PA gpio control */
#define CONFIG_4T4R_PA_CONTROL

/* TSEC enable */
#define CONFIG_TSEC_ENET
#define CONFIG_D4400_EVB_MDC_BUG_SW_FIX

#if defined(CONFIG_TSEC_ENET)

#define CONFIG_MII                      /* MII PHY management */
#define CONFIG_MII_DEFAULT_TSEC 1       /* Allow unregistered phys */

#define TSEC_BASE_ADDR		VETSEC0_GROUP0_BASE_ADDR
#define MDIO_BASE_ADDR		VETSEC0_MDIO_BASE_ADDR
#define TSEC_MDIO_OFFSET	0x21000
#define TSEC_SIZE               0x21000

#define TSEC1_FLAGS             (TSEC_REDUCED)
#define CONFIG_TSEC1		1
#define CONFIG_TSEC1_NAME       "eTSEC1"
#define TSEC1_PHY_ADDR		3 /* Set by hw, 4T4R board, lan87 chip */
#define TSEC1_PHY_ADDR_RGMII    3 /* Set by hw, 4T4R board, lan87 chip */
#define TSEC1_PHY_ADDR_RMII     3 /* Set by hw, 4T4R board, lan87 chip */
#define TSEC1_PHYIDX            0

#define CONFIG_ETHPRIME         "eTSEC1"
#undef CONFIG_PHY_GIGE         /* Undefine, no GbE detection */

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define	CONFIG_MII

#endif  /* CONFIG_TSEC_ENET */

/* Qspi flash */
#define CONFIG_SYS_NO_FLASH

/*VDD_VSEL Selection*/
#define CONFIG_OVDD_VSEL

/* Allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_CMD_MEMORY
#define CONFIG_CMD_GPIO
#define CONFIG_CMD_I2C
#define CONFIG_CMD_SPI

#define CONFIG_BOOTDELAY		5

#define CONFIG_LOADADDR			0x90800000
#define CONFIG_SYS_TEXT_BASE		0x90002000

/* Network settings */
#define CONFIG_IPADDR		10.69.12.210
#define CONFIG_GATEWAYIP	10.68.12.254
#define CONFIG_ETHADDR		02:20:30:40:50:60
#define CONFIG_NETMASK		255.255.255.0

#define CONFIG_EXTRA_ENV_SETTINGS\
	"uimage=uImage\0"\
	"consoledev=ttymxc2\0"\
	"baudrate=115200\0"\
	"ramdisksize=600000\0"\
	"loadaddr=0xb0000000\0"\
	"bootargs_all=setenv bootargs root=/dev/ram rw console=${consoledev},"\
		"${baudrate} ramdisk_size=${ramdisksize} rootfstype=ext2 "\
		"earlyprintk\0"\
	"bootcmd_qspi=echo Booting from QSPI FLASH ...; " \
		"run bootargs_all; sf probe 9:0 50000000 40; "\
		 "sf read ${loadaddr} 0x00120000 0x3F00000; "\
		 "bootm ${loadaddr}\0"\
	"bootcmd_ram=echo Booting from RAM ...; run test_tftp; "\
		"run bootargs_all;tftp ${loadaddr} ${tftp_path}kernel_fit.4t4r.itb;"\
		"bootm ${loadaddr}\0"\
	"bootcmd=run bootcmd_qspi\0"\
	"get_fit=tftp ${loadaddr} ${tftp_path}kernel_fit.4t4r.itb; "\
		"sf probe 9:0 50000000 40;sf erase 0x0120000 0x04000000; "\
		"sf write ${loadaddr} 0x0120000 ${filesize}\0 "\
	"get_rfs=tftp ${loadaddr} ${tftp_path}rootfs.jffs2; "\
		"sf probe 9:0 50000000 40;sf erase 0x04120000 0x03EE0000; "\
		"sf write ${loadaddr} 0x04120000 ${filesize}\0 "\
	"get_uboot1=tftp ${loadaddr} ${tftp_path}u-boot-sha256.4t4r.d4400; "\
		"sf probe 9:0 50000000 40;sf erase 0x0 0x80000; "\
		"sf write ${loadaddr} 0x0 ${filesize}\0 "\
	"get_uboot2=tftp ${loadaddr} ${tftp_path}u-boot-sha256.4t4r.d4400; "\
		"sf probe 9:0 50000000 40;sf erase 0xA0000 0x80000; "\
		"sf write ${loadaddr} 0xA0000 ${filesize}\0 "\
	"ipmi_verify1=i2c dev 4;i2c read 0x54 0.1 0x100 0x90800000; "\
		"md.b 0x90800000 0x100\0"\
	"ipmi_verify2=i2c dev 4;i2c read 0x54 0.2 0x100 0x90800000; "\
		"md.b 0x90800000 0x100\0"\
	"test_tftp=ping ${serverip}\0"\
	"tftp_path=/\0"\
	"serverip=10.69.13.69\0"

#define CONFIG_BOOTCOMMAND \
	"run bootcmd_qspi;"

#define CONFIG_ARP_TIMEOUT	200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT		"D4400 U-Boot => "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		256

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC_BASE_ADDR
#define PHYS_SDRAM_SIZE			(1u * 1024 * 1024 * 1024)

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_DATA_SIZE		0xFB8
#define CONFIG_SYS_RAM_VECTOR_SIZE	0x48

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - (CONFIG_SYS_DATA_SIZE + \
				     CONFIG_SYS_RAM_VECTOR_SIZE))
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_CMD_MEMTEST

#ifdef CONFIG_SPL_BUILD
#define CONFIG_SYS_MEMTEST_START	(PHYS_SDRAM + 0x00000000) /* 0 bytes */
#else
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_SYS_MEMTEST_SCRATCH      (PHYS_SDRAM + 0x00000000)
#define CONFIG_SYS_MEMTEST_START	(PHYS_SDRAM + 0x00000010)
#endif
#define CONFIG_SYS_MEMTEST_END		(PHYS_SDRAM + 0x00200000)

#define CONFIG_OF_LIBFDT
#define CONFIG_OF_BOARD_SETUP
#define CONFIG_CMD_BOOTZ

#define CONFIG_SYS_ARM_CACHE_WRITETHROUGH

/* Hide 256 MB of RAM from u-boot to be used
 * memory allocator used for VSPA and other
 * RF devices */
#define CONFIG_SYS_MEM_TOP_HIDE 0x10000000

/* I2C Configs */
#define CONFIG_I2C_MXC
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_I2C_SPEED	50000
#define CONFIG_SYS_MAX_I2C_BUS  11

/* SPI Configs */
#define CONFIG_MXC_SPI
#define CONFIG_HARD_SPI
#define CONFIG_DEFAULT_SPI_BUS          0
#define CONFIG_DEFAULT_SPI_MODE         (SPI_MODE_0)
#define MXC_ECSPI_SEPARATE_CLKS

/* Qspi flash */
#define CONFIG_FSL_D4400_QSPI

#define CONFIG_SPI_FLASH_SPEED_MIN_HZ	18000000
#define CONFIG_SPI_FLASH_SPEED_MAX_HZ	50000000
#define CONFIG_SPI_FLASH_SPEED_INIT_HZ	50000000 /* Initial speed setting */

#define CONFIG_SPI_FLASH

/* Specific flash support */
#define CONFIG_SPI_FLASH_MICRON
#define CONFIG_SPI_FLASH_SECTOR_SIZE	0x10000 /* 64KB sector */

/* D4400 qspi Numonyx mode erase bug work around */
#if defined CONFIG_SPI_FLASH_MICRON
	/* D440 qspi has 4-byte address/erase problem in Numonyx/Micron mode.
	 * Use Spansion mode for workaround.
	 */
	#define D4400_QSPI_NUMONYX_BUG_USE_SPANSION
#endif

/* Base address of Uboot */
#define CONFIG_SYS_MONITOR_BASE		0
/* Max len of Uboot */
#define CONFIG_SYS_MONITOR_LEN		0x80000	/* 512KB max, 8 sectors */

/* Environment variables size, 1 sector size of flash */
#define CONFIG_ENV_SECT_SIZE	CONFIG_SPI_FLASH_SECTOR_SIZE /* 1 sector */
/* Max size of environment data (1 sector) */
#define CONFIG_ENV_SIZE		CONFIG_ENV_SECT_SIZE

/* Environment data is located in 9th sector.
 * First 8 sector for u-boot.
 */
#define CONFIG_ENV_OFFSET		(8 * CONFIG_ENV_SECT_SIZE)

/* Secondary environmental data, located in 10th sector just
 * after the first env data.
 */
#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + \
					CONFIG_ENV_SIZE)
#define CONFIG_ENV_SIZE_REDUND	CONFIG_ENV_SIZE

/* Start of env data sector which is at the end of Uboot */
#define CONFIG_ENV_ADDR		(CONFIG_SYS_MONITOR_BASE +\
				 CONFIG_SYS_MONITOR_LEN)

#define CONFIG_ENV_IS_IN_SPI_FLASH	1
#define CONFIG_ENV_SPI_BUS		9
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SPI_FLASH_SPEED_INIT_HZ

#define CONFIG_ENV_SPI_MODE		SPI_QUAD_IO


/* Enable serial flash command utilties */
#define CONFIG_CMD_SF

#endif	/* __CONFIG_H */
