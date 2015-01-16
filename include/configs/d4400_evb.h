/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale d4400 evb board.
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
#define BOOT_FROM_NOR_FLASH	1
#define BOOT_FROM_ETHERNET	2
#define BOOT_FROM_SERIAL	3
#define BOOT_FROM_QSPI_FLASH	4
#define CONFIG_BOOT_FROM	BOOT_FROM_NOR_FLASH

/* Secondary image need to place in NOR flash at offset 0xc0000
 * For NOR flash, the header of the secondary image start at
 * offset 0xc1000 (initial 4k need to spare).  For QSPI flash,
 * the secondary header starts at 0xc0000.
 */
#if defined(CONFIG_D4400_UBOOT_VALIDATION_SHA256) && \
	defined(CONFIG_D4400_UBOOT_SECONDARY_IMAGE)
#define SECONDARY_IMAGE_OFFSET	0xc0000
#endif

#define CONFIG_CMD_EEPROM
#define CONFIG_SYS_I2C_EEPROM_ADDR              0x54
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS       4
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS   5
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN          1

#define CONFIG_VID
/* ZL6105 VID configuration */
#define CONFIG_ZL6105_VID_I2C_BUS_NUM	9
#define CONFIG_ZL6105_VID_I2C_SPEED	25000
#define CONFIG_ZL6105_VID_I2C_ADDR	0x23
/* R36021 VID configuration */
#define CONFIG_IR36021_VID_I2C_BUS_NUM	9
#define CONFIG_IR36021_VID_I2C_SPEED	25000
#define CONFIG_IR36021_VID_I2C_ADDR	0x38

/* Board monitor configuration */
#define CONFIG_BRD_MON_I2C_BUS_NUM	9
#define CONFIG_INA220_I2C_ADDR		0x40

/* RTC configuration */
#define CONFIG_CMD_DATE
#define CONFIG_RTC_DS3231
#define CONFIG_SYS_I2C_RTC_ADDR         0x68
#define CONFIG_SYS_RTC_BUS_NUM          9

/* UART configuration */
#define CONFIG_D4400_UART
#define CONFIG_D4400_UART_PORT		4
#define CONFIG_D4400_UART_BASE		UART4_BASE_ADDR

/* TSEC enable */
#define CONFIG_TSEC_ENET
#define CONFIG_D4400_EVB_MDC_BUG_SW_FIX
#if defined(CONFIG_TSEC_ENET)

#define TSEC_DEBUG
#define CONFIG_NET_MULTI

#define CONFIG_MII                      /* MII PHY management */
#define CONFIG_MII_DEFAULT_TSEC 1       /* Allow unregistered phys */

#define TSEC_BASE_ADDR		VETSEC0_GROUP0_BASE_ADDR
#define MDIO_BASE_ADDR		VETSEC0_MDIO_BASE_ADDR
#define TSEC_MDIO_OFFSET	0x21000
#define TSEC_SIZE               0x21000

#define TSEC1_FLAGS             (TSEC_SGMII | TSEC_GIGABIT | TSEC_REDUCED)
#define CONFIG_TSEC1    	1
#define CONFIG_TSEC1_NAME       "eTSEC1"
#define TSEC1_PHY_ADDR		0
#define TSEC1_PHY_ADDR_RGMII    1
#define TSEC1_PHY_ADDR_RMII     3
#define TSEC1_PHYIDX            0

#define TSEC2_FLAGS             (TSEC_SGMII | TSEC_GIGABIT | TSEC_REDUCED)
#define CONFIG_TSEC2    	1
#define CONFIG_TSEC2_NAME       "eTSEC2"
#define TSEC2_PHY_ADDR          2
#define TSEC2_PHYIDX            1

#define CONFIG_ETHPRIME         "eTSEC1"

#define CONFIG_PHY_GIGE         /* Include GbE speed/duplex detection */

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define	CONFIG_MII

#endif  /* CONFIG_TSEC_ENET */

/* WEIM-NOR */
#define CONFIG_CMD_WEIM_NOR
#define CONFIG_QIXIS
#define CONFIG_QIXIS_BASE_ADDR		(WEIM_BASE_ADDR + 0x10000000)

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
#define CONFIG_IPADDR   10.232.21.210
#define CONFIG_IP1ADDR   10.232.21.211
#define CONFIG_GATEWAYIP 10.232.135.254
#define CONFIG_ETHADDR	02:20:30:40:50:60
#define CONFIG_ETH1ADDR	02:20:30:40:50:61
#define CONFIG_NETMASK  255.255.0.0

#define CONFIG_EXTRA_ENV_SETTINGS\
	"uimage=uImage\0"\
	"consoledev=ttymxc3\0"\
	"baudrate=115200\0"\
	"ramdisksize=600000\0"\
	"loadaddr=0xb0000000\0"\
	"bootargs_all=setenv bootargs root=/dev/ram rw console=${consoledev},"\
		"${baudrate} ramdisk_size=${ramdisksize} rootfstype=ext2 "\
		"earlyprintk\0"\
	"bootcmd_nor=echo Booting from NOR FLASH ...; " \
		"run bootargs_all; cp.b 0x30140000 ${loadaddr} 0x3F00000; "\
		"bootm ${loadaddr}\0"\
	"bootcmd_ram=echo Booting from RAM ...; run test_tftp; "\
		"run bootargs_all;tftp ${loadaddr} ${tftp_path}kernel_fit.itb;"\
		"bootm ${loadaddr}\0"\
	"bootcmd_primarykernel=echo Booting primary kernel from NOR flash ...;"\
		"run bootargs_all; cp.b 0x30140000 ${loadaddr} 0x3F00000; "\
		"bootm ${loadaddr}\0"\
	"bootcmd_secondarykernel=echo Booting secondary kernel from NOR flash ...;"\
		"run bootargs_all; cp.b 0x34040000 ${loadaddr} 0x3F00000; "\
		"bootm ${loadaddr}\0"\
	"bootprimarykernel=run bootcmd_primarykernel\0"\
	"bootsecondarykernel=run bootcmd_secondarykernel\0"\
	"get_fit1=tftp ${loadaddr} ${tftp_path}kernel_fit.itb; "\
		"protect off 1:10-513; erase 1:10-513; "\
		"cp.b ${loadaddr} 0x30140000 ${filesize}; "\
		"protect on 1:10-513\0"\
	"get_fit2=tftp ${loadaddr} ${tftp_path}kernel_fit.itb; "\
		"protect off 1:514-1017; erase 1:514-1017; "\
		"cp.b ${loadaddr} 0x34040000 ${filesize}; "\
		"protect on 1:514-1017\0"\
	"get_rfs=tftp ${loadaddr} ${tftp_path}rootfs.jffs2; "\
		"protect off 2:0-1023; erase 2:0-1023; "\
		"cp.b ${loadaddr} 0x38000000 ${filesize}\0"\
	"get_uboot1=tftp ${loadaddr} ${tftp_path}u-boot-sha256.d4400; "\
		"protect off 1:0-3; erase 1:0-3; "\
		"cp.b ${loadaddr} 0x30001000 ${filesize}; protect on 1:0-3\0"\
	"get_uboot2=tftp ${loadaddr} ${tftp_path}u-boot-sha256.d4400; "\
		"protect off 1:6-9; erase 1:6-9; "\
		"cp.b ${loadaddr} 0x300c1000 ${filesize}; protect on 1:6-9\0"\
	"serverip=10.69.13.69\0"\
	"test_tftp=ping ${serverip}\0"\
	"tftp_path=\0"

#define CONFIG_BOOTCOMMAND \
	"run bootcmd_nor;"


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

#define CONFIG_SYS_DATA_SIZE           0xFB8
#define CONFIG_SYS_RAM_VECTOR_SIZE     0x48

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

/* WEIM NOR Config */
#ifdef CONFIG_CMD_WEIM_NOR
	#define CONFIG_SYS_FLASH_CFI
	#define CONFIG_FLASH_CFI_DRIVER
	#define CONFIG_SYS_FLASH_BASE		WEIM_BASE_ADDR
	#define CONFIG_SYS_FLASH_SIZE		(WEIM_END_ADDR - WEIM_BASE_ADDR)
	#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE
	#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
	#define CONFIG_SYS_FLASH_BANKS_LIST	{CONFIG_SYS_FLASH_BASE, \
		CONFIG_SYS_FLASH_BASE + (128*1024*1024) }
	#define CONFIG_SYS_FLASH_BANKS_SIZES	{128*1024*1024, 128*1024*1024}
	#define CONFIG_SYS_MAX_FLASH_BANKS	2
	#define CONFIG_SYS_MAX_FLASH_SECT	1024
	#define CONFIG_SYS_FLASH_PROTECTION
	#define CONFIG_SYS_FLASH_EMPTY_INFO
#endif /* CONFIG_CMD_WEIM_NOR */

/* FLASH and environment organization */
#define CONFIG_SYS_MONITOR_BASE	CONFIG_SYS_FLASH_BASE
#define CONFIG_SYS_MONITOR_LEN	0x80000

#define CONFIG_ENV_SECT_SIZE	0x20000   /* 128KiB sector size */
#define CONFIG_ENV_SIZE		CONFIG_ENV_SECT_SIZE

#define CONFIG_ENV_OFFSET	(4 * 128 * 1024)
#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + \
						CONFIG_ENV_SIZE)
#define CONFIG_ENV_SIZE_REDUND	CONFIG_ENV_SIZE
#define CONFIG_ENV_ADDR		(CONFIG_SYS_MONITOR_BASE + 0x0080000)
#define CONFIG_FSL_ENV_IN_FLASH

#define CONFIG_ENV_IS_IN_FLASH          1

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
#define CONFIG_SYS_I2C_SPEED	50000 /* for QIXIS */
#define CONFIG_SYS_MAX_I2C_BUS  11

/* SPI Configs */
#define CONFIG_MXC_SPI
#define CONFIG_HARD_SPI
#define CONFIG_DEFAULT_SPI_BUS          0
#define CONFIG_DEFAULT_SPI_MODE         (SPI_MODE_0)
#define MXC_ECSPI_SEPARATE_CLKS

/* QSPI Flash */
#define CONFIG_FSL_D4400_QSPI		/* 32-bit words are reversed for r/w op.
					   See mxc_spi.c _qspi_xfer_write() and
					  _qspi_xfer_read() functions.
					*/
#define CONFIG_QSPI_FLASH_SPANSION
#define CONFIG_QSPI_FLASH_SPEED_HZ	18000000 /* Spi clock, 50MHz max */
#define CONFIG_QSPI_QUAD_ENABLE		/* Enable qspi to do quad transfer. */

#endif	/* __CONFIG_H */
