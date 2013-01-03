/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Partha Hazra <b43678@freescale.com>
 *
 * Configuration settings for the Freescale i.MX6Q Armadillo2 board.
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

#define CONFIG_MX6
#define CONFIG_MX6Q

#include "mx6_common.h"

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

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART4_BASE

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_FSL_USDHC_NUM	2

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

/*WEIM-NOR*/
#define CONFIG_CMD_WEIM_NOR
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define	CONFIG_FEC_MXC
#define	CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define	CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_FEC_MXC_PHYADDR		0

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY		3

#define CONFIG_LOADADDR			0x10800000
#define CONFIG_SYS_TEXT_BASE		0x17800000

#ifdef CONFIG_MEDUSA_BOARD

#define CONFIG_EXTRA_ENV_SETTINGS \
	"uimage=uImage\0" \
	"console=ttymxc3\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"bootargs_all=setenv bootargs root=/dev/mtdblock4 "\
		"rootfstype=jffs2 rw console=ttymxc3,115200 earlyprintk\0" \
	"bootcmd_nor=echo Booting from nor flash ...; " \
		"run bootargs_all; " \
		"cp.b 0x080C0000 ${loadaddr} 0x400000;" \
		"cp.b 0x0ACC0000 0x11200000 0x20000;" \
		"bootm ${loadaddr} - 0x11200000 \0"

#define CONFIG_BOOTCOMMAND \
	"run bootcmd_nor;"

#else

#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"uimage=uImage\0" \
	"console="CONFIG_CONSOLE_DEV"\0" \
	"fdt_high=0xffffffff\0"   \
	"initrd_high=0xffffffff\0" \
	"mmcdev=0\0" \
	"mmcpart=1\0" \
	"mmcroot=/dev/mmcblk1p2 rootwait rw\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot}\0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loaduimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${uimage}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"bootm\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
	"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
		"netboot=echo Booting from net ...; " \
		"run netargs; " \
	"dhcp ${uimage}; bootm\0" \

#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev};" \
	"if mmc rescan ${mmcdev}; then " \
		"if run loadbootscript; then " \
			"run bootscript; " \
		"else " \
			"if run loaduimage; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \
		"fi; " \
	"else run netboot; fi"

#endif

#define CONFIG_ARP_TIMEOUT	200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT		"MX6qarm2-Medusa U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		256

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		0x10010000

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE			(2u * 1024 * 1024 * 1024)

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/*
 ** WEIM NOR Config
*/
#ifdef CONFIG_CMD_WEIM_NOR
	#define CONFIG_SYS_FLASH_CFI
	#define CONFIG_FLASH_CFI_DRIVER
	#define CONFIG_SYS_FLASH_BASE	0x08000000
	#define CONFIG_SYS_FLASH_SIZE	0x08000000
	#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE
	#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
	#define CONFIG_SYS_FLASH_BANKS_LIST	{CONFIG_SYS_FLASH_BASE}
	#define CONFIG_SYS_MAX_FLASH_BANKS	1
	#define CONFIG_SYS_MAX_FLASH_SECT	1024
	#define CONFIG_SYS_FLASH_PROTECTION
#endif


/* FLASH and environment organization */
#ifndef CONFIG_CMD_WEIM_NOR
	#define CONFIG_SYS_NO_FLASH
#else
	#define CONFIG_SYS_MONITOR_BASE	CONFIG_SYS_FLASH_BASE
	#define CONFIG_SYS_MONITOR_LEN	0x80000

	#define CONFIG_ENV_SECT_SIZE	0x00020000   /*128KiB sector size */
	#define CONFIG_ENV_SIZE		CONFIG_ENV_SECT_SIZE

	#define CONFIG_ENV_OFFSET	(8 * 64 * 1024)
	#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + \
							CONFIG_ENV_SIZE)
	#define CONFIG_ENV_SIZE_REDUND	CONFIG_ENV_SIZE
	#define CONFIG_ENV_ADDR		(CONFIG_SYS_MONITOR_BASE + 0x0080000)
	#define CONFIG_FSL_ENV_IN_FLASH
#endif

#ifdef CONFIG_FSL_ENV_IN_FLASH
	#define CONFIG_ENV_IS_IN_FLASH          1
#else
	#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
	#define CONFIG_ENV_SIZE			(8 * 1024)
	#define CONFIG_ENV_IS_IN_MMC
	#define CONFIG_SYS_MMC_ENV_DEV		1
#endif

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#endif				/* __CONFIG_H */
