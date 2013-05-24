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
#include <asm/arch/d4400-regs.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 2 * 1024 * 1024)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_D4400_GPIO

#define CONFIG_D4400_UART
#define CONFIG_D4400_UART_PORT		4
#define CONFIG_D4400_UART_BASE		UART4_BASE_ADDR

/* TSEC enable */
#define CONFIG_TSEC_ENET
#if defined(CONFIG_TSEC_ENET)

#define TSEC_DEBUG
#define CONFIG_NET_MULTI

#define CONFIG_MII                      /* MII PHY management */
#define CONFIG_MII_DEFAULT_TSEC 1       /* Allow unregistered phys */

#define TSEC1_FLAGS             (TSEC_GIGABIT | TSEC_REDUCED)
#define CONFIG_TSEC1    1
#define CONFIG_TSEC1_NAME       "eTSEC1"
#define TSEC1_PHY_ADDR          0
#define TSEC1_PHYIDX            0

#define TSEC2_FLAGS             (TSEC_GIGABIT | TSEC_REDUCED)
#define CONFIG_TSEC2    1
#define CONFIG_TSEC2_NAME       "eTSEC2"
#define TSEC2_PHY_ADDR          1
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

/* Allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY		3

#define CONFIG_LOADADDR			0x90800000
#define CONFIG_SYS_TEXT_BASE		0x90002000

#define CONFIG_EXTRA_ENV_SETTINGS\
	"uimage=uImage\0"\
	"fdt_high=0xffffffff\0"\
	"initrd_high=0xffffffff\0"\
	"loadaddr=0x90800000\0"\
	"consoledev=ttyS3\0"\
	"fdtaddr=0x91200000\0"\
	"bootargs_all=setenv bootargs root=/dev/mtdblock4 "\
		"rootfstype=jffs2 rw console=$consoledev,115200 earlyprintk\0" \
	"bootcmd_nor=echo Booting from nor flash ...; " \
		"run bootargs_all; " \
		"cp.b 0x300C0000 $loadaddr 0x400000;" \
		"cp.b 0x32CC0000 $fdtaddr 0x20000;" \
		"bootm $loadaddr - $fdtaddr \0"

#define CONFIG_BOOTCOMMAND \
	"run bootcmd_nor;"

#define CONFIG_ARP_TIMEOUT	200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT		"D4400-EVB U-Boot => "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		256

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START	MMDC_BASE_ADDR
#define CONFIG_SYS_MEMTEST_END		(MMDC_BASE_ADDR + 0x10000)

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

/* WEIM NOR Config */
#ifdef CONFIG_CMD_WEIM_NOR
	#define CONFIG_SYS_FLASH_CFI
	#define CONFIG_FLASH_CFI_DRIVER
	#define CONFIG_SYS_FLASH_BASE		WEIM_BASE_ADDR
	#define CONFIG_SYS_FLASH_SIZE		(WEIM_END_ADDR - WEIM_BASE_ADDR)
	#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE
	#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
	#define CONFIG_SYS_FLASH_BANKS_LIST	{CONFIG_SYS_FLASH_BASE}
	#define CONFIG_SYS_MAX_FLASH_BANKS	1
	#define CONFIG_SYS_MAX_FLASH_SECT	2048
	#define CONFIG_SYS_FLASH_PROTECTION
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
#define CONFIG_CMD_BOOTZ

#define CONFIG_SYS_DCACHE_OFF

#endif	/* __CONFIG_H */
