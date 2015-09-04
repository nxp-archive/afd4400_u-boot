/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ASM_ARCH_D4400_REGS_H__
#define __ASM_ARCH_D4400_REGS_H__

#define ARCH_D4400

#define CONFIG_SYS_CACHELINE_SIZE	32

#define ROMCP_BASE1_ADDR		0x00000000
#define ROMCP_END1_ADDR			0x00003FFF
#define ROMCP_BASE2_ADDR		0x00404000
#define ROMCP_END2_ADDR			0x00433FFF

#define AIPS1_BASE_ADDR			0x01000000
#define AIPS1_END_ADDR			0x011FFFFF
#define AIPS2_BASE_ADDR			0x01200000
#define AIPS2_END_ADDR			0x03FFFFFF
#define AIPS3_BASE_ADDR			0x04000000
#define AIPS3_END_ADDR			0x05FFFFFF

#define QSPI_FLASH_BASE_ADDR		0x08000000
#define QSPI_FLASH_END_ADDR		0x0FFFFFFF
#define OCRAM_BASE_ADDR			0x10000000
#define OCRAM_END_ADDR			0x1000FFFF
#define L2_CACHE_BASE_ADDR		0x10900000
#define L2_CACHE_END_ADDR		0x10903FFF
#define WEIM_BASE_ADDR			0x30000000
#define WEIM_END_ADDR			0x4FFFFFFF
#define OCRAM1_BASE_ADDR		0x60000000
#define OCRAM1_END_ADDR			0x6000FFFF
#define OCRAM2_BASE_ADDR		0x61000000
#define OCRAM2_END_ADDR			0x6100FFFF
#define OCRAM3_BASE_ADDR		0x70000000
#define OCRAM3_END_ADDR			0x7000FFFF
#define OCRAM4_BASE_ADDR		0x71000000
#define OCRAM4_END_ADDR			0x7100FFFF
#define MMDC_BASE_ADDR			0x90000000
#define MMDC_END_ADDR			0xEFFFFFFF
#define OCRAM5_BASE_ADDR		0xF0000000
#define OCRAM5_END_ADDR			0xF000FFFF
#define OCRAM6_BASE_ADDR		0xF1000000
#define OCRAM6_END_ADDR			0xF100FFFF

/* Defines for Blocks connected via AIPS */
#define ATZ1_BASE_ADDR		AIPS1_BASE_ADDR
#define ATZ2_BASE_ADDR		AIPS2_BASE_ADDR
#define ATZ3_BASE_ADDR		AIPS3_BASE_ADDR

#define AIPS1_ON_BASE_ADDR	(ATZ1_BASE_ADDR)
#define AIPS1_OFF_BASE_ADDR	(ATZ1_BASE_ADDR + 0x80000)

#define MMDC_CTRL_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x0000)
#define SERDES2_BASE_ADDR       (AIPS1_OFF_BASE_ADDR + 0x8000)
#define SRC_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0xC000)
#define CCM_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x14000)
#define WEIM_CTRL_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x18000)
#define ROMCP_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x1C000)
#define WDOG_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x20000)
#define ECSPI1_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x24000)
#define ECSPI2_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x28000)
#define ECSPI3_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x2C000)
#define ECSPI4_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x30000)
#define ECSPI5_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x34000)
#define ECSPI6_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x38000)
#define ECSPI7_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x3C000)
#define ECSPI8_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x40000)
#define I2C1_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x44000)
#define I2C2_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x48000)
#define I2C3_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x4C000)
#define I2C4_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x50000)
#define GPIO1_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x54000)
#define GPIO2_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x58000)
#define GPIO3_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x5C000)
#define GPIO4_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x60000)
#define GPIO5_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x64000)
#define EPIT_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x68000)
#define UART1_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x6C000)
#define UART2_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x70000)
#define UART3_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x74000)
#define UART4_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x78000)

#define AIPS2_ON_BASE_ADDR	(ATZ2_BASE_ADDR)
#define AIPS2_OFF_BASE_ADDR	(ATZ2_BASE_ADDR + 0x80000)

#define SCM_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x40000)
#define I2C5_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x5C000)
#define I2C6_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x60000)
#define I2C7_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x64000)
#define I2C8_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x68000)
#define I2C9_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x6C000)
#define I2C10_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x70000)
#define I2C11_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x74000)
#define IIM_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x78000)
#define QSPI_BASE_ADDR		(AIPS3_BASE_ADDR + 0x0A8000)

#define AIPS3_ON_BASE_ADDR	(ATZ3_BASE_ADDR)
#define AIPS3_OFF_BASE_ADDR	(ATZ3_BASE_ADDR + 0x80000)

#define VETSEC0_GROUP0_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x0000)
#define VETSEC1_GROUP0_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x1000)
#define VETSEC0_MDIO_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x4000)
#define IOMUXC_DDR_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0xC000)
#define GPC_BASE_ADDR		(AIPS3_OFF_BASE_ADDR + 0x18000)
#define IOMUXC_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x1C000)
#define VETSEC0_GROUP1_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x20000)
#define VETSEC1_GROUP1_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x21000)
#define VETSEC1_MDIO_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x24000)

#define WEIM_CONFIGURATION_REG               (WEIM_CTRL_BASE_ADDR + 0x90)
#define WEIM_CS0_GENERAL_CONFIGURATION_REG_1 (WEIM_CTRL_BASE_ADDR + 0x00)
#define WEIM_CS0_GENERAL_CONFIGURATION_REG_2 (WEIM_CTRL_BASE_ADDR + 0x04)
#define WEIM_CS0_READ_CONFIGURATION_REG_1    (WEIM_CTRL_BASE_ADDR + 0x08)
#define WEIM_CS0_READ_CONFIGURATION_REG_2    (WEIM_CTRL_BASE_ADDR + 0x0C)
#define WEIM_CS0_WRITE_CONFIGURATION_REG_1   (WEIM_CTRL_BASE_ADDR + 0x10)
#define WEIM_CS0_WRITE_CONFIGURATION_REG_2   (WEIM_CTRL_BASE_ADDR + 0x14)
#define WEIM_CS1_GENERAL_CONFIGURATION_REG_1 (WEIM_CTRL_BASE_ADDR + 0x18)
#define WEIM_CS1_GENERAL_CONFIGURATION_REG_2 (WEIM_CTRL_BASE_ADDR + 0x1C)
#define WEIM_CS1_READ_CONFIGURATION_REG_1    (WEIM_CTRL_BASE_ADDR + 0x20)
#define WEIM_CS1_READ_CONFIGURATION_REG_2    (WEIM_CTRL_BASE_ADDR + 0x24)
#define WEIM_CS1_WRITE_CONFIGURATION_REG_1   (WEIM_CTRL_BASE_ADDR + 0x28)
#define WEIM_CS1_WRITE_CONFIGURATION_REG_2   (WEIM_CTRL_BASE_ADDR + 0x2C)
#define WEIM_CS2_GENERAL_CONFIGURATION_REG_1 (WEIM_CTRL_BASE_ADDR + 0x30)
#define WEIM_CS2_GENERAL_CONFIGURATION_REG_2 (WEIM_CTRL_BASE_ADDR + 0x34)
#define WEIM_CS2_READ_CONFIGURATION_REG_1    (WEIM_CTRL_BASE_ADDR + 0x38)
#define WEIM_CS2_READ_CONFIGURATION_REG_2    (WEIM_CTRL_BASE_ADDR + 0x3C)
#define WEIM_CS2_WRITE_CONFIGURATION_REG_1   (WEIM_CTRL_BASE_ADDR + 0x40)
#define WEIM_CS2_WRITE_CONFIGURATION_REG_2   (WEIM_CTRL_BASE_ADDR + 0x44)
#define GCR_51_CONFIG_REG		     (SCM_BASE_ADDR + 0xE8)
#define GCR_0_CONFIG_REG          (SCM_BASE_ADDR + 0x0)

#define GCR0_SYS_PLL_OFFSET     19
#define GCR0_SYS_PLL_MASK       0x3
#define GCR0_DDR_PLL_OFFSET     21
#define GCR0_DDR_PLL_MASK       0x3

#define SRC_SBMR_REG			(SRC_BASE_ADDR + 0x00)
#define SRC_SBMR_BMOD_OFFSET		0
#define SRC_SBMR_BMOD_MASK		(0x3 << SRC_SBMR_BMOD_OFFSET)
#define SRC_SBMR_SER_DL_SEL_OFFSET	3
#define SRC_SBMR_SER_DL_SEL_MASK	(1 << SRC_SBMR_SER_DL_SEL_OFFSET)
#define SRC_SBMR_MEM_TYPE_OFFSET	4
#define SRC_SBMR_MEM_TYPE_MASK		(1 << SRC_SBMR_MEM_TYPE_OFFSET)
#define SRC_SBMR_MEM_BUS_WIDTH_OFFSET	5
#define SRC_SBMR_MEM_BUS_WIDTH_MASK	(0x3 << SRC_SBMR_MEM_BUS_WIDTH_OFFSET)
#define SRC_SBMR_ETH_MODE_OFFSET	10
#define SRC_SBMR_ETH_MODE_MASK		(0x3 << SRC_SBMR_ETH_MODE_OFFSET)
#define SRC_SBMR_BIV_EN_OFFSET		12
#define SRC_SBMR_BIV_EN_MASK		(1 << SRC_SBMR_BIV_EN_OFFSET)
#define SRC_SBMR_SPI_FLASH_VENID_OFFSET	13
#define SRC_SBMR_SPI_FLASH_VENID_MASK	(0x3 << SRC_SBMR_SPI_FLASH_VENID_OFFSET)
#define SRC_SBMR_REF_FREQ_OFFSET	15
#define SRC_SBMR_REF_FREQ_OFFSET_MASK	(1 << SRC_SBMR_REF_FREQ_OFFSET)

/* Check memory type of flash boot:
 *  FLASH_BOOT_MEM_TYPE_NOR	- NOR flash
 *  FLASH_BOOT_MEM_TYPE_QSPI	- qspi flash
 */
#define FLASH_BOOT_MEM_TYPE_NOR		0
#define FLASH_BOOT_MEM_TYPE_QSPI	1
#define QUERY_FLASH_BOOT_MEM_TYPE()\
(\
	((*(volatile u32 *)SRC_SBMR_REG & SRC_SBMR_MEM_TYPE_MASK) >> \
		SRC_SBMR_MEM_TYPE_OFFSET)\
)

/* Check qspi flash type:
 *  QSPI_FLASH_TYPE_WINBOND - Winbond
 *  QSPI_FLASH_TYPE_SPANSION- Spansion
 *  QSPI_FLASH_TYPE_MACRONIX- Macronix
 *  QSPI_FLASH_TYPE_NUMONYX - Numonyx
 */
#define QSPI_FLASH_TYPE_WINBOND		0
#define QSPI_FLASH_TYPE_SPANSION	1
#define QSPI_FLASH_TYPE_MACRONIX	2
#define QSPI_FLASH_TYPE_NUMONYX		3
#define QUERY_QSPI_FLASH_TYPE()\
(\
	((*(volatile u32 *)SRC_SBMR_REG & SRC_SBMR_SPI_FLASH_VENID_MASK) >> \
		SRC_SBMR_SPI_FLASH_VENID_OFFSET)\
)

/* Check boot mode:
 *  BOOT_MODE_FLASH		- flash (NOR/Qspi)
 *  BOOT_MODE_IPC		- IPC boot
 *  BOOT_MODE_EXT_DIRECT	- External direct boot
 *  BOOT_MODE_TEST_MODE		- Test mode
 */
#define BOOT_MODE_FLASH		0
#define BOOT_MODE_IPC		1
#define BOOT_MODE_EXT_DIRECT	2
#define BOOT_MODE_TEST_MODE	3
#define QUERY_BOOT_MODE()\
(\
	((*(volatile u32 *)SRC_SBMR_REG & SRC_SBMR_BMOD_MASK) >> \
		SRC_SBMR_BMOD_OFFSET)\
)

/*
 *  QSPI register definitions
 */
#define QSPI_MCR_REG		        (QSPI_BASE_ADDR + 0x000)
#define QSPI_LCR_REG		        (QSPI_BASE_ADDR + 0x004)
#define QSPI_SFAR_REG		        (QSPI_BASE_ADDR + 0x100)
#define QSPI_ICR_REG		        (QSPI_BASE_ADDR + 0x104)
#define QSPI_SMPR_REG		        (QSPI_BASE_ADDR + 0x108)
#define QSPI_RBSR_REG		        (QSPI_BASE_ADDR + 0x10C)
#define QSPI_RBCT_REG		        (QSPI_BASE_ADDR + 0x110)
#define QSPI_TBSR_REG		        (QSPI_BASE_ADDR + 0x150)
#define QSPI_TBDR_REG		        (QSPI_BASE_ADDR + 0x154) /* AFD4400 rev1 has 16 32-bit deep TX buffer */
#define QSPI_ACR_REG		        (QSPI_BASE_ADDR + 0x158)
#define QSPI_SR_REG		        (QSPI_BASE_ADDR + 0x15C)
#define QSPI_FR_REG		        (QSPI_BASE_ADDR + 0x160)
#define QSPI_RSER_REG		        (QSPI_BASE_ADDR + 0x164)
#define QSPI_RXDATA_BASE_REG		(QSPI_BASE_ADDR + 0x200)
#define QSPI_RXDATA_RBDR0		(QSPI_BASE_ADDR + 0x200)
#define QSPI_RXDATA_RBDR1		(QSPI_BASE_ADDR + 0x204)
#define QSPI_RXDATA_RBDR2		(QSPI_BASE_ADDR + 0x208)
#define QSPI_RXDATA_RBDR3		(QSPI_BASE_ADDR + 0x20C)
#define QSPI_RXDATA_RBDR4		(QSPI_BASE_ADDR + 0x210)
#define QSPI_RXDATA_RBDR5		(QSPI_BASE_ADDR + 0x214)
#define QSPI_RXDATA_RBDR6		(QSPI_BASE_ADDR + 0x218)
#define QSPI_RXDATA_RBDR7		(QSPI_BASE_ADDR + 0x21C)
#define QSPI_RXDATA_RBDR8		(QSPI_BASE_ADDR + 0x220)
#define QSPI_RXDATA_RBDR9		(QSPI_BASE_ADDR + 0x224)
#define QSPI_RXDATA_RBDR10		(QSPI_BASE_ADDR + 0x228)
#define QSPI_RXDATA_RBDR11		(QSPI_BASE_ADDR + 0x22C)
#define QSPI_RXDATA_RBDR12		(QSPI_BASE_ADDR + 0x230)
#define QSPI_RXDATA_RBDR13		(QSPI_BASE_ADDR + 0x234)
#define QSPI_RXDATA_RBDR14		(QSPI_BASE_ADDR + 0x238)
#define QSPI_RXDATA_RBDR15		(QSPI_BASE_ADDR + 0x23C)
#define QSPI_RXDATA_RBDR16		(QSPI_BASE_ADDR + 0x240)
#define QSPI_RXDATA_RBDR17		(QSPI_BASE_ADDR + 0x244)
#define QSPI_RXDATA_RBDR18		(QSPI_BASE_ADDR + 0x248)
#define QSPI_RXDATA_RBDR19		(QSPI_BASE_ADDR + 0x24C)
#define QSPI_RXDATA_RBDR20		(QSPI_BASE_ADDR + 0x250)
#define QSPI_RXDATA_RBDR21		(QSPI_BASE_ADDR + 0x254)
#define QSPI_RXDATA_RBDR22		(QSPI_BASE_ADDR + 0x258)
#define QSPI_RXDATA_RBDR23		(QSPI_BASE_ADDR + 0x25C)
#define QSPI_RXDATA_RBDR24		(QSPI_BASE_ADDR + 0x260)
#define QSPI_RXDATA_RBDR25		(QSPI_BASE_ADDR + 0x264)
#define QSPI_RXDATA_RBDR26		(QSPI_BASE_ADDR + 0x268)
#define QSPI_RXDATA_RBDR27		(QSPI_BASE_ADDR + 0x26C)
#define QSPI_RXDATA_RBDR28		(QSPI_BASE_ADDR + 0x270)
#define QSPI_RXDATA_RBDR29		(QSPI_BASE_ADDR + 0x274)
#define QSPI_RXDATA_RBDR30		(QSPI_BASE_ADDR + 0x278)
#define QSPI_RXDATA_RBDR31		(QSPI_BASE_ADDR + 0x27C)

#define QSPI_MCR_EXT_ADD_OFFSET		0
#define QSPI_MCR_EXT_ADD_MASK		(1 << QSPI_MCR_EXT_ADD_OFFSET)
#define QSPI_MCR_VMID_OFFSET		3
#define QSPI_MCR_VMID_MASK		(0xF << QSPI_MCR_VMID_OFFSET)
#define QSPI_MCR_CLR_RXF_OFFSET		10
#define QSPI_MCR_CLR_RXF_MASK		(1 << QSPI_MCR_CLR_RXF_OFFSET)
#define QSPI_MCR_CLR_TXF_OFFSET		11
#define QSPI_MCR_CLR_TXF_MASK		(1 << QSPI_MCR_CLR_TXF_OFFSET)
#define QSPI_MCR_MDIS_OFFSET		14
#define QSPI_MCR_MDIS_MASK		(1 << QSPI_MCR_MDIS_OFFSET)
#define QSPI_MCR_ISD2FA_OFFSET		16
#define QSPI_MCR_ISD2FA_MASK		(1 << QSPI_MCR_ISD2FA_OFFSET)
#define QSPI_MCR_ISD3FA_OFFSET		17
#define QSPI_MCR_ISD3FA_MASK		(1 << QSPI_MCR_ISD3FA_OFFSET)
#define QSPI_MCR_ISD2FB_OFFSET		18
#define QSPI_MCR_ISD2FB_MASK		(1 << QSPI_MCR_ISD2FB_OFFSET)
#define QSPI_MCR_ISD3FB_OFFSET		19
#define QSPI_MCR_ISD3FB_MASK		(1 << QSPI_MCR_ISD3FB_OFFSET)
#define QSPI_MCR_SCLKCFG_OFFSET		24
#define QSPI_MCR_SCLKCFG_MASK		(0xFF << QSPI_MCR_SCLKCFG_OFFSET)

#define QSPI_ICR_IC_OFFSET		0
#define QSPI_ICR_ICO_OFFSET		16  /* Errata in AFD4400 ref manual which says 8. */

#define QSPI_TBSR_TRBFL_OFFSET		8
#define QSPI_TBSR_TRBFL_MASK		(0x0F << QSPI_TBSR_TRBFL_OFFSET)
#define QSPI_TBSR_TRCTR_OFFSET		16
#define QSPI_TBSR_TRCTR_MASK		(0xFF << QSPI_TBSR_TRCTR_OFFSET)

#define QSPI_SMPR_HSENA_OFFSET		0
#define QSPI_SMPR_HSENA_MASK		(1 << QSPI_SMPR_HSENA_OFFSET)
#define QSPI_SMPR_HSPHS_OFFSET		1
#define QSPI_SMPR_HSPHS_MASK		(1 << QSPI_SMPR_HSDLY_OFFSET)
#define QSPI_SMPR_HSDLY_OFFSET		2
#define QSPI_SMPR_HSDLY_MASK		(1 << QSPI_SMPR_HSDLY_OFFSET)
#define QSPI_SMPR_FSPHS_OFFSET		5
#define QSPI_SMPR_FSPHS_MASK		(1 << QSPI_SMPR_FSPHS_OFFSET)
#define QSPI_SMPR_FSDLY_OFFSET		6
#define QSPI_SMPR_FSDLY_MASK		(1 << QSPI_SMPR_FSDLY_OFFSET)

#define QSPI_SR_BUSY_OFFSET		0
#define QSPI_SR_BUSY_MASK		(1 << QSPI_SR_BUSY_OFFSET)
#define QSPI_SR_IP_ACC_OFFSET		1
#define QSPI_SR_IP_ACC_MASK		(1 << QSPI_SR_IP_ACC_OFFSET)
#define QSPI_SR_AHB_ACC_OFFSET		2
#define QSPI_SR_AHB_ACC_MASK		(1 << QSPI_SR_ABH_ACC_OFFSET)
#define QSPI_SR_CNTMDFA_OFFSET		3
#define QSPI_SR_CNTMDFA_MASK		(1 << QSPI_SR_CNTMDFA_OFFSET)
#define QSPI_SR_AHBGNT_OFFSET		5
#define QSPI_SR_AHBGNT_MASK		(1 << QSPI_SR_AHBGNT_OFFSET)
#define QSPI_SR_AHBTRN_OFFSET		6
#define QSPI_SR_AHBTRN_MASK		(1 << QSPI_SR_AHBTRN_OFFSET)
#define QSPI_SR_AHBNE_OFFSET		8
#define QSPI_SR_AHBNE_MASK		(1 << QSPI_SR_AHBNE_OFFSET)
#define QSPI_SR_AHBFULL_OFFSET		11
#define QSPI_SR_AHBFULL_MASK		(1 << QSPI_SR_AHBFULL_OFFSET)
#define QSPI_SR_RXWE_OFFSET		16
#define QSPI_SR_RXWE_MASK		(1 << QSPI_SR_RXWE_OFFSET)
#define QSPI_SR_RXFULL_OFFSET		19
#define QSPI_SR_RXFULL_MASK		(1 << QSPI_SR_RXFULL_OFFSET)
#define QSPI_SR_RXDMA_OFFSET		23
#define QSPI_SR_RXDMA_MASK		(1 << QSPI_SR_RXDMA_OFFSET)
#define QSPI_SR_TXNE_OFFSET		24
#define QSPI_SR_TXNE_MASK		(1 << QSPI_SR_TXNE_OFFSET)
#define QSPI_SR_TXFULL_OFFSET		27
#define QSPI_SR_TXFULL_MASK		(1 << QSPI_SR_TXFULL_OFFSET)

#define QSPI_ACR_ARIC_OFFSET		0
#define QSPI_ACR_ARIC_MASK		(0xFF << QSPI_ACR_ARIC_OFFSET)
#define QSPI_ACR_ARSZ_OFFSET		11
#define QSPI_ACR_ARSZ_MASK		(0x1F << QSPI_ACR_ARSZ_OFFSET)
#define QSPI_ACR_ARMB_OFFSET		16
#define QSPI_ACR_ARMB_MASK		(0xFF << QSPI_ACR_ARMB_OFFSET)

#define	QSPI_TX_FIFO_SIZE	64	/* fifo size, 16 32-bit words */
#define QSPI_RX_FIFO_SIZE	128	/* fifo size, 32 32-bit words */

#define QSPI_SET_AMBA_READ_OPCODE(opcode) \
{\
	*((volatile u32*)QSPI_ACR_REG) &= ~QSPI_ACR_ARIC_MASK;\
	*((volatile u32*)QSPI_ACR_REG) |= \
		((opcode & 0xff) << QSPI_ACR_ARIC_OFFSET);\
}

#define QSPI_WAIT_WHILE_BUSY() \
{\
	while (*((volatile u32*)QSPI_SR_REG) & QSPI_SR_BUSY_MASK ) {}\
}
#define QSPI_CLEAR_RX_BUFPTR() \
{\
	*((volatile u32*)QSPI_MCR_REG) |= QSPI_MCR_CLR_RXF_MASK;\
}
#define QSPI_CLEAR_TX_BUFPTR() \
{\
	*((volatile u32*)QSPI_MCR_REG) |= QSPI_MCR_CLR_TXF_MASK;\
}
#define QSPI_CLEAR_RX_TX_BUFPTR() \
{\
	*((volatile u32*)QSPI_MCR_REG) |= \
		(QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK);\
}
#define QSPI_SET_ADDR(address) \
	*((volatile u32*)QSPI_SFAR_REG) = address;

/* Addressing mode
 * addr_mode: QSPI_ADDR_MODE_24BIT-24bit, QSPI_ADDR_MODE_32BIT-32bit
 */
#define QSPI_ADDR_MODE_24BIT	0
#define QSPI_ADDR_MODE_32BIT	1
#define QSPI_SET_ADDR_MODE(addr_mode) \
{\
	*((volatile u32*)QSPI_MCR_REG) |= QSPI_MCR_MDIS_MASK;\
	if (addr_mode == 0)\
		*((volatile u32*)QSPI_MCR_REG) &= ~QSPI_MCR_EXT_ADD_MASK;\
	else\
		*((volatile u32*)QSPI_MCR_REG) |= QSPI_MCR_EXT_ADD_MASK;\
	*((volatile u32*)QSPI_MCR_REG) &= ~QSPI_MCR_MDIS_MASK;\
}

#define CLEAR_BUFPTR_NONE 0
#define CLEAR_BUFPTR_RX   1
#define CLEAR_BUFPTR_TX	  2
#define CLEAR_BUFPTR_RXTX 3

#define QSPI_SEND_CMD(ico_value, cmd_value, clear_bufptr)\
	{\
		if (clear_bufptr == CLEAR_BUFPTR_RXTX)\
			QSPI_CLEAR_RX_TX_BUFPTR();\
		if (clear_bufptr == CLEAR_BUFPTR_RX)\
			QSPI_CLEAR_RX_BUFPTR();\
		if (clear_bufptr == CLEAR_BUFPTR_TX)\
			QSPI_CLEAR_TX_BUFPTR();\
		*((volatile u32*)QSPI_ICR_REG) =\
			(ico_value << QSPI_ICR_ICO_OFFSET) |\
			(cmd_value << QSPI_ICR_IC_OFFSET);\
		QSPI_WAIT_WHILE_BUSY();\
	}

/* For SET_QSPI_FLASH_TYPE() and GET_QSPI_FLASH_TYPE(), use the
 * the qspi type defined above for the SBMR register:
 *    QSPI_FLASH_TYPE_WINBOND  = 0  (1 in qspi MCR reg)
 *    QSPI_FLASH_TYPE_SPANSION = 1  (2 in qspi MCR reg)
 *    QSPI_FLASH_TYPE_MACRONIX = 2  (3 in qspi MCR reg)
 *    QSPI_FLASH_TYPE_NUMONYX  = 3  (4 in qspi MCR reg)
 *
 *  The qspi flash type from the SBMR reg is zero based whereas
 *  the type from the qspi MCR reg starts at 1.  So there's a +1
 */
#define SET_QSPI_FLASH_TYPE(type)\
{\
	*((volatile u32*)QSPI_MCR_REG) &= ~QSPI_MCR_VMID_MASK;\
	*((volatile u32*)QSPI_MCR_REG) |= ((type+1) << QSPI_MCR_VMID_OFFSET);\
}

#define GET_QSPI_FLASH_TYPE()\
(\
	((*((volatile u32*)QSPI_MCR_REG) & QSPI_MCR_VMID_MASK) >> QSPI_MCR_VMID_OFFSET) - 1\
)

/* The qspi register addresses are broken up hence padding is required. */
#if !(defined(__KERNEL_STRICT_NAMES) || defined(__ASSEMBLY__))
#include <asm/types.h>

/* QSPI registers */
struct qspi_regs {
	u32	mcr;			/* Module configuration */
	u32	lcr;			/* Latency configuration */
	u8	reserved_1[248];	/* Padding */
	u32	sfar;			/* Module configuration */
	u32	icr;			/* Instruction code */
	u32	smpr;			/* Sampling */
	u32	rbsr;			/* RX buffer status */
	u32	rbct;			/* RX buffer control */
	u8	reserved_2[60];		/* Padding */
	u32	tbsr;			/* TX buffer status */
	u32	tbdr;			/* TX buffer data */
	u32	acr;			/* AMBA control */
	u32	sr;			/* Status */
	u32	fr;			/* Flag */
	u32	rser;			/* IRQ/DMA request select/enable */
	u8	reserved_3[152];	/* Padding */
	u32	rbdr_base;		/* RX buffer */
};
#endif /* __ASSEMBLER__*/

#define FVDD_VSEL_REG	(IOMUXC_BASE_ADDR + 0x700)
#define JVDD_VSEL_REG	(IOMUXC_BASE_ADDR + 0x704)
#define GVDD1_VSEL_REG	(IOMUXC_BASE_ADDR + 0x708)
#define GVDD2_VSEL_REG	(IOMUXC_BASE_ADDR + 0x70C)
#define GVDD3_VSEL_REG	(IOMUXC_BASE_ADDR + 0x710)
#define GVDD4_VSEL_REG	(IOMUXC_BASE_ADDR + 0x714)
#define GVDD5_VSEL_REG	(IOMUXC_BASE_ADDR + 0x718)
#define GVDD6_VSEL_REG	(IOMUXC_BASE_ADDR + 0x71C)
#define GVDD7_VSEL_REG	(IOMUXC_BASE_ADDR + 0x720)
#define GVDD8_VSEL_REG	(IOMUXC_BASE_ADDR + 0x724)
#define GVDD9_VSEL_REG	(IOMUXC_BASE_ADDR + 0x728)


#define IRAM_BASE_ADDR	OCRAM_BASE_ADDR
#define IRAM_SIZE	(OCRAM_END_ADDR - OCRAM_BASE_ADDR)
#define IIM_FUSE_BITS_OFFSET	0x800

#define BOOT_ETH_MODE_SGMII 0

#define SRC_SBMR_ETH_MODE_SHIFT 0x0A

/*
 * CSPI register definitions
 */
#define MXC_ECSPI
#define MXC_ECSPI_SEPARATE_CLKS
#define MXC_CSPICTRL_EN         (1 << 0)
#define MXC_CSPICTRL_MODE       (1 << 1)
#define MXC_CSPICTRL_XCH        (1 << 2)
#define MXC_CSPICTRL_MODE_MASK (0xf << 4)
#define MXC_CSPICTRL_CHIPSELECT(x)      (((x) & 0x3) << 12)
#define MXC_CSPICTRL_BITCOUNT(x)        (((x) & 0xfff) << 20)
#define MXC_CSPICTRL_PREDIV(x)  (((x) & 0xF) << 12)
#define MXC_CSPICTRL_POSTDIV(x) (((x) & 0xF) << 8)
#define MXC_CSPICTRL_SELCHAN(x) (((x) & 0x3) << 18)
#define MXC_CSPICTRL_MAXBITS    0xfff
#define MXC_CSPICTRL_TC         (1 << 7)
#define MXC_CSPICTRL_RXOVF      (1 << 6)
#define MXC_CSPIPERIOD_32KHZ    (1 << 15)
#define MAX_SPI_BYTES   32

/* Bit position inside CTRL register to be associated with SS */
#define MXC_CSPICTRL_CHAN       18

/* Bit position inside CON register to be associated with SS */
#define MXC_CSPICON_POL         4
#define MXC_CSPICON_PHA         0
#define MXC_CSPICON_SSPOL       12
#define MXC_SPI_BASE_ADDRESSES \
	ECSPI1_BASE_ADDR, \
	ECSPI2_BASE_ADDR, \
	ECSPI3_BASE_ADDR, \
	ECSPI4_BASE_ADDR, \
	ECSPI5_BASE_ADDR, \
	ECSPI6_BASE_ADDR, \
	ECSPI7_BASE_ADDR, \
	ECSPI8_BASE_ADDR,

#if !(defined(__KERNEL_STRICT_NAMES) || defined(__ASSEMBLY__))
#include <asm/types.h>

/* System Reset Controller (SRC) */
struct src {
	u32	sbmr;
	u32	srsr;
	u32	sscr;
	u32	scrcr;
	u32	srbr;
	u32	sgpr;
};

/* CSPI registers */
struct cspi_regs {
	u32	rxdata;
	u32	txdata;
	u32	ctrl;
	u32	cfg;
	u32	intr;
	u32	dma;
	u32	stat;
	u32	period;
};

struct iim_fuse_t {
	u32 reserved0[0x08/4];
	u32 ext_eth_phy_addr;   /* 0x0008 */
	u32 reserved1[0xAC/4];  /* 0x000C-0x00B4 */
	u32 macaddr5;           /* 0x00B8 */
	u32 macaddr4;
	u32 macaddr3;
	u32 macaddr2;
	u32 macaddr1;
	u32 macaddr0;           /* 0x00CC */
	u32 reserved2[0x54/4];  /* 0x00D0-0x0120*/
	u32 vid1;               /* 0x0124 */
	u32 vid2;               /* 0x0128 */
};

struct iim_regs {
	u32	stat;
	u32	statm;
	u32	err;
	u32	emask;
	u32	fctl;
	u32	ua;
	u32	la;
	u32	sdat;
	u32	prev;
	u32	srev;
	u32	prg_p;
	u32	scs0;
	u32	scs1;
	u32	scs2;
	u32	scs3;
	struct	iim_fuse_t	iim_fuse;
};

struct aipstz_regs {
	u32	mpr1;
	u32	mpr2;
	u32	rsvd1[6];
	u32	pacr1;
	u32	pacr2;
	u32	pacr3;
	u32	pacr4;
	u32	rsvd2[4];
	u32	opacr1;
	u32	opacr2;
	u32	opacr3;
	u32	opacr4;
	u32	opacr5;
};

struct iomuxc_base_regs {
	u32	swmux_ctl;
	u32	gpio_swpad_ctl;
	u32	ddr_swpad_ctl;
};

#endif /* __ASSEMBLER__*/
#endif /* __ASM_ARCH_D4400_REGS_H__ */
