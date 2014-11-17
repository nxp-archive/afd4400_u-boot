/*
 * Copyright (C) 2013-2014 Freescale Semiconductor, Inc.
 *
 * Configuration settings Spansion S25FL qspi flash device.
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

#ifndef __QSPI_SPANSION_S25FL_CONFIG_H
#define __QSPI_SPANSION_S25FL_CONFIG_H

#include <asm/arch/imx-regs.h>

#define CLEAR_BUFPTR_NONE 0
#define CLEAR_BUFPTR_RX   1
#define CLEAR_BUFPTR_TX	  2
#define CLEAR_BUFPTR_RXTX 3

#define SPANSION_S25FL_CMD_READ_ID			0x90
#define SPANSION_S25FL_CMD_READ_ID_JEDEC		0x9F
#define SPANSION_S25FL_CMD_READ_ELEC_SIG		0xAB
#define SPANSION_S25FL_CMD_READ_SR1			0x05
#define SPANSION_S25FL_CMD_READ_SR2			0x07
#define SPANSION_S25FL_CMD_READ_CR1			0x35
#define SPANSION_S25FL_CMD_WRITE_REG			0x01
#define SPANSION_S25FL_CMD_CLEAR_SR1			0x30  /* Clears erase/prog failure status */
#define SPANSION_S25FL_CMD_WRITE_DISABLE		0x04
#define SPANSION_S25FL_CMD_WRITE_ENABLE			0x06
#define SPANSION_S25FL_CMD_BANKREG_READ			0x16
#define SPANSION_S25FL_CMD_BANKREG_WRITE		0x17
#define SPANSION_S25FL_CMD_PAGE_PROGRAM			0x02  /* 24/32 bit addressing */
#define SPANSION_S25FL_CMD_PAGE_PROGRAM_QUAD		0x32  /* 24/32 bit addressing */
#define SPANSION_S25FL_CMD_READ				0x03  /* 24/32 bit addressing */
#define SPANSION_S25FL_CMD_READ_QUAD			0x6B  /* 24/32 bit addressing */
#define SPANSION_S25FL_CMD_BULK_ERASE			0x60
#define SPANSION_S25FL_CMD_BULK_ERASE2			0xC7
#define SPANSION_S25FL_CMD_SECTOR_ERASE			0xD8 /* 24/32 bit addressing */

#define SPANSION_SECTOR_SIZE_BYTES	(256 * 1024)
#define SPANSION_PAGE_SIZE_BYTES	(256)

#define QSPI_SPANSION_FLASH_SEND_CMD(ico_value, cmd_value, clear_bufptr)\
	{\
		if (clear_bufptr == CLEAR_BUFPTR_RXTX)\
			QSPI_CLEAR_RX_TX_BUFPTR();\
		if (clear_bufptr == CLEAR_BUFPTR_RX)\
			QSPI_CLEAR_RX_BUFPTR();\
		if (clear_bufptr == CLEAR_BUFPTR_TX)\
			QSPI_CLEAR_TX_BUFPTR();\
		*((volatile u32*)QSPI_ICR_REG) =\
			 (ico_value << QSPI_ICR_ICO_OFFSET) | (cmd_value << QSPI_ICR_IC_OFFSET);\
		QSPI_WAIT_WHILE_BUSY();\
	}
/* Wait for done status.  Note that the WIP (device busy) bit of SR1 reg is NOT reliable
   way to tell if the device is busy.  It works at 50MHz but not at lower frequencies.  
   Instead, the WEL (write enable latch) bit of SR1 reg is used.  This bit is set at the
   before a write or erase operation and clears when the operation is complete.  It works 
   at all frequencies. The register value is located in b[31:24] of the D4400 qspi RX word 
   register.
*/
#define SPANSION_WAIT_FOR_WRITEERASE_DONE()\
	{\
		do {\
			QSPI_CLEAR_RX_TX_BUFPTR();\
			QSPI_SPANSION_FLASH_SEND_CMD(1, SPANSION_S25FL_CMD_READ_SR1, CLEAR_BUFPTR_RXTX);\
		} while (*((volatile u32*)QSPI_RXDATA_RBDR0) & 0x02000000);\
	}
#define QSPI_SPANSION_FLASH_WRITE(address, num_bytes)\
	{\
		*((volatile u32*)QSPI_SFAR_REG) = address;\
		QSPI_SPANSION_FLASH_SEND_CMD(0, SPANSION_S25FL_CMD_WRITE_ENABLE, CLEAR_BUFPTR_NONE);\
		QSPI_SPANSION_FLASH_SEND_CMD(num_bytes, SPANSION_S25FL_CMD_PAGE_PROGRAM, CLEAR_BUFPTR_NONE)\
		SPANSION_WAIT_FOR_WRITEERASE_DONE()\
	}
#define QSPI_SPANSION_FLASH_WRITE_QUAD(address, num_bytes)\
	{\
		*((volatile u32*)QSPI_SFAR_REG) = address;\
		QSPI_SPANSION_FLASH_SEND_CMD(0, SPANSION_S25FL_CMD_WRITE_ENABLE, CLEAR_BUFPTR_NONE);\
		QSPI_SPANSION_FLASH_SEND_CMD(num_bytes, SPANSION_S25FL_CMD_PAGE_PROGRAM_QUAD, CLEAR_BUFPTR_NONE)\
		SPANSION_WAIT_FOR_WRITEERASE_DONE()\
	}
#define QSPI_SPANSION_FLASH_READ(address, num_bytes)\
	{\
		*((volatile u32*)QSPI_SFAR_REG) = address;\
		QSPI_SPANSION_FLASH_SEND_CMD(num_bytes, SPANSION_S25FL_CMD_READ, CLEAR_BUFPTR_RX)\
	}
#define QSPI_SPANSION_FLASH_READ_QUAD(address, num_bytes)\
	{\
		*((volatile u32*)QSPI_SFAR_REG) = address;\
		QSPI_SPANSION_FLASH_SEND_CMD(num_bytes, SPANSION_S25FL_CMD_READ_QUAD, CLEAR_BUFPTR_RX)\
	}

/* Bulk erase */
#define QSPI_SPANSION_FLASH_BULK_ERASE()\
	{\
		QSPI_SPANSION_FLASH_SEND_CMD(0, SPANSION_S25FL_CMD_WRITE_ENABLE, CLEAR_BUFPTR_NONE);\
		QSPI_SPANSION_FLASH_SEND_CMD(0, SPANSION_S25FL_CMD_BULK_ERASE2, CLEAR_BUFPTR_NONE);\
		SPANSION_WAIT_FOR_WRITEERASE_DONE();\
	}

/* Sector erase */
#define QSPI_SPANSION_FLASH_SECTOR_ERASE(address)\
	{\
		*((volatile u32*)QSPI_SFAR_REG) = address;\
		QSPI_SPANSION_FLASH_SEND_CMD(0, SPANSION_S25FL_CMD_WRITE_ENABLE, CLEAR_BUFPTR_NONE);\
		QSPI_SPANSION_FLASH_SEND_CMD(0, SPANSION_S25FL_CMD_SECTOR_ERASE, CLEAR_BUFPTR_NONE);\
		SPANSION_WAIT_FOR_WRITEERASE_DONE();\
	}
/* Addressing mode
   addr_mode: 0-24bit, 1-32bit

   MSB of TX reg is written first, b[31:24] byte contains new bank reg value.
   Bank reg BRWR b[7] = EXTADD bit.  Set to 1 for 32bit, clear for 24bit addressing.
*/
#define SPANSION_ADDR_MODE_24BIT	0
#define SPANSION_ADDR_MODE_32BIT	1
#define QSPI_SPANSION_FLASH_SET_ADDR_MODE(addr_mode)\
	{\
		QSPI_CLEAR_TX_BUFPTR();\
		if (addr_mode == 0)\
			*((volatile u32*)QSPI_TBDR_REG) = 0x00000000;\
		else\
			*((volatile u32*)QSPI_TBDR_REG) = 0x80000000;\
		QSPI_SPANSION_FLASH_SEND_CMD(1, SPANSION_S25FL_CMD_BANKREG_WRITE, CLEAR_BUFPTR_NONE);\
	}
/* Quad i/o mode
   quad_mode: 0-single I/O, 1- quad I/O 

   Status/config write (status written first, config 2nd):
	b[31:24]- status SR1 reg, b1=1 WEL write enable, written 1st
	b[23:16]- config CR1 reg, b1=1 QUAD i/o enable, written 2nd 
   Quad bit in CR reg is non-volatile and requires wait time for it to be stored.
*/
#define SPANSION_IO_MODE_SINGLE		0
#define SPANSION_IO_MODE_QUAD		1
#define QSPI_SPANSION_FLASH_SET_QUADMODE(quad_mode)\
	{\
		QSPI_SPANSION_FLASH_SEND_CMD(0, SPANSION_S25FL_CMD_WRITE_ENABLE, CLEAR_BUFPTR_NONE);\
		QSPI_CLEAR_TX_BUFPTR();\
		if (quad_mode == 0)\
			*((volatile u32*)QSPI_TBDR_REG) = 0x02000000;\
		else\
			*((volatile u32*)QSPI_TBDR_REG) = 0x02020000;\
		QSPI_SPANSION_FLASH_SEND_CMD(2, SPANSION_S25FL_CMD_WRITE_REG, CLEAR_BUFPTR_NONE);\
		SPANSION_WAIT_FOR_WRITEERASE_DONE();\
	}

#endif /* __QSPI_SPANSION_S25FL_CONFIG_H */
