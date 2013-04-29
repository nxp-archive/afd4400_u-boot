/*
 * (C) Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <common.h>
#include <watchdog.h>
#include <asm/arch/d4400-regs.h>
#include <asm/arch/clock.h>
#include <serial.h>
#include <linux/compiler.h>

#define __REG(x)     (*((volatile u32 *)(x)))

#ifndef CONFIG_D4400_UART_BASE
#error "define CONFIG_D4400_UART_BASE to use the D4400 UART driver"
#endif

#define UART_PHYS	CONFIG_D4400_UART_BASE
/* Register definitions */
#define URXD  0x0  /* Receiver Register */
#define UTXD  0x40 /* Transmitter Register */
#define UCR1  0x80 /* Control Register 1 */
#define UCR2  0x84 /* Control Register 2 */
#define UCR3  0x88 /* Control Register 3 */
#define UCR4  0x8c /* Control Register 4 */
#define UFCR  0x90 /* FIFO Control Register */
#define USR1  0x94 /* Status Register 1 */
#define USR2  0x98 /* Status Register 2 */
#define UESC  0x9c /* Escape Character Register */
#define UTIM  0xa0 /* Escape Timer Register */
#define UBIR  0xa4 /* BRM Incremental Register */
#define UBMR  0xa8 /* BRM Modulator Register */
#define UBRC  0xac /* Baud Rate Count Register */
#define UTS   0xb4 /* UART Test Register */

/* UART Control Register Bit Fields.*/
#define  URXD_CHARRDY    (1<<15)
#define  URXD_ERR        (1<<14)
#define  URXD_OVRRUN     (1<<13)
#define  URXD_FRMERR     (1<<12)
#define  URXD_BRK        (1<<11)
#define  URXD_PRERR      (1<<10)
#define  URXD_RX_DATA    (0xFF)
#define  UCR1_ADEN       (1<<15) /* Auto dectect interrupt */
#define  UCR1_ADBR       (1<<14) /* Auto detect baud rate */
#define  UCR1_TRDYEN     (1<<13) /* Transmitter ready interrupt enable */
#define  UCR1_IDEN       (1<<12) /* Idle condition interrupt */
#define  UCR1_RRDYEN     (1<<9)  /* Recv ready interrupt enable */
#define  UCR1_RDMAEN     (1<<8)  /* Recv ready DMA enable */
#define  UCR1_IREN       (1<<7)  /* Infrared interface enable */
#define  UCR1_TXMPTYEN   (1<<6)  /* Transimitter empty interrupt enable */
#define  UCR1_RTSDEN     (1<<5)  /* RTS delta interrupt enable */
#define  UCR1_SNDBRK     (1<<4)  /* Send break */
#define  UCR1_TDMAEN     (1<<3)  /* Transmitter ready DMA enable */
#define  UCR1_UARTCLKEN  (1<<2)  /* UART clock enabled */
#define  UCR1_DOZE       (1<<1)  /* Doze */
#define  UCR1_UARTEN     (1<<0)  /* UART enabled */
#define  UCR2_ESCI       (1<<15) /* Escape seq interrupt enable */
#define  UCR2_IRTS       (1<<14) /* Ignore RTS pin */
#define  UCR2_CTSC       (1<<13) /* CTS pin control */
#define  UCR2_CTS        (1<<12) /* Clear to send */
#define  UCR2_ESCEN      (1<<11) /* Escape enable */
#define  UCR2_PREN       (1<<8)  /* Parity enable */
#define  UCR2_PROE       (1<<7)  /* Parity odd/even */
#define  UCR2_STPB       (1<<6)  /* Stop */
#define  UCR2_WS         (1<<5)  /* Word size */
#define  UCR2_RTSEN      (1<<4)  /* Request to send interrupt enable */
#define  UCR2_TXEN       (1<<2)  /* Transmitter enabled */
#define  UCR2_RXEN       (1<<1)  /* Receiver enabled */
#define  UCR2_SRST       (1<<0)  /* SW reset */
#define  UCR3_DTREN      (1<<13) /* DTR interrupt enable */
#define  UCR3_PARERREN   (1<<12) /* Parity enable */
#define  UCR3_FRAERREN   (1<<11) /* Frame error interrupt enable */
#define  UCR3_DSR        (1<<10) /* Data set ready */
#define  UCR3_DCD        (1<<9)  /* Data carrier detect */
#define  UCR3_RI         (1<<8)  /* Ring indicator */
#define  UCR3_ADNIMP     (1<<7)  /* Old Autobaud detection method */
#define  UCR3_RXDSEN     (1<<6)  /* Receive status interrupt enable */
#define  UCR3_AIRINTEN   (1<<5)  /* Async IR wake interrupt enable */
#define  UCR3_AWAKEN     (1<<4)  /* Async wake interrupt enable */
#define  UCR3_RXDMUXSEL  (1<<2)  /* RXD MUX Input Select */
#define  UCR3_INVT       (1<<1)  /* Inverted Infrared transmission */
#define  UCR3_ACIEN      (1<<0)  /* Autobaud counter interrupt enable */
#define  UCR4_CTSTL_32   (32<<10) /* CTS trigger level (32 chars) */
#define  UCR4_INVR       (1<<9)  /* Inverted infrared reception */
#define  UCR4_ENIRI      (1<<8)  /* Serial infrared interrupt enable */
#define  UCR4_WKEN       (1<<7)  /* Wake interrupt enable */
#define  UCR4_IDDMAEN    (1<<6)  /* DMA IDLE detect interrupt enable */
#define  UCR4_IRSC       (1<<5)  /* IR special case */
#define  UCR4_LPBYP      (1<<4)  /* Low Power Bypass */
#define  UCR4_TCEN       (1<<3)  /* Transmit complete interrupt enable */
#define  UCR4_BKEN       (1<<2)  /* Break condition interrupt enable */
#define  UCR4_OREN       (1<<1)  /* Receiver overrun interrupt enable */
#define  UCR4_DREN       (1<<0)  /* Recv data ready interrupt enable */
#define  UFCR_RXTL_SHF   0       /* Receiver trigger level shift */
#define  UFCR_RFDIV      (7<<7)  /* Reference freq divider mask */
#define  UFCR_RFDIV_BY_1 (5<<7)  /* Reference freq divider by 5 */
#define  UFCR_TXTL_SHF   10      /* Transmitter trigger level shift */
#define  USR1_PARITYERR  (1<<15) /* Parity error interrupt flag */
#define  USR1_RTSS       (1<<14) /* RTS pin status */
#define  USR1_TRDY       (1<<13) /* Transmitter ready interrupt/dma flag */
#define  USR1_RTSD       (1<<12) /* RTS delta */
#define  USR1_ESCF       (1<<11) /* Escape seq interrupt flag */
#define  USR1_FRAMERR    (1<<10) /* Frame error interrupt flag */
#define  USR1_RRDY       (1<<9)  /* Receiver ready interrupt/dma flag */
#define  USR1_AGTIM      (1<<8)  /* Ageing Timer Interrupt Flag */
#define  USR1_RXDS       (1<<6)  /* Receiver idle interrupt flag */
#define  USR1_AIRINT     (1<<5)  /* Async IR wake interrupt flag */
#define  USR1_AWAKE      (1<<4)  /* Aysnc wake interrupt flag */
#define  USR2_ADET       (1<<15) /* Auto baud rate detect complete */
#define  USR2_TXFE       (1<<14) /* Transmit buffer FIFO empty */
#define  USR2_DTRF       (1<<13) /* DTR edge interrupt flag */
#define  USR2_IDLE       (1<<12) /* Idle condition */
#define  USR2_ACST       (1<<11) /* Auto Baud Counter Stopped */
#define  USR2_IRINT      (1<<8)  /* Serial infrared interrupt flag */
#define  USR2_WAKE       (1<<7)  /* Wake */
#define  USR2_RTSF       (1<<4)  /* RTS edge interrupt flag */
#define  USR2_TXDC       (1<<3)  /* Transmitter complete */
#define  USR2_BRCD       (1<<2)  /* Break condition */
#define  USR2_ORE        (1<<1)  /* Overrun error */
#define  USR2_RDR        (1<<0)  /* Recv data ready */
#define  UTS_FRCPERR     (1<<13) /* Force parity error */
#define  UTS_LOOP        (1<<12) /* Loop tx and rx */
#define  UTS_TXEMPTY     (1<<6)  /* TxFIFO empty */
#define  UTS_RXEMPTY     (1<<5)  /* RxFIFO empty */
#define  UTS_TXFULL      (1<<4)  /* TxFIFO full */
#define  UTS_RXFULL      (1<<3)  /* RxFIFO full */
#define  UTS_SOFTRST     (1<<0)  /* Software reset */

#define RESET_REG	0

DECLARE_GLOBAL_DATA_PTR;

static void d4400_serial_setbrg(void)
{
	u32 clk = d4400_get_uart_clk(CONFIG_D4400_UART_PORT);

	if (!gd->baudrate)
		gd->baudrate = CONFIG_BAUDRATE;

	__REG(UART_PHYS + UFCR) = UFCR_RFDIV_BY_1;
	__REG(UART_PHYS + UBIR) = 0xf;
	__REG(UART_PHYS + UBMR) = clk / gd->baudrate;

}

static int d4400_serial_getc(void)
{
	while (!(__REG(UART_PHYS + USR2) & USR2_RDR))
		WATCHDOG_RESET();

	return __REG(UART_PHYS + URXD) & URXD_RX_DATA;
}

static void d4400_serial_putc(const char c)
{
	/* wait for transmitter to be ready */
	while (!(__REG(UART_PHYS + USR2) & USR2_TXDC))
		WATCHDOG_RESET();

	__REG(UART_PHYS + UTXD) = c;

	/* If \n, also do \r */
	if (c == '\n')
		serial_putc('\r');
}

/*
 * Test whether a character is in the RX buffer
 */
static int d4400_serial_tstc(void)
{
	if (__REG(UART_PHYS + USR2) & USR2_RDR)
		return 1;
	return 0;
}

/*
 * Initialise the serial port with the given baudrate. The settings
 * are always 8 data bits, no parity, 1 stop bit.
 * Flow control is disabled.
 */
static int d4400_serial_init(void)
{
	__REG(UART_PHYS + UCR1) = RESET_REG;
	__REG(UART_PHYS + UCR2) = RESET_REG;

	while (1) {
		if (__REG(UART_PHYS + UCR2) & UCR2_SRST)
			break;
	}
	__REG(UART_PHYS + UCR1) = UCR1_UARTEN;

	serial_setbrg();

	__REG(UART_PHYS + UCR2) = UCR2_WS | UCR2_IRTS | UCR2_RXEN |\
					UCR2_TXEN | UCR2_CTSC;

	__REG(UART_PHYS + UCR3) = UCR3_RXDMUXSEL | UCR3_DCD |\
							UCR3_DSR | UCR3_RI;

	__REG(UART_PHYS + UCR4) = UCR4_LPBYP;

	return 0;
}

static struct serial_device d4400_serial_drv = {
	.name	= "d4400_serial",
	.start	= d4400_serial_init,
	.stop	= NULL,
	.setbrg	= d4400_serial_setbrg,
	.putc	= d4400_serial_putc,
	.puts	= default_serial_puts,
	.getc	= d4400_serial_getc,
	.tstc	= d4400_serial_tstc,
};

void d4400_serial_initialize(void)
{
	serial_register(&d4400_serial_drv);
}

__weak struct serial_device *default_serial_console(void)
{
	return &d4400_serial_drv;
}
