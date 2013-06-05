/*
 * Copyright (C) 2011
 * Stefano Babic, DENX Software Engineering, <sbabic@denx.de>
 *
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


#ifndef __ASM_ARCH_D4400_GPIO_H
#define __ASM_ARCH_D4400_GPIO_H

#if !(defined(__KERNEL_STRICT_NAMES) || defined(__ASSEMBLY__))
/* GPIO registers */
struct gpio_regs {
	u32 gpio_dr;	/* data */
	u32 gpio_dir;	/* direction */
	u32 gpio_psr;	/* pad satus */
};
#endif

#define D4400_GPIO_NR(port, index)		((((port)-1)*32)+((index)&31))

/* used by gpio command for user friendly GPIO naming */
static inline int name_to_gpio(const char *name)
{
	int base = -1;
	int offset;
	/* equivalent to tolower() for ASCII A-Z */
	char gpio_set = *name | 0x20;

	/* Check for an initial letter for the GPIO bank */
	if (gpio_set >= 'a' && gpio_set <= 'z') {
		base = (gpio_set - 'a') * 32;
		++name;
	}

	/* After the bank letter it must be a decimal number */
	gpio_set = *name;
	if (gpio_set < '0' || gpio_set > '9')
		return -1;

	/* Get the numeric pin number */
	offset = simple_strtoul(name, NULL, 10);

	/* If a GPIO bank was used then the number must be less than 32 */
	if (base < 0)
		return offset; /* only a GPIO number was specified */
	else if (offset < 32)
		return base + offset; /* a GPIO bank and number was given */
	else
		return -1; /* the offset is too high */
}

#define name_to_gpio(n) name_to_gpio(n)

#endif
