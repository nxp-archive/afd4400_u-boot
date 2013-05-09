/*
 * Copyright 2009, 2013 Freescale Semiconductor, Inc.
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
 *
 */

#ifndef _ASM_CONFIG_H_
#define _ASM_CONFIG_H_

/* TSEC driver uses the PHYLIB infrastructure */
#ifndef CONFIG_PHYLIB
#if defined(CONFIG_TSEC_ENET)
#define CONFIG_PHYLIB

#include <config_phylib_all_drivers.h>
#endif /* TSEC_ENET */
#endif /* !CONFIG_PHYLIB */

#define CONFIG_LMB
#define CONFIG_SYS_BOOT_RAMDISK_HIGH
#endif
