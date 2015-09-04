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

#ifndef _D4400_4T4R_H
#define _D4400_4T4R_H

enum board_type {
        BOARD_TYPE_UNKNOWN = -1,
        BOARD_TYPE_D4400_4T4R = 0,
};

enum board_rev {
        BOARD_REV_UNKNOWN = -1,
        BOARD_REV_A = 0,
        BOARD_REV_B = 1,
        BOARD_REV_C = 2,
        BOARD_REV_D = 3,
        BOARD_REV_E = 4,
};

extern enum board_type get_board_type(void);
extern enum board_rev  get_board_rev(void);

#endif /* _D4400_4T4R_H */
