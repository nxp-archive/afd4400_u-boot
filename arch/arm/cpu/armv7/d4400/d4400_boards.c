/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <common.h>
#include <asm/errno.h>
#include <asm/arch/d4400_boards.h>

enum board_type ipmi_get_board_type(char *name_str)
{
	enum board_type type = BOARD_TYPE_UNKNOWN;

	if (strstr(name_str, D4400_EVB_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_EVB;
	else if (strstr(name_str, D4400_RDB_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_RDB;
	else if (strstr(name_str, D4400_4T4RK1_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_4T4RK1;
	else if (strstr(name_str, D4400_4T4R_NAME_STR) != NULL)
		/* strstr() used for catch-all for boards with "4T4R"
		 * substring.
		*/
		type = BOARD_TYPE_D4400_4T4R;
	else if (strstr(name_str, D4400_21RRH_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_21RRH;

	return type;
}

enum board_rev ipmi_get_board_rev(char *partnum_str)
{
	enum board_rev rev = BOARD_REV_UNKNOWN;

	if (strstr(partnum_str, REVA_STR) != NULL)
		rev = BOARD_REV_A;
	else if (strstr(partnum_str, REVB_STR) != NULL)
		rev = BOARD_REV_B;
	else if (strstr(partnum_str, REVC_STR) != NULL)
		rev = BOARD_REV_C;
	else if (strstr(partnum_str, REVD_STR) != NULL)
		rev = BOARD_REV_D;
	else if (strstr(partnum_str, REVE_STR) != NULL)
		rev = BOARD_REV_E;
	else if (strstr(partnum_str, REVF_STR) != NULL)
		rev = BOARD_REV_F;

	return rev;
}
