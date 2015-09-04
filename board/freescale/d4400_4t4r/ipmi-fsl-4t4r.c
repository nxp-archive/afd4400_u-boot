/*
 * Copyright 2015 Freescale Semiconductor, Inc.
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
#include "ipmi-fsl-4t4r.h"
#include <asm/arch/ipmi-eeprom-util.h>
#include "d4400_4t4r_priv.h"

/* Name string is matched against IPMI board name.  Only
 * part of the substring needs to match.
 */
#define IPMI_4T4R_NAME_STR	"4T4R"

#define IPMI_4T4R_REVA_STR	"REV A"
#define IPMI_4T4R_REVB_STR	"REV B"
#define IPMI_4T4R_REVC_STR	"REV C"
#define IPMI_4T4R_REVD_STR	"REV D"
#define IPMI_4T4R_REVE_STR	"REV E"

int ipmi_get_board_type(char *name_str)
{
	enum board_type type = BOARD_TYPE_UNKNOWN;

	if (strstr(name_str, IPMI_4T4R_NAME_STR) != NULL)
		type = BOARD_TYPE_D4400_4T4R;

	return type;
}

int ipmi_get_board_rev(char *partnum_str)
{
	enum board_rev rev = BOARD_REV_UNKNOWN;

	if (strstr(partnum_str, IPMI_4T4R_REVA_STR) != NULL)
		rev = BOARD_REV_A;
	else if (strstr(partnum_str, IPMI_4T4R_REVB_STR) != NULL)
		rev = BOARD_REV_B;
	else if (strstr(partnum_str, IPMI_4T4R_REVC_STR) != NULL)
		rev = BOARD_REV_C;
	else if (strstr(partnum_str, IPMI_4T4R_REVD_STR) != NULL)
		rev = BOARD_REV_D;
	else if (strstr(partnum_str, IPMI_4T4R_REVE_STR) != NULL)
		rev = BOARD_REV_E;

	return rev;
}
