/*
 * Copyright 2007-2013 Freescale Semiconductor, Inc.
 *
 * (C) Copyright 2000
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
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

#include <common.h>
#include <libfdt.h>
#include <fdt_support.h>

static void fdt_fixup_cpri_ethernet(void *fdt)
{
	int node, i, j;
	char enet[16], *tmp, *end;
	char mac[16] = "cpri0eth0addr";
	const char *path;
	unsigned char mac_addr[6];

	node = fdt_path_offset(fdt, "/aliases");
	if (node < 0)
		return;

	i = 0;
	while ((tmp = getenv(mac)) != NULL) {
		sprintf(enet, "cpri%dethernet%d", (i>>1), (i%2));
		path = fdt_getprop(fdt, node, enet, NULL);
		if (!path) {
			debug("No alias for %s\n", enet);
			i++;
			sprintf(mac, "cpri%deth%daddr", (i>>1), (i%2));
			continue;
		}

		for (j = 0; j < 6; j++) {
			mac_addr[j] = tmp ? simple_strtoul(tmp, &end, 16) : 0;
			if (tmp)
				tmp = (*end) ? end+1 : end;
		}

		do_fixup_by_path(fdt, path, "mac-address", &mac_addr, 6, 0);
		do_fixup_by_path(fdt, path, "local-mac-address",
				 &mac_addr, 6, 1);

		i++;
		sprintf(mac, "cpri%deth%daddr", (i>>1), (i%2));
	}
}

void ft_cpu_setup(void *blob, bd_t *bd)
{
	fdt_fixup_ethernet(blob);
	fdt_fixup_cpri_ethernet(blob);
}

