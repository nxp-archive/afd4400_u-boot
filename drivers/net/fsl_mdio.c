/*
 * Copyright 2009-2010, 2013 Freescale Semiconductor, Inc.
 *	Jun-jie Zhang <b18070@freescale.com>
 *	Mingkai Hu <Mingkai.hu@freescale.com>
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
#include <miiphy.h>
#include <phy.h>
#include <fsl_mdio.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/fsl_enet.h>

void mem_sync()
{
#ifdef CONFIG_ARM
	asm("dmb");
#endif
#ifdef CONFIG_PPC
	asm("sync");
#endif
}

void clrbits_32(volatile void *addr , unsigned int val)
{
#ifdef CONFIG_ARM
	clrbits_le32(addr, val);
#endif
#ifdef CONFIG_PPC
	clrbits_be32(addr, val);
#endif
}

void setbits_32(volatile void *addr , unsigned int val)
{
#ifdef CONFIG_ARM
	setbits_le32(addr, val);
#endif
#ifdef CONFIG_PPC
	setbits_be32(addr, val);
#endif
}

void write_32(volatile void *addr , unsigned int val)
{
#ifdef CONFIG_ARM
	out_le32(addr, val);
#endif
#ifdef CONFIG_PPC
	out_be32(addr, val);
#endif
}

unsigned int read_32(volatile void *addr)
{
#ifdef CONFIG_ARM
	return in_le32(addr);
#endif
#ifdef CONFIG_PPC
	return in_be32(addr);
#endif
}
void tsec_local_mdio_write(struct tsec_mii_mng *phyregs, int port_addr,
		int dev_addr, int regnum, int value)
{
	int timeout = 1000000;

	write_32(&phyregs->miimadd, (port_addr << 8) | (regnum & 0x1f));
	write_32(&phyregs->miimcon, value);
	mem_sync();

	while ((read_32(&phyregs->miimind) & MIIMIND_BUSY) && timeout--)
		;
}

int tsec_local_mdio_read(struct tsec_mii_mng *phyregs, int port_addr,
		int dev_addr, int regnum)
{
	int value;
	int timeout = 1000000;

	/* Put the address of the phy, and the register
	 * number into MIIMADD */
	write_32(&phyregs->miimadd, (port_addr << 8) | (regnum & 0x1f));

	/* Clear the command register, and wait */
	write_32(&phyregs->miimcom, 0);
	mem_sync();

	/* Initiate a read command, and wait */
	write_32(&phyregs->miimcom, MIIMCOM_READ_CYCLE);
	mem_sync();

	/* Wait for the the indication that the read is done */
	while ((read_32(&phyregs->miimind) & (MIIMIND_NOTVALID | MIIMIND_BUSY))
			&& timeout--)
		;

	/* Grab the value read from the PHY */
	value = read_32(&phyregs->miimstat);

	return value;
}

static int fsl_pq_mdio_reset(struct mii_dev *bus)
{
	struct tsec_mii_mng *regs = bus->priv;
#ifdef FPGA_DELAY
	int mask = 0;

	/*Code for FPGA delay*/
	printf("Waiting for FPGA INIT ");
	mask = read_32(&regs->miimcfg);
	while (0x00000007 != mask)
		mask = read_32(&regs->miimcfg);
	printf("0x%x\n", read_32(&regs->miimcfg));
#endif
	/* Reset MII (due to new addresses) */
	write_32(&regs->miimcfg, MIIMCFG_RESET_MGMT);

	write_32(&regs->miimcfg, MIIMCFG_INIT_VALUE);

	while (read_32(&regs->miimind) & MIIMIND_BUSY)
		;

	return 0;
}

int tsec_phy_read(struct mii_dev *bus, int addr, int dev_addr, int regnum)
{
	struct tsec_mii_mng *phyregs = bus->priv;

	return tsec_local_mdio_read(phyregs, addr, dev_addr, regnum);
}

int tsec_phy_write(struct mii_dev *bus, int addr, int dev_addr, int regnum,
			u16 value)
{
	struct tsec_mii_mng *phyregs = bus->priv;

	tsec_local_mdio_write(phyregs, addr, dev_addr, regnum, value);

	return 0;
}

int fsl_pq_mdio_init(bd_t *bis, struct fsl_pq_mdio_info *info)
{
	struct mii_dev *bus = mdio_alloc();

	if (!bus) {
		printf("Failed to allocate FSL MDIO bus\n");
		return -1;
	}

	bus->read = tsec_phy_read;
	bus->write = tsec_phy_write;
	bus->reset = fsl_pq_mdio_reset;
	sprintf(bus->name, info->name);

	bus->priv = info->regs;

	return mdio_register(bus);
}
