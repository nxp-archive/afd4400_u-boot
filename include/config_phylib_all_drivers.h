/*
 * Enable all PHYs
 *
 * This software may be used and distributed according to the
 * terms of the GNU Public License, Version 2, incorporated
 * herein by reference.
 *
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
 * author Andy Fleming
 *
 */
#ifndef _CONFIG_PHYLIB_ALL_H
#define _CONFIG_PHYLIB_ALL_H

#ifdef CONFIG_PHYLIB

#define CONFIG_PHY_VITESSE
#define CONFIG_PHY_SMSC

#ifdef CONFIG_PHYLIB_10G
#define CONFIG_PHY_TERANETICS
#endif /* CONFIG_PHYLIB_10G */

#endif /* CONFIG_PHYLIB */

#endif /*_CONFIG_PHYLIB_ALL_H */
