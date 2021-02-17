/*
 * SDRC register values for the Samsung K4X4G303PB
 *
 * Copyright (C) 2012 Barnes and Noble
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARCH_ARM_MACH_OMAP2_SDRAM_SAMSUNG_K4X2G323PD
#define ARCH_ARM_MACH_OMAP2_SDRAM_SAMSUNG_K4X2G323PD

#include <mach/sdrc.h>

/*--------------------------------------------------------------------------*/

/* Samsung K4X2G323PD mDDR (200MHz optimized) 5ns
 *
 *     ACTIMA
 *        -TDAL = Twr/Tck + Trp/tck = 12/5 + 15/5 = 3 + 2.4 = 5.4 -> 6
 *        -TDPL (Twr) = 12/5 = 2.4 -> 3
 *        -TRRD = 10/5       = 2
 *        -TRCD = 15/5       = 3
 *        -TRP = 15/5        = 3
 *        -TRAS = 40/5       = 8
 *        -TRC = 55/5        = 11
 *        -TRFC = 120/5      = 24
 *     ACTIMB
 *        -TWTR = 2 (TCDLR in the datasheet)
 *        -TCKE = 2
 *        -TXP  = 2
 *        -TXSR = 120/5 = 24
 */
#define TDAL_200   6
#define TDPL_200   3
#define TRRD_200   2
#define TRCD_200   3
#define TRP_200    3
#define TRAS_200   8
#define TRC_200   11
#define TRFC_200  24
#define V_ACTIMA_200 ((TRFC_200 << 27) | (TRC_200 << 22) | (TRAS_200 << 18) |\
          (TRP_200 << 15) | (TRCD_200 << 12) | (TRRD_200 << 9) | \
          (TDPL_200 << 6) | (TDAL_200))

#define TWTR_200  2
#define TCKE_200  2
#define TXP_200   2
#define TXSR_200 24
#define V_ACTIMB_200 (((TCKE_200 << 12) | (TXSR_200 << 0)) | \
          (TXP_200 << 8) | (TWTR_200 << 16))

#define SAMSUNG_RFR_CTRL_200MHz   0x0005e601 /* 7.8us/5ns - 50=0x5e6 */

/*--------------------------------------------------------------------------*/

/* Samsung K4X2G323PD mDDR (166MHz optimized) 6.02ns
 *
 *     ACTIMA
 *        -TDAL = Twr/Tck + Trp/tck = 12/6 + 15/6 = 2 + 2.5 = 4.5 -> 5
 *        -TDPL (Twr) = 12/6 = 2 -> 2
 *        -TRRD = 10/6       = 1.66 -> 2
 *        -TRCD = 15/6       = 2.5 -> 3
 *        -TRP = 15/6        = 2.5 -> 3
 *        -TRAS = 40/6       = 6.7 -> 7
 *        -TRC = 55/6        = 9.16 -> 10
 *        -TRFC = 120/6      = 20
 *     ACTIMB
 *        -TWTR = 2 (TCDLR in the datasheet)
 *        -TCKE = 2
 *        -TXP  = 2
 *        -TXSR = 120/6 = 20
 */
#define TDAL_166   5
#define TDPL_166   2
#define TRRD_166   2
#define TRCD_166   3
#define TRP_166    3
#define TRAS_166   7
#define TRC_166   10
#define TRFC_166  20
#define V_ACTIMA_166 ((TRFC_166 << 27) | (TRC_166 << 22) | (TRAS_166 << 18) |\
          (TRP_166 << 15) | (TRCD_166 << 12) | (TRRD_166 << 9) | \
          (TDPL_166 << 6) | (TDAL_166))

#define TWTR_166  2
#define TCKE_166  2
#define TXP_166   2
#define TXSR_166 20
#define V_ACTIMB_166 (((TCKE_166 << 12) | (TXSR_166 << 0)) | \
          (TXP_166 << 8) | (TWTR_166 << 16))

#define SAMSUNG_RFR_CTRL_166MHz   0x0004e201 /* 7.8us/6ns - 50=0x4e2 */

/*--------------------------------------------------------------------------*/

/* Samsung K4X2G323PD mDDR (100MHz optimized) 10ns
 *
 *     ACTIMA
 *        -TDAL = Twr/Tck + Trp/tck = 12/10 + 15/10 = 1.2 + 1.5 = 2.7 -> 3
 *        -TDPL (Twr) = 12/10 = 1.2 -> 2
 *        -TRRD = 10/10       = 1
 *        -TRCD = 15/10       = 1.5 -> 2
 *        -TRP = 15/10        = 1.5 -> 2
 *        -TRAS = 40/10       = 4
 *        -TRC = 55/10        = 5.5 -> 6
 *        -TRFC = 120/10      = 12
 *     ACTIMB
 *        -TWTR = 2 (TCDLR in the datasheet)
 *        -TCKE = 2
 *        -TXP  = 2
 *        -TXSR = 120/10 = 12
 */
#define TDAL_100   3
#define TDPL_100   2
#define TRRD_100   1
#define TRCD_100   2
#define TRP_100    2
#define TRAS_100   4
#define TRC_100    6
#define TRFC_100  12
#define V_ACTIMA_100 ((TRFC_100 << 27) | (TRC_100 << 22) | (TRAS_100 << 18) |\
          (TRP_100 << 15) | (TRCD_100 << 12) | (TRRD_100 << 9) | \
          (TDPL_100 << 6) | (TDAL_100))

#define TWTR_100  2
#define TCKE_100  2
#define TXP_100   2
#define TXSR_100 12
#define V_ACTIMB_100 (((TCKE_100 << 12) | (TXSR_100 << 0)) | \
          (TXP_100 << 8) | (TWTR_100 << 16))

#define SAMSUNG_RFR_CTRL_100MHz   0x0002da01 /* 7.8us/10ns - 50 = 0x2da */

/*--------------------------------------------------------------------------*/

/* Samsung K4X4G303PB mDDR (83MHz optimized) 12.05ns
 *
 *     ACTIMA
 *        -TDAL = Twr/Tck + Trp/tck = 12/12 + 15/12 = 1 + 1.25 = 2.25 -> 3
 *        -TDPL (Twr) = 12/12 = 1    -> 1
 *        -TRRD = 10/12       = 0.83 -> 1
 *        -TRCD = 15/12       = 1.25 -> 2
 *        -TRP = 15/12        = 1.25 -> 2
 *        -TRAS = 40/12       = 3.33 -> 4
 *        -TRC = 55/12        = 4.58 -> 5
 *        -TRFC = 120/12      = 10
 *     ACTIMB
 *        -TWTR = 2 (TCDLR in the datasheet)
 *        -TCKE = 2
 *        -TXP  = 2
 *        -TXSR = 120/12 = 10
 */
#define TDAL_83   3
#define TDPL_83   1
#define TRRD_83   1
#define TRCD_83   2
#define TRP_83    2
#define TRAS_83   4
#define TRC_83    5
#define TRFC_83  10
#define V_ACTIMA_83 ((TRFC_83 << 27) | (TRC_83 << 22) | (TRAS_83 << 18) |\
          (TRP_83 << 15) | (TRCD_83 << 12) | (TRRD_83 << 9) | \
          (TDPL_83 << 6) | (TDAL_83))

#define TWTR_83   2
#define TCKE_83   2
#define TXP_83    2
#define TXSR_83  10
#define V_ACTIMB_83 (((TCKE_83 << 12) | (TXSR_83 << 0)) | \
          (TXP_83 << 8) | (TWTR_83 << 16))

#define SAMSUNG_RFR_CTRL_83MHz  0x00025501 /* 7.8us/12ns - 50=597.33 -> 0x255 */

/*--------------------------------------------------------------------------*/

/* XXX Using ARE = 0x1 (no autorefresh burst) -- can this be changed? */
static struct omap_sdrc_params samsung_k4x2g323pd_sdrc_params[] = {
	[0] = {
		.rate	     = 200000000,
		.actim_ctrla = V_ACTIMA_200,
		.actim_ctrlb = V_ACTIMB_200,
		.rfr_ctrl    = SAMSUNG_RFR_CTRL_200MHz,
		.mr	     = 0x00000032,
	},
	[1] = {
		.rate	     = 166000000,
		.actim_ctrla = V_ACTIMA_166,
		.actim_ctrlb = V_ACTIMB_166,
		.rfr_ctrl    = SAMSUNG_RFR_CTRL_166MHz,
		.mr	     = 0x00000032,
	},
	[2] = {
		.rate	     = 165941176,
		.actim_ctrla = V_ACTIMA_166,
		.actim_ctrlb = V_ACTIMB_166,
		.rfr_ctrl    = SAMSUNG_RFR_CTRL_166MHz,
		.mr	     = 0x00000032,
	},
	[3] = {
		.rate	     = 100000000,
		.actim_ctrla = V_ACTIMA_100,
		.actim_ctrlb = V_ACTIMB_100,
		.rfr_ctrl    = SAMSUNG_RFR_CTRL_100MHz,
		.mr	     = 0x00000022,
	},
	[4] = {
		.rate	     = 83000000,
		.actim_ctrla = V_ACTIMA_83,
		.actim_ctrlb = V_ACTIMB_83,
		.rfr_ctrl    = SAMSUNG_RFR_CTRL_83MHz,
		.mr	     = 0x00000022,
	},
	[5] = {
		.rate	     = 82970588,
		.actim_ctrla = V_ACTIMA_83,
		.actim_ctrlb = V_ACTIMB_83,
		.rfr_ctrl    = SAMSUNG_RFR_CTRL_83MHz,
		.mr	     = 0x00000022,
	},
	[6] = {
		.rate	     = 0
	},
};

#endif
