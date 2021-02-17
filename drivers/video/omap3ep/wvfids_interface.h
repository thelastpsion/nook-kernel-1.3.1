/*
 * ========================================================================
 *                       MultiMedia Solutions
 *   (c) Copyright 2010, MultiMedia Solutions  All Rights Reserved.
 *
 *   Use of this software is controlled by the terms and conditions found
 *   in the license agreement under which this software has been supplied.
 *
 * ========================================================================
 *
 * This header file must be used whenever you need to write directly in the BFB.
 * Only WVFIDs defined here are known by the SocketNode.
 * There is no directly relationship between this header and any other header
 * file.
 * Keep all constants and structs defined here.
 *
 */


#ifndef _WVFIDS_INTERFACE_H_
#define _WVFIDS_INTERFACE_H_

#define OMAP3EPFB_DSP_WVFID_GC		0
#define OMAP3EPFB_DSP_WVFID_GU		1
#define OMAP3EPFB_DSP_WVFID_DU		2
#define OMAP3EPFB_DSP_WVFID_A2		3
#define OMAP3EPFB_DSP_WVFID_GL		4	///former _CUST1	4
#define OMAP3EPFB_DSP_WVFID_X1		5	///former _CUST2	5
#define OMAP3EPFB_DSP_WVFID_X2		6	///former _CUST3	6
#define OMAP3EPFB_DSP_WVFID_GLF		7       /// Do not use externally
#define OMAP3EPFB_DSP_WVFID_GLT		8       /// Do not use externally

#define OMAP3EPFB_DSP_WVFID_NUM		(9)
#define OMAP3EPFB_DSP_WVFID_AUTO	255

#define DU_VARIANT_DU2 (0)
#define DU_VARIANT_DU4 (1)

#ifndef OMAP3EPFB_IOCTL_API_VERSION
typedef struct {
       int du_variant;
       int sfto;
       int pmmo;
} wvf_feedback_st_t;
#endif

#endif///_WVFIDS_INTERFACE_H_
