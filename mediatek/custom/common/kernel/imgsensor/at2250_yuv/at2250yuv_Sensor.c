/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *  
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Leo Lee
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision: 1.2 $
 * $Modtime:$
 * $Log: at2250yuv_Sensor.c,v $
 * Revision 1.2  2013/06/08 13:32:34  xuzhoubin
 * no message
 *
 * Revision 1.1  2013/06/04 08:22:36  xuzhoubin
 * no message
 *
 *
 * [AT2250YUV V1.0.0]
 * 8.17.2012 Leo.Lee
 * .First Release
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GalaxyCoreinc. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
//#include <windows.h>
//#include <memory.h>
//#include <nkintr.h>
//#include <ceddk.h>
//#include <ceddk_exp.h>

//#include "kal_release.h"
//#include "i2c_exp.h"
//#include "gpio_exp.h"
//#include "msdk_exp.h"
//#include "msdk_sensor_exp.h"
//#include "msdk_isp_exp.h"
//#include "base_regs.h"
//#include "Sensor.h"
//#include "camera_sensor_para.h"
//#include "CameraCustomized.h"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "at2250yuv_Sensor.h"
#include "at2250yuv_Camera_Sensor_para.h"
#include "at2250yuv_CameraCustomized.h"

#define AT2250_TEST_PATTERN_CHECKSUM 0xc63aa951

#define AT2250YUV_DEBUG
#ifdef AT2250YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

#define AT2250_MIPI
//#define _MAXCLK_65MHz_ //petersun

//#define AT2250_SWITCH


kal_uint8 IsVideoPreview = 0;
kal_uint8 IsCapture = 0;

#ifdef AT2250_SWITCH
extern kal_uint8 IsAt2250 ;
#endif

kal_uint8 AT2250_BANDING = 0; // 0 50hz     1  60hz
BOOL AT2250_set_param_banding(UINT16 para);

UINT32 AT2250Close(void);


extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
/*************************************************************************
* FUNCTION
*    AT2250_write_cmos_sensor
*
* DESCRIPTION
*    This function wirte data to CMOS sensor through I2C
*
* PARAMETERS
*    addr: the 16bit address of register
*    para: the 8bit value of register
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void AT2250_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), AT2250_WRITE_ID); 

#if (defined(__AT2250_DEBUG_TRACE__))
  if (sizeof(out_buff) != rt) printk("I2C write %x, %x error\n", addr, para);
#endif
}

/*************************************************************************
* FUNCTION
*    AT2250_read_cmos_sensor
*
* DESCRIPTION
*    This function read data from CMOS sensor through I2C.
*
* PARAMETERS
*    addr: the 16bit address of register
*
* RETURNS
*    8bit data read through I2C
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint8 AT2250_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;

    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), AT2250_WRITE_ID)) {
        SENSORDB("ERROR: AT2250_read_cmos_sensor \n");
    }

#if (defined(__AT2250_DEBUG_TRACE__))
  if (size != rt) printk("I2C read %x error\n", addr);
#endif

  return in_buff[0];
}

UINT32 AT2250SetTestPatternMode(kal_bool bEnable)
{
	printk("[AT2250SetTestPatternMode]leanda test pattern bEnable:=%d\n",bEnable);

	if(bEnable)
	{			
		AT2250_write_cmos_sensor(0x12, 0x80);	//reset sensor	
		mDELAY(30); ////Delay 100ms 				 
		AT2250_write_cmos_sensor(0x12, 0x40);
		
		
		//analog							
		AT2250_write_cmos_sensor(0x0c, 0x04);
		AT2250_write_cmos_sensor(0x30, 0x96);
		AT2250_write_cmos_sensor(0x31, 0x10);
		AT2250_write_cmos_sensor(0x32, 0x0c);
		AT2250_write_cmos_sensor(0x33, 0x28);
		AT2250_write_cmos_sensor(0xaa, 0x28);
		AT2250_write_cmos_sensor(0xab, 0x11);
		AT2250_write_cmos_sensor(0xac, 0x15);
		AT2250_write_cmos_sensor(0xad, 0x25);
		AT2250_write_cmos_sensor(0xae, 0x40);
		AT2250_write_cmos_sensor(0xaf, 0xa0);
		AT2250_write_cmos_sensor(0xb0, 0x40);
		AT2250_write_cmos_sensor(0xb1, 0x00);
		AT2250_write_cmos_sensor(0xb2, 0x74);
		AT2250_write_cmos_sensor(0xb3, 0x39); 
		
		//PLL  
#if 0
		AT2250_write_cmos_sensor(0x0e, 0x10);
		AT2250_write_cmos_sensor(0x0f, 0x03);
		AT2250_write_cmos_sensor(0x10, 0x18);
		AT2250_write_cmos_sensor(0x11, 0x01);
		AT2250_write_cmos_sensor(0x0d, 0x88);
 #else
			AT2250_write_cmos_sensor(0x0e,0x11);
			AT2250_write_cmos_sensor(0x0f,0x07); //7.5 ~ 15 fps
			AT2250_write_cmos_sensor(0x10,0x30); //
			AT2250_write_cmos_sensor(0x11,0x41); 
			AT2250_write_cmos_sensor(0x0d,0xd0); //
		 
 #endif
		//DVP Port							 
		AT2250_write_cmos_sensor(0x1d, 0x00);
		AT2250_write_cmos_sensor(0x1e, 0x00);
							 
		//MIPI	 
		AT2250_write_cmos_sensor(0xb5, 0x50);
		
#if 0
		AT2250_write_cmos_sensor(0xb7, 0xb6);
		AT2250_write_cmos_sensor(0xb8, 0xd1);
		AT2250_write_cmos_sensor(0xb9, 0xab);
		AT2250_write_cmos_sensor(0xba, 0x55);
		AT2250_write_cmos_sensor(0xbb, 0x16);
		AT2250_write_cmos_sensor(0xbc, 0x1e);
		AT2250_write_cmos_sensor(0xbd, 0x80);
		AT2250_write_cmos_sensor(0xbe, 0x0c);
		AT2250_write_cmos_sensor(0xbf, 0x14);
#else
		AT2250_write_cmos_sensor(0xb7, 0xb6);
		AT2250_write_cmos_sensor(0xb8, 0xd1);
		AT2250_write_cmos_sensor(0xb9, 0xab);
		AT2250_write_cmos_sensor(0xba, 0x55);
		AT2250_write_cmos_sensor(0xbb, 0x16);
		AT2250_write_cmos_sensor(0xbc, 0x1e);
		AT2250_write_cmos_sensor(0xbd, 0x80);
		AT2250_write_cmos_sensor(0xbe, 0x0c);
		AT2250_write_cmos_sensor(0xbf, 0x15); // 
		
#endif
		//Resolution						
		AT2250_write_cmos_sensor(0x1b, 0x44);
		AT2250_write_cmos_sensor(0x1f, 0x40);
		AT2250_write_cmos_sensor(0x20, 0x8e);
		AT2250_write_cmos_sensor(0x21, 0x07);
		AT2250_write_cmos_sensor(0x22, 0xD8);
		AT2250_write_cmos_sensor(0x23, 0x04);
		AT2250_write_cmos_sensor(0x24, 0x40);
		AT2250_write_cmos_sensor(0x25, 0xB4);
		AT2250_write_cmos_sensor(0x26, 0x46);
		AT2250_write_cmos_sensor(0x27, 0xE3);
		AT2250_write_cmos_sensor(0x28, 0x0b);
		AT2250_write_cmos_sensor(0x29, 0x00);
		AT2250_write_cmos_sensor(0x2a, 0xd4);
		AT2250_write_cmos_sensor(0x2b, 0x10);
		AT2250_write_cmos_sensor(0x2c, 0x00);
		AT2250_write_cmos_sensor(0x2d, 0x00);
		AT2250_write_cmos_sensor(0x2e, 0x30);
		AT2250_write_cmos_sensor(0x2f, 0xa4);
		AT2250_write_cmos_sensor(0x38, 0x40);
		AT2250_write_cmos_sensor(0x39, 0xb0);
		AT2250_write_cmos_sensor(0x3a, 0x46);
		AT2250_write_cmos_sensor(0x3b, 0x40);
		AT2250_write_cmos_sensor(0x3c, 0xb0);
		AT2250_write_cmos_sensor(0x3d, 0x46);
		AT2250_write_cmos_sensor(0xbd, 0x80);
		AT2250_write_cmos_sensor(0xbe, 0x0c);
							 
		// AEC/AGC 
#if 0
		AT2250_write_cmos_sensor(0x13, 0x8e);
		AT2250_write_cmos_sensor(0x14, 0x4c);
		AT2250_write_cmos_sensor(0x15, 0x24);
		AT2250_write_cmos_sensor(0x16, 0x80);
		AT2250_write_cmos_sensor(0x17, 0x18);
		AT2250_write_cmos_sensor(0x18, 0xba);
		AT2250_write_cmos_sensor(0x19, 0x90);
		AT2250_write_cmos_sensor(0xa0, 0x55);
		AT2250_write_cmos_sensor(0xa1, 0x55);
		AT2250_write_cmos_sensor(0xa2, 0x55);
		AT2250_write_cmos_sensor(0xa3, 0x55);
		AT2250_write_cmos_sensor(0xa4, 0x04);
#else
		AT2250_write_cmos_sensor(0x13, 0x8e);
		AT2250_write_cmos_sensor(0x14,0x58);//0x48 140429
		AT2250_write_cmos_sensor(0x15, 0x23);
		AT2250_write_cmos_sensor(0x16, 0x72);
		AT2250_write_cmos_sensor(0x17, 0x24);
		AT2250_write_cmos_sensor(0x18, 0xba);
		AT2250_write_cmos_sensor(0x19, 0x90);
#if 0
		AT2250_write_cmos_sensor(0xa0, 0x55);
		AT2250_write_cmos_sensor(0xa1, 0x55);
		AT2250_write_cmos_sensor(0xa2, 0x55);
		AT2250_write_cmos_sensor(0xa3, 0x55);
		AT2250_write_cmos_sensor(0xa4, 0x04);
#else
		AT2250_write_cmos_sensor(0xa0, 0x69);
		AT2250_write_cmos_sensor(0xa1, 0xbe);
		AT2250_write_cmos_sensor(0xa2, 0xbe);
		AT2250_write_cmos_sensor(0xa3, 0x69);
		AT2250_write_cmos_sensor(0xa4, 0x04);	
#endif
		
#endif
		//ISP							  
		AT2250_write_cmos_sensor(0x36, 0x7b);
		AT2250_write_cmos_sensor(0x37, 0x00);		//mod PC 03202013
							 
		AT2250_write_cmos_sensor(0x40, 0x00);
							 
		//BLC								
		AT2250_write_cmos_sensor(0x49, 0x09);
		AT2250_write_cmos_sensor(0x4a, 0x0f);
							 
									 
		//AWB								
		AT2250_write_cmos_sensor(0x83,0x0a);
		AT2250_write_cmos_sensor(0x84,0x40);
		AT2250_write_cmos_sensor(0x82,0x1e);
		AT2250_write_cmos_sensor(0x80,0xf0);
		AT2250_write_cmos_sensor(0x81,0x1e);
		
		AT2250_write_cmos_sensor(0x85,0x1E);//0x44 140429
		AT2250_write_cmos_sensor(0x86,0x64);//0x80 140429
		AT2250_write_cmos_sensor(0x87,0x7e);//0xa6 140429
		AT2250_write_cmos_sensor(0x88,0x90);//0x78 140429
		AT2250_write_cmos_sensor(0x89,0x9b);//0x6a 140429
		AT2250_write_cmos_sensor(0x8A,0x94);//0x73 140429
		AT2250_write_cmos_sensor(0x8B,0xad);//0x8d 140429
		AT2250_write_cmos_sensor(0x8C,0x8e);//0x86 140429
		AT2250_write_cmos_sensor(0x8D,0x62);//0x5f 140429
		AT2250_write_cmos_sensor(0x8E,0x5b);//0x61 140429
		AT2250_write_cmos_sensor(0x8F,0x52);//0x2a 140429
		AT2250_write_cmos_sensor(0x90,0xC8);
		
		//SDE								
		AT2250_write_cmos_sensor(0x75, 0x00);
		AT2250_write_cmos_sensor(0x76, 0x40);
		AT2250_write_cmos_sensor(0x79, 0x00);
		AT2250_write_cmos_sensor(0x7a, 0x00);
		AT2250_write_cmos_sensor(0x7b, 0x00);
		AT2250_write_cmos_sensor(0x7c, 0x00);
		AT2250_write_cmos_sensor(0x7d, 0x00);
		AT2250_write_cmos_sensor(0x7e, 0x00);
		AT2250_write_cmos_sensor(0x7f, 0x80);
							 
		//Saturation						
		AT2250_write_cmos_sensor(0x77, 0x07);
		AT2250_write_cmos_sensor(0x78, 0x09);
							 
		//Gamma 							
		AT2250_write_cmos_sensor(0x50,0x04);
		AT2250_write_cmos_sensor(0x51,0x08);//0x06 140429 
		AT2250_write_cmos_sensor(0x52,0x0f);//0x0c 140429
		AT2250_write_cmos_sensor(0x53,0x1e);//0x1a 140429
		AT2250_write_cmos_sensor(0x54,0x2b);//0x28 140429
		AT2250_write_cmos_sensor(0x55,0x38);
		AT2250_write_cmos_sensor(0x56,0x45);//0x48 140429
		AT2250_write_cmos_sensor(0x57,0x51);//0x59 140429
		AT2250_write_cmos_sensor(0x58,0x68);//0x70 140429
		AT2250_write_cmos_sensor(0x59,0x7d);//0x86 140429
		AT2250_write_cmos_sensor(0x5a,0x91);//0x9a 140429
		AT2250_write_cmos_sensor(0x5b,0xa3);//0xa8 140429
		AT2250_write_cmos_sensor(0x5c,0xb4);
		AT2250_write_cmos_sensor(0x5d,0xd1);//0xcc 140429
		AT2250_write_cmos_sensor(0x5e,0xe7);//0xe0 140429
		AT2250_write_cmos_sensor(0x5f,0xff);
							 
		//Color matrix					   
		AT2250_write_cmos_sensor(0x91,0x1f);//0x11 140429
		AT2250_write_cmos_sensor(0x92,0x2a);//0x2f 140429
		AT2250_write_cmos_sensor(0x93,0x06);//0x07 140429
		AT2250_write_cmos_sensor(0x94,0x10);//0x12 140429
		AT2250_write_cmos_sensor(0x95,0x42);//0x49 140429
		AT2250_write_cmos_sensor(0x96,0x50);//0x5a 140429
		AT2250_write_cmos_sensor(0x97,0x51);//0x5f 140429
		AT2250_write_cmos_sensor(0x98,0x51);//0x5c 140429
		AT2250_write_cmos_sensor(0x99,0x02);
		AT2250_write_cmos_sensor(0x9a,0x00);
		AT2250_write_cmos_sensor(0x9b,0x80);
		AT2250_write_cmos_sensor(0x9c,0x80);
		AT2250_write_cmos_sensor(0x9d,0xCe);
		AT2250_write_cmos_sensor(0x9e,0x08);
										 
		//Lens Correction
		AT2250_write_cmos_sensor(0x66,0x78);//BAWANG+0x10 0xa0 140429
		AT2250_write_cmos_sensor(0x67,0x6f);//BAWANG+0x10 0x90 140429
		AT2250_write_cmos_sensor(0x68,0x70);//BAWANG+0x10-8 0x90 140429
		AT2250_write_cmos_sensor(0x69,0x6a);//BAWANG+0x10-7 0x88 140429
		AT2250_write_cmos_sensor(0x6a,0x5d);//BAWANG+0x10 0x7e 140429
		AT2250_write_cmos_sensor(0x6b,0x50);//BAWANG+0x10 0x9a 140429
		AT2250_write_cmos_sensor(0x6c,0x14);//0x18 140429
		AT2250_write_cmos_sensor(0x6d,0x5c);
		AT2250_write_cmos_sensor(0x6e,0xBB);
		AT2250_write_cmos_sensor(0x6f,0x20);//0x28 140429
		AT2250_write_cmos_sensor(0x70,0x52);//0x62 140429
		AT2250_write_cmos_sensor(0x71,0x10);//0x18 140429
		AT2250_write_cmos_sensor(0x72,0x40);//0x70 140429
		AT2250_write_cmos_sensor(0x73,0x0B);
							 
		//Edge/denoise						
		AT2250_write_cmos_sensor(0x3e,0xa8);
		AT2250_write_cmos_sensor(0x3f,0x26);
		AT2250_write_cmos_sensor(0x60,0x0a);
		AT2250_write_cmos_sensor(0x61,0x0d);
		AT2250_write_cmos_sensor(0x62,0x0a);
		AT2250_write_cmos_sensor(0x63,0x0d);
		AT2250_write_cmos_sensor(0x64,0x90);
		AT2250_write_cmos_sensor(0x65,0xaa);
		AT2250_write_cmos_sensor(0x05,0x70);
		AT2250_write_cmos_sensor(0x06,0x00);
		AT2250_write_cmos_sensor(0x07,0x60);
		AT2250_write_cmos_sensor(0x08,0x54);
		AT2250_write_cmos_sensor(0x09,0x04);

		
		AT2250_write_cmos_sensor(0xa5,0x21); //test pattern
		
		AT2250_write_cmos_sensor(0x12,0x02);
		mDELAY(30);

	}
	else
	{
		AT2250Close();

	}
	return ERROR_NONE;
}


/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define Sleep(ms) mdelay(ms)
#define mDELAY(ms) mdelay(ms)

#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/
/* Global Valuable */

static kal_uint32 zoom_factor = 0; 

static kal_bool AT2250_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool AT2250_sensor_cap_state = KAL_FALSE; //Preview or Capture

static kal_uint16 AT2250_exposure_lines=0, AT2250_extra_exposure_lines = 0;

static kal_uint16 AT2250_Capture_Shutter=0;
static kal_uint16 AT2250_Capture_Extra_Lines=0;

AT2250_isp_master_clock=0;

//static kal_uint32  AT2250_sensor_pclk=390;

static kal_int8 AT2250_DELAY_AFTER_PREVIEW = -1;

static kal_uint32 Preview_Shutter = 0;
static kal_uint32 Capture_Shutter = 0;

static MSDK_SENSOR_CONFIG_STRUCT AT2250SensorConfigData;



//#define DEBUG_SENSOR
#ifdef DEBUG_SENSOR
	#define AT2250_OP_CODE_INI		0x00		/* Initial value. */
	#define AT2250_OP_CODE_REG		0x01		/* Register */
	#define AT2250_OP_CODE_DLY		0x02		/* Delay */
	#define AT2250_OP_CODE_END		0x03		/* End of initial setting. */
	
	UINT32 AT2250_fromsd;
	kal_uint16 AT_CARD_tmp = 0 ;
	
		typedef struct
	{
		u16 init_reg;
		u16 init_val;	/* Save the register value and delay tick */
		u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
	} AT2250_initial_set_struct;

	AT2250_initial_set_struct AT2250_Init_Reg[1000];
	
 u32 strtol(const char *nptr, u8 base)
{
	u8 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{
		if((base==16 && *nptr>='A' && *nptr<='F') || 
			(base==16 && *nptr>='a' && *nptr<='f') || 
			(base>=10 && *nptr>='0' && *nptr<='9') ||
			(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

 u8 AT2250_Initialize_from_T_Flash()
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0, j = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */

	
	struct file *fp; 
    mm_segment_t fs; 
    loff_t pos = 0; 
	static u8 data_buff[10*1024] ;
 
	
 //for(AT_CARD_tmp=0;AT_CARD_tmp<4;AT_CARD_tmp++)
  {
    switch(AT_CARD_tmp)
	{
	case 0 :
		 fp = filp_open("/mnt/sdcard/at2250_sd", O_RDONLY , 0); 
		break;
	case 1 :
	 	fp = filp_open("/mnt/sdcard0/at2250_sd", O_RDONLY , 0); 
		break;
	case 2 :
		 fp = filp_open("/mnt/sdcard1/at2250_sd", O_RDONLY , 0); 
		break;
	case 3 :
		 fp = filp_open("/mnt/sdcard2/at2250_sd", O_RDONLY , 0); 
		break;

	}

   // fp = filp_open("/mnt/sdcard2/at2250_sd", O_RDONLY , 0); 
    if (IS_ERR(fp)) { 
        printk("create file error\n"); 
        return -1; 
    } 
    fs = get_fs(); 
    set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
    vfs_read(fp, data_buff, file_size, &pos); 
    //printk("%s %d %d\n", buf,iFileLen,pos); 
	filp_close(fp, NULL); 
    set_fs(fs);
	}	
	



	/* Start parse the setting witch read from t-flash. */
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size))
	{
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
		{
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
			{
				curr_ptr++;		/* Skip block comment code. */
			}

			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */
			
			continue ;
		}
		
		if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
		{
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}
		/* This just content one enter line. */
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		//printk(" curr_ptr1 = %s\n",curr_ptr);
		memcpy(func_ind, curr_ptr, 3);
	
						
		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */
			AT2250_Init_Reg[i].op_code = AT2250_OP_CODE_REG;
			
			AT2250_Init_Reg[i].init_reg = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */
		
			AT2250_Init_Reg[i].init_val = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */
		
		}
		else									/* DLY */
		{
			/* Need add delay for this setting. */
			curr_ptr += 4;	
			AT2250_Init_Reg[i].op_code = AT2250_OP_CODE_DLY;
			
			AT2250_Init_Reg[i].init_reg = 0xFF;
			AT2250_Init_Reg[i].init_val = strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
		}
		i++;
		

		/* Skip to next line directly. */
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}

	/* (0xFFFF, 0xFFFF) means the end of initial setting. */
	AT2250_Init_Reg[i].op_code = AT2250_OP_CODE_END;
	AT2250_Init_Reg[i].init_reg = 0xFF;
	AT2250_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
		//printk(" %x  ==  %x\n",AT2250_Init_Reg[j].init_reg, AT2250_Init_Reg[j].init_val);

	/* Start apply the initial setting to sensor. */
	#if 1
	for (j=0; j<i; j++)
	{
		if (AT2250_Init_Reg[j].op_code == AT2250_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (AT2250_Init_Reg[j].op_code == AT2250_OP_CODE_DLY)
		{
			msleep(AT2250_Init_Reg[j].init_val);		/* Delay */
		}
		else if (AT2250_Init_Reg[j].op_code == AT2250_OP_CODE_REG)
		{
		
			AT2250_write_cmos_sensor(AT2250_Init_Reg[j].init_reg, AT2250_Init_Reg[j].init_val);
		}
		else
		{
			printk("REG ERROR!\n");
		}
	}
#endif


	return 1;	
}

#endif


kal_uint16 AT2250_read_shutter(void)
{
return  (AT2250_read_cmos_sensor(0x02) << 8)|AT2250_read_cmos_sensor(0x01) ;
} /* AT2250 read_shutter */


static void AT2250_write_shutter(kal_uint32 shutter)
{
 	AT2250_write_cmos_sensor(0x01, (shutter & 0xFF));           //AEC[7:0]
	AT2250_write_cmos_sensor(0x02, ((shutter & 0xFF00) >>8));  //AEC[15:8]	

}    /* AT2250_write_shutter */


static void AT2250_set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 hvmirror = 0,reg0x27;

  hvmirror = AT2250_read_cmos_sensor(0x12);
  reg0x27=AT2250_read_cmos_sensor(0x27);
  

	switch (image_mirror)
	{
	case IMAGE_NORMAL:
		AT2250_write_cmos_sensor(0x12, hvmirror&0xcf);  //image normal   
		break;
	case IMAGE_H_MIRROR:
		AT2250_write_cmos_sensor(0x12, hvmirror|0x20);  
		AT2250_write_cmos_sensor(0x27, (AT2250_read_cmos_sensor(0x27)+1));  
		break;
	case IMAGE_V_MIRROR: 
		AT2250_write_cmos_sensor(0x12, hvmirror|0x10);  
		AT2250_write_cmos_sensor(0x27, (AT2250_read_cmos_sensor(0x27)));  
		break;		
	case IMAGE_HV_MIRROR:
		AT2250_write_cmos_sensor(0x12, hvmirror|0x30); 
		AT2250_write_cmos_sensor(0x27, (AT2250_read_cmos_sensor(0x27)+1));  
		break; 		
	default:
		break;
	}
}

static void AT2250_set_AE_mode(kal_bool AE_enable)
{
/*    kal_uint8 AeTemp;
	  AeTemp = AT2250_read_cmos_sensor(0x13);

    if (AE_enable == KAL_TRUE)
    {
        // turn on AEC/AGC
        AT2250_write_cmos_sensor(0x13,(AeTemp&(~0x01)));
    }
    else
    {
        // turn off AEC/AGC
      AT2250_write_cmos_sensor(0x13,(AeTemp|0x01));
    }
*/
}


static void AT2250_set_AWB_mode(kal_bool AWB_enable)
{
    kal_uint8 AwbTemp;
	  AwbTemp = AT2250_read_cmos_sensor(0x36);   

    if (AWB_enable == KAL_TRUE)
    {
             
	AT2250_write_cmos_sensor(0x36,AwbTemp|0x01); 

		
    }
    else
    {        
	AT2250_write_cmos_sensor(0x36,AwbTemp&(~0x01)); 
    }
}


/*************************************************************************
* FUNCTION
*	AT2250_night_mode
*
* DESCRIPTION
*	This function night mode of AT2250.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void AT2250_night_mode(kal_bool enable)
{
	
		/* ==Video Preview, Auto Mode, use 39MHz PCLK, 30fps; Night Mode use 39M, 15fps */
	kal_uint16 night = AT2250_read_cmos_sensor(0x13); 
		
	 kal_uint8 reg0x7e;

	reg0x7e = AT2250_read_cmos_sensor(0x7e);

	 
	if (enable)
	{ 
		            /* camera night mode */
	  #if 0
                AT2250_write_cmos_sensor(0x13,night&(~0x04)); // banding filter off

              AT2250_write_cmos_sensor(0x13,0x92);//enable auto frame rate adjust, max rate down 1/2

               mDELAY(33);//delay 33ms
                AT2250_write_cmos_sensor(0x13,0x92|0x04);//banding filter on
	  #else

	 
	   
	  AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xfb);
	  AT2250_write_cmos_sensor(0x7d, 0x30);

	  
	  #endif
				
    }
	else
	{   
	#if 0
				AT2250_write_cmos_sensor(0x13, night&(~0x04) );// banding filter off
				
				AT2250_write_cmos_sensor(0x13, /*night &*/ 0x4a);//disable auto frame rate adjust
				mDELAY(33);//delay 33ms
				AT2250_write_cmos_sensor(0x13, 0x4a | (0x04));//banding filter on
	#else
			//	AT2250_write_cmos_sensor(0x13 , 0x4e);
		AT2250_write_cmos_sensor(0x7e, reg0x7e | 0x04);
		AT2250_write_cmos_sensor(0x7d, 0x00);


	#endif
  }
	
}	/* AT2250_night_mode */



/*************************************************************************
* FUNCTION
*	AT2250_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS77
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 AT2250_GetSensorID(kal_uint32 *sensorID)

{
    int  retry = 3; 
	//AT2250_write_cmos_sensor(0x12,0x80);// Reset sensor
	//Sleep(30);
    // check if sensor ID correct
    do {
        *sensorID=(AT2250_read_cmos_sensor(0x0a) << 8) | AT2250_read_cmos_sensor(0x0b);
        if (*sensorID == AT2250_SENSOR_ID)
            break; 
        SENSORDB("Read Sensor ID Fail = 0x%04x\n", *sensorID); 
        retry--; 
    } while (retry > 0);

    if (*sensorID != AT2250_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
#ifdef AT2250_SWITCH

	IsAt2250 = 1;
#endif
    return ERROR_NONE;    
}   /* AT2250Open  */

#if  1

static void AT2250_Sensor_Init(void)
{
	zoom_factor = 0; 

#ifdef DEBUG_SENSOR
		if(AT2250_fromsd == 1)//是否从SD读取//gepeiwei   120903
			{	
				printk("________________from t!\n");
				AT2250_Initialize_from_T_Flash();//从SD卡读取的主要函数
			}
		else
#endif
{
#ifdef AT2250_MIPI
	AT2250_write_cmos_sensor(0x12, 0x80);	//reset sensor  
	mDELAY(30); ////Delay 100ms					 
	AT2250_write_cmos_sensor(0x12, 0x40);


	//analog                            
	AT2250_write_cmos_sensor(0x0c, 0x04);
	AT2250_write_cmos_sensor(0x30, 0x96);
	AT2250_write_cmos_sensor(0x31, 0x10);
	AT2250_write_cmos_sensor(0x32, 0x0c);
	AT2250_write_cmos_sensor(0x33, 0x28);
	AT2250_write_cmos_sensor(0xaa, 0x28);
	AT2250_write_cmos_sensor(0xab, 0x11);
	AT2250_write_cmos_sensor(0xac, 0x15);
	AT2250_write_cmos_sensor(0xad, 0x25);
	AT2250_write_cmos_sensor(0xae, 0x40);
	AT2250_write_cmos_sensor(0xaf, 0xa0);
	AT2250_write_cmos_sensor(0xb0, 0x40);
	AT2250_write_cmos_sensor(0xb1, 0x00);
	AT2250_write_cmos_sensor(0xb2, 0x74);
	AT2250_write_cmos_sensor(0xb3, 0x39); 

	//PLL  
	#if 0
	AT2250_write_cmos_sensor(0x0e, 0x10);
	AT2250_write_cmos_sensor(0x0f, 0x03);
	AT2250_write_cmos_sensor(0x10, 0x18);
	AT2250_write_cmos_sensor(0x11, 0x01);
	AT2250_write_cmos_sensor(0x0d, 0x88);
	 #else
		AT2250_write_cmos_sensor(0x0e,0x11);
		AT2250_write_cmos_sensor(0x0f,0x07); //7.5 ~ 15 fps
		AT2250_write_cmos_sensor(0x10,0x30); //
		AT2250_write_cmos_sensor(0x11,0x41); 
		AT2250_write_cmos_sensor(0x0d,0xd0); //
	 
	 #endif
	//DVP Port                           
	AT2250_write_cmos_sensor(0x1d, 0x00);
	AT2250_write_cmos_sensor(0x1e, 0x00);
	                     
	//MIPI   
	AT2250_write_cmos_sensor(0xb5, 0x50);

	#if 0
	AT2250_write_cmos_sensor(0xb7, 0xb6);
	AT2250_write_cmos_sensor(0xb8, 0xd1);
	AT2250_write_cmos_sensor(0xb9, 0xab);
	AT2250_write_cmos_sensor(0xba, 0x55);
	AT2250_write_cmos_sensor(0xbb, 0x16);
	AT2250_write_cmos_sensor(0xbc, 0x1e);
	AT2250_write_cmos_sensor(0xbd, 0x80);
	AT2250_write_cmos_sensor(0xbe, 0x0c);
	AT2250_write_cmos_sensor(0xbf, 0x14);
	#else
	AT2250_write_cmos_sensor(0xb7, 0xb6);
	AT2250_write_cmos_sensor(0xb8, 0xd1);
	AT2250_write_cmos_sensor(0xb9, 0xab);
	AT2250_write_cmos_sensor(0xba, 0x55);
	AT2250_write_cmos_sensor(0xbb, 0x16);
	AT2250_write_cmos_sensor(0xbc, 0x1e);
	AT2250_write_cmos_sensor(0xbd, 0x80);
	AT2250_write_cmos_sensor(0xbe, 0x0c);
	AT2250_write_cmos_sensor(0xbf, 0x15); // 
	
	#endif
	//Resolution                        
	AT2250_write_cmos_sensor(0x1b, 0x44);
	AT2250_write_cmos_sensor(0x1f, 0x40);
	AT2250_write_cmos_sensor(0x20, 0x8e);
	AT2250_write_cmos_sensor(0x21, 0x07);
	AT2250_write_cmos_sensor(0x22, 0xD8);
	AT2250_write_cmos_sensor(0x23, 0x04);
	AT2250_write_cmos_sensor(0x24, 0x40);
	AT2250_write_cmos_sensor(0x25, 0xB4);
	AT2250_write_cmos_sensor(0x26, 0x46);
	AT2250_write_cmos_sensor(0x27, 0xE4);//0xE3 140612
	AT2250_write_cmos_sensor(0x28, 0x0b);
	AT2250_write_cmos_sensor(0x29, 0x00);
	AT2250_write_cmos_sensor(0x2a, 0xd4);
	AT2250_write_cmos_sensor(0x2b, 0x10);
	AT2250_write_cmos_sensor(0x2c, 0x00);
	AT2250_write_cmos_sensor(0x2d, 0x00);
	AT2250_write_cmos_sensor(0x2e, 0x30);
	AT2250_write_cmos_sensor(0x2f, 0xa4);
	AT2250_write_cmos_sensor(0x38, 0x40);
	AT2250_write_cmos_sensor(0x39, 0xb0);
	AT2250_write_cmos_sensor(0x3a, 0x46);
	AT2250_write_cmos_sensor(0x3b, 0x40);
	AT2250_write_cmos_sensor(0x3c, 0xb0);
	AT2250_write_cmos_sensor(0x3d, 0x46);
	AT2250_write_cmos_sensor(0xbd, 0x80);
	AT2250_write_cmos_sensor(0xbe, 0x0c);
	                     
	// AEC/AGC 
	#if 0
	AT2250_write_cmos_sensor(0x13, 0x8e);
	AT2250_write_cmos_sensor(0x14, 0x4c);
	AT2250_write_cmos_sensor(0x15, 0x24);
	AT2250_write_cmos_sensor(0x16, 0x80);
	AT2250_write_cmos_sensor(0x17, 0x18);
	AT2250_write_cmos_sensor(0x18, 0xba);
	AT2250_write_cmos_sensor(0x19, 0x90);
	AT2250_write_cmos_sensor(0xa0, 0x55);
	AT2250_write_cmos_sensor(0xa1, 0x55);
	AT2250_write_cmos_sensor(0xa2, 0x55);
	AT2250_write_cmos_sensor(0xa3, 0x55);
	AT2250_write_cmos_sensor(0xa4, 0x04);
	#else
	AT2250_write_cmos_sensor(0x13, 0x8e);
	AT2250_write_cmos_sensor(0x14, 0x58);//0x48 140429 0x58//0x5c 140523
	AT2250_write_cmos_sensor(0x15, 0x23);
	AT2250_write_cmos_sensor(0x16, 0x72);
	AT2250_write_cmos_sensor(0x17, 0x38);//0x24
	AT2250_write_cmos_sensor(0x18, 0x93);//0xba 24M0x74
	AT2250_write_cmos_sensor(0x19, 0x91);//0x90 24M0x91
	#if 0
	AT2250_write_cmos_sensor(0xa0, 0x55);
	AT2250_write_cmos_sensor(0xa1, 0x55);
	AT2250_write_cmos_sensor(0xa2, 0x55);
	AT2250_write_cmos_sensor(0xa3, 0x55);
	AT2250_write_cmos_sensor(0xa4, 0x04);
	#else
	AT2250_write_cmos_sensor(0xa0, 0x69);
	AT2250_write_cmos_sensor(0xa1, 0xbe);
	AT2250_write_cmos_sensor(0xa2, 0xbe);
	AT2250_write_cmos_sensor(0xa3, 0x69);
	AT2250_write_cmos_sensor(0xa4, 0x04);//0x04
	#endif
	
	#endif
	//ISP                             
	AT2250_write_cmos_sensor(0x36, 0x7b);
	AT2250_write_cmos_sensor(0x37, 0x00);		//mod PC 03202013
	                     
	AT2250_write_cmos_sensor(0x40, 0x00);
	                     
	//BLC      
	AT2250_write_cmos_sensor(0x49, 0x00);//0x09 140527
	AT2250_write_cmos_sensor(0x4a, 0x0f);
	                     
	                             
	//AWB                               
	AT2250_write_cmos_sensor(0x83,0x0a);
	AT2250_write_cmos_sensor(0x84,0x40);
	AT2250_write_cmos_sensor(0x82,0x1e);
	AT2250_write_cmos_sensor(0x80,0xf0);
	AT2250_write_cmos_sensor(0x81,0x1e);
	
	AT2250_write_cmos_sensor(0x85,0x1E);//0x44 140429
	AT2250_write_cmos_sensor(0x86,0x64);//0x80 140429
	AT2250_write_cmos_sensor(0x87,0x7e);//0xa6 140429
	AT2250_write_cmos_sensor(0x88,0x90);//0x78 140429
	AT2250_write_cmos_sensor(0x89,0x9b);//0x6a 140429
	AT2250_write_cmos_sensor(0x8A,0x94);//0x73 140429
	AT2250_write_cmos_sensor(0x8B,0xad);//0x8d 140429
	AT2250_write_cmos_sensor(0x8C,0x8e);//0x86 140429
	AT2250_write_cmos_sensor(0x8D,0x62);//0x5f 140429
	AT2250_write_cmos_sensor(0x8E,0x5b);//0x61 140429
	AT2250_write_cmos_sensor(0x8F,0x52);//0x2a 140429
	AT2250_write_cmos_sensor(0x90,0xC8);
	
	//SDE                               
	AT2250_write_cmos_sensor(0x75, 0x00);
	AT2250_write_cmos_sensor(0x76, 0x40);
	AT2250_write_cmos_sensor(0x79, 0x00);
	AT2250_write_cmos_sensor(0x7a, 0x00);
	AT2250_write_cmos_sensor(0x7b, 0x00);
	AT2250_write_cmos_sensor(0x7c, 0x00);
	AT2250_write_cmos_sensor(0x7d, 0x00);
	AT2250_write_cmos_sensor(0x7e, 0x00);
	AT2250_write_cmos_sensor(0x7f, 0x80);
	                     
	//Saturation                        
	AT2250_write_cmos_sensor(0x77, 0x07);
	AT2250_write_cmos_sensor(0x78, 0x1b); //0x09
	                     
	//Gamma                             
	#if 1 //lj_test bird
			AT2250_write_cmos_sensor(0x50,0x06);
			AT2250_write_cmos_sensor(0x51,0x0b);
			AT2250_write_cmos_sensor(0x52,0x13);
			AT2250_write_cmos_sensor(0x53,0x20);
			AT2250_write_cmos_sensor(0x54,0x2d);
			AT2250_write_cmos_sensor(0x55,0x3a);
			AT2250_write_cmos_sensor(0x56,0x46);
			AT2250_write_cmos_sensor(0x57,0x50);
			AT2250_write_cmos_sensor(0x58,0x63);
			AT2250_write_cmos_sensor(0x59,0x73);
			AT2250_write_cmos_sensor(0x5a,0x82);
			AT2250_write_cmos_sensor(0x5b,0x91);
			AT2250_write_cmos_sensor(0x5c,0x9f);
			AT2250_write_cmos_sensor(0x5d,0xb9);
			AT2250_write_cmos_sensor(0x5e,0xd1);
			AT2250_write_cmos_sensor(0x5f,0xff);
	
/*
			AT2250_write_cmos_sensor(0x50,0x03);
			AT2250_write_cmos_sensor(0x51,0x07);
			AT2250_write_cmos_sensor(0x52,0x0c);
			AT2250_write_cmos_sensor(0x53,0x1a);
			AT2250_write_cmos_sensor(0x54,0x28);
			AT2250_write_cmos_sensor(0x55,0x38);
			AT2250_write_cmos_sensor(0x56,0x48);
			AT2250_write_cmos_sensor(0x57,0x59);
			AT2250_write_cmos_sensor(0x58,0x70);
			AT2250_write_cmos_sensor(0x59,0x86);
			AT2250_write_cmos_sensor(0x5a,0x9a);
			AT2250_write_cmos_sensor(0x5b,0xa8);
			AT2250_write_cmos_sensor(0x5c,0xb4);
			AT2250_write_cmos_sensor(0x5d,0xcc);
			AT2250_write_cmos_sensor(0x5e,0xe0);
			AT2250_write_cmos_sensor(0x5f,0xff);

	*/		
	#else
	AT2250_write_cmos_sensor(0x50,0x04);
	AT2250_write_cmos_sensor(0x51,0x08);//0x06 140429 
	AT2250_write_cmos_sensor(0x52,0x0f);//0x0c 140429
	AT2250_write_cmos_sensor(0x53,0x1e);//0x1a 140429
	AT2250_write_cmos_sensor(0x54,0x2b);//0x28 140429
	AT2250_write_cmos_sensor(0x55,0x38);
	AT2250_write_cmos_sensor(0x56,0x45);//0x48 140429
	AT2250_write_cmos_sensor(0x57,0x51);//0x59 140429
	AT2250_write_cmos_sensor(0x58,0x68);//0x70 140429
	AT2250_write_cmos_sensor(0x59,0x7d);//0x86 140429
	AT2250_write_cmos_sensor(0x5a,0x91);//0x9a 140429
	AT2250_write_cmos_sensor(0x5b,0xa3);//0xa8 140429
	AT2250_write_cmos_sensor(0x5c,0xb4);
	AT2250_write_cmos_sensor(0x5d,0xd1);//0xcc 140429
	AT2250_write_cmos_sensor(0x5e,0xe7);//0xe0 140429
	AT2250_write_cmos_sensor(0x5f,0xff);
	#endif
	                     
	//Color matrix                     
	AT2250_write_cmos_sensor(0x91,0x24);//0x11 140429 0x1f//0x2c 140523
	AT2250_write_cmos_sensor(0x92,0x2a);//0x2f 140429
	AT2250_write_cmos_sensor(0x93,0x06);//0x07 140429
	AT2250_write_cmos_sensor(0x94,0x13);//0x12 140429 0x10
	AT2250_write_cmos_sensor(0x95,0x4f);//0x49 140429 0x42
	AT2250_write_cmos_sensor(0x96,0x60);//0x5a 140429 0x50
	AT2250_write_cmos_sensor(0x97,0x61);//0x5f 140429 0x51//0x61 140523
	AT2250_write_cmos_sensor(0x98,0x61);//0x5c 140429 0x51
	AT2250_write_cmos_sensor(0x99,0x02);
	AT2250_write_cmos_sensor(0x9a,0x00);
	AT2250_write_cmos_sensor(0x9b,0x80);
	AT2250_write_cmos_sensor(0x9c,0x80);
	AT2250_write_cmos_sensor(0x9d,0xCe);
	AT2250_write_cmos_sensor(0x9e,0x08);
	                                 
	//Lens Correction
	AT2250_write_cmos_sensor(0x66,0x78);//BAWANG+0x10 0xa0 140429
	AT2250_write_cmos_sensor(0x67,0x6f);//BAWANG+0x10 0x90 140429
	AT2250_write_cmos_sensor(0x68,0x70);//BAWANG+0x10-8 0x90 140429
	AT2250_write_cmos_sensor(0x69,0x6a);//BAWANG+0x10-7 0x88 140429
	AT2250_write_cmos_sensor(0x6a,0x5d);//BAWANG+0x10 0x7e 140429
	AT2250_write_cmos_sensor(0x6b,0x50);//BAWANG+0x10 0x9a 140429
	AT2250_write_cmos_sensor(0x6c,0x14);//0x18 140429
	AT2250_write_cmos_sensor(0x6d,0x5c);
	AT2250_write_cmos_sensor(0x6e,0xBB);
	AT2250_write_cmos_sensor(0x6f,0x20);//0x28 140429
	AT2250_write_cmos_sensor(0x70,0x52);//0x62 140429
	AT2250_write_cmos_sensor(0x71,0x10);//0x18 140429
	AT2250_write_cmos_sensor(0x72,0x40);//0x70 140429
	AT2250_write_cmos_sensor(0x73,0x0B);
	                     
	//Edge/denoise                      
	AT2250_write_cmos_sensor(0x3e,0xa8);
	AT2250_write_cmos_sensor(0x3f,0x26);
	AT2250_write_cmos_sensor(0x60,0x1a);
	AT2250_write_cmos_sensor(0x61,0x1d);
	AT2250_write_cmos_sensor(0x62,0x1a);
	AT2250_write_cmos_sensor(0x63,0x1d);
	AT2250_write_cmos_sensor(0x64,0x90);
	AT2250_write_cmos_sensor(0x65,0xff);//0xaa 140527
	
	AT2250_write_cmos_sensor(0x05,0xa9);
	AT2250_write_cmos_sensor(0x06,0x00);
	AT2250_write_cmos_sensor(0x07,0x51);
	AT2250_write_cmos_sensor(0x08,0x54);
	AT2250_write_cmos_sensor(0x09,0x05);
	
	AT2250_write_cmos_sensor(0x12,0x32);
	mDELAY(30);      
}

#else
	AT2250_write_cmos_sensor(0x12,0x80); // soft reset 
	mDELAY(30);

#ifdef DEBUG_SENSOR
		if(AT2250_fromsd == 1)//是否从SD读取//gepeiwei   120903
			{
			
		printk("________________from t!\n");
		AT2250_Initialize_from_T_Flash();//从SD卡读取的主要函数
			}
		else
#endif
{
	
	AT2250_write_cmos_sensor(0x12,0x40); // sleep -- soft pwd

	//analog
	AT2250_write_cmos_sensor(0x0c,0x04);
	
	AT2250_write_cmos_sensor(0x30,0x96);
	AT2250_write_cmos_sensor(0x31,0x10);
	AT2250_write_cmos_sensor(0x32,0x0c);
	AT2250_write_cmos_sensor(0x33,0x28);
	AT2250_write_cmos_sensor(0xaa,0x28);
	AT2250_write_cmos_sensor(0xab,0x11);
	AT2250_write_cmos_sensor(0xac,0x15);
	AT2250_write_cmos_sensor(0xad,0x25);
	AT2250_write_cmos_sensor(0xae,0x40);
	AT2250_write_cmos_sensor(0xaf,0xa0);

	AT2250_write_cmos_sensor(0xb0,0x40);
	AT2250_write_cmos_sensor(0xb1,0x00);
	AT2250_write_cmos_sensor(0xb2,0x74);
	AT2250_write_cmos_sensor(0xb3,0x39);

	#if 0 // PLL
	AT2250_write_cmos_sensor(0x0e,0x10);
	AT2250_write_cmos_sensor(0x0f,0x07);
	AT2250_write_cmos_sensor(0x10,0x18);
	AT2250_write_cmos_sensor(0x11,0x41);
	AT2250_write_cmos_sensor(0x0d,0xd8);
	#else
		AT2250_write_cmos_sensor(0x0e,0x11);
		AT2250_write_cmos_sensor(0x0f,0x07); //7.5 ~ 15 fps
		AT2250_write_cmos_sensor(0x10,0x30); //
		AT2250_write_cmos_sensor(0x11,0x41); 
		AT2250_write_cmos_sensor(0x0d,0xd0); //
	#endif
	//DVP Port    
	AT2250_write_cmos_sensor(0x1d,0xff);
	AT2250_write_cmos_sensor(0x1e,0x9f);	     //0x9f

	//MIPI   
	AT2250_write_cmos_sensor(0xb5,0xF0);
	
	AT2250_write_cmos_sensor(0xb7,0xb6);
	AT2250_write_cmos_sensor(0xb8,0x71);
	AT2250_write_cmos_sensor(0xb9,0x8a);
	AT2250_write_cmos_sensor(0xba,0x47);
	AT2250_write_cmos_sensor(0xbb,0x16);
	AT2250_write_cmos_sensor(0xbc,0x1e);
	AT2250_write_cmos_sensor(0xbd,0x80);
	AT2250_write_cmos_sensor(0xbe,0x0c);
	AT2250_write_cmos_sensor(0xbf,0x1a);

		//Resolution                        
  #if 0
	AT2250_write_cmos_sensor(0x1b,0x44);
	AT2250_write_cmos_sensor(0x1f,0x50); //40
	AT2250_write_cmos_sensor(0x20,0x8e);
	AT2250_write_cmos_sensor(0x21,0x07);
	AT2250_write_cmos_sensor(0x22,0xD8);
	AT2250_write_cmos_sensor(0x23,0x04);
	AT2250_write_cmos_sensor(0x24,0x40);
	AT2250_write_cmos_sensor(0x25,0xB4);
	AT2250_write_cmos_sensor(0x26,0x46);
	AT2250_write_cmos_sensor(0x27,0xE3);
	AT2250_write_cmos_sensor(0x28,0x0b);
	AT2250_write_cmos_sensor(0x29,0x00);
	AT2250_write_cmos_sensor(0x2a,0xd4);
	AT2250_write_cmos_sensor(0x2b,0x10);
	AT2250_write_cmos_sensor(0x2c,0x00);
	AT2250_write_cmos_sensor(0x2d,0x00);
	AT2250_write_cmos_sensor(0x2e,0x30);
	AT2250_write_cmos_sensor(0x2f,0xa4);
	AT2250_write_cmos_sensor(0x38,0x40);
	AT2250_write_cmos_sensor(0x39,0xb0);
	AT2250_write_cmos_sensor(0x3a,0x46);
	AT2250_write_cmos_sensor(0x3b,0x40);
	AT2250_write_cmos_sensor(0x3c,0xb0);
	AT2250_write_cmos_sensor(0x3d,0x46);
	AT2250_write_cmos_sensor(0xbd,0x80);
	AT2250_write_cmos_sensor(0xbe,0x0c);
#else
		AT2250_write_cmos_sensor(0x1b,0x44);
		AT2250_write_cmos_sensor(0x1f,0x60);//40
		AT2250_write_cmos_sensor(0x20,0x8e);
		AT2250_write_cmos_sensor(0x21,0x07);
		AT2250_write_cmos_sensor(0x22,0xD8);
		AT2250_write_cmos_sensor(0x23,0x04);
		AT2250_write_cmos_sensor(0x24,0x40);
		AT2250_write_cmos_sensor(0x25,0xB4);
		AT2250_write_cmos_sensor(0x26,0x46);
		AT2250_write_cmos_sensor(0x27,0xE4);//  E3
		AT2250_write_cmos_sensor(0x28,0x0b);
		AT2250_write_cmos_sensor(0x29,0x00);
		AT2250_write_cmos_sensor(0x2a,0xd4);
		AT2250_write_cmos_sensor(0x2b,0x10);
		AT2250_write_cmos_sensor(0x2c,0x00);
		AT2250_write_cmos_sensor(0x2d,0x00);
		AT2250_write_cmos_sensor(0x2e,0x30);
		AT2250_write_cmos_sensor(0x2f,0xa4);
		AT2250_write_cmos_sensor(0x38,0x40);
		AT2250_write_cmos_sensor(0x39,0xb0);
		AT2250_write_cmos_sensor(0x3a,0x46);
		AT2250_write_cmos_sensor(0x3b,0x40);
		AT2250_write_cmos_sensor(0x3c,0xb0);
		AT2250_write_cmos_sensor(0x3d,0x46);
		AT2250_write_cmos_sensor(0xbd,0x80);
		AT2250_write_cmos_sensor(0xbe,0x0c);

#endif
	// AEC/AGC 
	AT2250_write_cmos_sensor(0x13,0x8e);//0x86
	AT2250_write_cmos_sensor(0x14,0x48);
	AT2250_write_cmos_sensor(0x15,0x23);
	AT2250_write_cmos_sensor(0x16,0x72);
	AT2250_write_cmos_sensor(0x17,0x24);
	AT2250_write_cmos_sensor(0x18,0xba);
	AT2250_write_cmos_sensor(0x19,0x90);

	#if 0
	AT2250_write_cmos_sensor(0xa0,0x55);
	AT2250_write_cmos_sensor(0xa1,0x55);
	AT2250_write_cmos_sensor(0xa2,0x55);
	AT2250_write_cmos_sensor(0xa3,0x55);
	AT2250_write_cmos_sensor(0xa4,0x04);
	#else
	AT2250_write_cmos_sensor(0xa0, 0x69);
	AT2250_write_cmos_sensor(0xa1, 0xbe);
	AT2250_write_cmos_sensor(0xa2, 0xbe);
	AT2250_write_cmos_sensor(0xa3, 0x69);
	AT2250_write_cmos_sensor(0xa4, 0x04);		
	#endif
	//ISP  
	AT2250_write_cmos_sensor(0x36,0x7b);
	AT2250_write_cmos_sensor(0x37,0x00);		//mod PC 03202013
	AT2250_write_cmos_sensor(0x40,0x00);
	//BLC  
	AT2250_write_cmos_sensor(0x49,0x09);
	AT2250_write_cmos_sensor(0x4a,0x0f);

	AT2250_write_cmos_sensor(0x12,0x02);	
	//AWB
	#if 0
	AT2250_write_cmos_sensor(0x83,0x02);
	AT2250_write_cmos_sensor(0x84,0x50);
	AT2250_write_cmos_sensor(0x85,0x16);
	AT2250_write_cmos_sensor(0x86,0x64);
	AT2250_write_cmos_sensor(0x87,0x7A);
	AT2250_write_cmos_sensor(0x88,0x9D);
	AT2250_write_cmos_sensor(0x89,0xd0);
	AT2250_write_cmos_sensor(0x8A,0xD0);
	AT2250_write_cmos_sensor(0x8B,0xE8);
	AT2250_write_cmos_sensor(0x8C,0x90);
	AT2250_write_cmos_sensor(0x8D,0x75);
	AT2250_write_cmos_sensor(0x8E,0x5D);
	AT2250_write_cmos_sensor(0x8F,0x50);
	AT2250_write_cmos_sensor(0x90,0xC8);
	#else
	AT2250_write_cmos_sensor(0x83,0x0a);
	AT2250_write_cmos_sensor(0x84,0x40);
	AT2250_write_cmos_sensor(0x82,0x1e);
	AT2250_write_cmos_sensor(0x80,0xf0);
	AT2250_write_cmos_sensor(0x81,0x1e);
	
	AT2250_write_cmos_sensor(0x85,0x44);
	AT2250_write_cmos_sensor(0x86,0x80);
	AT2250_write_cmos_sensor(0x87,0xa6);
	AT2250_write_cmos_sensor(0x88,0x78);
	AT2250_write_cmos_sensor(0x89,0x6a);
	AT2250_write_cmos_sensor(0x8A,0x73);
	AT2250_write_cmos_sensor(0x8B,0x8d);
	AT2250_write_cmos_sensor(0x8C,0x86);
	AT2250_write_cmos_sensor(0x8D,0x5f);
	AT2250_write_cmos_sensor(0x8E,0x61);
	AT2250_write_cmos_sensor(0x8F,0x2a);
	AT2250_write_cmos_sensor(0x90,0xC8);


	
	#endif
	//SDE 
	AT2250_write_cmos_sensor(0x75,0x00);
	AT2250_write_cmos_sensor(0x76,0x40);
	AT2250_write_cmos_sensor(0x79,0x00);
	AT2250_write_cmos_sensor(0x7a,0x00);
	AT2250_write_cmos_sensor(0x7b,0x00);
	AT2250_write_cmos_sensor(0x7c,0x00);
	AT2250_write_cmos_sensor(0x7d,0x00);
	AT2250_write_cmos_sensor(0x7e,0x00);
	AT2250_write_cmos_sensor(0x7f,0x80);
	//Saturation  
	
	AT2250_write_cmos_sensor(0x77,0x07);
	AT2250_write_cmos_sensor(0x78,0x09);
	//gamma	
#if	0															//mod PC 03202013
	AT2250_write_cmos_sensor(0x50,0x04);
	AT2250_write_cmos_sensor(0x51,0x08);
	AT2250_write_cmos_sensor(0x52,0x0f);
	AT2250_write_cmos_sensor(0x53,0x1e);
	AT2250_write_cmos_sensor(0x54,0x2b);
	AT2250_write_cmos_sensor(0x55,0x38);
	AT2250_write_cmos_sensor(0x56,0x45);
	AT2250_write_cmos_sensor(0x57,0x51);
	AT2250_write_cmos_sensor(0x58,0x68);
	AT2250_write_cmos_sensor(0x59,0x7d);
	AT2250_write_cmos_sensor(0x5a,0x91);
	AT2250_write_cmos_sensor(0x5b,0xa3);
	AT2250_write_cmos_sensor(0x5c,0xb4);
	AT2250_write_cmos_sensor(0x5d,0xd1);
	AT2250_write_cmos_sensor(0x5e,0xe7);
	AT2250_write_cmos_sensor(0x5f,0xff);
#else
	AT2250_write_cmos_sensor(0x50,0x04);
	AT2250_write_cmos_sensor(0x51,0x06);
	AT2250_write_cmos_sensor(0x52,0x0c);
	AT2250_write_cmos_sensor(0x53,0x1a);
	AT2250_write_cmos_sensor(0x54,0x28);
	AT2250_write_cmos_sensor(0x55,0x38);
	AT2250_write_cmos_sensor(0x56,0x48);
	AT2250_write_cmos_sensor(0x57,0x59);
	AT2250_write_cmos_sensor(0x58,0x70);
	AT2250_write_cmos_sensor(0x59,0x86);
	AT2250_write_cmos_sensor(0x5a,0x9a);
	AT2250_write_cmos_sensor(0x5b,0xa8);
	AT2250_write_cmos_sensor(0x5c,0xb4);
	AT2250_write_cmos_sensor(0x5d,0xcc);
	AT2250_write_cmos_sensor(0x5e,0xe0);
	AT2250_write_cmos_sensor(0x5f,0xff);

#endif
	//Color matrix                     
	AT2250_write_cmos_sensor(0x91,0x11);
	AT2250_write_cmos_sensor(0x92,0x2f);
	AT2250_write_cmos_sensor(0x93,0x07);
	AT2250_write_cmos_sensor(0x94,0x12);
	AT2250_write_cmos_sensor(0x95,0x49);
	AT2250_write_cmos_sensor(0x96,0x5a);
	AT2250_write_cmos_sensor(0x97,0x5f);
	AT2250_write_cmos_sensor(0x98,0x5c);
	AT2250_write_cmos_sensor(0x99,0x02);
	AT2250_write_cmos_sensor(0x9a,0x00);
	AT2250_write_cmos_sensor(0x9b,0x80);
	AT2250_write_cmos_sensor(0x9c,0x80);
	AT2250_write_cmos_sensor(0x9d,0xCe);
	AT2250_write_cmos_sensor(0x9e,0x08);
	//Lens Correction
	AT2250_write_cmos_sensor(0x66,0xa0);//BAWANG+0x10
	AT2250_write_cmos_sensor(0x67,0x90);//BAWANG+0x10
	AT2250_write_cmos_sensor(0x68,0x90);//BAWANG+0x10-8
	AT2250_write_cmos_sensor(0x69,0x88);//BAWANG+0x10-7
	AT2250_write_cmos_sensor(0x6a,0x7e);//BAWANG+0x10
	AT2250_write_cmos_sensor(0x6b,0x9a);//BAWANG+0x10
	AT2250_write_cmos_sensor(0x6c,0x18);//20  now left panel
	AT2250_write_cmos_sensor(0x6d,0x5c);//58
	AT2250_write_cmos_sensor(0x6e,0xBB);
	AT2250_write_cmos_sensor(0x6f,0x28);
	AT2250_write_cmos_sensor(0x70,0x62);
	AT2250_write_cmos_sensor(0x71,0x18);
	AT2250_write_cmos_sensor(0x72,0x70);
	AT2250_write_cmos_sensor(0x73,0x0B);
	//Edge/denoise                      
	AT2250_write_cmos_sensor(0x3e,0xa8);
	AT2250_write_cmos_sensor(0x3f,0x26);
	AT2250_write_cmos_sensor(0x60,0x0a);
	AT2250_write_cmos_sensor(0x61,0x0d);
	AT2250_write_cmos_sensor(0x62,0x0a);
	AT2250_write_cmos_sensor(0x63,0x0d);
	AT2250_write_cmos_sensor(0x64,0x88);
	AT2250_write_cmos_sensor(0x65,0xaa);
	


	AT2250_write_cmos_sensor(0x05,0xcc);
	AT2250_write_cmos_sensor(0x06,0x00);
	AT2250_write_cmos_sensor(0x07,0x60);
	AT2250_write_cmos_sensor(0x08,0x54);
	AT2250_write_cmos_sensor(0x09,0x04);



		{
	//	AT2250_write_cmos_sensor(0x12,0x42);
	//	AT2250_write_cmos_sensor(0x1d,0xff);	  //enable DVP pads
	//	AT2250_write_cmos_sensor(0x1e,0x9f);		//enable DVP pads  0x9f
	//	AT2250_write_cmos_sensor(0xb5,0xf0);		//disabled mipi

	/*	AT2250_write_cmos_sensor(0x18,0xba);		
		AT2250_write_cmos_sensor(0x0e,0x11);
		AT2250_write_cmos_sensor(0x0f,0x07); //7.5 ~ 15 fps
		AT2250_write_cmos_sensor(0x10,0x30); //
		AT2250_write_cmos_sensor(0x11,0x41); 
		AT2250_write_cmos_sensor(0x0d,0xd0); //mod Der 0530  drive cap 00 to 10//15fps 
	*/
	
	//	AT2250_write_cmos_sensor(0x0e,0x11);
	//	AT2250_write_cmos_sensor(0x0f,0x17);
	//	AT2250_write_cmos_sensor(0x10,0x40);
	//	AT2250_write_cmos_sensor(0x11,0x01);//10FPS
	//	AT2250_write_cmos_sensor(0x0d,0x98);


	//Resolution                        
		
	/*	AT2250_write_cmos_sensor(0x1b,0x44);
		AT2250_write_cmos_sensor(0x1f,0x60);//40
		AT2250_write_cmos_sensor(0x20,0x8e);
		AT2250_write_cmos_sensor(0x21,0x07);
		AT2250_write_cmos_sensor(0x22,0xD8);
		AT2250_write_cmos_sensor(0x23,0x04);
		AT2250_write_cmos_sensor(0x24,0x40);
		AT2250_write_cmos_sensor(0x25,0xB4);
		AT2250_write_cmos_sensor(0x26,0x46);
		AT2250_write_cmos_sensor(0x27,0xE4);//  E3
		AT2250_write_cmos_sensor(0x28,0x0b);
		AT2250_write_cmos_sensor(0x29,0x00);
		AT2250_write_cmos_sensor(0x2a,0xd4);
		AT2250_write_cmos_sensor(0x2b,0x10);
		AT2250_write_cmos_sensor(0x2c,0x00);
		AT2250_write_cmos_sensor(0x2d,0x00);
		AT2250_write_cmos_sensor(0x2e,0x30);
		AT2250_write_cmos_sensor(0x2f,0xa4);
		AT2250_write_cmos_sensor(0x38,0x40);
		AT2250_write_cmos_sensor(0x39,0xb0);
		AT2250_write_cmos_sensor(0x3a,0x46);
		AT2250_write_cmos_sensor(0x3b,0x40);
		AT2250_write_cmos_sensor(0x3c,0xb0);
		AT2250_write_cmos_sensor(0x3d,0x46);
		AT2250_write_cmos_sensor(0xbd,0x80);
		AT2250_write_cmos_sensor(0xbe,0x0c);
*/

	//	AT2250_write_cmos_sensor(0x19,0x90);
		AT2250_write_cmos_sensor(0x36,0x7b);



		AT2250_write_cmos_sensor(0x12,0x02);// 0x32  jinfewei 
			}
	
	mDELAY(400);
}	
#endif
}

#endif
static void AT2250_Sensor_UXGA(void)
{
//	AT2250_write_cmos_sensor(0xb5, 0x50);
	AT2250_write_cmos_sensor(0x12, 0x40);

	AT2250_write_cmos_sensor(0x0e,0x11);
	AT2250_write_cmos_sensor(0x0f,0x07); //7.5 ~ 15 fps
	AT2250_write_cmos_sensor(0x10,0x30); //
	AT2250_write_cmos_sensor(0x11,0x41); 
	AT2250_write_cmos_sensor(0x0d,0xd0); //
	
	AT2250_write_cmos_sensor(0x13,0x8E);

	//Resolution                        
	AT2250_write_cmos_sensor(0x1b, 0x44);
	AT2250_write_cmos_sensor(0x1f, 0x40);
	AT2250_write_cmos_sensor(0x20, 0x8e);
	AT2250_write_cmos_sensor(0x21, 0x07);
	AT2250_write_cmos_sensor(0x22, 0xD8);
	AT2250_write_cmos_sensor(0x23, 0x04);
	AT2250_write_cmos_sensor(0x24, 0x40);
	AT2250_write_cmos_sensor(0x25, 0xB4);
	AT2250_write_cmos_sensor(0x26, 0x46);
	AT2250_write_cmos_sensor(0x27, 0xE4);//0xE3
	AT2250_write_cmos_sensor(0x28, 0x0b);
	AT2250_write_cmos_sensor(0x29, 0x00);
	AT2250_write_cmos_sensor(0x2a, 0xd4);
	AT2250_write_cmos_sensor(0x2b, 0x10);
	AT2250_write_cmos_sensor(0x2c, 0x00);
	AT2250_write_cmos_sensor(0x2d, 0x00);
	AT2250_write_cmos_sensor(0x2e, 0x30);
	AT2250_write_cmos_sensor(0x2f, 0xa4);
	AT2250_write_cmos_sensor(0x38, 0x40);
	AT2250_write_cmos_sensor(0x39, 0xb0);
	AT2250_write_cmos_sensor(0x3a, 0x46);
	AT2250_write_cmos_sensor(0x3b, 0x40);
	AT2250_write_cmos_sensor(0x3c, 0xb0);
	AT2250_write_cmos_sensor(0x3d, 0x46);
	AT2250_write_cmos_sensor(0xbd, 0x80);
	AT2250_write_cmos_sensor(0xbe, 0x0c);
	
	AT2250_write_cmos_sensor(0xb7, 0xb6);
	AT2250_write_cmos_sensor(0xb8, 0xd1);
	AT2250_write_cmos_sensor(0xb9, 0xab);
	AT2250_write_cmos_sensor(0xba, 0x55);
	AT2250_write_cmos_sensor(0xbb, 0x16);
	AT2250_write_cmos_sensor(0xbc, 0x1e);
	AT2250_write_cmos_sensor(0xbd, 0x80);
	AT2250_write_cmos_sensor(0xbe, 0x0c);
	AT2250_write_cmos_sensor(0xbf, 0x15); // 

	AT2250_write_cmos_sensor(0xa4, 0x04);//add 
	AT2250_write_cmos_sensor(0x13, AT2250_read_cmos_sensor(0x13) & 0xfb); //bandong off

	AT2250_write_cmos_sensor(0x14, 0x58);//0x48 140429 0x58//0x5c 140523
	AT2250_write_cmos_sensor(0x15, 0x23);
	AT2250_write_cmos_sensor(0x16, 0x72);
	AT2250_write_cmos_sensor(0x17, 0x38);//0x24
	
	AT2250_write_cmos_sensor(0x18, 0x93); //0xba 24M0x74
	AT2250_write_cmos_sensor(0x19, 0x91); //0x90 24M0x91
	AT2250_write_cmos_sensor(0x13, AT2250_read_cmos_sensor(0x13) | 0x04); //bandong off

	AT2250_write_cmos_sensor(0x3e,0xac);//0xa8  140523
	AT2250_write_cmos_sensor(0x3F,0x26);//add  140523//0x22  140527
	AT2250_write_cmos_sensor(0x65,0xff);//add  140527

	AT2250_write_cmos_sensor(0x78,0x1b);
	//AT2250_write_cmos_sensor(0xa4,0x06);


	AT2250_write_cmos_sensor(0x36,0x7b);
	
	AT2250_write_cmos_sensor(0x12, 0x32);//0x02
	
	AT2250_write_cmos_sensor(0x91,0x24);//0x11 140429 0x1f//0x2c 140523
	
	AT2250_write_cmos_sensor(0xb5, 0x50);
}
static void AT2250_Sensor_VGA(void)
{
	// windows and fps
	//640x480bin, YUV 24Mhz, 21fps
	   
	//AT2250_write_cmos_sensor(0xb5,0xF0);
	#if 1 // ZTE
	AT2250_write_cmos_sensor(0x12,0x43);

	AT2250_write_cmos_sensor(0x0e,0x10);//	
	AT2250_write_cmos_sensor(0x0f,0x17);//
	AT2250_write_cmos_sensor(0x10,0x1b);//
	AT2250_write_cmos_sensor(0x11,0x01);//
	AT2250_write_cmos_sensor(0x0d,0x90);//
	AT2250_write_cmos_sensor(0xa4, 0x06);//add 
	AT2250_write_cmos_sensor(0x13,0x86);


	AT2250_write_cmos_sensor(0x1b,0x44);
	AT2250_write_cmos_sensor(0x1f,0x42);
	AT2250_write_cmos_sensor(0x20,0x52);
	AT2250_write_cmos_sensor(0x21,0x06);
	AT2250_write_cmos_sensor(0x22,0x6a);
	AT2250_write_cmos_sensor(0x23,0x02);
	AT2250_write_cmos_sensor(0x24,0x20);
	AT2250_write_cmos_sensor(0x25,0x58);
	AT2250_write_cmos_sensor(0x26,0x23);
	AT2250_write_cmos_sensor(0x27,0x8e);//
	AT2250_write_cmos_sensor(0x28,0x08);//
	AT2250_write_cmos_sensor(0x29,0x01);
	AT2250_write_cmos_sensor(0x2a,0x7E);
	AT2250_write_cmos_sensor(0x2b,0x11);
	AT2250_write_cmos_sensor(0x2c,0x02);//
	AT2250_write_cmos_sensor(0x2d,0x03);//
	AT2250_write_cmos_sensor(0x2e,0x31);
	AT2250_write_cmos_sensor(0x2f,0x24);

	AT2250_write_cmos_sensor(0x38,0x00);//
	AT2250_write_cmos_sensor(0x39,0x40);//
	AT2250_write_cmos_sensor(0x3a,0x23);
	AT2250_write_cmos_sensor(0x3b,0x80);
	AT2250_write_cmos_sensor(0x3c,0xE0);
	AT2250_write_cmos_sensor(0x3d,0x12);
	AT2250_write_cmos_sensor(0xbd,0x00);
	AT2250_write_cmos_sensor(0xbe,0x05);
		

	AT2250_write_cmos_sensor(0xb7,0x6E);//
	AT2250_write_cmos_sensor(0xb8,0x6A);//
	AT2250_write_cmos_sensor(0xb9,0x69);//
	AT2250_write_cmos_sensor(0xba,0x33);//
	AT2250_write_cmos_sensor(0xbb,0x14);//
	AT2250_write_cmos_sensor(0xbf,0x0f);//


	AT2250_write_cmos_sensor(0x13, AT2250_read_cmos_sensor(0x13) & 0xfb); //bandong off

	AT2250_write_cmos_sensor(0x14, 0x58);//0x48 140429 0x58//0x5c 140523
	AT2250_write_cmos_sensor(0x15, 0x23);
	AT2250_write_cmos_sensor(0x16, 0x80);
	AT2250_write_cmos_sensor(0x17, 0x24);//0x24

	AT2250_write_cmos_sensor(0x18, 0x96);//7D  24M0x77
	AT2250_write_cmos_sensor(0x19, 0x91);//90  24M0x91
	AT2250_write_cmos_sensor(0x13, AT2250_read_cmos_sensor(0x13) | 0x04); //bandong off

    	AT2250_write_cmos_sensor(0x3e,0xa0);//sharp  //0x9a 140523

	AT2250_write_cmos_sensor(0x3F,0x26);//add  140523
	AT2250_write_cmos_sensor(0x65,0xff);//add  140527    

	AT2250_write_cmos_sensor(0x78,0x1b);//0x09
	//AT2250_write_cmos_sensor(0xa4,0x04);
	AT2250_write_cmos_sensor(0x91,0x2f);//add

	
	AT2250_write_cmos_sensor(0x36,0x7f);
	AT2250_write_cmos_sensor(0x12,0x33);//0x03

	AT2250_write_cmos_sensor(0xb5,0x50);
	#else //it is ok
	AT2250_write_cmos_sensor(0x12,0x43);

	AT2250_write_cmos_sensor(0x0e,0x10);	
	AT2250_write_cmos_sensor(0x0f,0x17);
	AT2250_write_cmos_sensor(0x10,0x1c);	
	AT2250_write_cmos_sensor(0x11,0x01);	
	AT2250_write_cmos_sensor(0x0d,0x90);
	
	AT2250_write_cmos_sensor(0x13,0x86);


	AT2250_write_cmos_sensor(0x1b,0x44);
	AT2250_write_cmos_sensor(0x1f,0x42);
	AT2250_write_cmos_sensor(0x20,0x52);
	AT2250_write_cmos_sensor(0x21,0x06);
	AT2250_write_cmos_sensor(0x22,0x6a);
	AT2250_write_cmos_sensor(0x23,0x02);
	AT2250_write_cmos_sensor(0x24,0x20);
	AT2250_write_cmos_sensor(0x25,0x58);
	AT2250_write_cmos_sensor(0x26,0x23);
	AT2250_write_cmos_sensor(0x27,0x8e); //8d
	AT2250_write_cmos_sensor(0x28,0x07);
	AT2250_write_cmos_sensor(0x29,0x01);
	AT2250_write_cmos_sensor(0x2a,0x7E);
	AT2250_write_cmos_sensor(0x2b,0x11);
	AT2250_write_cmos_sensor(0x2c,0x00);
	AT2250_write_cmos_sensor(0x2d,0x00);
	AT2250_write_cmos_sensor(0x2e,0x31);
	AT2250_write_cmos_sensor(0x2f,0x24);

	AT2250_write_cmos_sensor(0x38,0x20);
	AT2250_write_cmos_sensor(0x39,0x58);
	AT2250_write_cmos_sensor(0x3a,0x23);
	AT2250_write_cmos_sensor(0x3b,0x80);
	AT2250_write_cmos_sensor(0x3c,0xE0);
	AT2250_write_cmos_sensor(0x3d,0x12);
	AT2250_write_cmos_sensor(0xbd,0x00);
	AT2250_write_cmos_sensor(0xbe,0x05);
		

	AT2250_write_cmos_sensor(0xb7,0x6E);
	AT2250_write_cmos_sensor(0xb8,0x8A);
	AT2250_write_cmos_sensor(0xb9,0x6A);
	AT2250_write_cmos_sensor(0xba,0x43);
	AT2250_write_cmos_sensor(0xbb,0x04);
	AT2250_write_cmos_sensor(0xbf,0x06);

	AT2250_write_cmos_sensor(0x13, AT2250_read_cmos_sensor(0x13) & 0xfb); //bandong off
	AT2250_write_cmos_sensor(0x18,0x86);//82 
	AT2250_write_cmos_sensor(0x19,0x91);//90
	AT2250_write_cmos_sensor(0x13, AT2250_read_cmos_sensor(0x13) | 0x04); //bandong off

       AT2250_write_cmos_sensor(0x3e,0x9a);//sharp
	   
	AT2250_write_cmos_sensor(0x36,0x7f);
	AT2250_write_cmos_sensor(0x12,0x33);//0x03

	AT2250_write_cmos_sensor(0xb5,0x50);
	#endif
}

static void AT2250_Sensor_SVGA(void)
{
	AT2250_write_cmos_sensor(0x13,AT2250_read_cmos_sensor(0x13)|0x01);//close auto ALC
	AT2250_write_cmos_sensor(0x13,AT2250_read_cmos_sensor(0x13)&(~0x01)); //open auto ALC
#ifdef AT2250_MIPI
			AT2250_write_cmos_sensor(0x12,0x43);
			AT2250_write_cmos_sensor(0x1d,0x00);	  //disable DVP pads
			AT2250_write_cmos_sensor(0x1e,0x00);	  //disable DVP pads
			AT2250_write_cmos_sensor(0xb5,0x50);		// enabled mipi
			AT2250_write_cmos_sensor(0x0e,0x10);
			AT2250_write_cmos_sensor(0x0f,0x17);
			AT2250_write_cmos_sensor(0x10,0x18);
			AT2250_write_cmos_sensor(0x11,0x01);
			AT2250_write_cmos_sensor(0x0d,0xd8);
			AT2250_write_cmos_sensor(0x1b,0x44);
			AT2250_write_cmos_sensor(0x1f,0x42);
			AT2250_write_cmos_sensor(0x20,0xA6);
			AT2250_write_cmos_sensor(0x21,0x04);
			AT2250_write_cmos_sensor(0x22,0xF0);
			AT2250_write_cmos_sensor(0x23,0x03);
			AT2250_write_cmos_sensor(0x24,0x20);
			AT2250_write_cmos_sensor(0x25,0x58);
			AT2250_write_cmos_sensor(0x26,0x23);
			AT2250_write_cmos_sensor(0x27,0x8D);
			AT2250_write_cmos_sensor(0x28,0x07);
			AT2250_write_cmos_sensor(0x29,0x01);
			AT2250_write_cmos_sensor(0x2a,0x7E);
			AT2250_write_cmos_sensor(0x2b,0x11);
			AT2250_write_cmos_sensor(0x2c,0x00);
			AT2250_write_cmos_sensor(0x2d,0x00);
			AT2250_write_cmos_sensor(0x2e,0x31);
			AT2250_write_cmos_sensor(0x2f,0xa4);
			AT2250_write_cmos_sensor(0x38,0x20);
			AT2250_write_cmos_sensor(0x39,0x58);
			AT2250_write_cmos_sensor(0x3a,0x23);
			AT2250_write_cmos_sensor(0x3b,0x20);
			AT2250_write_cmos_sensor(0x3c,0x58);
			AT2250_write_cmos_sensor(0x3d,0x23);
			AT2250_write_cmos_sensor(0xbd,0x40);
			AT2250_write_cmos_sensor(0xbe,0x06);
			AT2250_write_cmos_sensor(0xb7,0x6a);

			AT2250_write_cmos_sensor(0xb8,0x69);
			AT2250_write_cmos_sensor(0xb9,0x69);
			AT2250_write_cmos_sensor(0xba,0x33);
			AT2250_write_cmos_sensor(0xbb,0x16);
			AT2250_write_cmos_sensor(0xbf,0x24);
			AT2250_write_cmos_sensor(0x18,0x97);
			AT2250_write_cmos_sensor(0x19,0x90);
			AT2250_write_cmos_sensor(0x36,0x7b);
			AT2250_write_cmos_sensor(0x12,0x03);
	
#else
	//DVP
			AT2250_write_cmos_sensor(0x12,0x43);
			AT2250_write_cmos_sensor(0x1d,0xff);	  //enable DVP pads
			AT2250_write_cmos_sensor(0x1e,0x9f);		//enable DVP pads	//0x9f
			AT2250_write_cmos_sensor(0xb5,0xf0);		//disabled mipi
			
			AT2250_write_cmos_sensor(0x0e,0x10);
			AT2250_write_cmos_sensor(0x0f,0x09);	//0x07);
			AT2250_write_cmos_sensor(0x10,0x19);	//mod PC 03202013
			AT2250_write_cmos_sensor(0x11,0x42);
			AT2250_write_cmos_sensor(0x0d,0xd8);
			AT2250_write_cmos_sensor(0x1b,0x44);
			AT2250_write_cmos_sensor(0x1f,0x42);
			AT2250_write_cmos_sensor(0x20,0xA6);
			AT2250_write_cmos_sensor(0x21,0x04);
			AT2250_write_cmos_sensor(0x22,0xF0);
			AT2250_write_cmos_sensor(0x23,0x03);
			AT2250_write_cmos_sensor(0x24,0x20);
			AT2250_write_cmos_sensor(0x25,0x58);
			AT2250_write_cmos_sensor(0x26,0x23);
			AT2250_write_cmos_sensor(0x27,0x8D);
			AT2250_write_cmos_sensor(0x28,0x07);
			AT2250_write_cmos_sensor(0x29,0x01);
			AT2250_write_cmos_sensor(0x2a,0x7E);
			AT2250_write_cmos_sensor(0x2b,0x11);
			AT2250_write_cmos_sensor(0x2c,0x00);
			AT2250_write_cmos_sensor(0x2d,0x00);
			AT2250_write_cmos_sensor(0x2e,0x31);
			AT2250_write_cmos_sensor(0x2f,0xa4);
			AT2250_write_cmos_sensor(0x38,0x20);
			AT2250_write_cmos_sensor(0x39,0x58);
			AT2250_write_cmos_sensor(0x3a,0x23);
			AT2250_write_cmos_sensor(0x3b,0x20);
			AT2250_write_cmos_sensor(0x3c,0x58);
			AT2250_write_cmos_sensor(0x3d,0x23);
			AT2250_write_cmos_sensor(0xbd,0x40);
			AT2250_write_cmos_sensor(0xbe,0x06);
			AT2250_write_cmos_sensor(0x18,0x97);
			AT2250_write_cmos_sensor(0x19,0x90);
			AT2250_write_cmos_sensor(0x36,0x7b);
			AT2250_write_cmos_sensor(0x12,0x03);		  
			AT2250_write_cmos_sensor(0xa5,0x21);
#endif

}

static void AT2250_Sensor_2M(void)
{
	    #ifdef AT2250_MIPI
		AT2250_write_cmos_sensor(0x12,0x42);
		AT2250_write_cmos_sensor(0x1d,0x00);	  //disable DVP pads
		AT2250_write_cmos_sensor(0x1e,0x00);	  //disable DVP pads
		AT2250_write_cmos_sensor(0xb5,0x50);		// enabled mipi
		AT2250_write_cmos_sensor(0x0e,0x11);
		AT2250_write_cmos_sensor(0x0f,0x17);
		AT2250_write_cmos_sensor(0x10,0x40);
		AT2250_write_cmos_sensor(0x11,0x01);//10FPS
		AT2250_write_cmos_sensor(0x0d,0x98);// 98drv capa
		AT2250_write_cmos_sensor(0x1b,0x44);
		AT2250_write_cmos_sensor(0x1f,0x50);
		AT2250_write_cmos_sensor(0x20,0x8e);
		AT2250_write_cmos_sensor(0x21,0x07);
		AT2250_write_cmos_sensor(0x22,0xD8);
		AT2250_write_cmos_sensor(0x23,0x04);
		AT2250_write_cmos_sensor(0x24,0x40);
		AT2250_write_cmos_sensor(0x25,0xB4);
		AT2250_write_cmos_sensor(0x26,0x46);
		AT2250_write_cmos_sensor(0x27,0xE3);
		AT2250_write_cmos_sensor(0x28,0x0b);
		AT2250_write_cmos_sensor(0x29,0x00);
		AT2250_write_cmos_sensor(0x2a,0xd4);
		AT2250_write_cmos_sensor(0x2b,0x10);
		AT2250_write_cmos_sensor(0x2c,0x00);
		AT2250_write_cmos_sensor(0x2d,0x00);
		AT2250_write_cmos_sensor(0x2e,0x30);
		AT2250_write_cmos_sensor(0x2f,0xa4);
		AT2250_write_cmos_sensor(0x38,0x40);
		AT2250_write_cmos_sensor(0x39,0xb0);
		AT2250_write_cmos_sensor(0x3a,0x46);
		AT2250_write_cmos_sensor(0x3b,0x40);
		AT2250_write_cmos_sensor(0x3c,0xb0);
		AT2250_write_cmos_sensor(0x3d,0x46);
		AT2250_write_cmos_sensor(0xbd,0x80);
		AT2250_write_cmos_sensor(0xbe,0x0c);
		AT2250_write_cmos_sensor(0xb7,0x6e);
		AT2250_write_cmos_sensor(0xb8,0xac);
		AT2250_write_cmos_sensor(0xb9,0x6a);
		AT2250_write_cmos_sensor(0xba,0x43);
		AT2250_write_cmos_sensor(0xbb,0x16);
		AT2250_write_cmos_sensor(0xbf,0x1c);
		AT2250_write_cmos_sensor(0x18,0x7c);
		AT2250_write_cmos_sensor(0x19,0x90);
		AT2250_write_cmos_sensor(0x36,0x7b);
		AT2250_write_cmos_sensor(0x12,0x02);
	
#else	//DVP
		AT2250_write_cmos_sensor(0x12,0x42);
		AT2250_write_cmos_sensor(0x1d,0xff);	  //enable DVP pads
		AT2250_write_cmos_sensor(0x1e,0x9f);		//enable DVP pads  0x9f
		AT2250_write_cmos_sensor(0xb5,0xf0);		//disabled mipi

		AT2250_write_cmos_sensor(0x18,0x7c);		
		//AT2250_write_cmos_sensor(0x0e,0x10);
		//AT2250_write_cmos_sensor(0x0f,0x07); //mod PC 03202013 10-7.5fps
		//AT2250_write_cmos_sensor(0x10,0x18); //mod PC 03202013 10-7.5fps
		//AT2250_write_cmos_sensor(0x11,0x41); 
		//AT2250_write_cmos_sensor(0x0d,0xD8); //mod Der 0530  drive cap 00 to 10//15fps 

		AT2250_write_cmos_sensor(0x0e,0x11);
		AT2250_write_cmos_sensor(0x0f,0x17);
		AT2250_write_cmos_sensor(0x10,0x40);
		AT2250_write_cmos_sensor(0x11,0x01);//10FPS
		AT2250_write_cmos_sensor(0x0d,0x98);
		
		AT2250_write_cmos_sensor(0x1b,0x44);
		AT2250_write_cmos_sensor(0x1f,0x50);//40
		AT2250_write_cmos_sensor(0x20,0x8e);
		AT2250_write_cmos_sensor(0x21,0x07);
		AT2250_write_cmos_sensor(0x22,0xD8);
		AT2250_write_cmos_sensor(0x23,0x04);
		AT2250_write_cmos_sensor(0x24,0x40);
		AT2250_write_cmos_sensor(0x25,0xB4);
		AT2250_write_cmos_sensor(0x26,0x46);
		AT2250_write_cmos_sensor(0x27,0xE4);//  E3
		AT2250_write_cmos_sensor(0x28,0x0b);
		AT2250_write_cmos_sensor(0x29,0x00);
		AT2250_write_cmos_sensor(0x2a,0xd4);
		AT2250_write_cmos_sensor(0x2b,0x10);
		AT2250_write_cmos_sensor(0x2c,0x00);
		AT2250_write_cmos_sensor(0x2d,0x00);
		AT2250_write_cmos_sensor(0x2e,0x30);
		AT2250_write_cmos_sensor(0x2f,0xa4);
		AT2250_write_cmos_sensor(0x38,0x40);
		AT2250_write_cmos_sensor(0x39,0xb0);
		AT2250_write_cmos_sensor(0x3a,0x46);
		AT2250_write_cmos_sensor(0x3b,0x40);
		AT2250_write_cmos_sensor(0x3c,0xb0);
		AT2250_write_cmos_sensor(0x3d,0x46);
		AT2250_write_cmos_sensor(0xbd,0x80);
		AT2250_write_cmos_sensor(0xbe,0x0c);

		AT2250_write_cmos_sensor(0x19,0x90);
		AT2250_write_cmos_sensor(0x36,0x7b);



		AT2250_write_cmos_sensor(0x12,0x02);// 0x32  jinfewei 
		#endif

}

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	AT2250Open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 AT2250Open(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	zoom_factor = 0; 
	AT2250_write_cmos_sensor(0x12,0x80);// Reset sensor
	Sleep(10);


	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = (AT2250_read_cmos_sensor(0x0a) << 8) | AT2250_read_cmos_sensor(0x0b);
		SENSORDB("AT2250 READ ID :%x",sensor_id);
		if(sensor_id != AT2250_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	
	SENSORDB("AT2250 Sensor Read ID OK \r\n");

#ifdef DEBUG_SENSOR 

 struct file *fp; 
    mm_segment_t fs; 
    loff_t pos = 0; 
	static char buf[10*1024] ;
	//kal_uint16 AT_CARD_tmp = 0 ;
 for(AT_CARD_tmp=0;AT_CARD_tmp<4;AT_CARD_tmp++)
  {
    switch(AT_CARD_tmp)
	{
	case 0 :
		 fp = filp_open("/mnt/sdcard/at2250_sd", O_RDONLY , 0); 
		break;
	case 1 :
	 	fp = filp_open("/mnt/sdcard0/at2250_sd", O_RDONLY , 0); 
		break;
	case 2 :
		 fp = filp_open("/mnt/sdcard1/at2250_sd", O_RDONLY , 0); 
		break;
	case 3 :
		 fp = filp_open("/mnt/sdcard2/at2250_sd", O_RDONLY , 0); 
		break;

	}
 //   fp = filp_open("/mnt/sdcard2/at2250_sd", O_RDONLY , 0); 
    if (IS_ERR(fp)) { 
		AT2250_fromsd = 0;   
		printk("open file error\n");
		
    } 
	else 
		{
		AT2250_fromsd = 1;
	printk("open file ok\n");
		
	//AT2250_Initialize_from_T_Flash();


	filp_close(fp, NULL); 
	    set_fs(fs);
	break;
	}
	
	}
#endif

	
	AT2250_Sensor_Init();
//	AT2250_set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}	/* AT2250Open() */

/*************************************************************************
* FUNCTION
*	AT2250Close
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 AT2250Close(void)
{
//	CISModulePowerOn(FALSE);
  #ifdef AT2250_MIPI
		AT2250_write_cmos_sensor(0x1d,0x00);
		AT2250_write_cmos_sensor(0x1e,0x00);
		AT2250_write_cmos_sensor(0xb5,0xf0);
		AT2250_write_cmos_sensor(0x12,0x40);

  #else	//DVP		
		AT2250_write_cmos_sensor(0x1d,0x00);
		AT2250_write_cmos_sensor(0x1e,0x00);
		AT2250_write_cmos_sensor(0xb5,0xf0);
		AT2250_write_cmos_sensor(0x12,0x40);
 #endif
	return ERROR_NONE;
}	/* AT2250Close() */

/*************************************************************************
* FUNCTION
*	AT2250Preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 AT2250Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint8 iTemp, temp_AE_reg, temp_AWB_reg;
	kal_uint16 iDummyPixels = 0, iDummyLines = 0, iStartX = 0, iStartY = 0;

	SENSORDB("AT2250Previe\n");

	AT2250_sensor_cap_state = KAL_FALSE;

//fcs  test	AT2250_Sensor_2M();
	//AT2250_write_shutter(Preview_Shutter);
//fcs  test	AT2250_set_AE_mode(KAL_TRUE); 
//fcs  test	AT2250_set_AWB_mode(KAL_TRUE);
	image_window->GrabStartX=0;
	image_window->GrabStartY=1;

if(IsVideoPreview)
{
	printk("HT_TRACE AT2250Preview IsVideoPreview=1 \n");
	image_window->ExposureWindowWidth=640 - 8;
	image_window->ExposureWindowHeight=480 - 6;	
	AT2250_Sensor_VGA();
	AT2250_set_param_banding(AT2250_BANDING);

	mDELAY(100);//800 // 400
}
else
{
	printk("HT_TRACE AT2250Control IsVideoPreview=0 \n");
	image_window->ExposureWindowWidth=AT2250_IMAGE_SENSOR_PV_WIDTH - 16 ;
	image_window->ExposureWindowHeight=AT2250_IMAGE_SENSOR_PV_HEIGHT - 12 ;	
	if (IsCapture == 0)
	{
		AT2250_Sensor_UXGA();	 
		AT2250_set_param_banding(AT2250_BANDING);

		mDELAY(800);
	}
	IsCapture = 0;
}


	// copy sensor_config_data
	memcpy(&AT2250SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* AT2250Preview() */

UINT32 AT2250Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  
        volatile kal_uint32 shutter = 0,Capture_Shutter=0;
		kal_uint8 reg0x18, reg0x19,GainValue;
		kal_uint8 bandstep = 0;
		kal_uint16 bandvalue_pre = 0;
		kal_uint16 bandvalue_cap = 0;
		kal_uint16 VTS_new			 = 0;
	#if 0			
		reg0x18 = AT2250_read_cmos_sensor(0x18);
		reg0x19 = AT2250_read_cmos_sensor(0x19);
		bandvalue_pre = ((reg0x19 & 0x03) << 8)| reg0x18 ;
    	if(AT2250_sensor_cap_state == KAL_FALSE)
		{
			AT2250_set_AE_mode(KAL_FALSE);
			shutter = AT2250_read_shutter();
			bandstep 	= (shutter / bandvalue_pre)* 2 ;

			AT2250_Sensor_2M();  ////  2M_setting

			mDELAY(30);
			reg0x18 = AT2250_read_cmos_sensor(0x18);
			reg0x19 = AT2250_read_cmos_sensor(0x19);
			bandvalue_cap = ((reg0x19 & 0x03) << 8)| reg0x18 ;

			if (shutter < bandvalue_pre)									// shutter less than 1 band value
			{				
			Capture_Shutter = shutter * 2;
			}			
			else
			{			
			Capture_Shutter = bandvalue_cap * bandstep;					//calculate exposure time for capture
			}													
			if(Capture_Shutter  <= AT2250_FULL_PERIOD_LINE_NUMS)
			 {
			 	AT2250_write_shutter(Capture_Shutter);
			 	mDELAY(30);
				}
			else
			{
				VTS_new = Capture_Shutter + 2;
				AT2250_write_cmos_sensor(0x23, (VTS_new >> 8) & 0xff);
				AT2250_write_cmos_sensor(0x22, VTS_new & 0xff);
				AT2250_write_shutter(Capture_Shutter);
			 	mDELAY(30);
			}
    		}

	#endif
		AT2250_sensor_cap_state = KAL_TRUE;
		IsCapture = 1;
		 /* 2M FULL Mode */
		SENSORDB("AT2250Capture 2M\n");
		image_window->GrabStartX=0;
		image_window->GrabStartY=1;
		image_window->ExposureWindowWidth=AT2250_IMAGE_SENSOR_FULL_WIDTH - 16;
		image_window->ExposureWindowHeight=AT2250_IMAGE_SENSOR_FULL_HEIGHT - 12;    	 
//add below 6 line 
	GainValue = AT2250_read_cmos_sensor(0x00);
	if(GainValue < 0x30)//light on
	{
		AT2250_write_cmos_sensor(0x65,0xaa);//add  140527
		AT2250_write_cmos_sensor(0x3F,0x24);//add  140527
	}

    	AT2250_DELAY_AFTER_PREVIEW = 4;
    	memcpy(&AT2250SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* AT2250Capture() */

UINT32 AT2250GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=AT2250_IMAGE_SENSOR_FULL_WIDTH - 16;
	pSensorResolution->SensorFullHeight=AT2250_IMAGE_SENSOR_FULL_HEIGHT - 12;
	pSensorResolution->SensorPreviewWidth=AT2250_IMAGE_SENSOR_PV_WIDTH - 16;
	pSensorResolution->SensorPreviewHeight=AT2250_IMAGE_SENSOR_PV_HEIGHT - 12;
	pSensorResolution->SensorVideoWidth=640 - 8;
  	pSensorResolution->SensorVideoHeight=480 - 6;
	return ERROR_NONE;
}	/* AT2250GetResolution() */

UINT32 AT2250GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=AT2250_IMAGE_SENSOR_PV_WIDTH - 16;
	pSensorInfo->SensorPreviewResolutionY=AT2250_IMAGE_SENSOR_PV_HEIGHT - 12;
	pSensorInfo->SensorFullResolutionX=AT2250_IMAGE_SENSOR_FULL_WIDTH - 16;
	pSensorInfo->SensorFullResolutionY=AT2250_IMAGE_SENSOR_FULL_HEIGHT - 12;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=4;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=4;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	/*??? */
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->CaptureDelayFrame = 1;// 4; 
	pSensorInfo->PreviewDelayFrame = 4;//1; 
	pSensorInfo->VideoDelayFrame = 1;// 4;//0; 
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_6MA;

	pSensorInfo->YUVAwbDelayFrame = 2; 
	pSensorInfo->YUVEffectDelayFrame = 2;  

#ifdef AT2250_MIPI
	pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_MIPI;
#else
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;
#endif
	//hupeng 1219,mtk remove SensorISOBinningInfo
	#if 0
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=FALSE;	
	#endif
	
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
		pSensorInfo->SensorClockFreq=25;
		pSensorInfo->SensorClockDividCount=5;
		pSensorInfo->SensorClockRisingCount= 0;
		pSensorInfo->SensorClockFallingCount= 2;
		pSensorInfo->SensorPixelClockCount= 3;
		pSensorInfo->SensorDataLatchCount= 2;
		pSensorInfo->SensorGrabStartX = 0;//8; 
		pSensorInfo->SensorGrabStartY = 1;//8;
		pSensorInfo->SensorPreviewResolutionX=AT2250_IMAGE_SENSOR_PV_WIDTH - 16;
		pSensorInfo->SensorPreviewResolutionY=AT2250_IMAGE_SENSOR_PV_HEIGHT - 12;
		pSensorInfo->SensorFullResolutionX=AT2250_IMAGE_SENSOR_FULL_WIDTH - 16;
		pSensorInfo->SensorFullResolutionY=AT2250_IMAGE_SENSOR_FULL_HEIGHT - 12;
#ifdef AT2250_MIPI
		pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		
		pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
		pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
		pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
		pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
		pSensorInfo->SensorPacketECCOrder = 1;
#endif

		break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			pSensorInfo->SensorClockFreq=25;
			pSensorInfo->SensorClockDividCount=5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
	        pSensorInfo->SensorGrabStartX = 0;//8; 
	        pSensorInfo->SensorGrabStartY = 1;//8;
			pSensorInfo->SensorPreviewResolutionX=640 - 8;
			pSensorInfo->SensorPreviewResolutionY=480 - 6;
			pSensorInfo->SensorFullResolutionX=AT2250_IMAGE_SENSOR_FULL_WIDTH - 16;
			pSensorInfo->SensorFullResolutionY=AT2250_IMAGE_SENSOR_FULL_HEIGHT - 12;
#ifdef AT2250_MIPI
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
#endif
	
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			pSensorInfo->SensorClockFreq=25;
			pSensorInfo->SensorClockDividCount=5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = 0; 
            pSensorInfo->SensorGrabStartY = 1;
			pSensorInfo->SensorPreviewResolutionX=AT2250_IMAGE_SENSOR_PV_WIDTH - 16;
			pSensorInfo->SensorPreviewResolutionY=AT2250_IMAGE_SENSOR_PV_HEIGHT - 12;
			pSensorInfo->SensorFullResolutionX=AT2250_IMAGE_SENSOR_FULL_WIDTH - 16;
			pSensorInfo->SensorFullResolutionY=AT2250_IMAGE_SENSOR_FULL_HEIGHT - 12;

#ifdef AT2250_MIPI
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
#endif
			
		break;
		default:
			pSensorInfo->SensorClockFreq=25;
			pSensorInfo->SensorClockDividCount=5;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
            pSensorInfo->SensorGrabStartX = 0; 
            pSensorInfo->SensorGrabStartY = 1;             
			pSensorInfo->SensorPreviewResolutionX=AT2250_IMAGE_SENSOR_PV_WIDTH - 16;
			pSensorInfo->SensorPreviewResolutionY=AT2250_IMAGE_SENSOR_PV_HEIGHT - 12;
			pSensorInfo->SensorFullResolutionX=AT2250_IMAGE_SENSOR_FULL_WIDTH - 16;
			pSensorInfo->SensorFullResolutionY=AT2250_IMAGE_SENSOR_FULL_HEIGHT - 12;
		break;
	}
	memcpy(pSensorConfigData, &AT2250SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* AT2250GetInfo() */


UINT32 AT2250Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	printk("HT_TRACE AT2250Control\n");
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			IsVideoPreview = 0;			
			AT2250Preview(pImageWindow, pSensorConfigData);
	printk("HT_TRACE AT2250Control MSDK_SCENARIO_ID_CAMERA_PREVIEW\n");
		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			IsVideoPreview = 1;	
			AT2250Preview(pImageWindow, pSensorConfigData);
	printk("HT_TRACE AT2250Control MSDK_SCENARIO_ID_VIDEO_PREVIEW\n");
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			IsVideoPreview = 0;
			AT2250Capture(pImageWindow, pSensorConfigData);
	printk("HT_TRACE AT2250Control MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG\n");
		break;
		default:
			IsVideoPreview = 0;
	printk("HT_TRACE AT2250Control default\n");
		    break; 
	}
	return ERROR_NONE;
}	/* AT2250Control() */

BOOL AT2250_set_param_wb(UINT16 para)
{
    switch (para)
    {                   
        case AWB_MODE_AUTO:		
			//AT2250_set_AWB_mode(KAL_TRUE);
			if( (AT2250_read_cmos_sensor(0x36) & 0x01) == 0)
			{
				AT2250_write_cmos_sensor(0x05,0xa9);
				AT2250_write_cmos_sensor(0x06,0x00);
				AT2250_write_cmos_sensor(0x07,0x51);
				AT2250_write_cmos_sensor(0x08,0x54);
				AT2250_write_cmos_sensor(0x09,0x05);
			}
			AT2250_set_AWB_mode(KAL_TRUE);
			break;
        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			AT2250_set_AWB_mode(KAL_FALSE); 
			AT2250_write_cmos_sensor(0x05,0xc9);
			AT2250_write_cmos_sensor(0x06,0x00);
			AT2250_write_cmos_sensor(0x07,0xd6);
			AT2250_write_cmos_sensor(0x08,0x54);
			AT2250_write_cmos_sensor(0x09,0x04);
            break;
        case AWB_MODE_DAYLIGHT: //sunny
			AT2250_set_AWB_mode(KAL_FALSE); 
			AT2250_write_cmos_sensor(0x05,0x00);
			AT2250_write_cmos_sensor(0x06,0x00);
			AT2250_write_cmos_sensor(0x07,0x6b);
			AT2250_write_cmos_sensor(0x08,0x64);
			AT2250_write_cmos_sensor(0x09,0x04);
			break;
        case AWB_MODE_INCANDESCENT: //office
			AT2250_set_AWB_mode(KAL_FALSE); 
			AT2250_write_cmos_sensor(0x05,0x32);
			AT2250_write_cmos_sensor(0x06,0x00);
			AT2250_write_cmos_sensor(0x07,0xfd);
			AT2250_write_cmos_sensor(0x08,0x44);
			AT2250_write_cmos_sensor(0x09,0x05);
			break; 
		case AWB_MODE_TUNGSTEN:
            AT2250_set_AWB_mode(KAL_FALSE);     
            break;
        case AWB_MODE_FLUORESCENT:
            AT2250_set_AWB_mode(KAL_FALSE); 
            AT2250_write_cmos_sensor(0x05,0x60);
			AT2250_write_cmos_sensor(0x06,0x00);
			AT2250_write_cmos_sensor(0x07,0xab);
			AT2250_write_cmos_sensor(0x08,0x54);
			AT2250_write_cmos_sensor(0x09,0x05);
            break;
            default:
            return FALSE;
    }
       return TRUE;
} /* AT2250_set_param_wb */

BOOL AT2250_set_param_effect(UINT16 para)
{
   	kal_uint8 reg0x37;
	kal_uint8 reg0x7e;
		
	reg0x37 = AT2250_read_cmos_sensor(0x37);
	reg0x7e = AT2250_read_cmos_sensor(0x7e);

	switch (para)
	{
		case MEFFECT_OFF:
			AT2250_write_cmos_sensor(0x37, reg0x37 & 0xef);
			AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xf4);
			AT2250_write_cmos_sensor(0x7b, 0x00);
			AT2250_write_cmos_sensor(0x7c, 0x00);
			AT2250_write_cmos_sensor(0x79, 0x00);
			AT2250_write_cmos_sensor(0x7a, 0x00);
		break;

		case MEFFECT_SEPIA:
			#if 0
			AT2250_write_cmos_sensor(0x37, reg0x37 & 0xef);
			AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xf4);
			AT2250_write_cmos_sensor(0x7e, reg0x7e | 0x01);
			AT2250_write_cmos_sensor(0x7b, 0x20);
			AT2250_write_cmos_sensor(0x7c, 0x20);
			AT2250_write_cmos_sensor(0x79, 0x00);
			AT2250_write_cmos_sensor(0x7a, 0x00);
			#else

			AT2250_write_cmos_sensor(0x37, reg0x37 & 0xef);
			AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xf4);
			AT2250_write_cmos_sensor(0x7e, reg0x7e | 0x01);
			AT2250_write_cmos_sensor(0x7b, 0x28);
			AT2250_write_cmos_sensor(0x7c, 0x16);
			AT2250_write_cmos_sensor(0x79, 0x46);
			AT2250_write_cmos_sensor(0x7a, 0x36);
			
			#endif
			
		break;  

		case MEFFECT_NEGATIVE:		
			AT2250_write_cmos_sensor(0x37, reg0x37 | 0x10);
			AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xf4);
			AT2250_write_cmos_sensor(0x7b, 0x00);
			AT2250_write_cmos_sensor(0x7c, 0x00);
			AT2250_write_cmos_sensor(0x79, 0x00);
			AT2250_write_cmos_sensor(0x7a, 0x00);
		break; 

		case MEFFECT_SEPIAGREEN:		
			AT2250_write_cmos_sensor(0x37, reg0x37 & 0xef);
			AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xf4);
			AT2250_write_cmos_sensor(0x7e, reg0x7e | 0x03);
			AT2250_write_cmos_sensor(0x7b, 0x20);
			AT2250_write_cmos_sensor(0x7c, 0x20);
			AT2250_write_cmos_sensor(0x79, 0x00);
			AT2250_write_cmos_sensor(0x7a, 0x00);
		break;

		case MEFFECT_SEPIABLUE:	
			AT2250_write_cmos_sensor(0x37, reg0x37 & 0xef);
			AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xf4);
			AT2250_write_cmos_sensor(0x7e, reg0x7e | 0x02);
			AT2250_write_cmos_sensor(0x7b, 0x20);
			AT2250_write_cmos_sensor(0x7c, 0x20);
			AT2250_write_cmos_sensor(0x79, 0x00);
			AT2250_write_cmos_sensor(0x7a, 0x00);
		break;

		case MEFFECT_MONO:				
			AT2250_write_cmos_sensor(0x37, reg0x37 & 0xef);
			AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xf4);
			AT2250_write_cmos_sensor(0x7e, reg0x7e | 0x08);
			AT2250_write_cmos_sensor(0x7b, 0x00);
			AT2250_write_cmos_sensor(0x7c, 0x00);
			AT2250_write_cmos_sensor(0x79, 0xff);
			AT2250_write_cmos_sensor(0x7a, 0xff);

		break;

		default:
		return FALSE;
    }



	return TRUE;
} /* AT2250_set_param_effect */

BOOL AT2250_set_param_banding(UINT16 para)
{

    printk("AT2250_set_param_banding  para= %d\n", para);
	switch (para)
    {
        case AE_FLICKER_MODE_50HZ: 
		{
			AT2250_BANDING = 0;
			if(IsVideoPreview)
			{
				#if 1//ZTE
				AT2250_write_cmos_sensor(0x18, 0x96);//7D  24M0x77
				AT2250_write_cmos_sensor(0x19, 0x91);//90  24M0x91			
				#else //it is ok
				AT2250_write_cmos_sensor(0x18, 0x86);//82 
				AT2250_write_cmos_sensor(0x19, 0x91);//90
				#endif
			}
			else
			{
				AT2250_write_cmos_sensor(0x18, 0x93);  //0xba 24M0x74
				AT2250_write_cmos_sensor(0x19, 0x91);  //0x90 24M0x91
			}
		            break;
		}
        case AE_FLICKER_MODE_60HZ:
		{
			AT2250_BANDING = 1;
			if(IsVideoPreview)
			{
				#if 1//ZTE
				AT2250_write_cmos_sensor(0x18, 0x52);//68  24M0x38
				AT2250_write_cmos_sensor(0x19, 0x91);//90 24M0x91
				#else// it is ok
				AT2250_write_cmos_sensor(0x18, 0x44);//6c  
				AT2250_write_cmos_sensor(0x19, 0x91);//90
				#endif
			}
			else
			{
				AT2250_write_cmos_sensor(0x18, 0xf8);   //0x9b 24M0x36
				AT2250_write_cmos_sensor(0x19, 0x91);	//0x90 24M0x91
			}
			 break;

		}
		
        

         default:
	 {
		AT2250_BANDING = 0;
		if(IsVideoPreview)
		{
			#if 1//ZTE
			AT2250_write_cmos_sensor(0x18, 0x96);//7D  24M0x77
			AT2250_write_cmos_sensor(0x19, 0x91);//90  24M0x91			
			#else //it is ok
			AT2250_write_cmos_sensor(0x18, 0x86);//82 
			AT2250_write_cmos_sensor(0x19, 0x91);//90
			#endif
		}
		else
		{
			AT2250_write_cmos_sensor(0x18, 0x93);  //0xba 24M0x74
			AT2250_write_cmos_sensor(0x19, 0x91);  //0x90 24M0x91
		}
	            break;
	}
        return FALSE;
    }

    return TRUE;
} /* AT2250_set_param_banding */

BOOL AT2250_set_param_exposure(UINT16 para)
{
   kal_uint8 reg0x7e;
	reg0x7e = AT2250_read_cmos_sensor(0x7e);


	switch (para)
	{
		case AE_EV_COMP_n13:

			AT2250_write_cmos_sensor(0x7e, reg0x7e | 0x04);
			AT2250_write_cmos_sensor(0x7d, 0x40);


		break;
		case AE_EV_COMP_n10:

			AT2250_write_cmos_sensor(0x7e, reg0x7e | 0x04);
			AT2250_write_cmos_sensor(0x7d, 0x30);
		break;
		case AE_EV_COMP_n07:

			AT2250_write_cmos_sensor(0x7e, reg0x7e | 0x04);
			AT2250_write_cmos_sensor(0x7d, 0x20);

		break;
		case AE_EV_COMP_n03:

			AT2250_write_cmos_sensor(0x7e, reg0x7e | 0x04);
			AT2250_write_cmos_sensor(0x7d, 0x10);

		break;
		case AE_EV_COMP_00:

			AT2250_write_cmos_sensor(0x7e, reg0x7e | 0x04);
			AT2250_write_cmos_sensor(0x7d, 0x00);

		break;
		case AE_EV_COMP_03:

			AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xfb);
			AT2250_write_cmos_sensor(0x7d, 0x10);
			
		break;
		case AE_EV_COMP_07:

			AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xfb);
			AT2250_write_cmos_sensor(0x7d, 0x20);
		break;
		case AE_EV_COMP_10:

			AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xfb);
			AT2250_write_cmos_sensor(0x7d, 0x30);

		break;
		case AE_EV_COMP_13:

			AT2250_write_cmos_sensor(0x7e, reg0x7e & 0xfb);
			AT2250_write_cmos_sensor(0x7d, 0x40);

		break;
		default:
		return FALSE;
	}
	return TRUE;
} /* AT2250_set_param_exposure */

UINT32 AT2250YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
		//   if( AT2250_sensor_cap_state == KAL_TRUE)
		//	   return TRUE;

#ifdef DEBUG_SENSOR
return TRUE;

#endif

		switch (iCmd) {
		case FID_SCENE_MODE:	    
		//	    printk("Set Scene Mode:%d\n", iPara); 
		if (iPara == SCENE_MODE_OFF)
		{
		AT2250_night_mode(0); 
		}
		else if (iPara == SCENE_MODE_NIGHTSCENE)
		{
		   AT2250_night_mode(1); 
		}	    
		break; 	    
		case FID_AWB_MODE:
		//	    printk("Set AWB Mode:%d\n", iPara); 	    
		AT2250_set_param_wb(iPara);
		break;
		case FID_COLOR_EFFECT:
		//	    printk("Set Color Effect:%d\n", iPara); 	    	    
		AT2250_set_param_effect(iPara);
		break;
		case FID_AE_EV:
		//           printk("Set EV:%d\n", iPara); 	    	    
		AT2250_set_param_exposure(iPara);
		break;
		case FID_AE_FLICKER:
		//           printk("Set Flicker:%d\n", iPara); 	    	    	    
		AT2250_set_param_banding(iPara);
		break;
		case FID_AE_SCENE_MODE: 
		if (iPara == AE_MODE_OFF) {
		    AT2250_set_AE_mode(KAL_FALSE);
		}
		else {
		    AT2250_set_AE_mode(KAL_TRUE);
		}
		break; 
		case FID_ZOOM_FACTOR:
		zoom_factor = iPara; 
		break; 
		default:
		break;
		}
		return TRUE;
}   /* AT2250YUVSensorSetting */

UINT32 AT2250YUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 iTemp;
    /* to fix VSYNC, to fix frame rate */
    //printk("Set YUV Video Mode \n");  

    if (u2FrameRate == 30)
    {
    }
    else if (u2FrameRate == 15)       
    {
    }
    else 
    {
     printk("Wrong frame rate setting \n");
    }
    AT2250_VEDIO_encode_mode = KAL_TRUE; 

	//	mDELAY(400);
		
    return TRUE;
}

UINT32 AT2250FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=AT2250_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=AT2250_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=AT2250_PV_PERIOD_PIXEL_NUMS;
			*pFeatureReturnPara16=AT2250_PV_PERIOD_LINE_NUMS;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 =48;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			 AT2250_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
		break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			AT2250_isp_master_clock=*pFeatureData32;
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			AT2250_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = AT2250_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &AT2250SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:

		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=0;
            *pFeatureParaLen=4;	    
		    break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			 AT2250_GetSensorID(pFeatureData32);
			 break;
		case SENSOR_FEATURE_SET_YUV_CMD:
		       //printk("AT2250 YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			AT2250YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		      AT2250YUVSetVideoMode(*pFeatureData16);
		       break; 
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			AT2250SetTestPatternMode((BOOL)*pFeatureData16);            
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=AT2250_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;
		default:
			break;			
	}
	return ERROR_NONE;
}	/* AT2250FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncAT2250= 
{
	AT2250Open,
	AT2250GetInfo,
	AT2250GetResolution,
	AT2250FeatureControl,
	AT2250Control,
	AT2250Close
};

UINT32 AT2250_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
	*pfFunc=&SensorFuncAT2250;
	return ERROR_NONE;
}	/* SensorInit() */
