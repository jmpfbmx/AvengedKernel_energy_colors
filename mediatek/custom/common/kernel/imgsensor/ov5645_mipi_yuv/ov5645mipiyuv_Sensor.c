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
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/system.h>	 
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"
#include "ov5645mipiyuv_Sensor.h"
#include "ov5645mipiyuv_Camera_Sensor_para.h"
#include "ov5645mipiyuv_CameraCustomized.h" 



#define OV5645MIPIYUV_DEBUG
#ifdef OV5645MIPIYUV_DEBUG
#define OV5645MIPISENSORDB printk
#else
#define OV5645MIPISENSORDB(x,...)   
#endif

//static int AF_Power = false;

static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define OV5645MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,OV5645MIPI_WRITE_ID)
#define mDELAY(ms)  mdelay(ms)
static bool AF_Power = false;
#define OV5645_TAF_TOLERANCE (100)

typedef enum
{
    PRV_W=1280,
    PRV_H=960
}PREVIEW_VIEW_SIZE;
kal_uint16 OV5645MIPIYUV_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV5645MIPI_WRITE_ID);
    return get_byte;
}

static struct
{
	//kal_uint8   Banding;
	kal_bool	  NightMode;
	kal_bool	  VideoMode;
	kal_uint16  Fps;
	kal_uint16  ShutterStep;
	kal_uint8   IsPVmode;
	kal_uint32  PreviewDummyPixels;
	kal_uint32  PreviewDummyLines;
	kal_uint32  CaptureDummyPixels;
	kal_uint32  CaptureDummyLines;
	kal_uint32  PreviewPclk;
	kal_uint32  CapturePclk;
	kal_uint32  PreviewShutter;
	kal_uint32  PreviewExtraShutter;
	kal_uint32  SensorGain;
	kal_uint32  AF_window_x;
	kal_uint32  AF_window_y;
	OV5645MIPI_SENSOR_MODE SensorMode;
	
} OV5645MIPISensor;
/* Global Valuable */
static kal_uint32 zoom_factor = 0; 
static kal_int8 OV5645MIPI_DELAY_AFTER_PREVIEW = -1;
static kal_uint8 OV5645MIPI_Banding_setting = AE_FLICKER_MODE_50HZ; 
static kal_bool OV5645MIPI_AWB_ENABLE = KAL_TRUE; 
static kal_bool OV5645MIPI_AE_ENABLE = KAL_TRUE; 
MSDK_SENSOR_CONFIG_STRUCT OV5645MIPISensorConfigData;

#define OV5645_TEST_PATTERN_CHECKSUM 0x96f31f11
static DEFINE_SPINLOCK(ov5645mipi_drv_lock);

typedef enum
{
    AE_SECTION_INDEX_BEGIN=0, 
    AE_SECTION_INDEX_1=AE_SECTION_INDEX_BEGIN, 
    AE_SECTION_INDEX_2, 
    AE_SECTION_INDEX_3, 
    AE_SECTION_INDEX_4, 
    AE_SECTION_INDEX_5, 
    AE_SECTION_INDEX_6, 
    AE_SECTION_INDEX_7, 
    AE_SECTION_INDEX_8, 
    AE_SECTION_INDEX_9, 
    AE_SECTION_INDEX_10, 
    AE_SECTION_INDEX_11, 
    AE_SECTION_INDEX_12, 
    AE_SECTION_INDEX_13, 
    AE_SECTION_INDEX_14, 
    AE_SECTION_INDEX_15, 
    AE_SECTION_INDEX_16,  
    AE_SECTION_INDEX_MAX
}AE_SECTION_INDEX;
typedef enum
{
    AE_VERTICAL_BLOCKS=4,
    AE_VERTICAL_BLOCKS_MAX,
    AE_HORIZONTAL_BLOCKS=4,
    AE_HORIZONTAL_BLOCKS_MAX
}AE_VERTICAL_HORIZONTAL_BLOCKS;
static UINT32 line_coordinate[AE_VERTICAL_BLOCKS_MAX] = {0};//line[0]=0      line[1]=160     line[2]=320     line[3]=480     line[4]=640
static UINT32 row_coordinate[AE_HORIZONTAL_BLOCKS_MAX] = {0};//line[0]=0       line[1]=120     line[2]=240     line[3]=360     line[4]=480
static BOOL AE_1_ARRAY[AE_SECTION_INDEX_MAX] = {FALSE};
static BOOL AE_2_ARRAY[AE_HORIZONTAL_BLOCKS][AE_VERTICAL_BLOCKS] = {{FALSE},{FALSE},{FALSE},{FALSE}};//how to ....
//=====================touch AE begin==========================//
void writeAEReg(void)
{	
    UINT8 temp;
    //write 1280X960
     OV5645MIPI_write_cmos_sensor(0x5680,0x00); 
     OV5645MIPI_write_cmos_sensor(0x5681,0x00);
     OV5645MIPI_write_cmos_sensor(0x5682,0x00);  
     OV5645MIPI_write_cmos_sensor(0x5683,0x00);
     OV5645MIPI_write_cmos_sensor(0x5684,0x05); //width=256  
     OV5645MIPI_write_cmos_sensor(0x5685,0x00);
     OV5645MIPI_write_cmos_sensor(0x5686,0x03); //heght=256
     OV5645MIPI_write_cmos_sensor(0x5687,0xc0);
    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_1]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_2]==TRUE)    { temp=temp|0xF0;}
    //write 0x5688
    OV5645MIPI_write_cmos_sensor(0x5688,temp);
    
    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_3]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_4]==TRUE)    { temp=temp|0xF0;}
    //write 0x5689
    OV5645MIPI_write_cmos_sensor(0x5689,temp);

    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_5]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_6]==TRUE)    { temp=temp|0xF0;}
    //write 0x568A
    OV5645MIPI_write_cmos_sensor(0x568A,temp);
    
    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_7]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_8]==TRUE)    { temp=temp|0xF0;}
    //write 0x568B
    OV5645MIPI_write_cmos_sensor(0x568B,temp);

    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_9]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_10]==TRUE)  { temp=temp|0xF0;}
	//write 0x568C
    OV5645MIPI_write_cmos_sensor(0x568C,temp);

    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_11]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_12]==TRUE)    { temp=temp|0xF0;}
    //write 0x568D
    OV5645MIPI_write_cmos_sensor(0x568D,temp);    
    
    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_13]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_14]==TRUE)    { temp=temp|0xF0;}
	//write 0x568E
    OV5645MIPI_write_cmos_sensor(0x568E,temp);

    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_15]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_16]==TRUE)    { temp=temp|0xF0;}
	//write 0x568F
    OV5645MIPI_write_cmos_sensor(0x568F,temp);
}


void printAE_1_ARRAY(void)
{
    UINT32 i;
    for(i=0; i<AE_SECTION_INDEX_MAX; i++)
    {
        OV5645MIPISENSORDB("AE_1_ARRAY[%2d]=%d\n", i, AE_1_ARRAY[i]);
    }
}

void printAE_2_ARRAY(void)
{
    UINT32 i, j;
    OV5645MIPISENSORDB("\t\t");
    for(i=0; i<AE_VERTICAL_BLOCKS; i++)
    {
        OV5645MIPISENSORDB("      line[%2d]", i);
    }
    printk("\n");
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        OV5645MIPISENSORDB("\trow[%2d]", j);
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        {
            //SENSORDB("AE_2_ARRAY[%2d][%2d]=%d\n", j,i,AE_2_ARRAY[j][i]);
            OV5645MIPISENSORDB("  %7d", AE_2_ARRAY[j][i]);
        }
        OV5645MIPISENSORDB("\n");
    }
}

void clearAE_2_ARRAY(void)
{
    UINT32 i, j;
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        {AE_2_ARRAY[j][i]=FALSE;}
    }
}

void mapAE_2_ARRAY_To_AE_1_ARRAY(void)
{
    UINT32 i, j;
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        { AE_1_ARRAY[j*AE_VERTICAL_BLOCKS+i] = AE_2_ARRAY[j][i];}
    }
}

void mapMiddlewaresizePointToPreviewsizePoint(
    UINT32 mx,
    UINT32 my,
    UINT32 mw,
    UINT32 mh,
    UINT32 * pvx,
    UINT32 * pvy,
    UINT32 pvw,
    UINT32 pvh
)
{
    *pvx = pvw * mx / mw;
    *pvy = pvh * my / mh;
    OV5645MIPISENSORDB("mapping middlware x[%d],y[%d], [%d X %d]\n\t\tto x[%d],y[%d],[%d X %d]\n ",
        mx, my, mw, mh, *pvx, *pvy, pvw, pvh);
}


void calcLine(void)
{//line[5]
    UINT32 i;
    UINT32 step = PRV_W / AE_VERTICAL_BLOCKS;
    for(i=0; i<=AE_VERTICAL_BLOCKS; i++)
    {
        *(&line_coordinate[0]+i) = step*i;
        OV5645MIPISENSORDB("line[%d]=%d\t",i, *(&line_coordinate[0]+i));
    }
    OV5645MIPISENSORDB("\n");
}

void calcRow(void)
{//row[5]
    UINT32 i;
    UINT32 step = PRV_H / AE_HORIZONTAL_BLOCKS;
    for(i=0; i<=AE_HORIZONTAL_BLOCKS; i++)
    {
        *(&row_coordinate[0]+i) = step*i;
        OV5645MIPISENSORDB("row[%d]=%d\t",i,*(&row_coordinate[0]+i));
    }
    OV5645MIPISENSORDB("\n");
}

void calcPointsAELineRowCoordinate(UINT32 x, UINT32 y, UINT32 * linenum, UINT32 * rownum)
{
    UINT32 i;
    i = 1;
    while(i<=AE_VERTICAL_BLOCKS)
    {
        if(x<line_coordinate[i])
        {
            *linenum = i;
            break;
        }
        *linenum = i++;
    }
    
    i = 1;
    while(i<=AE_HORIZONTAL_BLOCKS)
    {
        if(y<row_coordinate[i])
        {
            *rownum = i;
            break;
        }
        *rownum = i++;
    }
    OV5645MIPISENSORDB("PV point [%d, %d] to section line coordinate[%d] row[%d]\n",x,y,*linenum,*rownum);
}



MINT32 clampSection(UINT32 x, UINT32 min, UINT32 max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

void mapCoordinate(UINT32 linenum, UINT32 rownum, UINT32 * sectionlinenum, UINT32 * sectionrownum)
{
    *sectionlinenum = clampSection(linenum-1,0,AE_VERTICAL_BLOCKS-1);
    *sectionrownum = clampSection(rownum-1,0,AE_HORIZONTAL_BLOCKS-1);	
    OV5645MIPISENSORDB("mapCoordinate from[%d][%d] to[%d][%d]\n",
		linenum, rownum,*sectionlinenum,*sectionrownum);
}

void mapRectToAE_2_ARRAY(UINT32 x0, UINT32 y0, UINT32 x1, UINT32 y1)
{
    UINT32 i, j;
    OV5645MIPISENSORDB("([%d][%d]),([%d][%d])\n", x0,y0,x1,y1);
    clearAE_2_ARRAY();
    x0=clampSection(x0,0,AE_VERTICAL_BLOCKS-1);
    y0=clampSection(y0,0,AE_HORIZONTAL_BLOCKS-1);
    x1=clampSection(x1,0,AE_VERTICAL_BLOCKS-1);
    y1=clampSection(y1,0,AE_HORIZONTAL_BLOCKS-1);

    for(j=y0; j<=y1; j++)
    {
        for(i=x0; i<=x1; i++)
        {
            AE_2_ARRAY[j][i]=TRUE;
        }
    }
}

void resetPVAE_2_ARRAY(void)
{
    mapRectToAE_2_ARRAY(1,1,2,2);
}

//update ae window
//@input zone[] addr
void OV5645_FOCUS_Set_AE_Window(UINT32 zone_addr)
{
	//update global zone
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645_FOCUS_Set_AE_Window function\n");
	//input:
	UINT32 FD_XS;
	UINT32 FD_YS;	
	UINT32 x0, y0, x1, y1;
	UINT32 pvx0, pvy0, pvx1, pvy1;
	UINT32 linenum, rownum;
	UINT32 rightbottomlinenum,rightbottomrownum;
	UINT32 leftuplinenum,leftuprownum;
	UINT32* zone = (UINT32*)zone_addr;
	x0 = *zone;
	y0 = *(zone + 1);
	x1 = *(zone + 2);
	y1 = *(zone + 3);	
	FD_XS = *(zone + 4);
	FD_YS = *(zone + 5);

	OV5645MIPISENSORDB("AE x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",x0, y0, x1, y1, FD_XS, FD_YS);	
    
	//print_sensor_ae_section();
	//print_AE_section();	

	//1.transfer points to preview size
	//UINT32 pvx0, pvy0, pvx1, pvy1;
	mapMiddlewaresizePointToPreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, PRV_W, PRV_H);
	mapMiddlewaresizePointToPreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, PRV_W, PRV_H);
    
	//2.sensor AE line and row coordinate
	calcLine();
	calcRow();

	//3.calc left up point to section
	//UINT32 linenum, rownum;
	calcPointsAELineRowCoordinate(pvx0,pvy0,&linenum,&rownum);    
	//UINT32 leftuplinenum,leftuprownum;
	mapCoordinate(linenum, rownum, &leftuplinenum, &leftuprownum);
	//SENSORDB("leftuplinenum=%d,leftuprownum=%d\n",leftuplinenum,leftuprownum);

	//4.calc right bottom point to section
	calcPointsAELineRowCoordinate(pvx1,pvy1,&linenum,&rownum);    
	//UINT32 rightbottomlinenum,rightbottomrownum;
	mapCoordinate(linenum, rownum, &rightbottomlinenum, &rightbottomrownum);
	//SENSORDB("rightbottomlinenum=%d,rightbottomrownum=%d\n",rightbottomlinenum,rightbottomrownum);

	//5.update global section array
	mapRectToAE_2_ARRAY(leftuplinenum, leftuprownum, rightbottomlinenum, rightbottomrownum);
	//print_AE_section();

	//6.write to reg
	mapAE_2_ARRAY_To_AE_1_ARRAY();
	//printAE_1_ARRAY();
	printAE_2_ARRAY();
	writeAEReg();
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645_FOCUS_Set_AE_Window function\n");
}

//=====================touch AE end==========================//
/*************************************************************************
* FUNCTION
*	OV5645MIPI_set_dummy
*
* DESCRIPTION
*	This function set the dummy pixels(Horizontal Blanking) & dummy lines(Vertical Blanking), it can be
*	used to adjust the frame rate or gain more time for back-end process.
*	
*	IMPORTANT NOTICE: the base shutter need re-calculate for some sensor, or else flicker may occur.
*
* PARAMETERS
*	1. kal_uint32 : Dummy Pixels (Horizontal Blanking)
*	2. kal_uint32 : Dummy Lines (Vertical Blanking)
*
* RETURNS
*	None
*
*************************************************************************/
static void OV5645MIPIinitalvariable()
{
	spin_lock(&ov5645mipi_drv_lock);
	OV5645MIPISensor.VideoMode = KAL_FALSE;
	OV5645MIPISensor.NightMode = KAL_FALSE;
	OV5645MIPISensor.Fps = 100;
	OV5645MIPISensor.ShutterStep= 0xde;
	OV5645MIPISensor.CaptureDummyPixels = 0;
	OV5645MIPISensor.CaptureDummyLines = 0;
	OV5645MIPISensor.PreviewDummyPixels = 0;
	OV5645MIPISensor.PreviewDummyLines = 0;
	OV5645MIPISensor.SensorMode= SENSOR_MODE_INIT;
	OV5645MIPISensor.IsPVmode= KAL_TRUE;	
	OV5645MIPISensor.PreviewPclk= 560;
	OV5645MIPISensor.CapturePclk= 900;
	OV5645MIPISensor.PreviewShutter=0x0375; //0375
	OV5645MIPISensor.PreviewExtraShutter=0x00; 
	OV5645MIPISensor.SensorGain=0x10;
	OV5645MIPISensor.AF_window_x=0;
	OV5645MIPISensor.AF_window_y=0;
	spin_unlock(&ov5645mipi_drv_lock);
}
static void OV5645MIPISetDummy(kal_uint32 dummy_pixels, kal_uint32 dummy_lines)
{
		OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPISetDummy function:\n ");
		if (OV5645MIPISensor.IsPVmode)  
        {
            dummy_pixels = dummy_pixels+OV5645MIPI_PV_PERIOD_PIXEL_NUMS; 
            OV5645MIPI_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));         
            OV5645MIPI_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
      
            dummy_lines= dummy_lines+OV5645MIPI_PV_PERIOD_LINE_NUMS; 
            OV5645MIPI_write_cmos_sensor(0x380F,(dummy_lines&0xFF));       
            OV5645MIPI_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
        } 
        else
        {
            dummy_pixels = dummy_pixels+OV5645MIPI_FULL_PERIOD_PIXEL_NUMS; 
            OV5645MIPI_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));         
            OV5645MIPI_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
      
            dummy_lines= dummy_lines+OV5645MIPI_FULL_PERIOD_LINE_NUMS; 
            OV5645MIPI_write_cmos_sensor(0x380F,(dummy_lines&0xFF));       
            OV5645MIPI_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
        } 
		OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPISetDummy function:\n ");
}    /* OV5645MIPI_set_dummy */

/*************************************************************************
* FUNCTION
*	OV5645MIPIWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void OV5645MIPIWriteShutter(kal_uint32 shutter)
{
	kal_uint32 extra_exposure_vts = 0;
	
	if (shutter < 1)
	{
		shutter = 1;
	}
	if (shutter > OV5645MIPI_FULL_EXPOSURE_LIMITATION) 
	{
		extra_exposure_vts =shutter+4;
		OV5645MIPI_write_cmos_sensor(0x380f, extra_exposure_vts & 0xFF);          // EXVTS[b7~b0]
		OV5645MIPI_write_cmos_sensor(0x380e, (extra_exposure_vts & 0xFF00) >> 8); // EXVTS[b15~b8]
		OV5645MIPI_write_cmos_sensor(0x350D,0x00); 
		OV5645MIPI_write_cmos_sensor(0x350C,0x00);
	}
	shutter*=16;
	OV5645MIPI_write_cmos_sensor(0x3502, (shutter & 0x00FF));           //AEC[7:0]
	OV5645MIPI_write_cmos_sensor(0x3501, ((shutter & 0x0FF00) >>8));  //AEC[15:8]
	OV5645MIPI_write_cmos_sensor(0x3500, ((shutter & 0xFF0000) >> 16));	
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIWriteShutter function:\n ");
}    /* OV5645MIPI_write_shutter */
/*************************************************************************
* FUNCTION
*	OV5645MIPIExpWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void OV5645MIPIWriteExpShutter(kal_uint32 shutter)
{
	shutter*=16;
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIWriteExpShutter function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3502, (shutter & 0x00FF));           //AEC[7:0]
	OV5645MIPI_write_cmos_sensor(0x3501, ((shutter & 0x0FF00) >>8));  //AEC[15:8]
	OV5645MIPI_write_cmos_sensor(0x3500, ((shutter & 0xFF0000) >> 16));	
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIWriteExpShutter function:\n ");
}    /* OV5645MIPI_write_shutter */

/*************************************************************************
* FUNCTION
*	OV5645MIPIExtraWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void OV5645MIPIWriteExtraShutter(kal_uint32 shutter)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIWriteExtraShutter function:\n ");
	OV5645MIPI_write_cmos_sensor(0x350D, shutter & 0xFF);          // EXVTS[b7~b0]
	OV5645MIPI_write_cmos_sensor(0x350C, (shutter & 0xFF00) >> 8); // EXVTS[b15~b8]
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIWriteExtraShutter function:\n ");
}    /* OV5645MIPI_write_shutter */

/*************************************************************************
* FUNCTION
*	OV5645MIPIWriteSensorGain
*
* DESCRIPTION
*	This function used to write the sensor gain.
*
* PARAMETERS
*	1. kal_uint32 : The sensor gain want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void OV5645MIPIWriteSensorGain(kal_uint32 gain)
{
	kal_uint16 temp_reg ;
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIWriteSensorGain function:\n ");
	if(gain > 1024)  ASSERT(0);
	temp_reg = 0;
	temp_reg=gain&0x0FF;	
	OV5645MIPI_write_cmos_sensor(0x350B,temp_reg);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIWriteSensorGain function:\n ");
}  /* OV5645MIPI_write_sensor_gain */

/*************************************************************************
* FUNCTION
*	OV5645MIPIReadShutter
*
* DESCRIPTION
*	This function read current shutter for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current shutter value.
*
*************************************************************************/
static kal_uint32 OV5645MIPIReadShutter(void)
{
	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIReadShutter function:\n ");
	temp_reg1 = OV5645MIPIYUV_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV5645MIPIYUV_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV5645MIPIYUV_read_cmos_sensor(0x3502);    // AEC[b7~b0]

	spin_lock(&ov5645mipi_drv_lock);
	OV5645MIPISensor.PreviewShutter  = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);
	spin_unlock(&ov5645mipi_drv_lock);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIReadShutter function:\n ");	
	return OV5645MIPISensor.PreviewShutter;
} /* OV5645MIPI_read_shutter */

/*************************************************************************
* FUNCTION
*	OV5645MIPIReadExtraShutter
*
* DESCRIPTION
*	This function read current shutter for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current shutter value.
*
*************************************************************************/
static kal_uint32 OV5645MIPIReadExtraShutter(void)
{
	kal_uint16 temp_reg1, temp_reg2 ;
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIReadExtraShutter function:\n ");
	temp_reg1 = OV5645MIPIYUV_read_cmos_sensor(0x350c);    // AEC[b15~b8]
	temp_reg2 = OV5645MIPIYUV_read_cmos_sensor(0x350d);    // AEC[b7~b0]
	spin_lock(&ov5645mipi_drv_lock);
	OV5645MIPISensor.PreviewExtraShutter  = ((temp_reg1<<8)| temp_reg2);
	spin_unlock(&ov5645mipi_drv_lock);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIReadExtraShutter function:\n ");		
	return OV5645MIPISensor.PreviewExtraShutter;
} /* OV5645MIPI_read_shutter */
/*************************************************************************
* FUNCTION
*	OV5645MIPIReadSensorGain
*
* DESCRIPTION
*	This function read current sensor gain for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current sensor gain value.
*
*************************************************************************/
static kal_uint32 OV5645MIPIReadSensorGain(void)
{
	kal_uint32 sensor_gain = 0;
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIReadSensorGain function:\n ");
	sensor_gain=(OV5645MIPIYUV_read_cmos_sensor(0x350B)&0xFF); 
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIReadSensorGain function:\n ");
	return sensor_gain;
}  /* OV5645MIPIReadSensorGain */
/*************************************************************************
* FUNCTION
*	OV5645MIPI_set_AE_mode
*
* DESCRIPTION
*	This function OV5645MIPI_set_AE_mode.
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
static void OV5645MIPI_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 AeTemp;
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPI_set_AE_mode function:\n ");
    AeTemp = OV5645MIPIYUV_read_cmos_sensor(0x3503);
    if (AE_enable == KAL_TRUE)
    {
        // turn on AEC/AGC
        OV5645MIPI_write_cmos_sensor(0x3503,(AeTemp&(~0x07)));
    }
    else
    {
        // turn off AEC/AGC
      OV5645MIPI_write_cmos_sensor(0x3503,(AeTemp|0x07));
    }
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPI_set_AE_mode function:\n ");
}

/*************************************************************************
* FUNCTION
*	OV5645MIPI_set_AWB_mode
*
* DESCRIPTION
*	This function OV5645MIPI_set_AWB_mode.
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
static void OV5645MIPI_set_AWB_mode(kal_bool AWB_enable)
{
    kal_uint8 AwbTemp;
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPI_set_AWB_mode function:\n ");
	  AwbTemp = OV5645MIPIYUV_read_cmos_sensor(0x3406);   

    if (AWB_enable == KAL_TRUE)
    {
             
		OV5645MIPI_write_cmos_sensor(0x3406,AwbTemp&0xFE); 
		
    }
    else
    {             
		OV5645MIPI_write_cmos_sensor(0x3406,AwbTemp|0x01); 
		
    }
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPI_set_AWB_mode function:\n ");
}


/*************************************************************************
* FUNCTION
*	OV5645MIPI_night_mode
*
* DESCRIPTION
*	This function night mode of OV5645MIPI.
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
void OV5645MIPI_night_mode(kal_bool enable)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPI_night_mode function:\n ");
	kal_uint16 night = OV5645MIPIYUV_read_cmos_sensor(0x3A00); 
	if (enable)
	{ 
            /* camera night mode */
			OV5645MIPI_write_cmos_sensor(0x3A00,night|0x04); // 30fps-5fps
			OV5645MIPI_write_cmos_sensor(0x3a02,0x17); 
			OV5645MIPI_write_cmos_sensor(0x3a03,0x10);                         
			OV5645MIPI_write_cmos_sensor(0x3a14,0x17); 
			OV5645MIPI_write_cmos_sensor(0x3a15,0x10);
			OV5645MIPI_write_cmos_sensor(0x3a19,0xc0);       
	}
	else
	{             /* camera normal mode */                
			OV5645MIPI_write_cmos_sensor(0x3A00,night|0x04); //30fps-10fps               
			OV5645MIPI_write_cmos_sensor(0x3a02,0x0b);
			OV5645MIPI_write_cmos_sensor(0x3a03,0x88);
			OV5645MIPI_write_cmos_sensor(0x3a14,0x0b); 
			OV5645MIPI_write_cmos_sensor(0x3a15,0x88);								
			OV5645MIPI_write_cmos_sensor(0x3a19,0x60);      
	} 
	spin_lock(&ov5645mipi_drv_lock);
	OV5645MIPISensor.NightMode=enable;
	spin_unlock(&ov5645mipi_drv_lock);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPI_night_mode function:\n ");
}	/* OV5645MIPI_night_mode */
/*************************************************************************
* FUNCTION
*	OV5645MIPI_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
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
//static 
kal_uint32 OV5645MIPI_GetSensorID(kal_uint32 *sensorID)
{
    volatile signed char i;
	kal_uint32 sensor_id=0;
	kal_uint8 temp_sccb_addr = 0;
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPI_GetSensorID function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3008,0x82);// Reset sensor
	mDELAY(10);
	for(i=0;i<3;i++)
	{
		sensor_id = (OV5645MIPIYUV_read_cmos_sensor(0x300A) << 8) | OV5645MIPIYUV_read_cmos_sensor(0x300B);
		OV5645MIPISENSORDB("OV5645MIPI READ ID: %x",sensor_id);
		if(sensor_id != OV5645MIPI_SENSOR_ID)
		{	
			*sensorID =0xffffffff;
			return ERROR_SENSOR_CONNECT_FAIL;
		}
		else
			{
			*sensorID=OV5645MIPI_SENSOR_ID;
		        break;
			}
	}
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPI_GetSensorID function:\n ");
   return ERROR_NONE;    
}   
/*************************************************************************
* FUNCTION
*    OV5645MIPIInitialSetting
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
//static 
void OV5645MIPIInitialSetting(void)
{
	//;OV5645MIPI 1280x960,30fps
	//56Mhz, 224Mbps/Lane, 2 Lane
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIInitialSetting function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3103, 0x11);//	; PLL clock selection
	OV5645MIPI_write_cmos_sensor(0x3008, 0x82);//	; software reset	
	mDELAY(5);            										//; delay 2ms                           					
	OV5645MIPI_write_cmos_sensor(0x3008, 0x42);//	; software power down
	OV5645MIPI_write_cmos_sensor(0x3103, 0x03);//	; clock from PLL
	OV5645MIPI_write_cmos_sensor(0x3503, 0x07);//	; AGC manual, AEC manual
	OV5645MIPI_write_cmos_sensor(0x3000, 0x30);
	OV5645MIPI_write_cmos_sensor(0x3004, 0xef);
	OV5645MIPI_write_cmos_sensor(0x3002, 0x1c);//	; system reset
	OV5645MIPI_write_cmos_sensor(0x3006, 0xc3);//	; clock enable
	OV5645MIPI_write_cmos_sensor(0x300e, 0x45);//	; MIPI 2 lane
	OV5645MIPI_write_cmos_sensor(0x3017, 0x40);//	; Frex, CSK input, Vsync output
	OV5645MIPI_write_cmos_sensor(0x3018, 0x00);//	; GPIO input
	OV5645MIPI_write_cmos_sensor(0x302c, 0x02);//	; GPIO input
	OV5645MIPI_write_cmos_sensor(0x302e, 0x0b);//
	OV5645MIPI_write_cmos_sensor(0x3031, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x3611, 0x06);//
	OV5645MIPI_write_cmos_sensor(0x3612, 0xab);//
	OV5645MIPI_write_cmos_sensor(0x3614, 0x50);//
	OV5645MIPI_write_cmos_sensor(0x3618, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x4800, 0x04);//chage mipi data free/gate
	OV5645MIPI_write_cmos_sensor(0x3034, 0x18);//	; PLL, MIPI 8-bit mode
	OV5645MIPI_write_cmos_sensor(0x3035, 0x21);//	; PLL
	OV5645MIPI_write_cmos_sensor(0x3036, 0x70);//	; PLL
	OV5645MIPI_write_cmos_sensor(0x3037, 0x13); // ; PLL
	OV5645MIPI_write_cmos_sensor(0x3108, 0x01); // ; PLL
	OV5645MIPI_write_cmos_sensor(0x3824, 0x01); // ; PLL
	OV5645MIPI_write_cmos_sensor(0x460c, 0x20); // ; PLL
	OV5645MIPI_write_cmos_sensor(0x3500, 0x00);//	; exposure = 0x100
	OV5645MIPI_write_cmos_sensor(0x3501, 0x01);//	; exposure
	OV5645MIPI_write_cmos_sensor(0x3502, 0x00);//	; exposure
	OV5645MIPI_write_cmos_sensor(0x350a, 0x00);//	; gain = 0x3f
	OV5645MIPI_write_cmos_sensor(0x350b, 0x3f);//	; gain
	OV5645MIPI_write_cmos_sensor(0x3600, 0x09);//
	OV5645MIPI_write_cmos_sensor(0x3601, 0x43);//
	OV5645MIPI_write_cmos_sensor(0x3620, 0x33);//
	OV5645MIPI_write_cmos_sensor(0x3621, 0xe0);//
	OV5645MIPI_write_cmos_sensor(0x3622, 0x01);//
	OV5645MIPI_write_cmos_sensor(0x3630, 0x2d);//
	OV5645MIPI_write_cmos_sensor(0x3631, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x3632, 0x32);//
	OV5645MIPI_write_cmos_sensor(0x3633, 0x52);//
	OV5645MIPI_write_cmos_sensor(0x3634, 0x70);//
	OV5645MIPI_write_cmos_sensor(0x3635, 0x13);//
	OV5645MIPI_write_cmos_sensor(0x3636, 0x03);//
	OV5645MIPI_write_cmos_sensor(0x3702, 0x6e);//
	OV5645MIPI_write_cmos_sensor(0x3703, 0x52);//
	OV5645MIPI_write_cmos_sensor(0x3704, 0xa0);//
	OV5645MIPI_write_cmos_sensor(0x3705, 0x33);//
	OV5645MIPI_write_cmos_sensor(0x3708, 0x66);//
	OV5645MIPI_write_cmos_sensor(0x3709, 0x12);//
	OV5645MIPI_write_cmos_sensor(0x370b, 0x61);//
	OV5645MIPI_write_cmos_sensor(0x370c, 0xc3);//
	OV5645MIPI_write_cmos_sensor(0x370f, 0x10);//
	OV5645MIPI_write_cmos_sensor(0x3715, 0x08);//
	OV5645MIPI_write_cmos_sensor(0x3717, 0x01);//
	OV5645MIPI_write_cmos_sensor(0x371b, 0x20);//
	OV5645MIPI_write_cmos_sensor(0x3731, 0x22);//
	OV5645MIPI_write_cmos_sensor(0x3739, 0x70);//
	OV5645MIPI_write_cmos_sensor(0x3901, 0x0a);//
	OV5645MIPI_write_cmos_sensor(0x3905, 0x02);//
	OV5645MIPI_write_cmos_sensor(0x3906, 0x10);//
	OV5645MIPI_write_cmos_sensor(0x3719, 0x86);//
	OV5645MIPI_write_cmos_sensor(0x3800, 0x00);//	; HS = 0
	OV5645MIPI_write_cmos_sensor(0x3801, 0x00);//	; HS
	OV5645MIPI_write_cmos_sensor(0x3802, 0x00);//	; VS = 250
	OV5645MIPI_write_cmos_sensor(0x3803, 0x06);//	; VS
	OV5645MIPI_write_cmos_sensor(0x3804, 0x0a);//	; HW = 2623
	OV5645MIPI_write_cmos_sensor(0x3805, 0x3f);//	; HW
	OV5645MIPI_write_cmos_sensor(0x3806, 0x07);//	; VH = 1705
	OV5645MIPI_write_cmos_sensor(0x3807, 0x9d);//	; VH
	OV5645MIPI_write_cmos_sensor(0x3808, 0x05);//	; DVPHO = 1280
	OV5645MIPI_write_cmos_sensor(0x3809, 0x00);//	; DVPHO
	OV5645MIPI_write_cmos_sensor(0x380a, 0x03);//	; DVPHO
	OV5645MIPI_write_cmos_sensor(0x380b, 0xc0);//	; DVPVO
	OV5645MIPI_write_cmos_sensor(0x380c, 0x07);//	; HTS = 2160
	OV5645MIPI_write_cmos_sensor(0x380d, 0x68);//	; HTS
	OV5645MIPI_write_cmos_sensor(0x380e, 0x03);//	; VTS = 740
	OV5645MIPI_write_cmos_sensor(0x380f, 0xd8);//	; VTS
	OV5645MIPI_write_cmos_sensor(0x3810, 0x00);//	; H OFF = 16
	OV5645MIPI_write_cmos_sensor(0x3811, 0x10);//	; H OFF
	OV5645MIPI_write_cmos_sensor(0x3812, 0x00);//	; V OFF = 4
	OV5645MIPI_write_cmos_sensor(0x3813, 0x06);//	; V OFF
	OV5645MIPI_write_cmos_sensor(0x3814, 0x31);//	; X INC
	OV5645MIPI_write_cmos_sensor(0x3815, 0x31);//	; Y INC
	OV5645MIPI_write_cmos_sensor(0x3820, 0x41);//	; flip off, V bin on
	OV5645MIPI_write_cmos_sensor(0x3821, 0x07);//	; mirror on, H bin on
	OV5645MIPI_write_cmos_sensor(0x3826, 0x03); // 
	OV5645MIPI_write_cmos_sensor(0x3828, 0x08);//
	OV5645MIPI_write_cmos_sensor(0x3a02, 0x03);//	; max exp 60 = 740
	OV5645MIPI_write_cmos_sensor(0x3a03, 0xd8);//	; max exp 60
	OV5645MIPI_write_cmos_sensor(0x3a08, 0x01);//	; B50 = 222
	OV5645MIPI_write_cmos_sensor(0x3a09, 0x27); // ; B50
	OV5645MIPI_write_cmos_sensor(0x3a0a, 0x00); // ; B60 = 185
	OV5645MIPI_write_cmos_sensor(0x3a0b, 0xf6); // ; B60
	OV5645MIPI_write_cmos_sensor(0x3a0e, 0x03); // ; max 50
	OV5645MIPI_write_cmos_sensor(0x3a0d, 0x04); // ; max 60
	OV5645MIPI_write_cmos_sensor(0x3a14, 0x03); // ; max exp 50 = 740
	OV5645MIPI_write_cmos_sensor(0x3a15, 0xd8);//	; max exp 50
	OV5645MIPI_write_cmos_sensor(0x3a18, 0x00);//	; gain ceiling = 15.5x
	OV5645MIPI_write_cmos_sensor(0x3a19, 0x60);//	; gain ceiling
	OV5645MIPI_write_cmos_sensor(0x3a05, 0x30);//	; enable band insert, ken,  
	OV5645MIPI_write_cmos_sensor(0x3c01, 0xb4); // ;manual banding mode
	OV5645MIPI_write_cmos_sensor(0x3c00, 0x04); // ;50 Banding mode 
	OV5645MIPI_write_cmos_sensor(0x3c04, 0x28);//
	OV5645MIPI_write_cmos_sensor(0x3c05, 0x98);//
	OV5645MIPI_write_cmos_sensor(0x3c07, 0x07);//
	OV5645MIPI_write_cmos_sensor(0x3c08, 0x01);//
	OV5645MIPI_write_cmos_sensor(0x3c09, 0xc2);//
	OV5645MIPI_write_cmos_sensor(0x3c0a, 0x9c);//
	OV5645MIPI_write_cmos_sensor(0x3c0b, 0x40);//
	OV5645MIPI_write_cmos_sensor(0x4001, 0x02);//	; BLC start line
	OV5645MIPI_write_cmos_sensor(0x4004, 0x02);//	; BLC line number
	OV5645MIPI_write_cmos_sensor(0x4005, 0x18);//	; BLC update triggered by gain change
	OV5645MIPI_write_cmos_sensor(0x4050, 0x6e);//	; BLC line number
	OV5645MIPI_write_cmos_sensor(0x4051, 0x8f);//	; BLC update triggered by gain change
	OV5645MIPI_write_cmos_sensor(0x4300, 0x30);//	; YUV 422, YUYV
	OV5645MIPI_write_cmos_sensor(0x4514, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x4520, 0xb0);//
	OV5645MIPI_write_cmos_sensor(0x460b, 0x37);//
	OV5645MIPI_write_cmos_sensor(0x4818, 0x01);//
	OV5645MIPI_write_cmos_sensor(0x481d, 0xf0);//
	OV5645MIPI_write_cmos_sensor(0x481f, 0x50);//
	OV5645MIPI_write_cmos_sensor(0x4823, 0x70);//
	OV5645MIPI_write_cmos_sensor(0x4831, 0x14);//
	OV5645MIPI_write_cmos_sensor(0x4837, 0x11);//
	OV5645MIPI_write_cmos_sensor(0x5000, 0xa7);//	; Lenc on, raw gamma on, BPC on, WPC on, color interpolation on
	OV5645MIPI_write_cmos_sensor(0x5001, 0xa3);//	; SDE on, scale off, UV adjust off, color matrix on, AWB on
	OV5645MIPI_write_cmos_sensor(0x5002, 0x80);//   
	OV5645MIPI_write_cmos_sensor(0x501d, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x501f, 0x00);//	; select ISP YUV 422
	OV5645MIPI_write_cmos_sensor(0x503d, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x505c, 0x30);//
	OV5645MIPI_write_cmos_sensor(0x5181, 0x59);//
	OV5645MIPI_write_cmos_sensor(0x5183, 0x00);//0x00 leanda 
	OV5645MIPI_write_cmos_sensor(0x5191, 0xf0);//
	OV5645MIPI_write_cmos_sensor(0x5192, 0x03);//    
	OV5645MIPI_write_cmos_sensor(0x5684, 0x10);//
	OV5645MIPI_write_cmos_sensor(0x5685, 0xa0);//
	OV5645MIPI_write_cmos_sensor(0x5686, 0x0c);//
	OV5645MIPI_write_cmos_sensor(0x5687, 0x78);//
	OV5645MIPI_write_cmos_sensor(0x5a00, 0x08);//
	OV5645MIPI_write_cmos_sensor(0x5a21, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x5a24, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x3008, 0x02);//	; wake up from software standby
	OV5645MIPI_write_cmos_sensor(0x3503, 0x00);//	; AGC on, AEC on
	OV5645MIPI_write_cmos_sensor(0x5180, 0xff);// ;awb
	OV5645MIPI_write_cmos_sensor(0x5181, 0xf2);//
	OV5645MIPI_write_cmos_sensor(0x5182, 0x0 );//
	OV5645MIPI_write_cmos_sensor(0x5183, 0x14);//0x14 leanda 
	OV5645MIPI_write_cmos_sensor(0x5184, 0x25);//
	OV5645MIPI_write_cmos_sensor(0x5185, 0x24);//
	OV5645MIPI_write_cmos_sensor(0x5186, 0xC);//
	OV5645MIPI_write_cmos_sensor(0x5187, 0x15);//
	OV5645MIPI_write_cmos_sensor(0x5188, 0xC);//
	OV5645MIPI_write_cmos_sensor(0x5189, 0x7B);//
	OV5645MIPI_write_cmos_sensor(0x518a, 0x5C);//
	OV5645MIPI_write_cmos_sensor(0x518b, 0xC3);//
	OV5645MIPI_write_cmos_sensor(0x518c, 0x14);//
	OV5645MIPI_write_cmos_sensor(0x518d, 0x3F);//
	OV5645MIPI_write_cmos_sensor(0x518e, 0x30);//
	OV5645MIPI_write_cmos_sensor(0x518f, 0x57);//
	OV5645MIPI_write_cmos_sensor(0x5190, 0x3D);//
	OV5645MIPI_write_cmos_sensor(0x5191, 0xf8);//
	OV5645MIPI_write_cmos_sensor(0x5192, 0x4 );//
	OV5645MIPI_write_cmos_sensor(0x5193, 0x70);//
	OV5645MIPI_write_cmos_sensor(0x5194, 0xf0);//
	OV5645MIPI_write_cmos_sensor(0x5195, 0xf0);//
	OV5645MIPI_write_cmos_sensor(0x5196, 0x3 );//
	OV5645MIPI_write_cmos_sensor(0x5197, 0x1 );//
	OV5645MIPI_write_cmos_sensor(0x5198, 0x5 );//
	OV5645MIPI_write_cmos_sensor(0x5199, 0x1A);//
	OV5645MIPI_write_cmos_sensor(0x519a, 0x4 );//
	OV5645MIPI_write_cmos_sensor(0x519b, 0x0 );//
	OV5645MIPI_write_cmos_sensor(0x519c, 0x4 );//
	OV5645MIPI_write_cmos_sensor(0x519d, 0xB2);//
	OV5645MIPI_write_cmos_sensor(0x519e, 0x38);//
	OV5645MIPI_write_cmos_sensor(0x5381, 0x27);//ccm  27
	OV5645MIPI_write_cmos_sensor(0x5382, 0x50);//50
	OV5645MIPI_write_cmos_sensor(0x5383, 0x11);//11
	OV5645MIPI_write_cmos_sensor(0x5384, 0x0a);//0a
	OV5645MIPI_write_cmos_sensor(0x5385, 0x80);//66
	OV5645MIPI_write_cmos_sensor(0x5386, 0x83);//71
	OV5645MIPI_write_cmos_sensor(0x5387, 0x81);//7c
	OV5645MIPI_write_cmos_sensor(0x5388, 0x6f);//6b
	OV5645MIPI_write_cmos_sensor(0x5389, 0x0f);//11
	OV5645MIPI_write_cmos_sensor(0x538a, 0x01);//
	OV5645MIPI_write_cmos_sensor(0x538b, 0x98);//
	OV5645MIPI_write_cmos_sensor(0x5300, 0x08);//	; sharpen MT th1
	OV5645MIPI_write_cmos_sensor(0x5301, 0x30);//	; sharpen MT th2
	OV5645MIPI_write_cmos_sensor(0x5302, 0x18);//	; sharpen MT off1
	OV5645MIPI_write_cmos_sensor(0x5303, 0x08);//	; sharpen MT off2
	OV5645MIPI_write_cmos_sensor(0x5304, 0x08);//	; DNS th1
	OV5645MIPI_write_cmos_sensor(0x5305, 0x30);//	; DNS th2
	OV5645MIPI_write_cmos_sensor(0x5306, 0x08);//	; DNS off1
	OV5645MIPI_write_cmos_sensor(0x5307, 0x16);//	; DNS off2
	OV5645MIPI_write_cmos_sensor(0x5309, 0x08);//	; sharpen TH th1
	OV5645MIPI_write_cmos_sensor(0x530a, 0x30);//	; sharpen TH th2
	OV5645MIPI_write_cmos_sensor(0x530b, 0x04);//	; sharpen TH th2
	OV5645MIPI_write_cmos_sensor(0x530c, 0x06);//	; sharpen TH off2
	OV5645MIPI_write_cmos_sensor(0x5480, 0x01);//	; bias on
	OV5645MIPI_write_cmos_sensor(0x5481, 0x08);//	; Y yst 00
	OV5645MIPI_write_cmos_sensor(0x5482, 0x14);//
	OV5645MIPI_write_cmos_sensor(0x5483, 0x28);//
	OV5645MIPI_write_cmos_sensor(0x5484, 0x51);//
	OV5645MIPI_write_cmos_sensor(0x5485, 0x65);//
	OV5645MIPI_write_cmos_sensor(0x5486, 0x71);//
	OV5645MIPI_write_cmos_sensor(0x5487, 0x7D);//
	OV5645MIPI_write_cmos_sensor(0x5488, 0x87);//
	OV5645MIPI_write_cmos_sensor(0x5489, 0x91);//
	OV5645MIPI_write_cmos_sensor(0x548a, 0x9A);//
	OV5645MIPI_write_cmos_sensor(0x548b, 0xaA);//
	OV5645MIPI_write_cmos_sensor(0x548c, 0xB8);//
	OV5645MIPI_write_cmos_sensor(0x548d, 0xcD);//
	OV5645MIPI_write_cmos_sensor(0x548e, 0xdD);//
	OV5645MIPI_write_cmos_sensor(0x548f, 0xeA);//
	OV5645MIPI_write_cmos_sensor(0x5490, 0x1D);//
	OV5645MIPI_write_cmos_sensor(0x5588, 0x01);//
	OV5645MIPI_write_cmos_sensor(0x5580, 0x06);//
	OV5645MIPI_write_cmos_sensor(0x5583, 0x40);//
	OV5645MIPI_write_cmos_sensor(0x5584, 0x30);//
	OV5645MIPI_write_cmos_sensor(0x5589, 0x18);//
	OV5645MIPI_write_cmos_sensor(0x558a, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x558b, 0x3c);//
	OV5645MIPI_write_cmos_sensor(0x5800, 0x35);//lsc
	OV5645MIPI_write_cmos_sensor(0x5801, 0x18);//
	OV5645MIPI_write_cmos_sensor(0x5802, 0x13);//
	OV5645MIPI_write_cmos_sensor(0x5803, 0x14);//
	OV5645MIPI_write_cmos_sensor(0x5804, 0x18);//
	OV5645MIPI_write_cmos_sensor(0x5805, 0x3e);//
	OV5645MIPI_write_cmos_sensor(0x5806, 0xe );//
	OV5645MIPI_write_cmos_sensor(0x5807, 0xa );//
	OV5645MIPI_write_cmos_sensor(0x5808, 0x8 );//
	OV5645MIPI_write_cmos_sensor(0x5809, 0x8 );//
	OV5645MIPI_write_cmos_sensor(0x580a, 0xb );//
	OV5645MIPI_write_cmos_sensor(0x580b, 0xf );//
	OV5645MIPI_write_cmos_sensor(0x580c, 0x9 );//
	OV5645MIPI_write_cmos_sensor(0x580d, 0x3 );//
	OV5645MIPI_write_cmos_sensor(0x580e, 0x0 );//
	OV5645MIPI_write_cmos_sensor(0x580f, 0x1 );//
	OV5645MIPI_write_cmos_sensor(0x5810, 0x4 );//
	OV5645MIPI_write_cmos_sensor(0x5811, 0xc );//
	OV5645MIPI_write_cmos_sensor(0x5812, 0x8 );//
	OV5645MIPI_write_cmos_sensor(0x5813, 0x3 );//
	OV5645MIPI_write_cmos_sensor(0x5814, 0x1 );//
	OV5645MIPI_write_cmos_sensor(0x5815, 0x1 );//
	OV5645MIPI_write_cmos_sensor(0x5816, 0x4 );//
	OV5645MIPI_write_cmos_sensor(0x5817, 0xa );//
	OV5645MIPI_write_cmos_sensor(0x5818, 0xc );//
	OV5645MIPI_write_cmos_sensor(0x5819, 0x8 );//
	OV5645MIPI_write_cmos_sensor(0x581a, 0x5 );//
	OV5645MIPI_write_cmos_sensor(0x581b, 0x6 );//
	OV5645MIPI_write_cmos_sensor(0x581c, 0x9 );//
	OV5645MIPI_write_cmos_sensor(0x581d, 0xf );//
	OV5645MIPI_write_cmos_sensor(0x581e, 0x26);//
	OV5645MIPI_write_cmos_sensor(0x581f, 0x13);//
	OV5645MIPI_write_cmos_sensor(0x5820, 0xf );//
	OV5645MIPI_write_cmos_sensor(0x5821, 0x10);//
	OV5645MIPI_write_cmos_sensor(0x5822, 0x13);//
	OV5645MIPI_write_cmos_sensor(0x5823, 0x27);//
	OV5645MIPI_write_cmos_sensor(0x5824, 0x8 );//
	OV5645MIPI_write_cmos_sensor(0x5825, 0xc );//
	OV5645MIPI_write_cmos_sensor(0x5826, 0xe );//
	OV5645MIPI_write_cmos_sensor(0x5827, 0xc );//
	OV5645MIPI_write_cmos_sensor(0x5828, 0x4a);//
	OV5645MIPI_write_cmos_sensor(0x5829, 0xc );//
	OV5645MIPI_write_cmos_sensor(0x582a, 0x6 );//
	OV5645MIPI_write_cmos_sensor(0x582b, 0x4 );//
	OV5645MIPI_write_cmos_sensor(0x582c, 0x8 );//
	OV5645MIPI_write_cmos_sensor(0x582d, 0xa );//
	OV5645MIPI_write_cmos_sensor(0x582e, 0xc );//
	OV5645MIPI_write_cmos_sensor(0x582f, 0x22);//
	OV5645MIPI_write_cmos_sensor(0x5830, 0x20);//
	OV5645MIPI_write_cmos_sensor(0x5831, 0x22);//
	OV5645MIPI_write_cmos_sensor(0x5832, 0xa );//
	OV5645MIPI_write_cmos_sensor(0x5833, 0xc );//
	OV5645MIPI_write_cmos_sensor(0x5834, 0x6 );//
	OV5645MIPI_write_cmos_sensor(0x5835, 0x4 );//
	OV5645MIPI_write_cmos_sensor(0x5836, 0x6 );//
	OV5645MIPI_write_cmos_sensor(0x5837, 0xa );//
	OV5645MIPI_write_cmos_sensor(0x5838, 0xc );//
	OV5645MIPI_write_cmos_sensor(0x5839, 0xc );//
	OV5645MIPI_write_cmos_sensor(0x583a, 0xe );//
	OV5645MIPI_write_cmos_sensor(0x583b, 0xc );//
	OV5645MIPI_write_cmos_sensor(0x583c, 0xa );//
	OV5645MIPI_write_cmos_sensor(0x583d, 0xee);//
	OV5645MIPI_write_cmos_sensor(0x583e, 0x10);//
	OV5645MIPI_write_cmos_sensor(0x583f, 0x08);//
	OV5645MIPI_write_cmos_sensor(0x5840, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x5025, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x3a00, 0x38);//	; ae mode	
	OV5645MIPI_write_cmos_sensor(0x3a0f, 0x30);//	; AEC in H
	OV5645MIPI_write_cmos_sensor(0x3a10, 0x28);//	; AEC in L
	OV5645MIPI_write_cmos_sensor(0x3a1b, 0x30);//	; AEC out H
	OV5645MIPI_write_cmos_sensor(0x3a1e, 0x26);//	; AEC out L
	OV5645MIPI_write_cmos_sensor(0x3a11, 0x60);//	; control zone H
	OV5645MIPI_write_cmos_sensor(0x3a1f, 0x14);//	; control zone L
	OV5645MIPI_write_cmos_sensor(0x501d, 0x10);
  OV5645MIPI_write_cmos_sensor(0x5680, 0x00); 
  OV5645MIPI_write_cmos_sensor(0x5681, 0x00);
  OV5645MIPI_write_cmos_sensor(0x5682, 0x00);  
  OV5645MIPI_write_cmos_sensor(0x5683, 0x00);
  OV5645MIPI_write_cmos_sensor(0x5684, 0x05); //width=256  
  OV5645MIPI_write_cmos_sensor(0x5685, 0x00);
  OV5645MIPI_write_cmos_sensor(0x5686, 0x03); //heght=256
  OV5645MIPI_write_cmos_sensor(0x5687, 0xc0);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIInitialSetting function:\n ");
		
} 
/*****************************************************************
* FUNCTION
*    OV5645MIPIPreviewSetting
*
* DESCRIPTION
*    This function config Preview setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void OV5645MIPIPreviewSetting_SVGA(void)
{
	//;OV5645MIPI 1280x960,30fps
	//56Mhz, 224Mbps/Lane, 2Lane.
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIPreviewSetting_SVGA function:\n ");
	OV5645MIPI_write_cmos_sensor(0x4202, 0x0f);//	; stop mipi stream
	OV5645MIPI_write_cmos_sensor(0x300e, 0x45);//	; MIPI 2 lane
	OV5645MIPI_write_cmos_sensor(0x3034, 0x18);// PLL, MIPI 8-bit mode
	OV5645MIPI_write_cmos_sensor(0x3035, 0x21);// PLL
	OV5645MIPI_write_cmos_sensor(0x3036, 0x70);// PLL
	OV5645MIPI_write_cmos_sensor(0x3037, 0x13);// PLL
	OV5645MIPI_write_cmos_sensor(0x3108, 0x01);// PLL
	OV5645MIPI_write_cmos_sensor(0x3824, 0x01);// PLL
	OV5645MIPI_write_cmos_sensor(0x460c, 0x20);// PLL
	OV5645MIPI_write_cmos_sensor(0x3618, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x3600, 0x09);//
	OV5645MIPI_write_cmos_sensor(0x3601, 0x43);//
	OV5645MIPI_write_cmos_sensor(0x3708, 0x66);//
	OV5645MIPI_write_cmos_sensor(0x3709, 0x12);//
	OV5645MIPI_write_cmos_sensor(0x370c, 0xc3);//
	OV5645MIPI_write_cmos_sensor(0x3800, 0x00); // HS = 0
	OV5645MIPI_write_cmos_sensor(0x3801, 0x00); // HS
	OV5645MIPI_write_cmos_sensor(0x3802, 0x00); // VS = 250
	OV5645MIPI_write_cmos_sensor(0x3803, 0x06); // VS
	OV5645MIPI_write_cmos_sensor(0x3804, 0x0a); // HW = 2623
	OV5645MIPI_write_cmos_sensor(0x3805, 0x3f);//	; HW
	OV5645MIPI_write_cmos_sensor(0x3806, 0x07);//	; VH = 
	OV5645MIPI_write_cmos_sensor(0x3807, 0x9d);//	; VH
	OV5645MIPI_write_cmos_sensor(0x3808, 0x05);//	; DVPHO = 1280
	OV5645MIPI_write_cmos_sensor(0x3809, 0x00);//	; DVPHO
	OV5645MIPI_write_cmos_sensor(0x380a, 0x03);//	; DVPVO = 960
	OV5645MIPI_write_cmos_sensor(0x380b, 0xc0);//	; DVPVO
	OV5645MIPI_write_cmos_sensor(0x380c, 0x07);//	; HTS = 2160
	OV5645MIPI_write_cmos_sensor(0x380d, 0x68);//	; HTS
	OV5645MIPI_write_cmos_sensor(0x380e, 0x03);//	; VTS = 740
	OV5645MIPI_write_cmos_sensor(0x380f, 0xd8);//	; VTS
	OV5645MIPI_write_cmos_sensor(0x3810, 0x00); // H OFF = 16
	OV5645MIPI_write_cmos_sensor(0x3811, 0x10); // H OFF
	OV5645MIPI_write_cmos_sensor(0x3812, 0x00); // V OFF = 4
	OV5645MIPI_write_cmos_sensor(0x3813, 0x06);//	; V OFF
	OV5645MIPI_write_cmos_sensor(0x3814, 0x31);//	; X INC
	OV5645MIPI_write_cmos_sensor(0x3815, 0x31);//	; Y INC
	OV5645MIPI_write_cmos_sensor(0x3820, 0x41);//	; flip off, V bin on
	OV5645MIPI_write_cmos_sensor(0x3821, 0x07);//	; mirror on, H bin on
	OV5645MIPI_write_cmos_sensor(0x3a02, 0x03);//	; max exp 60 = 740
	OV5645MIPI_write_cmos_sensor(0x3a03, 0xd8);//	; max exp 60
	OV5645MIPI_write_cmos_sensor(0x3a08, 0x01);//	; B50 = 222
	OV5645MIPI_write_cmos_sensor(0x3a09, 0x27);//	; B50
	OV5645MIPI_write_cmos_sensor(0x3a0a, 0x00);//	; B60 = 185
	OV5645MIPI_write_cmos_sensor(0x3a0b, 0xf6);//	; B60
	OV5645MIPI_write_cmos_sensor(0x3a0e, 0x03);//	; max 50
	OV5645MIPI_write_cmos_sensor(0x3a0d, 0x04);//	; max 60
	OV5645MIPI_write_cmos_sensor(0x3a14, 0x03);//	; max exp 50 = 740
	OV5645MIPI_write_cmos_sensor(0x3a15, 0xd8);//	; max exp 50
	OV5645MIPI_write_cmos_sensor(0x3c07, 0x07);//	; 50/60 auto detect
	OV5645MIPI_write_cmos_sensor(0x3c08, 0x01);//	; 50/60 auto detect
	OV5645MIPI_write_cmos_sensor(0x3c09, 0xc2);//	; 50/60 auto detect
	OV5645MIPI_write_cmos_sensor(0x4004, 0x02);//	; BLC line number
	OV5645MIPI_write_cmos_sensor(0x4005, 0x18);//	; BLC triggered by gain change
	OV5645MIPI_write_cmos_sensor(0x4837, 0x11); // MIPI global timing 16           
	OV5645MIPI_write_cmos_sensor(0x503d, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x5000, 0xa7);//
	OV5645MIPI_write_cmos_sensor(0x5001, 0x83);//
	OV5645MIPI_write_cmos_sensor(0x5002, 0x80);//
	OV5645MIPI_write_cmos_sensor(0x5003, 0x08);//
	OV5645MIPI_write_cmos_sensor(0x3032, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x4000, 0x89);//
	OV5645MIPI_write_cmos_sensor(0x5583, 0x40);//
	OV5645MIPI_write_cmos_sensor(0x5584, 0x30);//
	OV5645MIPI_write_cmos_sensor(0x3a00, 0x3c);//	; ae mode	
	
	OV5645MIPI_write_cmos_sensor(0x5302, 0x2a);//	; 
	OV5645MIPI_write_cmos_sensor(0x5303, 0x1a);//	; 
	OV5645MIPI_write_cmos_sensor(0x5306, 0x18);//	; DNS off1
	OV5645MIPI_write_cmos_sensor(0x5307, 0x28);//	; DNS off2
	
	OV5645MIPI_write_cmos_sensor(0x4202, 0x00);//	; open mipi stream
	OV5645MIPIWriteExpShutter(OV5645MIPISensor.PreviewShutter);
	OV5645MIPIWriteExtraShutter(OV5645MIPISensor.PreviewExtraShutter);
	spin_lock(&ov5645mipi_drv_lock);
	OV5645MIPISensor.SensorMode= SENSOR_MODE_PREVIEW;
	OV5645MIPISensor.IsPVmode = KAL_TRUE;
	OV5645MIPISensor.PreviewPclk= 560;	
	spin_unlock(&ov5645mipi_drv_lock);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIPreviewSetting_SVGA function:\n ");
}

/*************************************************************************
* FUNCTION
*     OV5645MIPIFullSizeCaptureSetting
*
* DESCRIPTION
*    This function config full size capture setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void OV5645MIPIFullSizeCaptureSetting(void)
{
	//OV5645MIPI 2592x1944,10fps
	//90Mhz, 360Mbps/Lane, 2Lane.15
	OV5645MIPI_write_cmos_sensor(0x4202, 0x0f);//	; stop mipi stream
	OV5645MIPI_write_cmos_sensor(0x3a00, 0x38);//	; ae mode	
	OV5645MIPI_write_cmos_sensor(0x300e, 0x45);//	; MIPI 2 lane
	OV5645MIPI_write_cmos_sensor(0x3034, 0x18); //PLL, MIPI 8-bit mode
	OV5645MIPI_write_cmos_sensor(0x3035, 0x11); //PLL
	OV5645MIPI_write_cmos_sensor(0x3036, 0x5a); //PLL
	OV5645MIPI_write_cmos_sensor(0x3037, 0x13); //PLL
	OV5645MIPI_write_cmos_sensor(0x3108, 0x01); //PLL
	OV5645MIPI_write_cmos_sensor(0x3824, 0x01); //PLL
	OV5645MIPI_write_cmos_sensor(0x460c, 0x20); //PLL
	OV5645MIPI_write_cmos_sensor(0x3618, 0x04);//
	OV5645MIPI_write_cmos_sensor(0x3600, 0x08);//
	OV5645MIPI_write_cmos_sensor(0x3601, 0x33);//
	OV5645MIPI_write_cmos_sensor(0x3708, 0x63);//
	OV5645MIPI_write_cmos_sensor(0x3709, 0x12);//
	OV5645MIPI_write_cmos_sensor(0x370c, 0xc0);//
	OV5645MIPI_write_cmos_sensor(0x3800, 0x00); //HS = 0
	OV5645MIPI_write_cmos_sensor(0x3801, 0x00); //HS
	OV5645MIPI_write_cmos_sensor(0x3802, 0x00); //VS = 0
	OV5645MIPI_write_cmos_sensor(0x3803, 0x00); //VS
	OV5645MIPI_write_cmos_sensor(0x3804, 0x0a); //HW = 2623
	OV5645MIPI_write_cmos_sensor(0x3805, 0x3f);//	; HW
	OV5645MIPI_write_cmos_sensor(0x3806, 0x07);//	; VH = 1705
	OV5645MIPI_write_cmos_sensor(0x3807, 0x9f);//	; VH
	OV5645MIPI_write_cmos_sensor(0x3808, 0x0a);//	; DVPHO = 2560
	OV5645MIPI_write_cmos_sensor(0x3809, 0x20);//	; DVPHO
	OV5645MIPI_write_cmos_sensor(0x380a, 0x07);//	; DVPVO = 1440
	OV5645MIPI_write_cmos_sensor(0x380b, 0x98);//	; DVPVO
	OV5645MIPI_write_cmos_sensor(0x380c, 0x0b);//	; HTS = 2984
	OV5645MIPI_write_cmos_sensor(0x380d, 0xec);//	; HTS
	OV5645MIPI_write_cmos_sensor(0x380e, 0x07);//	; VTS = 1464
	OV5645MIPI_write_cmos_sensor(0x380f, 0xb0);//	; VTS
	OV5645MIPI_write_cmos_sensor(0x3810, 0x00); //H OFF = 16
	OV5645MIPI_write_cmos_sensor(0x3811, 0x10); //H OFF
	OV5645MIPI_write_cmos_sensor(0x3812, 0x00); //V OFF = 4
	OV5645MIPI_write_cmos_sensor(0x3813, 0x06);//	; V OFF
	OV5645MIPI_write_cmos_sensor(0x3814, 0x11);//	; X INC
	OV5645MIPI_write_cmos_sensor(0x3815, 0x11);//	; Y INC
	OV5645MIPI_write_cmos_sensor(0x3820, 0x40);//	; flip off, V bin off
	OV5645MIPI_write_cmos_sensor(0x3821, 0x06);//	; mirror on, H bin off
	OV5645MIPI_write_cmos_sensor(0x3a0e, 0x06);//	; flip off, V bin off
	OV5645MIPI_write_cmos_sensor(0x3a0d, 0x08);//	; mirror on, H bin off
	OV5645MIPI_write_cmos_sensor(0x4004, 0x06);//	; BLC line number
	OV5645MIPI_write_cmos_sensor(0x4837, 0x16);//; MIPI global timing  
	OV5645MIPI_write_cmos_sensor(0x503d, 0x00);//
	OV5645MIPI_write_cmos_sensor(0x5000, 0xa7);//
	OV5645MIPI_write_cmos_sensor(0x5001, 0x83);//
	OV5645MIPI_write_cmos_sensor(0x5002, 0x80);
	OV5645MIPI_write_cmos_sensor(0x5003, 0x08);
	OV5645MIPI_write_cmos_sensor(0x3032, 0x00);
	OV5645MIPI_write_cmos_sensor(0x4000, 0x89);
	OV5645MIPI_write_cmos_sensor(0x350c, 0x00);
	OV5645MIPI_write_cmos_sensor(0x350d, 0x00);
	OV5645MIPI_write_cmos_sensor(0x4202, 0x00);//	; open mipi stream
	spin_lock(&ov5645mipi_drv_lock);
	OV5645MIPISensor.IsPVmode = KAL_FALSE;
	OV5645MIPISensor.CapturePclk= 900;	
	OV5645MIPISensor.SensorMode= SENSOR_MODE_CAPTURE;
	spin_unlock(&ov5645mipi_drv_lock);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIFullSizeCaptureSetting function:\n ");
}
/*************************************************************************
* FUNCTION
*    OV5645MIPISetHVMirror
*
* DESCRIPTION
*    This function set sensor Mirror
*
* PARAMETERS
*    Mirror
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void OV5645MIPISetHVMirror(kal_uint8 Mirror, kal_uint8 Mode)
{
  	kal_uint8 mirror= 0, flip=0;
    OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPISetHVMirror function:\n ");
		flip = OV5645MIPIYUV_read_cmos_sensor(0x3820);
		mirror=OV5645MIPIYUV_read_cmos_sensor(0x3821);
	
	if (Mode==SENSOR_MODE_PREVIEW)
	{
		switch (Mirror)
		{
		case IMAGE_V_MIRROR:
			OV5645MIPI_write_cmos_sensor(0x3820, flip&0xf9);     
			OV5645MIPI_write_cmos_sensor(0x3821, mirror&0xf9);
			OV5645MIPI_write_cmos_sensor(0x4514, 0x00);
			break;
		case IMAGE_H_MIRROR:
			OV5645MIPI_write_cmos_sensor(0x3820, flip&0xf9);     
			OV5645MIPI_write_cmos_sensor(0x3821, mirror|0x06);
			OV5645MIPI_write_cmos_sensor(0x4514, 0x00);
			break;
		case IMAGE_NORMAL: 
			OV5645MIPI_write_cmos_sensor(0x3820, flip|0x06);     
			OV5645MIPI_write_cmos_sensor(0x3821, mirror&0xf9);
			OV5645MIPI_write_cmos_sensor(0x4514, 0x88);
			break;		
		case IMAGE_HV_MIRROR:
			OV5645MIPI_write_cmos_sensor(0x3820, flip|0x06);     
			OV5645MIPI_write_cmos_sensor(0x3821, mirror|0x06);
			OV5645MIPI_write_cmos_sensor(0x4514, 0x00);
			break; 		
		default:
			ASSERT(0);
		}
	}
	else if (Mode== SENSOR_MODE_CAPTURE)
	{
		switch (Mirror)
		{
		case IMAGE_V_MIRROR:
			OV5645MIPI_write_cmos_sensor(0x3820, flip&0xf9);     
			OV5645MIPI_write_cmos_sensor(0x3821, mirror&0xf9);
			OV5645MIPI_write_cmos_sensor(0x4514, 0x00);
			break;
		case IMAGE_H_MIRROR:
			OV5645MIPI_write_cmos_sensor(0x3820, flip&0xf9);     
			OV5645MIPI_write_cmos_sensor(0x3821, mirror|0x06);
			OV5645MIPI_write_cmos_sensor(0x4514, 0x00);
			break;
		case IMAGE_NORMAL: 
			OV5645MIPI_write_cmos_sensor(0x3820, flip|0x06);     
			OV5645MIPI_write_cmos_sensor(0x3821, mirror&0xf9);
			OV5645MIPI_write_cmos_sensor(0x4514, 0x88);
			break;		
		case IMAGE_HV_MIRROR:
			OV5645MIPI_write_cmos_sensor(0x3820, flip|0x06);     
			OV5645MIPI_write_cmos_sensor(0x3821, mirror|0x06);
			OV5645MIPI_write_cmos_sensor(0x4514, 0xbb);
			break; 		
		default:
			ASSERT(0);
		}
	}
	
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPISetHVMirror function:\n ");
}

void OV5645MIPI_Standby(void)
{
	OV5645MIPI_write_cmos_sensor(0x3008,0x42);
}

void OV5645MIPI_Wakeup(void)
{
	OV5645MIPI_write_cmos_sensor(0x3008,0x02);
}
/*************************************************************************
* FUNCTION
*   OV5645_FOCUS_OVT_AFC_Init
* DESCRIPTION
*   This function is to load micro code for AF function
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/
static void OV5645_FOCUS_OVT_AFC_Init(void)
{
	int i, length, address, AF_status;
	OV5645MIPI_write_cmos_sensor(0x3000,0x20);

	length = sizeof(firmware)/sizeof(int);
	address = 0x8000;
	for(i=0;i<length;i++) {
		OV5645MIPI_write_cmos_sensor(address, firmware[i]);
		address++;
	}

	OV5645MIPI_write_cmos_sensor(0x3022,0x00);
	OV5645MIPI_write_cmos_sensor(0x3023,0x00);
	OV5645MIPI_write_cmos_sensor(0x3024,0x00);
	OV5645MIPI_write_cmos_sensor(0x3025,0x00);
	OV5645MIPI_write_cmos_sensor(0x3026,0x00);
	OV5645MIPI_write_cmos_sensor(0x3027,0x00);
	OV5645MIPI_write_cmos_sensor(0x3028,0x00);
	OV5645MIPI_write_cmos_sensor(0x3029,0x7f);
	OV5645MIPI_write_cmos_sensor(0x3000, 0x00); 	   // Enable MCU
	if(false == AF_Power)
	{
		OV5645MIPI_write_cmos_sensor(0x3602,0x00);
		OV5645MIPI_write_cmos_sensor(0x3603,0x00);
	}
}


/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Constant_Focus
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void OV5645_FOCUS_OVT_AFC_Constant_Focus(void)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645_FOCUS_OVT_AFC_Constant_Focus function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3023,0x01);
	OV5645MIPI_write_cmos_sensor(0x3022,0x04);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645_FOCUS_OVT_AFC_Constant_Focus function:\n ");
}     
/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Single_Focus
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void OV5645_FOCUS_OVT_AFC_Single_Focus()
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645_FOCUS_OVT_AFC_Single_Focus function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3023,0x01);
	OV5645MIPI_write_cmos_sensor(0x3022,0x81);
	mDELAY(20);
	OV5645MIPI_write_cmos_sensor(0x3022,0x03);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645_FOCUS_OVT_AFC_Single_Focus function:\n ");
}

/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Pause_Focus
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void OV5645_FOCUS_OVT_AFC_Pause_Focus()
{
	OV5645MIPI_write_cmos_sensor(0x3023,0x01);
    OV5645MIPI_write_cmos_sensor(0x3022,0x06);
}
static void OV5645_FOCUS_Get_AF_Max_Num_Focus_Areas(UINT32 *pFeatureReturnPara32)
{ 	  
	*pFeatureReturnPara32 = 1;    
	OV5645MIPISENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);	
}


static void OV5645_FOCUS_Get_AE_Max_Num_Metering_Areas(UINT32 *pFeatureReturnPara32)
{ 	
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
	*pFeatureReturnPara32 = 1;    
	OV5645MIPISENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
}

static UINT32 OV5645_FOCUS_OVT_AFC_Touch_AF(UINT32 coor_x,UINT32 coor_y)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645_FOCUS_OVT_AFC_Touch_AF function:\n ");
	int x_view,y_view;
	int x_tmp,y_tmp;
	UINT32 coor_x1,coor_y1;
	//check if we really need to update AF window? the tolerance shall less than 10 pixels
	//on the both x/y direction for inner & ouuter window
	if (!((coor_x> (OV5645MIPISensor.AF_window_x + OV5645_TAF_TOLERANCE)) ||
		((coor_x + OV5645_TAF_TOLERANCE) < OV5645MIPISensor.AF_window_x) ||
		(coor_y > (OV5645MIPISensor.AF_window_y + OV5645_TAF_TOLERANCE)) ||
		((coor_y + OV5645_TAF_TOLERANCE) < OV5645MIPISensor.AF_window_y)))
	{
		OV5645MIPISENSORDB("[OV5645MIPI] AF window is very near the previous'.......\n");
		return ERROR_NONE;
	}
	OV5645MIPISensor.AF_window_x = coor_x;
	OV5645MIPISensor.AF_window_y = coor_y ;
	coor_x1=coor_x/16;
	coor_y1=coor_y/16;
	if(coor_x1<1)
	{
		x_view=1;
	}
	else if(coor_x1>79)
	{
		x_view=79;
	}
	else
	{
		x_view= coor_x1;
	}
     
	if(coor_y1<1)
	{
		y_view=1;
	}
	else if(coor_y1>59)
	{
		y_view=59;
	}
	else
	{
		y_view= coor_y1;
	}
	OV5645MIPISENSORDB("[OV5645MIPI]AF x_view=%d,y_view=%d\n",x_view, y_view);
	OV5645MIPI_write_cmos_sensor(0x3024,x_view);
	OV5645MIPI_write_cmos_sensor(0x3025,y_view);   
	x_tmp = OV5645MIPIYUV_read_cmos_sensor(0x3024);
	y_tmp = OV5645MIPIYUV_read_cmos_sensor(0x3025);
	OV5645MIPISENSORDB("[OV5645MIPI]AF x_tmp1=%d,y_tmp1=%d\n",x_tmp, y_tmp);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645_FOCUS_OVT_AFC_Touch_AF function:\n ");
	return ERROR_NONE;
}


static void OV5645_FOCUS_Set_AF_Window(UINT32 zone_addr)
{//update global zone
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645_FOCUS_Set_AF_Window function:\n ");
	UINT32 FD_XS;
	UINT32 FD_YS;   
	UINT32 x0, y0, x1, y1;
	UINT32 pvx0, pvy0, pvx1, pvy1;
	UINT32 linenum, rownum;
	UINT32 AF_pvx, AF_pvy;
	UINT32* zone = (UINT32*)zone_addr;
	x0 = *zone;
	y0 = *(zone + 1);
	x1 = *(zone + 2);
	y1 = *(zone + 3);   
	FD_XS = *(zone + 4);
	FD_YS = *(zone + 5);
	  
	OV5645MIPISENSORDB("AE x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",x0, y0, x1, y1, FD_XS, FD_YS);  
	mapMiddlewaresizePointToPreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, PRV_W, PRV_H);
	mapMiddlewaresizePointToPreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, PRV_W, PRV_H);  
	OV5645MIPISENSORDB("[OV5645MIPI]AF pvx0=%d,pvy0=%d\n",pvx0, pvy0);
	OV5645MIPISENSORDB("[OV5645MIPI]AF pvx0=%d,pvy0=%d\n",pvx1, pvy1);
	AF_pvx =(pvx0+pvx1)/32;
	AF_pvy =(pvy0+pvy1)/32;
	OV5645MIPISENSORDB("[OV5645MIPI]AF AF_pvx=%d,AF_pvy=%d\n",AF_pvx, AF_pvy);
	OV5645_FOCUS_OVT_AFC_Touch_AF(AF_pvx ,AF_pvy);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645_FOCUS_Set_AF_Window function:\n ");
}

static void OV5645_FOCUS_Get_AF_Macro(UINT32 *pFeatureReturnPara32)
{
	*pFeatureReturnPara32 = 0;
}

static void OV5645_FOCUS_Get_AF_Inf(UINT32 * pFeatureReturnPara32)
{
	*pFeatureReturnPara32 = 0;
}

/*************************************************************************
//此函数变量未定义,请根据编译定义数据变量.
//prview 1280*960 
//16的整数倍 ; n*16*80/1280
//16的整数倍 ; n*16*60/960
//touch_x  为对应preview的横坐标[0-1280]
//touch_y  为对应preview的纵坐标[0-960]

*************************************************************************/ 
static UINT32 OV5645_FOCUS_Move_to(UINT32 a_u2MovePosition)//??how many bits for ov3640??
{
}
/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Get_AF_Status
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/                        
static void OV5645_FOCUS_OVT_AFC_Get_AF_Status(UINT32 *pFeatureReturnPara32)
{
	UINT32 state_3028=0;
	UINT32 state_3029=0;
	*pFeatureReturnPara32 = SENSOR_AF_IDLE;
	state_3028 = OV5645MIPIYUV_read_cmos_sensor(0x3028);
	state_3029 = OV5645MIPIYUV_read_cmos_sensor(0x3029);
	mDELAY(1);
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645_FOCUS_OVT_AFC_Get_AF_Status function:state_3028=%d,state_3029=%d\n",state_3028,state_3029);
	if (state_3028==0)
	{
		*pFeatureReturnPara32 = SENSOR_AF_ERROR;	
	}
	else if (state_3028==1)
	{
		switch (state_3029)
		{
			case 0x70:
				*pFeatureReturnPara32 = SENSOR_AF_IDLE;
				break;
			case 0x00:
				*pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
				break;
			case 0x10:
				*pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
				break;
			case 0x20:
				*pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
				break;
			default:
				*pFeatureReturnPara32 = SENSOR_AF_SCENE_DETECTING; 
				break;
		}								   
	}	 
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645_FOCUS_OVT_AFC_Get_AF_Status function:state_3028=%d,state_3029=%d\n",state_3028,state_3029);
}

static void OV5645_FOCUS_OVT_AFC_Cancel_Focus()
{
    //OV5645MIPI_write_cmos_sensor(0x3023,0x01);
    //OV5645MIPI_write_cmos_sensor(0x3022,0x08);    
}

/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Cancel_Focus
* DESCRIPTION
*   cancel af 
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/     
static void OV5640_FOCUS_OVT_AFC_Cancel_Focus()
{
    OV5645MIPI_write_cmos_sensor(0x3023,0x01);
    OV5645MIPI_write_cmos_sensor(0x3022,0x08);    
}

/*************************************************************************
* FUNCTION
*   OV5645WBcalibattion
* DESCRIPTION
*   color calibration
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void OV5645WBcalibattion(kal_uint32 color_r_gain,kal_uint32 color_b_gain)
{
		kal_uint32 color_r_gain_w = 0;
		kal_uint32 color_b_gain_w = 0;
		OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645WBcalibattion function:\n ");
		kal_uint8 temp = OV5645MIPIYUV_read_cmos_sensor(0x350b); 
		
		if(temp>=0xb0)
		{	
			color_r_gain_w=color_r_gain*97/100;																																														
			color_b_gain_w=color_b_gain*99/100;  
		}
		else if (temp>=0x70)
		{
			color_r_gain_w=color_r_gain *97/100;																																														
			color_b_gain_w=color_b_gain*99/100;
		}
		else if (temp>=0x30)
		{
			color_r_gain_w=color_r_gain*98/100;																																														
			color_b_gain_w=color_b_gain*99/100;
		}
		else
		{
			color_r_gain_w=color_r_gain*98/100;																																														
			color_b_gain_w=color_b_gain*99/100; 
		}																																																																						
		OV5645MIPI_write_cmos_sensor(0x3400,(color_r_gain_w & 0xff00)>>8);																																														
		OV5645MIPI_write_cmos_sensor(0x3401,color_r_gain_w & 0xff); 			
		OV5645MIPI_write_cmos_sensor(0x3404,(color_b_gain_w & 0xff00)>>8);																																														
		OV5645MIPI_write_cmos_sensor(0x3405,color_b_gain_w & 0xff); 
		OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645WBcalibattion function:\n ");
}	
/*************************************************************************
* FUNCTION
*	OV5645MIPIOpen
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
UINT32 OV5645MIPIOpen(void)
{
	volatile signed int i;
	kal_uint16 sensor_id = 0;
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIOpen function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3103,0x11);
	OV5645MIPI_write_cmos_sensor(0x3008,0x82);
    mDELAY(10);
	for(i=0;i<3;i++)
	{
		sensor_id = (OV5645MIPIYUV_read_cmos_sensor(0x300A) << 8) | OV5645MIPIYUV_read_cmos_sensor(0x300B);
		OV5645MIPISENSORDB("OV5645MIPI READ ID :%x",sensor_id);
		if(sensor_id != OV5645MIPI_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	OV5645MIPIinitalvariable();
	OV5645MIPIInitialSetting();
	OV5645_FOCUS_OVT_AFC_Init();
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIOpen function:\n ");
	
#if 1
		if(false == AF_Power)
		{
			OV5645MIPISENSORDB("[OV5645Sensor] AF Power on.\n");
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,"OV5645_AF"))
			{
				printk("[CAMERA SENSOR AF] Fail to enable analog power\n");
				return -EIO;
			}  
			AF_Power = true;
		}
		else
		{
			OV5645MIPISENSORDB("[OV5645Sensor] AF Power has already on.\n");
		}
		mdelay(8);	
#endif
	return ERROR_NONE;
}	/* OV5645MIPIOpen() */

/*************************************************************************
* FUNCTION
*	OV5645MIPIClose
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
UINT32 OV5645MIPIClose(void)
{
  //CISModulePowerOn(FALSE);
  #if 1
		if(true == AF_Power)
		{
			OV5645MIPISENSORDB("[OV5645Sensor] AF Power down.\n");
		  printk("[OV5645Sensor] AF Power down.\n");
		  if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,"OV5645_AF"))
			{
				printk("[CAMERA SENSOR AF] Fail to power down analog power\n");
				return -EIO;
			}  
			AF_Power = false;
		}
		else
		{
			OV5645MIPISENSORDB("[OV5645Sensor] AF Power has already down.\n");
		}
		mdelay(8);
	#endif
	
	return ERROR_NONE;
}	/* OV5645MIPIClose() */
/*************************************************************************
* FUNCTION
*	OV5645MIPIPreview
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
UINT32 OV5645MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	//kal_uint8 iTemp, temp_AE_reg, temp_AWB_reg;
	//kal_uint16 iDummyPixels = 0, iDummyLines = 0, iStartX = 1, iStartY = 1;
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIPreview function:\n ");
	OV5645MIPI_set_AE_mode(KAL_TRUE);
	OV5645MIPI_set_AWB_mode(KAL_TRUE);
	switch(CurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			   OV5645MIPIFullSizeCaptureSetting();			
			   break;
		default:
			   OV5645MIPIPreviewSetting_SVGA();
			   break;
	}
	mDELAY(30);	
	OV5645MIPI_night_mode(OV5645MIPISensor.NightMode);
	OV5645MIPISetHVMirror(sensor_config_data->SensorImageMirror,SENSOR_MODE_PREVIEW);
	//OV5645_FOCUS_OVT_AFC_Constant_Focus();
	mDELAY(50);	
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIPreview function:\n ");	
	return ERROR_NONE ;
	
}	/* OV5645MIPIPreview() */

UINT32 OV5645MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 shutter = 0;	
	kal_uint32 extshutter = 0;
	kal_uint32 color_r_gain = 0;
	kal_uint32 color_b_gain = 0;
	kal_uint32 readgain=0;
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPICapture function:\n ");
	if(SENSOR_MODE_PREVIEW == OV5645MIPISensor.SensorMode )
	{		
	shutter=OV5645MIPIReadShutter();
	extshutter=OV5645MIPIReadExtraShutter();
	readgain=OV5645MIPIReadSensorGain();
	spin_lock(&ov5645mipi_drv_lock);
	OV5645MIPISensor.PreviewShutter=shutter;
	OV5645MIPISensor.PreviewExtraShutter=extshutter;	
	OV5645MIPISensor.SensorGain=readgain;
	spin_unlock(&ov5645mipi_drv_lock);
	//OV5640_FOCUS_OVT_AFC_Pause_Focus();
	OV5645MIPI_set_AE_mode(KAL_FALSE);
	OV5645MIPI_set_AWB_mode(KAL_FALSE);
	color_r_gain=((OV5645MIPIYUV_read_cmos_sensor(0x3401)&0xFF)+((OV5645MIPIYUV_read_cmos_sensor(0x3400)&0xFF)*256));  
	color_b_gain=((OV5645MIPIYUV_read_cmos_sensor(0x3405)&0xFF)+((OV5645MIPIYUV_read_cmos_sensor(0x3404)&0xFF)*256)); 
	OV5645MIPIFullSizeCaptureSetting();
    OV5645WBcalibattion(color_r_gain,color_b_gain);      
	shutter = shutter*2;
	OV5645MIPISetHVMirror(sensor_config_data->SensorImageMirror,SENSOR_MODE_CAPTURE);
	//OV5645MIPIWriteSensorGain(OV5645MIPISensor.SensorGain);	
	OV5645MIPIWriteShutter(shutter);
	mDELAY(100);
	}
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPICapture function:\n ");
	return ERROR_NONE; 
}/* OV5645MIPICapture() */

UINT32 OV5645MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIGetResolution function:\n ");
	pSensorResolution->SensorPreviewWidth= OV5645MIPI_IMAGE_SENSOR_SVGA_WIDTH-4;
	pSensorResolution->SensorPreviewHeight= OV5645MIPI_IMAGE_SENSOR_SVGA_HEIGHT-3;
	pSensorResolution->SensorFullWidth= OV5645MIPI_IMAGE_SENSOR_QSXGA_WITDH-4; 
	pSensorResolution->SensorFullHeight= OV5645MIPI_IMAGE_SENSOR_QSXGA_HEIGHT-3;
	pSensorResolution->SensorVideoWidth= OV5645MIPI_IMAGE_SENSOR_SVGA_WIDTH-4; 
	pSensorResolution->SensorVideoHeight= OV5645MIPI_IMAGE_SENSOR_SVGA_HEIGHT-3;
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIGetResolution function:\n ");
	return ERROR_NONE;
}	/* OV5645MIPIGetResolution() */

UINT32 OV5645MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,MSDK_SENSOR_INFO_STRUCT *pSensorInfo,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIGetInfo function:\n ");
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorPreviewResolutionX=OV5645MIPI_IMAGE_SENSOR_QSXGA_WITDH ;
			pSensorInfo->SensorPreviewResolutionY=OV5645MIPI_IMAGE_SENSOR_QSXGA_HEIGHT ;	
			break;
		default:
			pSensorInfo->SensorPreviewResolutionX=OV5645MIPI_IMAGE_SENSOR_SVGA_WIDTH ;
			pSensorInfo->SensorPreviewResolutionY=OV5645MIPI_IMAGE_SENSOR_SVGA_HEIGHT ;	
			break;
	}
	pSensorInfo->SensorFullResolutionX= OV5645MIPI_IMAGE_SENSOR_QSXGA_WITDH;
	pSensorInfo->SensorFullResolutionY= OV5645MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=10;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;  
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
	pSensorInfo->CaptureDelayFrame = 2;
	pSensorInfo->PreviewDelayFrame = 2; 
	pSensorInfo->VideoDelayFrame = 4; 		
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->YUVAwbDelayFrame = 2;
	pSensorInfo->YUVEffectDelayFrame= 2; 
  pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;   		
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = OV5645MIPI_PV_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = OV5645MIPI_PV_GRAB_START_Y;   
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;  	
			pSensorInfo->SensorPacketECCOrder = 1;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:

			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = OV5645MIPI_FULL_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = OV5645MIPI_FULL_GRAB_START_Y;             
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 7; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;
			pSensorInfo->SensorPacketECCOrder = 1;
			break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = OV5645MIPI_PV_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = OV5645MIPI_PV_GRAB_START_Y; 			
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;
			pSensorInfo->SensorHightSampling = 0;	
			pSensorInfo->SensorPacketECCOrder = 1;
		  break;
	}
	memcpy(pSensorConfigData, &OV5645MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));	
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIGetInfo function:\n ");	
	return ERROR_NONE;
}	/* OV5645MIPIGetInfo() */

UINT32 OV5645MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	  OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIControl function:\n ");
	  spin_lock(&ov5645mipi_drv_lock);
	  CurrentScenarioId = ScenarioId;
	  spin_unlock(&ov5645mipi_drv_lock);
	  switch (ScenarioId)
	  {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 OV5645MIPIPreview(pImageWindow, pSensorConfigData);
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			 OV5645MIPICapture(pImageWindow, pSensorConfigData);
	  	     break;
		default:
			return ERROR_INVALID_SCENARIO_ID;
	}
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIControl function:\n ");
	return ERROR_NONE;
}	/* OV5645MIPIControl() */

/* [TC] YUV sensor */	

BOOL OV5645MIPI_set_param_wb(UINT16 para)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPI_set_param_wb function:\n ");
	switch (para)
    {
        case AWB_MODE_OFF:
							spin_lock(&ov5645mipi_drv_lock);
							OV5645MIPI_AWB_ENABLE = KAL_FALSE; 
							spin_unlock(&ov5645mipi_drv_lock);
							OV5645MIPI_set_AWB_mode(OV5645MIPI_AWB_ENABLE);
							break;                    
        case AWB_MODE_AUTO:
							spin_lock(&ov5645mipi_drv_lock);
							OV5645MIPI_AWB_ENABLE = KAL_TRUE; 
							spin_unlock(&ov5645mipi_drv_lock);
							OV5645MIPI_set_AWB_mode(OV5645MIPI_AWB_ENABLE);
							break;
        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
							OV5645MIPI_set_AWB_mode(KAL_FALSE);         	                
							OV5645MIPI_write_cmos_sensor(0x3400,0x06); 
							OV5645MIPI_write_cmos_sensor(0x3401,0x30); 
							OV5645MIPI_write_cmos_sensor(0x3402,0x04); 
							OV5645MIPI_write_cmos_sensor(0x3403,0x00); 
							OV5645MIPI_write_cmos_sensor(0x3404,0x04); 
							OV5645MIPI_write_cmos_sensor(0x3405,0x30);                          
              break;
        case AWB_MODE_DAYLIGHT: //sunny
							OV5645MIPI_set_AWB_mode(KAL_FALSE);                           
							OV5645MIPI_write_cmos_sensor(0x3400,0x06); 
							OV5645MIPI_write_cmos_sensor(0x3401,0x10); 
							OV5645MIPI_write_cmos_sensor(0x3402,0x04); 
							OV5645MIPI_write_cmos_sensor(0x3403,0x00); 
							OV5645MIPI_write_cmos_sensor(0x3404,0x04); 
							OV5645MIPI_write_cmos_sensor(0x3405,0x48);                           
							break;
        case AWB_MODE_INCANDESCENT: //office
							OV5645MIPI_set_AWB_mode(KAL_FALSE);                           
							OV5645MIPI_write_cmos_sensor(0x3400,0x04); 
							OV5645MIPI_write_cmos_sensor(0x3401,0xe0); 
							OV5645MIPI_write_cmos_sensor(0x3402,0x04); 
							OV5645MIPI_write_cmos_sensor(0x3403,0x00); 
							OV5645MIPI_write_cmos_sensor(0x3404,0x05); 
							OV5645MIPI_write_cmos_sensor(0x3405,0xa0);                           
							break; 
		case AWB_MODE_TUNGSTEN:
							OV5645MIPI_set_AWB_mode(KAL_FALSE);                         
							OV5645MIPI_write_cmos_sensor(0x3400,0x05); 
							OV5645MIPI_write_cmos_sensor(0x3401,0x48); 
							OV5645MIPI_write_cmos_sensor(0x3402,0x04); 
							OV5645MIPI_write_cmos_sensor(0x3403,0x00); 
							OV5645MIPI_write_cmos_sensor(0x3404,0x05); 
							OV5645MIPI_write_cmos_sensor(0x3405,0xe0); 
							break;
        case AWB_MODE_FLUORESCENT:
							OV5645MIPI_set_AWB_mode(KAL_FALSE);                           
							OV5645MIPI_write_cmos_sensor(0x3400,0x04); 
							OV5645MIPI_write_cmos_sensor(0x3401,0x00); 
							OV5645MIPI_write_cmos_sensor(0x3402,0x04); 
							OV5645MIPI_write_cmos_sensor(0x3403,0x00); 
							OV5645MIPI_write_cmos_sensor(0x3404,0x06); 
							OV5645MIPI_write_cmos_sensor(0x3405,0x50);                           
							break;
        default:
			return FALSE;
    }
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPI_set_param_wb function:\n ");
       return TRUE;
} /* OV5645MIPI_set_param_wb */

BOOL OV5645MIPI_set_param_effect(UINT16 para)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPI_set_param_effect function:\n ");
	switch (para)
    {
        case MEFFECT_OFF:                      
						OV5645MIPI_write_cmos_sensor(0x5001,0xa3); 
						OV5645MIPI_write_cmos_sensor(0x5580,0x06); 
						OV5645MIPI_write_cmos_sensor(0x5583,0x38); //40
						OV5645MIPI_write_cmos_sensor(0x5584,0x28); //30
	         break;
        case MEFFECT_SEPIA:                      
			 OV5645MIPI_write_cmos_sensor(0x5001,0xa3);
             OV5645MIPI_write_cmos_sensor(0x5583,0x40); 
             OV5645MIPI_write_cmos_sensor(0x5584,0xa0); 
             OV5645MIPI_write_cmos_sensor(0x5580,0x1e);
			 break;
        case MEFFECT_NEGATIVE:
			 OV5645MIPI_write_cmos_sensor(0x5001,0xa3);
			 OV5645MIPI_write_cmos_sensor(0x5580,0x46); 										                                                            
			 break;
        case MEFFECT_SEPIAGREEN:                      			 
			 OV5645MIPI_write_cmos_sensor(0x5001,0xa3);
			 OV5645MIPI_write_cmos_sensor(0x5583,0x60); 
			 OV5645MIPI_write_cmos_sensor(0x5584,0x60);                      
		     OV5645MIPI_write_cmos_sensor(0x5580,0x1e);		                      
			 break;
        case MEFFECT_SEPIABLUE:
			 OV5645MIPI_write_cmos_sensor(0x5001,0xa3);
             OV5645MIPI_write_cmos_sensor(0x5583,0xa0); 
             OV5645MIPI_write_cmos_sensor(0x5584,0x40);                      
             OV5645MIPI_write_cmos_sensor(0x5580,0x1e);                      
             break;
		case MEFFECT_MONO: //B&W
			 OV5645MIPI_write_cmos_sensor(0x5001,0xa3);
        	 OV5645MIPI_write_cmos_sensor(0x5583,0x80); 
        	 OV5645MIPI_write_cmos_sensor(0x5584,0x80); 
        	 OV5645MIPI_write_cmos_sensor(0x5580,0x1e); 
        	 break;
        default:
             return KAL_FALSE;
    }    
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPI_set_param_effect function:\n ");
    return KAL_FALSE;
} /* OV5645MIPI_set_param_effect */

BOOL OV5645MIPI_set_param_banding(UINT16 para)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPI_set_param_banding function:\n ");
	switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
						spin_lock(&ov5645mipi_drv_lock);
						OV5645MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;
						spin_unlock(&ov5645mipi_drv_lock);
						OV5645MIPI_write_cmos_sensor(0x3c00,0x04);
						OV5645MIPI_write_cmos_sensor(0x3c01,0x80);
            break;
        case AE_FLICKER_MODE_60HZ:			
						spin_lock(&ov5645mipi_drv_lock);
						OV5645MIPI_Banding_setting = AE_FLICKER_MODE_60HZ;
						spin_unlock(&ov5645mipi_drv_lock);
						OV5645MIPI_write_cmos_sensor(0x3c00,0x00);
						OV5645MIPI_write_cmos_sensor(0x3c01,0x80);
            break;
            default:
                 return FALSE;
    }
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPI_set_param_banding function:\n ");
        return TRUE;
} /* OV5645MIPI_set_param_banding */

BOOL OV5645MIPI_set_param_exposure(UINT16 para)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPI_set_param_exposure function:\n ");
	switch (para)
    {	
       case AE_EV_COMP_20:	                   
				OV5645MIPI_write_cmos_sensor(0x3a0f, 0x50);//	; AEC in H
				OV5645MIPI_write_cmos_sensor(0x3a10, 0x48);//	; AEC in L
				OV5645MIPI_write_cmos_sensor(0x3a1b, 0x50);//	; AEC out H
				OV5645MIPI_write_cmos_sensor(0x3a1e, 0x46);//	; AEC out L
				OV5645MIPI_write_cmos_sensor(0x3a11, 0xA0);//	; control zone H
				OV5645MIPI_write_cmos_sensor(0x3a1f, 0x1C);//	; control zone L   
			  break;
       case AE_EV_COMP_10:	                   
				OV5645MIPI_write_cmos_sensor(0x3a0f, 0x40);//	; AEC in H
				OV5645MIPI_write_cmos_sensor(0x3a10, 0x38);//	; AEC in L
				OV5645MIPI_write_cmos_sensor(0x3a1b, 0x40);//	; AEC out H
				OV5645MIPI_write_cmos_sensor(0x3a1e, 0x38);//	; AEC out L
				OV5645MIPI_write_cmos_sensor(0x3a11, 0x80);//	; control zone H
				OV5645MIPI_write_cmos_sensor(0x3a1f, 0x18);//	; control zone L   
			  break;
		case AE_EV_COMP_00:
				OV5645MIPI_write_cmos_sensor(0x3a0f, 0x30);//	; AEC in H
				OV5645MIPI_write_cmos_sensor(0x3a10, 0x28);//	; AEC in L
				OV5645MIPI_write_cmos_sensor(0x3a1b, 0x30);//	; AEC out H
				OV5645MIPI_write_cmos_sensor(0x3a1e, 0x26);//	; AEC out L
				OV5645MIPI_write_cmos_sensor(0x3a11, 0x60);//	; control zone H
				OV5645MIPI_write_cmos_sensor(0x3a1f, 0x14);//	; control zone L   
			  break;
   		 case AE_EV_COMP_n10:
				OV5645MIPI_write_cmos_sensor(0x3a0f, 0x20);//	; AEC in H
				OV5645MIPI_write_cmos_sensor(0x3a10, 0x18);//	; AEC in L
				OV5645MIPI_write_cmos_sensor(0x3a1b, 0x20);//	; AEC out H
				OV5645MIPI_write_cmos_sensor(0x3a1e, 0x18);//	; AEC out L
				OV5645MIPI_write_cmos_sensor(0x3a11, 0x40);//	; control zone H
				OV5645MIPI_write_cmos_sensor(0x3a1f, 0x10);//	; control zone L   
			  break;	
	     case AE_EV_COMP_n20:
				OV5645MIPI_write_cmos_sensor(0x3a0f, 0x10);//	; AEC in H
				OV5645MIPI_write_cmos_sensor(0x3a10, 0x08);//	; AEC in L
				OV5645MIPI_write_cmos_sensor(0x3a1b, 0x10);//	; AEC out H
				OV5645MIPI_write_cmos_sensor(0x3a1e, 0x0A);//	; AEC out L
				OV5645MIPI_write_cmos_sensor(0x3a11, 0x20);//	; control zone H
				OV5645MIPI_write_cmos_sensor(0x3a1f, 0x0C);//	; control zone L   
			 break;
                default:
                         return FALSE;
    }
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPI_set_param_exposure function:\n ");
    return TRUE;
} /* OV5645MIPI_set_param_exposure */
#if 0//afc
BOOL OV5645MIPI_set_param_afmode(UINT16 para)
{
    switch (para)
    {
			case AF_MODE_AFS:
				OV5640_FOCUS_OVT_AFC_Single_Focus();
			break;
			case AF_MODE_AFC:
				OV5640_FOCUS_OVT_AFC_Constant_Focus();
			break;            
      	default:
      	return FALSE;
    }
        return TRUE;
} /* OV5645MIPI_set_param_banding */
#endif
UINT32 OV5645MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	OV5645MIPISENSORDB("OV5645MIPIYUVSensorSetting:iCmd=%d,iPara=%d, %d \n",iCmd, iPara);
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIYUVSensorSetting function:\n ");
	switch (iCmd) {
		case FID_SCENE_MODE:
				OV5645MIPISENSORDB("Night Mode:%d\n", iPara); 
	    	if (iPara == SCENE_MODE_OFF)
	    	{
	      		  OV5645MIPI_night_mode(KAL_FALSE); 
	    	}
	    	else if (iPara == SCENE_MODE_NIGHTSCENE)
	    	{
          		OV5645MIPI_night_mode(KAL_TRUE); 
	    	}	    
	    	break; 	    
		case FID_AWB_MODE:
				OV5645MIPI_set_param_wb(iPara);
			  break;
		case FID_COLOR_EFFECT:	    	    
				OV5645MIPI_set_param_effect(iPara);
		 	  break;
		case FID_AE_EV:   
				OV5645MIPI_set_param_exposure(iPara);
		    break;
		case FID_AE_FLICKER:    	    	    
				OV5645MIPI_set_param_banding(iPara);
		 	  break;
		case FID_AE_SCENE_MODE: 
				if (iPara == AE_MODE_OFF) 
				{
					spin_lock(&ov5645mipi_drv_lock);
		 			OV5645MIPI_AE_ENABLE = KAL_FALSE; 
					spin_unlock(&ov5645mipi_drv_lock);
        }
        else 
        {
					spin_lock(&ov5645mipi_drv_lock);
		 			OV5645MIPI_AE_ENABLE = KAL_TRUE; 
					spin_unlock(&ov5645mipi_drv_lock);
	     	}
				OV5645MIPI_set_AE_mode(OV5645MIPI_AE_ENABLE);
        break; 
    case FID_ZOOM_FACTOR:
   		    OV5645MIPISENSORDB("FID_ZOOM_FACTOR:%d\n", iPara); 	    
					spin_lock(&ov5645mipi_drv_lock);
	        zoom_factor = iPara; 
					spin_unlock(&ov5645mipi_drv_lock);
            break; 
#if 0 //afc
		case FID_AF_MODE:
	    	 OV5645MIPI_set_param_afmode(iPara);
					break;     
#endif            
	  default:
		 	      break;
	}
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIYUVSensorSetting function:\n ");
	  return TRUE;
}   /* OV5645MIPIYUVSensorSetting */

UINT32 OV5645MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{
		kal_uint8 mirror= 0, flip=0;
		flip = OV5645MIPIYUV_read_cmos_sensor(0x3820);
		mirror=OV5645MIPIYUV_read_cmos_sensor(0x3821);

	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIYUVSetVideoMode function:\n ");
	if (u2FrameRate == 30)
	{
		//;OV5645MIPI 1280x960,30fps
		//56Mhz, 224Mbps/Lane, 2Lane.
		OV5645MIPISENSORDB("[OV5645MIPI]OV5645MIPIYUVSetVideoMode enter u2FrameRate == 30 setting  :\n ");	
		OV5645MIPI_write_cmos_sensor(0x300e, 0x45);//	; MIPI 2 lane
		OV5645MIPI_write_cmos_sensor(0x3034, 0x18); // PLL, MIPI 8-bit mode
		OV5645MIPI_write_cmos_sensor(0x3035, 0x21); // PLL
		OV5645MIPI_write_cmos_sensor(0x3036, 0x70); // PLL
		OV5645MIPI_write_cmos_sensor(0x3037, 0x13); // PLL
		OV5645MIPI_write_cmos_sensor(0x3108, 0x01); // PLL
		OV5645MIPI_write_cmos_sensor(0x3824, 0x01); // PLL
		OV5645MIPI_write_cmos_sensor(0x460c, 0x20); // PLL
		OV5645MIPI_write_cmos_sensor(0x3618, 0x00);//
		OV5645MIPI_write_cmos_sensor(0x3600, 0x09);//
		OV5645MIPI_write_cmos_sensor(0x3601, 0x43);//
		OV5645MIPI_write_cmos_sensor(0x3708, 0x66);//
		OV5645MIPI_write_cmos_sensor(0x3709, 0x12);//
		OV5645MIPI_write_cmos_sensor(0x370c, 0xc3);//
		OV5645MIPI_write_cmos_sensor(0x3800, 0x00); // HS = 0
		OV5645MIPI_write_cmos_sensor(0x3801, 0x00); // HS
		OV5645MIPI_write_cmos_sensor(0x3802, 0x00); // VS = 250
		OV5645MIPI_write_cmos_sensor(0x3803, 0x06); // VS
		OV5645MIPI_write_cmos_sensor(0x3804, 0x0a); // HW = 2623
		OV5645MIPI_write_cmos_sensor(0x3805, 0x3f);//	; HW
		OV5645MIPI_write_cmos_sensor(0x3806, 0x07);//	; VH = 
		OV5645MIPI_write_cmos_sensor(0x3807, 0x9d);//	; VH
		OV5645MIPI_write_cmos_sensor(0x3808, 0x05);//	; DVPHO = 1280
		OV5645MIPI_write_cmos_sensor(0x3809, 0x00);//	; DVPHO
		OV5645MIPI_write_cmos_sensor(0x380a, 0x03);//	; DVPVO = 960
		OV5645MIPI_write_cmos_sensor(0x380b, 0xc0);//	; DVPVO
		OV5645MIPI_write_cmos_sensor(0x380c, 0x07);//	; HTS = 2160
		OV5645MIPI_write_cmos_sensor(0x380d, 0x68);//	; HTS
		OV5645MIPI_write_cmos_sensor(0x380e, 0x03);//	; VTS = 740
		OV5645MIPI_write_cmos_sensor(0x380f, 0xd8);//	; VTS
		OV5645MIPI_write_cmos_sensor(0x3810, 0x00); // H OFF = 16
		OV5645MIPI_write_cmos_sensor(0x3811, 0x10); // H OFF
		OV5645MIPI_write_cmos_sensor(0x3812, 0x00); // V OFF = 4
		OV5645MIPI_write_cmos_sensor(0x3813, 0x06);//	; V OFF
		OV5645MIPI_write_cmos_sensor(0x3814, 0x31);//	; X INC
		OV5645MIPI_write_cmos_sensor(0x3815, 0x31);//	; Y INC
		OV5645MIPI_write_cmos_sensor(0x3820, 0x41);//	; flip off, V bin on
		OV5645MIPI_write_cmos_sensor(0x3821, 0x01);//	; mirror on, H bin on
		OV5645MIPI_write_cmos_sensor(0x3a00, 0x38);//	; ae mode	
		OV5645MIPI_write_cmos_sensor(0x3a02, 0x03);//	; max exp 60 = 740
		OV5645MIPI_write_cmos_sensor(0x3a03, 0xd8);//	; max exp 60
		OV5645MIPI_write_cmos_sensor(0x3a08, 0x01);//	; B50 = 222
		OV5645MIPI_write_cmos_sensor(0x3a09, 0x27);//	; B50
		OV5645MIPI_write_cmos_sensor(0x3a0a, 0x00);//	; B60 = 185
		OV5645MIPI_write_cmos_sensor(0x3a0b, 0xf6);//	; B60
		OV5645MIPI_write_cmos_sensor(0x3a0e, 0x03);//	; max 50
		OV5645MIPI_write_cmos_sensor(0x3a0d, 0x04);//	; max 60
		OV5645MIPI_write_cmos_sensor(0x3a14, 0x03);//	; max exp 50 = 740
		OV5645MIPI_write_cmos_sensor(0x3a15, 0xd8);//	; max exp 50
		OV5645MIPI_write_cmos_sensor(0x3c07, 0x07);//	; 50/60 auto detect
		OV5645MIPI_write_cmos_sensor(0x3c08, 0x01);//	; 50/60 auto detect
		OV5645MIPI_write_cmos_sensor(0x3c09, 0xc2);//	; 50/60 auto detect
		OV5645MIPI_write_cmos_sensor(0x4004, 0x02);//	; BLC line number
		OV5645MIPI_write_cmos_sensor(0x4005, 0x18);//	; BLC triggered by gain change
		OV5645MIPI_write_cmos_sensor(0x4837, 0x11); // MIPI global timing 16           
		OV5645MIPI_write_cmos_sensor(0x503d, 0x00);//
		OV5645MIPI_write_cmos_sensor(0x5000, 0xa7);//
		OV5645MIPI_write_cmos_sensor(0x5001, 0xa3);//
		OV5645MIPI_write_cmos_sensor(0x5002, 0x80);//
		OV5645MIPI_write_cmos_sensor(0x5003, 0x08);//
		OV5645MIPI_write_cmos_sensor(0x3032, 0x00);//
		OV5645MIPI_write_cmos_sensor(0x4000, 0x89);//
		OV5645MIPI_write_cmos_sensor(0x350c, 0x00);//
		OV5645MIPI_write_cmos_sensor(0x350d, 0x00);//
		OV5645MIPISENSORDB("[OV5645MIPI]OV5645MIPIYUVSetVideoMode exit u2FrameRate == 30 setting  :\n ");
		}
    else if (u2FrameRate == 15)   
	{
		//;OV5645MIPI 1280x960,15fps
		//28Mhz, 112Mbps/Lane, 2Lane.
		OV5645MIPISENSORDB("[OV5645MIPI]OV5645MIPIYUVSetVideoMode enter u2FrameRate == 15 setting  :\n ");	
		OV5645MIPI_write_cmos_sensor(0x300e, 0x45);//	; MIPI 2 lane
		OV5645MIPI_write_cmos_sensor(0x3034, 0x18); // PLL, MIPI 8-bit mode
		OV5645MIPI_write_cmos_sensor(0x3035, 0x21); // PLL
		OV5645MIPI_write_cmos_sensor(0x3036, 0x38); // PLL
		OV5645MIPI_write_cmos_sensor(0x3037, 0x13); // PLL
		OV5645MIPI_write_cmos_sensor(0x3108, 0x01); // PLL
		OV5645MIPI_write_cmos_sensor(0x3824, 0x01); // PLL
		OV5645MIPI_write_cmos_sensor(0x460c, 0x20); // PLL
		OV5645MIPI_write_cmos_sensor(0x3618, 0x00);//
		OV5645MIPI_write_cmos_sensor(0x3600, 0x09);//
		OV5645MIPI_write_cmos_sensor(0x3601, 0x43);//
		OV5645MIPI_write_cmos_sensor(0x3708, 0x66);//
		OV5645MIPI_write_cmos_sensor(0x3709, 0x12);//
		OV5645MIPI_write_cmos_sensor(0x370c, 0xc3);//
		OV5645MIPI_write_cmos_sensor(0x3800, 0x00); // HS = 0
		OV5645MIPI_write_cmos_sensor(0x3801, 0x00); // HS
		OV5645MIPI_write_cmos_sensor(0x3802, 0x00); // VS = 250
		OV5645MIPI_write_cmos_sensor(0x3803, 0x06); // VS
		OV5645MIPI_write_cmos_sensor(0x3804, 0x0a); // HW = 2623
		OV5645MIPI_write_cmos_sensor(0x3805, 0x3f);//	; HW
		OV5645MIPI_write_cmos_sensor(0x3806, 0x07);//	; VH = 
		OV5645MIPI_write_cmos_sensor(0x3807, 0x9d);//	; VH
		OV5645MIPI_write_cmos_sensor(0x3808, 0x05);//	; DVPHO = 1280
		OV5645MIPI_write_cmos_sensor(0x3809, 0x00);//	; DVPHO
		OV5645MIPI_write_cmos_sensor(0x380a, 0x03);//	; DVPVO = 960
		OV5645MIPI_write_cmos_sensor(0x380b, 0xc0);//	; DVPVO
		OV5645MIPI_write_cmos_sensor(0x380c, 0x07);//	; HTS = 2160
		OV5645MIPI_write_cmos_sensor(0x380d, 0x68);//	; HTS
		OV5645MIPI_write_cmos_sensor(0x380e, 0x03);//	; VTS = 740
		OV5645MIPI_write_cmos_sensor(0x380f, 0xd8);//	; VTS
		OV5645MIPI_write_cmos_sensor(0x3810, 0x00); // H OFF = 16
		OV5645MIPI_write_cmos_sensor(0x3811, 0x10); // H OFF
		OV5645MIPI_write_cmos_sensor(0x3812, 0x00); // V OFF = 4
		OV5645MIPI_write_cmos_sensor(0x3813, 0x06);//	; V OFF
		OV5645MIPI_write_cmos_sensor(0x3814, 0x31);//	; X INC
		OV5645MIPI_write_cmos_sensor(0x3815, 0x31);//	; Y INC
		OV5645MIPI_write_cmos_sensor(0x3820, 0x41);//	; flip off, V bin on
		OV5645MIPI_write_cmos_sensor(0x3821, 0x01);//	; mirror on, H bin on
		OV5645MIPI_write_cmos_sensor(0x3a00, 0x38);//	; ae mode	
		OV5645MIPI_write_cmos_sensor(0x3a02, 0x03);//	; max exp 60 = 740
		OV5645MIPI_write_cmos_sensor(0x3a03, 0xd8);//	; max exp 60
		OV5645MIPI_write_cmos_sensor(0x3a08, 0x00);//	; B50 = 222
		OV5645MIPI_write_cmos_sensor(0x3a09, 0x94);//	; B50
		OV5645MIPI_write_cmos_sensor(0x3a0a, 0x00);//	; B60 = 185
		OV5645MIPI_write_cmos_sensor(0x3a0b, 0x7b);//	; B60
		OV5645MIPI_write_cmos_sensor(0x3a0e, 0x06);//	; max 50
		OV5645MIPI_write_cmos_sensor(0x3a0d, 0x07);//	; max 60
		OV5645MIPI_write_cmos_sensor(0x3a14, 0x03);//	; max exp 50 = 740
		OV5645MIPI_write_cmos_sensor(0x3a15, 0xd8);//	; max exp 50
		OV5645MIPI_write_cmos_sensor(0x3c07, 0x08);//	; 50/60 auto detect
		OV5645MIPI_write_cmos_sensor(0x3c08, 0x00);//	; 50/60 auto detect
		OV5645MIPI_write_cmos_sensor(0x3c09, 0x1c);//	; 50/60 auto detect
		OV5645MIPI_write_cmos_sensor(0x4004, 0x02);//	; BLC line number
		OV5645MIPI_write_cmos_sensor(0x4005, 0x1a);//	; BLC triggered by gain change 18
		OV5645MIPI_write_cmos_sensor(0x4837, 0x11); // MIPI global timing 16           
		OV5645MIPI_write_cmos_sensor(0x503d, 0x00);//
		OV5645MIPI_write_cmos_sensor(0x5000, 0xa7);//
		OV5645MIPI_write_cmos_sensor(0x5001, 0xa3);//
		OV5645MIPI_write_cmos_sensor(0x5002, 0x80);//
		OV5645MIPI_write_cmos_sensor(0x5003, 0x08);//
		OV5645MIPI_write_cmos_sensor(0x3032, 0x00);//
		OV5645MIPI_write_cmos_sensor(0x4000, 0x89);//
		OV5645MIPI_write_cmos_sensor(0x350c, 0x00);//
		OV5645MIPI_write_cmos_sensor(0x350d, 0x00);//
		OV5645MIPISENSORDB("[OV5645MIPI]OV5645MIPIYUVSetVideoMode exit u2FrameRate == 15 setting  :\n ");
	}   
    else 
    {
        printk("Wrong frame rate setting \n");
    } 
	//FLIP XY
	OV5645MIPI_write_cmos_sensor(0x3820, flip|0x06);     
	OV5645MIPI_write_cmos_sensor(0x3821, mirror&0xf9);
	OV5645MIPI_write_cmos_sensor(0x4514, 0x88);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIYUVSetVideoMode function:\n ");
    return TRUE; 
}

/**************************/
static void OV5645MIPIGetEvAwbRef(UINT32 pSensorAEAWBRefStruct)
{

	PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
	Ref->SensorAERef.AeRefLV05Shutter=0x170c;
	Ref->SensorAERef.AeRefLV05Gain=0x30;
	Ref->SensorAERef.AeRefLV13Shutter=0x24e;
	Ref->SensorAERef.AeRefLV13Gain=0x10;
	Ref->SensorAwbGainRef.AwbRefD65Rgain=0x610;
	Ref->SensorAwbGainRef.AwbRefD65Bgain=0x448;
	Ref->SensorAwbGainRef.AwbRefCWFRgain=0x4e0;
	Ref->SensorAwbGainRef.AwbRefCWFBgain=0x5a0;
	
}

static void OV5645MIPIGetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct)
{

	PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
	Info->SensorAECur.AeCurShutter=OV5645MIPIReadShutter();
	Info->SensorAECur.AeCurGain=OV5645MIPIReadSensorGain() ;
	Info->SensorAwbGainCur.AwbCurRgain=((OV5645MIPIYUV_read_cmos_sensor(0x3401)&&0xff)+((OV5645MIPIYUV_read_cmos_sensor(0x3400)&&0xff)*256));
	Info->SensorAwbGainCur.AwbCurBgain=((OV5645MIPIYUV_read_cmos_sensor(0x3405)&&0xff)+((OV5645MIPIYUV_read_cmos_sensor(0x3404)&&0xff)*256));

}

// add for  control flash --yyf
#define EXP_FLAG    0x1f
static void ov5645_get_exp_flag(unsigned int *pFeatureData32)
{
	  unsigned int NormBr;

    NormBr = OV5645MIPIYUV_read_cmos_sensor(0x56a1);
    //printk(" ov5645_get_exp_flag NormBr=%x\n",NormBr);
	
    if (NormBr > EXP_FLAG)
    {
       *pFeatureData32 = FALSE;
        return;
    }

    *pFeatureData32 = TRUE;
    return;
}
//end

UINT32 OV5645MIPIMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
	{
		kal_uint32 pclk;
		kal_int16 dummyLine;
		kal_uint16 lineLength,frameHeight;
		OV5645MIPISENSORDB("OV5645MIPIMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
		OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIMaxFramerateByScenario function:\n ");
		switch (scenarioId) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				pclk = 56000000;
				lineLength = OV5645MIPI_IMAGE_SENSOR_SVGA_WIDTH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - OV5645MIPI_IMAGE_SENSOR_SVGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&ov5645mipi_drv_lock);
				OV5645MIPISensor.SensorMode= SENSOR_MODE_PREVIEW;
				OV5645MIPISensor.PreviewDummyLines = dummyLine;
				spin_unlock(&ov5645mipi_drv_lock);
				OV5645MIPISetDummy(OV5645MIPISensor.PreviewDummyPixels, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pclk = 56000000;
				//lineLength = OV5645MIPI_IMAGE_SENSOR_VIDEO_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				//dummyLine = frameHeight - OV5645MIPI_IMAGE_SENSOR_VIDEO_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				//spin_lock(&ov5645mipi_drv_lock);
				//ov8825.sensorMode = SENSOR_MODE_VIDEO;
				//spin_unlock(&ov5645mipi_drv_lock);
				OV5645MIPISetDummy(0, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:			
				pclk = 84000000;
				lineLength = OV5645MIPI_IMAGE_SENSOR_QSXGA_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - OV5645MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&ov5645mipi_drv_lock);
				OV5645MIPISensor.CaptureDummyLines = dummyLine;
				OV5645MIPISensor.SensorMode= SENSOR_MODE_CAPTURE;
				spin_unlock(&ov5645mipi_drv_lock);
				OV5645MIPISetDummy(OV5645MIPISensor.CaptureDummyPixels, dummyLine);			
				break;		
			case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
				break;		
			default:
				break;
		}	
		OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIMaxFramerateByScenario function:\n ");
		return ERROR_NONE;
	}
UINT32 OV5645MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPIGetDefaultFramerateByScenario function:\n ");
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 300;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIGetDefaultFramerateByScenario function:\n ");
	return ERROR_NONE;
}
void OV5645MIPI_get_AEAWB_lock(UINT32 *pAElockRet32, UINT32 *pAWBlockRet32)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPI_get_AEAWB_lock function:\n ");
	*pAElockRet32 =1;
	*pAWBlockRet32=1;
	OV5645MIPISENSORDB("[OV5645MIPI]OV5645MIPI_get_AEAWB_lock,AE=%d,AWB=%d\n",*pAElockRet32,*pAWBlockRet32);
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPI_get_AEAWB_lock function:\n ");
}

void OV5645MIPI_GetDelayInfo(UINT32 delayAddr)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPI_GetDelayInfo function:\n ");
	SENSOR_DELAY_INFO_STRUCT *pDelayInfo=(SENSOR_DELAY_INFO_STRUCT*)delayAddr;
	pDelayInfo->InitDelay=0;
	pDelayInfo->EffectDelay=0;
	pDelayInfo->AwbDelay=0;
	pDelayInfo->AFSwitchDelayFrame=50;
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPI_GetDelayInfo function:\n ");
}
void OV5645MIPI_AutoTestCmd(UINT32 *cmd,UINT32 *para)
{
	OV5645MIPISENSORDB("[OV5645MIPI]enter OV5645MIPI_AutoTestCmd function:\n ");
	switch(*cmd)
	{
		case YUV_AUTOTEST_SET_SHADDING:
			OV5645MIPISENSORDB("YUV_AUTOTEST_SET_SHADDING:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_GAMMA:
			OV5645MIPISENSORDB("YUV_AUTOTEST_SET_GAMMA:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_AE:
			OV5645MIPISENSORDB("YUV_AUTOTEST_SET_AE:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_SHUTTER:
			OV5645MIPISENSORDB("YUV_AUTOTEST_SET_SHUTTER:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_GAIN:
			OV5645MIPISENSORDB("YUV_AUTOTEST_SET_GAIN:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_GET_SHUTTER_RANGE:
			*para=8228;
			break;
		default:
			OV5645MIPISENSORDB("YUV AUTOTEST NOT SUPPORT CMD:%d\n",*cmd);
			break;	
	}
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPI_AutoTestCmd function:\n ");
}
UINT32 OV5645SetTestPatternMode(kal_bool bEnable)
{
	OV5645MIPISENSORDB("[OV5645MIPI_OV5645SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
	if(bEnable)
	{
		OV5645MIPI_write_cmos_sensor(0x503d,0x80);
	}
	else
	{
		OV5645MIPI_write_cmos_sensor(0x503d,0x00);
	}
	return ERROR_NONE;
}
UINT32 OV5645MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	UINT32 Tony_Temp1 = 0;
	UINT32 Tony_Temp2 = 0;
	Tony_Temp1 = pFeaturePara[0];
	Tony_Temp2 = pFeaturePara[1];
	OV5645MIPISENSORDB("[OV5645MIPI][OV5645MIPIFeatureControl]feature id=%d \n",FeatureId);
	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=OV5645MIPI_IMAGE_SENSOR_QSXGA_WITDH;
			*pFeatureReturnPara16=OV5645MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=OV5645MIPI_FULL_PERIOD_PIXEL_NUMS + OV5645MIPISensor.CaptureDummyPixels;
					*pFeatureReturnPara16=OV5645MIPI_FULL_PERIOD_LINE_NUMS + OV5645MIPISensor.CaptureDummyLines;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara16++=OV5645MIPI_PV_PERIOD_PIXEL_NUMS + OV5645MIPISensor.PreviewDummyPixels;
					*pFeatureReturnPara16=OV5645MIPI_PV_PERIOD_LINE_NUMS + OV5645MIPISensor.PreviewDummyLines;
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV5645MIPISensor.CapturePclk * 1000 *100;	 //unit: Hz				
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV5645MIPISensor.PreviewPclk * 1000 *100;	 //unit: Hz
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			OV5645MIPI_night_mode((BOOL) *pFeatureData16);
			break;
		case SENSOR_FEATURE_SET_GAIN:
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			OV5645MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = OV5645MIPIYUV_read_cmos_sensor(pSensorRegData->RegAddr);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &OV5645MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
			break;
		/*case SENSOR_FEATURE_SET_TEST_PATTERN:            
			OV5645SetTestPatternMode((BOOL)*pFeatureData16);            
			break;*/
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			OV5645MIPI_GetSensorID(pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=OV5645_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			OV5645MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		    OV5645MIPIYUVSetVideoMode(*pFeatureData16);
		    break;
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			OV5645MIPIGetEvAwbRef(*pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			OV5645MIPIGetCurAeAwbInfo(*pFeatureData32);			
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			OV5645MIPIMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			OV5645MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,(MUINT32 *)*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			OV5645MIPISENSORDB("[OV5645MIPI]SENSOR_FEATURE_GET_AF_STATUS\n");
			OV5645MIPI_get_AEAWB_lock(*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DELAY_INFO:
			OV5645MIPISENSORDB("SENSOR_FEATURE_GET_DELAY_INFO\n");
			OV5645MIPI_GetDelayInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_AUTOTEST_CMD:
			OV5645MIPISENSORDB("SENSOR_FEATURE_AUTOTEST_CMD\n");
			OV5645MIPI_AutoTestCmd(*pFeatureData32,*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_INITIALIZE_AF:  
			OV5645MIPISENSORDB("[OV5645MIPI]SENSOR_FEATURE_INITIALIZE_AF\n");
			OV5645_FOCUS_OVT_AFC_Init();
             break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            OV5645MIPISENSORDB("[OV5645MIPI]SENSOR_FEATURE_MOVE_FOCUS_LENS\n");
			OV5645_FOCUS_Move_to(*pFeatureData16);
            break;
        case SENSOR_FEATURE_GET_AF_STATUS:
            OV5645MIPISENSORDB("[OV5645MIPI]SENSOR_FEATURE_GET_AF_STATUS\n");
			OV5645_FOCUS_OVT_AFC_Get_AF_Status(pFeatureReturnPara32);            
			*pFeatureParaLen=4;
            break;
		case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
		    OV5645MIPISENSORDB("[OV5645MIPI]SENSOR_FEATURE_SINGLE_FOCUS_MODE\n");
			OV5645_FOCUS_OVT_AFC_Single_Focus();
            break;	
		case SENSOR_FEATURE_CONSTANT_AF:
			OV5645MIPISENSORDB("[OV5645MIPI]SENSOR_FEATURE_CONSTANT_AF\n");
			OV5645_FOCUS_OVT_AFC_Constant_Focus();
			 break;
	    case SENSOR_FEATURE_CANCEL_AF:
            OV5645MIPISENSORDB("[OV5645MIPI]SENSOR_FEATURE_CANCEL_AF\n");
			OV5645_FOCUS_OVT_AFC_Cancel_Focus();
            break;	
		
        case SENSOR_FEATURE_GET_AF_INF:
            OV5645MIPISENSORDB("[OV5645MIPI]SENSOR_FEATURE_GET_AF_INF\n");
			OV5645_FOCUS_Get_AF_Inf(pFeatureReturnPara32);
			*pFeatureParaLen=4;       
            break;
        case SENSOR_FEATURE_GET_AF_MACRO:
            OV5645MIPISENSORDB("[OV5645MIPI]SENSOR_FEATURE_GET_AF_MACRO\n");
			OV5645_FOCUS_Get_AF_Macro(pFeatureReturnPara32);
			*pFeatureParaLen=4;           
            break;

        case SENSOR_FEATURE_SET_AF_WINDOW:       
			OV5645MIPISENSORDB("[OV5645MIPI]SENSOR_FEATURE_SET_AF_WINDOW\n");
			OV5645_FOCUS_Set_AF_Window(*pFeatureData32);
            break;				
        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
            OV5645MIPISENSORDB("[OV5645MIPI]SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS\n");
			OV5645_FOCUS_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);            
			*pFeatureParaLen=4;
            break;        
        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
            OV5645MIPISENSORDB("[OV5645MIPI]AE zone addr = 0x%x\n",*pFeatureData32);
			OV5645_FOCUS_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32);            
			*pFeatureParaLen=4;
            break;        
        case SENSOR_FEATURE_SET_AE_WINDOW:
            OV5645MIPISENSORDB("[OV5645MIPI]AE zone addr = 0x%x\n",*pFeatureData32);			
			OV5645_FOCUS_Set_AE_Window(*pFeatureData32);
            break; 
       		// add for control flash ---yyf
		   case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
			      printk("func:%s,line:%d",__func__,__LINE__);
			      ov5645_get_exp_flag(pFeatureData32);
			     break;
		// end --yyf
		default:
			OV5645MIPISENSORDB("OV5645MIPIFeatureControl:default \n");
			break;			
	}
	OV5645MIPISENSORDB("[OV5645MIPI]exit OV5645MIPIFeatureControl function:\n ");
	return ERROR_NONE;
}	/* OV5645MIPIFeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncOV5645MIPI=
{
	OV5645MIPIOpen,
	OV5645MIPIGetInfo,
	OV5645MIPIGetResolution,
	OV5645MIPIFeatureControl,
	OV5645MIPIControl,
	OV5645MIPIClose
};

UINT32 OV5645_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncOV5645MIPI;
	return ERROR_NONE;
}	/* SensorInit() */



