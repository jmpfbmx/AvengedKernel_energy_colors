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
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <linux/kernel.h>//for printk


#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_ERR, PFX , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg); \
                } while(0)

				
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

#define CAMERADB(fmt, args...) printk(KERN_ERR"[CAMERADB]%s %d : "fmt, __FUNCTION__, __LINE__, ##args)

#define IDX_SENSOR_CMRST_PIN 			0
#define IDX_SENSOR_CMPDN_PIN			1
#define IDX_SENSOR_CMRST_PIN_MODE	2
#define IDX_SENSOR_CMPDN_PIN_MODE	3

void VCAMA_LDO_EN_fuc(int enable)
{
       
       if(mt_set_gpio_mode(GPIO_VCAMA_LDO_EN_PIN,GPIO_VCAMA_LDO_EN_PIN_M_GPIO))
       {
               PK_DBG("[VCAMA_LDO_EN_fuc] set gpio mode failed!! \n");
       }
       if(mt_set_gpio_dir(GPIO_VCAMA_LDO_EN_PIN,GPIO_DIR_OUT))
       {
               PK_DBG("[VCAMA_LDO_EN_fuc] set gpio dir failed!! \n");
       }
       
       if(enable==1)
       {
               if(mt_set_gpio_out(GPIO_VCAMA_LDO_EN_PIN,GPIO_OUT_ONE))
               {
                       PK_DBG("[VCAMA_LDO_EN_fuc ] set gpio high failed!! \n");
               }
       }
       else
       {
               if(mt_set_gpio_out(GPIO_VCAMA_LDO_EN_PIN,GPIO_OUT_ZERO))
               {
                       PK_DBG("[VCAMA_LDO_EN_fuc ] set gpio low failed!! \n");
               }
       }
               
 }

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
	u32 pinSetIdx = 0;//default main sensor
	u32 pinSet_Sensor[3][4]= 
	{
		//for main sensor 
		{
			GPIO_CAMERA_CMRST_PIN,
			GPIO_CAMERA_CMPDN_PIN,
			GPIO_CAMERA_CMRST_PIN_M_GPIO,
			GPIO_CAMERA_CMPDN_PIN_M_GPIO
		},
		//for sub sensor 
		{
			GPIO_CAMERA_CMRST1_PIN,
			GPIO_CAMERA_CMPDN1_PIN,
			GPIO_CAMERA_CMRST1_PIN_M_GPIO,
			GPIO_CAMERA_CMPDN1_PIN_M_GPIO
		},	
		//for main_2 sensor 
		{
		/*
			GPIO_CAMERA_2_CMRST_PIN,
			GPIO_CAMERA_2_CMPDN_PIN,
			GPIO_CAMERA_2_CMRST_PIN_M_GPIO,
			GPIO_CAMERA_2_CMPDN_PIN_M_GPIO
			*/
		},
	};

	CAMERADB("SensorIdx=%d,currSensorName=%s\n",SensorIdx,currSensorName);

	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
		pinSetIdx = 0;
	}
	else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
		pinSetIdx = 1;
	}
	else if (DUAL_CAMERA_MAIN_SECOND_SENSOR == SensorIdx) {
		pinSetIdx = 2;
		CAMERADB("SensorIdx==3,return error\n");
		return -EIO;
	}

	if(currSensorName)
	{		

			if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI258_MIPI_YUV,currSensorName)))
		{
			//power ON
			if (On)
			{
				//standby other sensor
				CAMERADB("SENSOR_DRVNAME_HI258_MIPI_YUV  Power Up\r\n");	

				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){CAMERADB("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){CAMERADB("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){CAMERADB("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){CAMERADB("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){CAMERADB("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){CAMERADB("[CAMERA SENSOR] set gpio failed!! \n");}
				}
	
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){CAMERADB("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){CAMERADB("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){CAMERADB("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){CAMERADB("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){CAMERADB("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){CAMERADB("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
					CAMERADB("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(500) ;	      
				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					CAMERADB("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(500);    
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))//zhaoshaopeng ov5647 1.5
				{
					CAMERADB("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
				udelay(500) ;	      
				//if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				//{
				//	CAMERADB("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
				//	goto _kdCISModulePowerOn_exit_;
				//}

				mdelay(10);
				//enable active sensor
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(20);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(20);
			}
			else 
			{
				printk("SENSOR_DRVNAME_HI258_MIPI_YUV  Power Down\r\n");
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				udelay(20);
				
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(50);
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}     	
				udelay(50);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
				//if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
				//{
				//	PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
				//	goto _kdCISModulePowerOn_exit_;
				//}    

				///????????????????????????
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}

			}//
		}
		else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_SP0A19_YUV,currSensorName)))
		{
			//power ON
			if (On)
			{
				/*
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				*/
				//in case
				printk("SENSOR_DRVNAME_SP0A19_YUV  Power Up\r\n");		
				//PDN pin
				/*
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				*/

				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(5);
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}		  
				mdelay(5);
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}	
				mdelay(10);
			//	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				//{
					//PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
			//		goto _kdCISModulePowerOn_exit_;
				//}
				
				//enable active sensor
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
					mdelay(10);
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
					mdelay(20);
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
					mdelay(10);
				}
				//reset pin
				/*
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				*/
				mdelay(20);
			}
			else 
			{
				printk("SENSOR_DRVNAME_SP0A19_YUV  Power Down\r\n");
				//PDN pin
				/*
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(5);
				*/
				//reset pin
				/*
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				*/
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(5);
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}		
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
			//	if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
				//{
					//PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
			//		goto _kdCISModulePowerOn_exit_;
				//}	
				mdelay(20);
			#if 0
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
			#endif
			}
		}
		else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC0329_YUV,currSensorName)))
		{
			//power ON
			if (On)
			{
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				//in case
				printk("SENSOR_DRVNAME_GC0329_YUV  Power Up\r\n");		
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(500);		  
				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(500); 		  
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}	
			//	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				//{
					//PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
			//		goto _kdCISModulePowerOn_exit_;
				//}

				mdelay(10);
				//enable active sensor
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(10);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(20);
			}
			else 
			{
				printk("SENSOR_DRVNAME_GC0329_YUV  Power Down\r\n");
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(5);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(5);
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}		
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
			//	if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
				//{
					//PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
			//		goto _kdCISModulePowerOn_exit_;
				//}	
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
			}
		}
		else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2035_YUV,currSensorName)))
		{
			//power ON
			if (On)
			{
				//standby other sensor
				printk("SENSOR_DRVNAME_GC2035_YUV  Power Up\r\n");	

				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
	
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(500) ;	      
				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(500);    
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))//zhaoshaopeng ov5647 1.5
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
				//if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				//{
				//	PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
				//	goto _kdCISModulePowerOn_exit_;
				//}

				mdelay(10);
				//enable active sensor
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(20);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(20);
			}
			else 
			{
				printk("SENSOR_DRVNAME_GC2035_YUV  Power Down\r\n");
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(5);
				
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(5);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(5);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(5);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}     	
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
				//if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
				//{
				//	PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
				//	goto _kdCISModulePowerOn_exit_;
				//}

				///????????????????????????
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				
			}//
		}
		else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2035MIPI_YUV,currSensorName)))
		{
			//power ON
			if (On)
			{
				//standby other sensor
				printk("SENSOR_DRVNAME_GC2035_YUV  Power Up\r\n");	

				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
	
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(500) ;	 
				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(500);    
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))//zhaoshaopeng ov5647 1.5
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
				//if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				//{
				//	PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
				//	goto _kdCISModulePowerOn_exit_;
				//}

				mdelay(10);
				//enable active sensor
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(20);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(20);
			}
			else 
			{
				printk("SENSOR_DRVNAME_GC2035_YUV  Power Down\r\n");
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(5);
				
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(5);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(5);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(5);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}     	
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
				//if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
				//{
				//	PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
				//	goto _kdCISModulePowerOn_exit_;
				//}

				///????????????????????????
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				
			}//
		}
		else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5CAGX_YUV,currSensorName)))
              {
			//power ON
			if (On)
			{
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				//in case
				printk("SENSOR_DRVNAME_S5K5CAGX_YUV  Power Up\r\n");		
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
                            //zhaoshaopeng vcore
                            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))//zhaoshaopeng from VOL_1500
                            {
                                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                                 //return -EIO;
                                 goto _kdCISModulePowerOn_exit_;
                            }    		
                            mdelay(50);
                            VCAMA_LDO_EN_fuc(1);
                            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
                            {
                                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                                //return -EIO;
                                goto _kdCISModulePowerOn_exit_;
                            }
                            mdelay(50);
                            //zhaoshaopeng af
                            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
                            {
                                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                                //return -EIO;
                                goto _kdCISModulePowerOn_exit_;
                            } 
                    
                            // wait power to be stable 
                            mdelay(5); 
                      	//zhaoshaopeng vdd_io	
                            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
                            {
                                PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                                //return -EIO;
                                goto _kdCISModulePowerOn_exit_;
                            }
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(5);
							
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
				
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(20);
			}
			else 
			{
				printk("SENSOR_DRVNAME_S5K5CAGX_YUV  Power Down\r\n");
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(10);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);

				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(10);
                            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
                            {
                                PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                                //return -EIO;
                                goto _kdCISModulePowerOn_exit_;
                            }                    	
				udelay(100);
				            VCAMA_LDO_EN_fuc(0);
                        	if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
                                //return -EIO;
                                goto _kdCISModulePowerOn_exit_;
                            }
				udelay(100);
                            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
                            {
                                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                                //return -EIO;
                                goto _kdCISModulePowerOn_exit_;
                            }     	
				udelay(100);
                            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
                                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                                //return -EIO;
                                goto _kdCISModulePowerOn_exit_;
                            }
                    		mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
							
			}
		}		
					
         else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI704_YUV,currSensorName)))
		{
			//power ON
			if (On)
			{
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				//in case
				printk("SENSOR_DRVNAME_HI704_YUV  Power Up\r\n");		
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(100);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	

				udelay(100);	
				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(100);		  
			//	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				//{
				//	PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
			///		goto _kdCISModulePowerOn_exit_;
				//}
				
				//reset pin
				mdelay(20);
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(2);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(20);
			}
			else 
			{
				printk("SENSOR_DRVNAME_HI704_YUV  Power Down\r\n");
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(10);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	
				
		//		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
				//{
					//PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
		//			goto _kdCISModulePowerOn_exit_;
				//}	
                            mdelay(1);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
	
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
			}
		}
             else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5EAYX_YUV,currSensorName)))
		{
			//power ON
			if (On)
			{
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				//in case
				printk("SENSOR_DRVNAME_S5K5EAYX_YUV  Power Up\r\n");		
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	

				udelay(100);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(100);
				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(100);		  
			//	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
			//	{
			//		PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//		//return -EIO;
			//		goto _kdCISModulePowerOn_exit_;
			//	}

				//udelay(500); 
				
				//reset pin
				//if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
				//	if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				//}
				mdelay(20);
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(2);
				//enable active sensor
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(5);
			}
			else 
			{
			  //  return 0;
				printk("SENSOR_DRVNAME_S5K5EAYX_YUV  Power Down\r\n");
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);

                                //PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(10);
                
			//	if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
			//	{
			//		PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//		//return -EIO;
			//		goto _kdCISModulePowerOn_exit_;
			//	}	

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
			}

            }
		else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5EAYX_YUV_GZ,currSensorName)))
		{
			//power ON
			if (On)
			{
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				//in case
				printk("SENSOR_DRVNAME_S5K5EAYX_YUV_GZ  Power Up\r\n");		
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	
				udelay(100);	
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(100);
				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(100);		  
			//	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
			//	{
			//		PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
			//		goto _kdCISModulePowerOn_exit_;
			//	}

				//udelay(500); 
				
				//reset pin
				//if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
				//	if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				//}
				mdelay(20);
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(2);
				//enable active sensor
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(5);
			}
			else 
			{
			  //  return 0;
				printk("SENSOR_DRVNAME_S5K5EAYX_YUV_GZ  Power Down\r\n");
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);

                                //PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(10);
                
			//	if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
			//	{
			//		PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//		//return -EIO;
			//		goto _kdCISModulePowerOn_exit_;
			//	}	

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
			}

            }
         else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5645_MIPI_YUV,currSensorName)))
         {
			//power ON
			if (On)
			{
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				//in case
				printk("SENSOR_DRVNAME_OV5645_YUV  Power Up\r\n");		
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	

				udelay(100);	
				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(100);		  
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
			/*	udelay(100);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}		*/		
                            mdelay(7);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(2);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}				
				mdelay(20);
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
			mdelay(20);
			}
			else 
			{
				printk("SENSOR_DRVNAME_OV5645_YUV  Power Down\r\n");
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(10);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);

			/*	if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}	*/

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
	
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
			}
		}

	   else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_AT2250_YUV,currSensorName)))
		{
			//power ON
			if (On)
			{
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				//in case
				printk("SENSOR_DRVNAME_AT2250_YUV  Power Up\r\n");		
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(100);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	

				udelay(100);	
				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(100);		  
			//	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				//{
				//	PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
			///		goto _kdCISModulePowerOn_exit_;
				//}
				
				//reset pin
				mdelay(20);
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(2);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(20);
			}
			else 
			{
				printk("SENSOR_DRVNAME_AT2250_YUV  Power Down\r\n");
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(10);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	
				
		//		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
				//{
					//PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
		//			goto _kdCISModulePowerOn_exit_;
				//}	
                            mdelay(1);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
	
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
			}
		}
	   else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2145MIPI_YUV,currSensorName)))
		{
			//power ON
			if (On)
			{
				//standby other sensor
				printk("SENSOR_DRVNAME_GC2145MIPI_YUV  Power Up\r\n");	

				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
	
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(500) ;	 
				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(500);    
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))//zhaoshaopeng ov5647 1.5
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
				//if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				//{
				//	PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
				//	goto _kdCISModulePowerOn_exit_;
				//}

				mdelay(10);
				//enable active sensor
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(20);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(20);
			}
			else 
			{
				printk("SENSOR_DRVNAME_GC2145MIPI_YUV  Power Down\r\n");
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(5);
				
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(5);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(5);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(5);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}     	
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
				//if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
				//{
				//	PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
				//	goto _kdCISModulePowerOn_exit_;
				//}

				///????????????????????????
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				
			}//
		}
	  
	  else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV2685MIPI_YUV,currSensorName)))
		  {
			 //power ON
			 if (On)
			 {
			 	/*
				 if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					 if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					 if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					 if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				 }
				 //reset pin
				 
				 if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					 if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					 if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					 if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				 }
				 */
				 //in case
				 printk("SENSOR_DRVNAME_OV2685MIPI_YUV	Power Up\r\n"); 	 
				 //PDN pin
				 if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					 if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					 if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					 if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				 }
				 //reset pin
				 /*
				 if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					 if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					 if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					 if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				 }
				 */
				 mdelay(10);
				 if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				 {
						 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						 //return -EIO;
						 goto _kdCISModulePowerOn_exit_;
				 }	 
	  
				 udelay(100);	 
				 VCAMA_LDO_EN_fuc(1);
				 if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				 {
					 PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					 //return -EIO;
					 goto _kdCISModulePowerOn_exit_;
				 }
				 udelay(100);		   
				 if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
				 {
					 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					 //return -EIO;
					 goto _kdCISModulePowerOn_exit_;
				 }
			  //  udelay(100);
			  // if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
			  //	 {
			  //	 PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			  //		 //return -EIO;
			  //	 goto _kdCISModulePowerOn_exit_;
			  // }	 
							 mdelay(7);
				 //PDN pin
				 if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					 if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				 }
				 mdelay(2);
				 //reset pin
				 /*
				 if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					 if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				 }				 
				 mdelay(20);
				 if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					 if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				 }
				 */
			 mdelay(20);
			 }
			 else 
			 {
				 printk("SENSOR_DRVNAME_OV2685MIPI_YUV	Power Down\r\n");
				 //PDN pin
				 if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					 if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					 if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					 if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				 }
				 mdelay(1);
				 //reset pin
				 /*
				 if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					 if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					 if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					 if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				 }
				 mdelay(10);
				 */
	  
			 //  if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
			 //	 {
			 //		 PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			 //		 //return -EIO;
			 //		 goto _kdCISModulePowerOn_exit_;
			 //	 }	 
	  
				 if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					 PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					 //return -EIO;
					 goto _kdCISModulePowerOn_exit_;
				 }
				 VCAMA_LDO_EN_fuc(0);
				 if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				 {
						 PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
						 //return -EIO;
						 goto _kdCISModulePowerOn_exit_;
				 }	 
	  
				 if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				 {
					 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					 //return -EIO;
					 goto _kdCISModulePowerOn_exit_;
				 }	
	  
				 mdelay(20);
				 //PDN pin
				 /*
				 if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]){
					 if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				 }
				 */
				 static int index_for_sp0a19 = 0;
				 if(index_for_sp0a19 == 0){
				 	index_for_sp0a19 = 1;
				 if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
				 	if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					 if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					 if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				 }
				 }
				 
			 }
		 }	  
		else//other sensor
		{
			//power ON
			if (On)
			{
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				
				//in case
				printk("SENSOR_DRVNAME_DEFAULT  Power Up\r\n");		
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(100);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	

				udelay(100);		  
				VCAMA_LDO_EN_fuc(1);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				udelay(100);		  
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				
				//reset pin
				mdelay(20);
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(2);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(20);
			}
			else 
			{
				printk("SENSOR_DRVNAME_DEFAULT  Power Down\r\n");
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				mdelay(10);
				//reset pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN]) {
					if(mt_set_gpio_mode(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMRST_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				}
				mdelay(10);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}	

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}
				VCAMA_LDO_EN_fuc(0);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
				{
						PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
						//return -EIO;
						goto _kdCISModulePowerOn_exit_;
				}	

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					//return -EIO;
					goto _kdCISModulePowerOn_exit_;
				}  
	
				mdelay(20);
				//PDN pin
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
				if (GPIO_CAMERA_INVALID != pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN]){
					if(mt_set_gpio_out(pinSet_Sensor[1-pinSetIdx][IDX_SENSOR_CMPDN_PIN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				}
			}
		}
	}
	else
	{
		CAMERADB("currSensorName == NULL \n");
		goto _kdCISModulePowerOn_exit_;
	}
	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);



