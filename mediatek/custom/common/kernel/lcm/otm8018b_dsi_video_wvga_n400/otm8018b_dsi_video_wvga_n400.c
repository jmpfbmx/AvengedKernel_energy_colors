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

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>

#define Lcd_Log printf
#else
#include <linux/string.h>
#include <linux/kernel.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define Lcd_Log printk
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (800)
#define LCM_ID       (0x1202)
#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER
#define GPIO_LCD_ID_PIN GPIO_LCM_ID_PIN

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static unsigned int lcm_compare_id();

#define LCM_DSI_CMD_MODE    0

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


//CMI 3.97" IPS
static struct LCM_setting_table lcm_initialization_setting[] = {
	{0x00, 1, {0x00}},
	{0xFF, 3, {0x80,0x09,0x01}},

	{0x00, 1, {0x80}},
	{0xFF, 2, {0x80,0x09}},

  {0x00, 1, {0xC6}},
  {0xB0, 1, {0x03}},
  
  {0x00, 1, {0x8B}},
  {0xB0, 1, {0x40}},
  
  {0x00, 1, {0x90}},
  {0xB3, 1, {0x00}},
  
  {0x00, 1, {0x92}},
  {0xB3, 1, {0x05}},
  
	{0x00, 1, {0x80}},
	{0xC0, 9, {0x00,0x58,0x00,0x15,0x15,0x00,0x58,0x15,0x15}},

	{0x00, 1, {0x90}},
	{0xC0, 6, {0x00,0x44,0x00,0x00,0x00,0x03}},

	{0x00, 1, {0xA3}},
	{0xC0, 1, {0x00}},

	{0x00, 1, {0xB4}},
	{0xC0, 1, {0x50}},

	{0x00, 1, {0x81}},
	{0xC1, 1, {0x66}},

	{0x00, 1, {0xA0}},
	{0xC1, 2, {0xEA,0x08}},

	{0x00, 1, {0xA6}},
	{0xC1, 3, {0x01,0x00,0x00}},

	{0x00, 1, {0x81}},
	{0xC4, 1, {0x83}},

	{0x00, 1, {0x89}},
	{0xC4, 1, {0x08}},

	{0x00, 1, {0x82}},
	{0xC5, 1, {0xA3}},

	{0x00, 1, {0x90}},
	{0xC5, 3, {0x96,0x87,0x01}},

	{0x00, 1, {0xB1}},
	{0xC5, 1, {0xA9}},

	{0x00, 1, {0xC0}},
	{0xC5, 1, {0x00}},

	{0x00, 1, {0xB2}},
	{0xF5, 4, {0x15,0x00,0x15,0x00}},

	{0x00, 1, {0xC7}},
	{0xCF, 1, {0x02}},

	{0x00, 1,  {0x80}},
	{0xCE, 12, {0x87,0x03,0x00,0x86,0x03,0x00,0x85,0x03,0x00,0x84,0x03,0x00}},

	{0x00, 1, {0x90}},
	{0xCE, 14, {0x33,0x1C,0x00,0x33,0x1D,0x00,0x33,0x1E,0x00,0x33,0x1F,0x00,0x00,0x00}},

	{0x00, 1, {0xA0}},
	{0xCE, 14, {0x38,0x05,0x03,0x20,0x00,0x00,0x00,0x38,0x04,0x03,0x21,0x00,0x00,0x00}},

	{0x00, 1, {0xB0}},
	{0xCE, 14, {0x38,0x03,0x03,0x22,0x00,0x00,0x00,0x38,0x02,0x03,0x23,0x00,0x00,0x00}},

	{0x00, 1, {0xC0}},
	{0xCE, 14, {0x38,0x01,0x03,0x24,0x00,0x00,0x00,0x38,0x00,0x03,0x25,0x00,0x00,0x00}},

	{0x00, 1, {0xD0}},
	{0xCE, 14, {0x30,0x00,0x03,0x26,0x00,0x00,0x00,0x30,0x01,0x03,0x27,0x00,0x00,0x00}},

	{0x00, 1, {0xC3}},
	{0xCB, 8, {0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04}},

	{0x00, 1, {0xD8}},
	{0xCB, 7, {0x04,0x04,0x04,0x04,0x04,0x04,0x04}},

	{0x00, 1, {0xE0}},
	{0xCB, 1, {0x04}},

	{0x00, 1, {0x83}},
	{0xCC, 7, {0x03,0x01,0x09,0x0B,0x0D,0x0F,0x05}},

	{0x00, 1, {0x90}},
	{0xCC, 1, {0x07}},

	{0x00, 1, {0x9D}},
	{0xCC, 2, {0x04,0x02}},

	{0x00, 1, {0xA0}},
	{0xCC, 6, {0x0A,0x0C,0x0E,0x10,0x06,0x08}},

	{0x00, 1, {0xB3}},
	{0xCC, 7, {0x06,0x08,0x0A,0x10,0x0E,0x0C,0x04}},

	{0x00, 1, {0xC0}},
	{0xCC, 1, {0x02}},

	{0x00, 1, {0xCD}},
	{0xCC, 1, {0x05}},

	{0x00, 1, {0xCD}},
	{0xCC, 1, {0x07}},

	{0x00, 1, {0xD0}},
	{0xCC, 6, {0x09,0x0F,0x0D,0x0B,0x03,0x01}},

	{0x00, 1, {0x00}},
	{0xD8, 2, {0x8F,0x8F}},
	
	{0x00, 1, {0x00}},
	{0xD9, 1, {0x61}},

	{0x00, 1, {0x00}},
	{0xE1, 16, {0x03,0x08,0x0F,0x0E,0x09,0x1F,0x0F,0x0F,0x00,0x04,0x01,0x06,0x0E,0x1D,0x1A,0x0A}},
	
	{0x00, 1, {0x00}},
	{0xE2, 16, {0x03,0x08,0x0F,0x0E,0x09,0x1F,0x0F,0x0F,0x00,0x04,0x01,0x06,0x0E,0x1D,0x1A,0x0A}},
	
  {0x00, 1, {0x00}},
	{0xFF, 3, {0xFF,0xFF,0xFF}},
	
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 200, {}}, 

	{0x29, 0, {0x00}},

	{0x2C, 0, {0x00}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}




        };


static struct LCM_setting_table lcm_set_window[] = {
            {0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
            {0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
            {REGFLAG_END_OF_TABLE, 0x00, {}}
        };


static struct LCM_setting_table lcm_sleep_out_setting[] = {

            // Sleep Out
            {0x11, 1, {0x00}},
            {REGFLAG_DELAY, 120, {}},

            // Display ON
            {0x29, 1, {0x00}},
            {REGFLAG_DELAY, 20, {}},

            //{REGFLAG_END_OF_TABLE, 0x00, {}}
        };


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
            // Display off sequence
            {0x28, 1, {0x00}},
            {REGFLAG_DELAY, 50, {}},

            // Sleep Mode On
            {0x10, 1, {0x00}},
            {REGFLAG_DELAY, 100, {}},

            //{REGFLAG_END_OF_TABLE, 0x00, {}}
        };

static struct LCM_setting_table lcm_compare_id_setting[] = {

            {0x00,1,{0x00}},
            {0xff,3,{0x80,0x09,0x01}},
            {0x00,1,{0x80}},
            {0xff,2,{0x80,0x09}},
        };

static struct LCM_setting_table lcm_backlight_level_setting[] = {
            {0x51, 1, {0xFF}},
            {REGFLAG_END_OF_TABLE, 0x00, {}}
        };


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

        case REGFLAG_DELAY :
            MDELAY(table[i].count);
            break;

            //case REGFLAG_END_OF_TABLE :
            //  break;

        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
            //MDELAY(1);//MDELAY(10);
        }
    }

}


static void init_lcm_registers(void)
{
    int id;

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
	{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

///#if (LCM_DSI_CMD_MODE)
		///params->dsi.mode   = CMD_MODE;
///#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
////#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;


		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 10;
		params->dsi.vertical_frontporch					= 20;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 6;
		params->dsi.horizontal_backporch				= 90;
		params->dsi.horizontal_frontporch				= 10;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

                    params->dsi.pll_div1=2;//48;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		params->dsi.pll_div2=0; 		// div2=0~15: fout=fvo/(2*div2)
                params->dsi.fbk_div =32;

}


static void lcm_init(void)
{
#if 0
#ifdef BUILD_LK
    pmic_config_interface(0x0532,5,0x7,5);//add by libo for VGP2 POWER ON
    pmic_config_interface(0x050C,1,0x1,15);
#else
    hwPowerOn(MT6323_POWER_LDO_VGP2,VOL_2800,"LCM");
#endif
    MDELAY(100);
#endif
    SET_RESET_PIN(1);
    MDELAY(25);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(100);
    init_lcm_registers();

}

static void lcm_suspend(void)
{
    SET_RESET_PIN(1);
    MDELAY(25);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(100);

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(50);//Must > 10ms
}

static void lcm_resume(void)
{
//    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	lcm_init();
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);


    unsigned int data_array[16];


    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}
#if 0
static int get_lcd_id(void)
{
    mt_set_gpio_mode(GPIO_LCD_ID_PIN,0);
    mt_set_gpio_dir(GPIO_LCD_ID_PIN,0);
    mt_set_gpio_pull_enable(GPIO_LCD_ID_PIN,1);
    mt_set_gpio_pull_select(GPIO_LCD_ID_PIN,0);
    MDELAY(1);

    return mt_get_gpio_in(GPIO_LCD_ID_PIN);
}
#endif
#define LCM_ID_OTM8018B  0x8009
static unsigned int lcm_compare_id(void)
{
 	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xa1, buffer, 5);

	id_high = buffer[2];
	id_low = buffer[3];
	id = (id_high<<8) | id_low;

	#if defined(BUILD_UBOOT)
		printf("OTM8018B uboot %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		//printk("OTM8018B kernel %s \n", __func__);
		//printk("%s id = 0x%08x \n", __func__, id);
	#endif

	return (LCM_ID_OTM8018B == id)?1:0;
}

LCM_DRIVER otm8018b_dsi_video_wvga_n400_lcm_drv = 
{
    .name			= "otm8018b_dsi_video_wvga_n400",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};

