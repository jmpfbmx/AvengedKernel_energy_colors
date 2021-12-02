#ifdef BUILD_LK

#else
#include <linux/string.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)

#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

#ifndef FALSE
  #define FALSE (0)
#endif

#ifndef TRUE
  #define TRUE  (1)
#endif


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                                                                   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)



struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {

	/*
	Note :

	Data ID will depends on the following rule.

		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag

	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
	{0XFF, 5, {0XFF, 0X98, 0X06, 0X04, 0X01}},
	{0X8, 1, {0X10}},
	{0X21, 1, {0X01}},
	{0X30, 1, {0X02}},
	{0X31, 1, {0X02}},
	{0X40, 1, {0X16}},
	{0X41, 1, {0X33}},
	{0X42, 1, {0X00}},
	{0X43, 1, {0X85}},
	{0X44, 1, {0X8B}},
	{0X45, 1, {0X1B}},
	{0X50, 1, {0X78}},
	{0X51, 1, {0X78}},
	{0X52, 1, {0X00}},
	{0X53, 1, {0X42}},
	{0X57, 1, {0X50}},
	{0X60, 1, {0X07}},
	{0X61, 1, {0X00}},
	{0X62, 1, {0X07}},
	{0X63, 1, {0X00}},
	{0XA0, 1, {0X00}},
	{0XA1, 1, {0X0B}},
	{0XA2, 1, {0X12}},
	{0XA3, 1, {0X0C}},
	{0XA4, 1, {0X05}},
	{0XA5, 1, {0X0C}},
	{0XA6, 1, {0X07}},
	{0XA7, 1, {0X16}},
	{0XA8, 1, {0X06}},
	{0XA9, 1, {0X0A}},
	{0xAA, 1, {0x0F}},
	{0xAB, 1, {0x06}},
	{0XAC, 1, {0X0E}},
	{0XAD, 1, {0X1A}},
	{0XAE, 1, {0X12}},
	{0XAF, 1, {0X00}},
	{0XC0, 1, {0X00}},
	{0XC1, 1, {0X0B}},
	{0XC2, 1, {0X12}},
	{0XC3, 1, {0X0C}},
	{0XC4, 1, {0X05}},
	{0XC5, 1, {0X0C}},
	{0XC6, 1, {0X07}},
	{0XC7, 1, {0X16}},
	{0XC8, 1, {0X06}},
	{0XC9, 1, {0X0A}},
	{0XCA, 1, {0X0F}},
	{0XCB, 1, {0X06}},
	{0XCC, 1, {0X0E}},
	{0XCD, 1, {0X1A}},
	{0XCE, 1, {0X12}},
	{0XCF, 1, {0X00}},
	{0XFF, 5, {0XFF, 0X98, 0X06, 0X04, 0X06}},
	{0X0, 1, {0XA0}},
	{0X1, 1, {0X05}},
	{0X2, 1, {0X00}},
	{0X3, 1, {0X00}},
	{0X4, 1, {0X01}},
	{0X5, 1, {0X01}},
	{0X6, 1, {0X88}},
	{0X7, 1, {0X04}},
	{0X8, 1, {0X01}},
	{0X9, 1, {0X90}},
	{0XA, 1, {0X04}},
	{0XB, 1, {0X01}},
	{0XC, 1, {0X01}},
	{0XD, 1, {0X01}},
	{0XE, 1, {0X00}},
	{0XF, 1, {0X00}},
	{0X10, 1, {0X55}},
	{0X11, 1, {0X50}},
	{0X12, 1, {0X01}},
	{0X13, 1, {0X85}},
	{0X14, 1, {0X85}},
	{0X15, 1, {0XC0}},
	{0X16, 1, {0X0B}},
	{0X17, 1, {0X00}},
	{0X18, 1, {0X00}},
	{0X19, 1, {0X00}},
	{0X1A, 1, {0X00}},
	{0X1B, 1, {0X00}},
	{0X1C, 1, {0X00}},
	{0X1D, 1, {0X00}},
	{0X20, 1, {0X01}},
	{0X21, 1, {0X23}},
	{0X22, 1, {0X45}},
	{0X23, 1, {0X67}},
	{0X24, 1, {0X01}},
	{0X25, 1, {0X23}},
	{0X26, 1, {0X45}},
	{0X27, 1, {0X67}},
	{0X30, 1, {0X02}},
	{0X31, 1, {0X22}},
	{0X32, 1, {0X11}},
	{0X33, 1, {0XAA}},
	{0X34, 1, {0XBB}},
	{0X35, 1, {0X66}},
	{0X36, 1, {0X00}},
	{0X37, 1, {0X22}},
	{0X38, 1, {0X22}},
	{0X39, 1, {0X22}},
	{0X3A, 1, {0X22}},
	{0X3B, 1, {0X22}},
	{0X3C, 1, {0X22}},
	{0X3D, 1, {0X22}},
	{0X3E, 1, {0X22}},
	{0X3F, 1, {0X22}},
	{0X40, 1, {0X22}},
	{0X52, 1, {0X10}},
	{0X53, 1, {0X10}},
	{0XFF, 5, {0XFF, 0X98, 0X06, 0X04, 0X07}},
	{0X17, 1, {0X22}},
	{0X2, 1, {0X77}},
	{0XE1, 1, {0X79}},
	{0XFF, 5, {0XFF, 0X98, 0X06, 0X04, 0X00}},
	{0X35, 1, {0X00}},
	{0X11, 0, {0}},
	{REGFLAG_DELAY, 0x96, {0}},
	{0X29, 0, {0}},
	{REGFLAG_DELAY, 60, {0}},
	{REGFLAG_END_OF_TABLE, 0, {0}}
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
	{0xFF, 5 ,{0xFF,0x98,0x06,0x04,0x01}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 0xC8, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 0xC8, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count)
{
	unsigned int i;

	for(i = 0; i < count; i++) {

	unsigned cmd;
	cmd = table[i].cmd;

	switch (cmd) {

	    case REGFLAG_DELAY :
	        MDELAY(table[i].count);
	        break;

	    case REGFLAG_END_OF_TABLE :
	        break;

	    default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, 1);//force_update);
	   	}
	}

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

	params->height = FRAME_HEIGHT;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 12;
	params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DBI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.vertical_frontporch = 20;
	params->dsi.compatibility_for_nvk = 0;
	params->dsi.pll_div2 = 0;
	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->dsi.LANE_NUM = LCM_TWO_LANE;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = 480*3;
	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 80;
	params->dsi.horizontal_frontporch = 80;
	params->dsi.horizontal_blanking_pixel = 80;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.pll_div1 = 2;
	params->dsi.fbk_div = 28;
}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(10 );
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table));
}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table));
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	//SET_RESET_PIN(1);
	//MDELAY(120);

}

static void lcm_resume(void)
{
	// push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
#if 0
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_midd=0;
	char id_low=0;
	int id=0,lcd_id=0;

	mt_set_gpio_mode(GPIO_LCM_ID, GPIO_LCM_ID_M_GPIO);
	mt_set_gpio_dir(GPIO_LCM_ID, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_LCM_ID,GPIO_PULL_DISABLE);
	mt_set_gpio_pull_select(GPIO_LCM_ID,GPIO_PULL_DOWN);
	MDELAY(1);

	//Do reset here
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(50);

	array[0]=0x00063902;
	array[1]=0x0698ffff;
	array[2]=0x00000104;
	dsi_set_cmdq(array, 3, 1);
	MDELAY(10);
	array[0] = 0x00033700;
	dsi_set_cmdq(array, 1, 1);
	//read_reg_9806(0x04, buffer, 3);//if read 0x04,should get 0x008000,that is both OK.
	read_reg_v2(0x00, buffer,1);
	id_high = buffer[0]; /////////////////////////0x98
	read_reg_v2(0x01, buffer,1);
	id_midd = buffer[0]; ///////////////////////0x06
	read_reg_v2(0x02, buffer,1);
	id_low = buffer[0]; ///////////////////////0x04
	id = (id_midd << 8) | id_low;

	lcd_id =  mt_get_gpio_in(GPIO_LCM_ID);
#if defined(BUILD_LK)
	printf("*******lanhong 9806E***************  id_high=%x,id_midd=%x,id_low=%x ,id=%x ,lcd_id=%d \n", id_high,id_midd, id_low,id,lcd_id);
#endif

	return (lcd_id ==0 && 0x0604 == id) ? 1: 0;
#endif
	unsigned int array[4];
	unsigned int id;
	unsigned char buffer[4];

	SET_RESET_PIN(1);
	MDELAY(2);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table));

	array[0] = 0x13700;
	dsi_set_cmdq(array, 1, 1);
	memset(buffer, 0, 4);
	read_reg_v2(0, buffer, 1);

	array[0] = 0x13700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(1, &buffer[1], 1);

	array[0] = 0x13700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(2, &buffer[2], 1);
#if defined(BUILD_LK)
	printf("### zhaoshaopeng  ili9806 lcm_compare_id buffer[0] = 0x%x,0x%x,0x%x,0x%x", buffer[0],buffer[1],buffer[2],buffer[3]);
#endif
	if ( ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]) == 0x980604 ) {
		mt_set_gpio_mode(GPIO_LCM_ID, GPIO_LCM_ID_M_GPIO);
		mt_set_gpio_dir(GPIO_LCM_ID, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_LCM_ID,GPIO_PULL_DISABLE);
		mt_set_gpio_pull_select(GPIO_LCM_ID,GPIO_PULL_DOWN);
		id = mt_get_gpio_in(GPIO_LCM_ID);
		//return id - 1 + (id - 1 <= 0) - (id - 1); //idk wtf is this
		//return (id == 0) ? 1 : 0; //maybe this?
		return 1; //ok, good
	}
	else
	{
		return 0;
	}
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER tongxinta_ili9806e_wvga_dsi_vdo_mid4006_lcm_drv =
{
	.name          = "tongxinta_ili9806e_wvga_dsi_vdo_mid4006",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.update         = lcm_update,
	//.set_backlight	= lcm_setbacklight,
	//.set_backlight_mode = lcm_setbacklight_mode,
	//.set_pwm        = lcm_setpwm,
	//.get_pwm        = lcm_getpwm
	.compare_id    = lcm_compare_id,
	//.esd_check      = lcm_esd_check,
	//.esd_recover    = lcm_esd_recover,

};
