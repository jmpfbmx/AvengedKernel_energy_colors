#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
#include <mach/pmic_mt6329_hw_bank1.h>
#include <mach/pmic_mt6329_sw_bank1.h>
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#include <mach/mt_gpio.h>
#include <linux/delay.h>
#include <mach/irqs.h>

#define LCD_PWM_MOD_VALUE 255

#ifndef GPIO_LCD_PULSE_CONTRL
#define GPIO_LCD_PULSE_CONTRL GPIO134
#endif

#define ERROR_BL_LEVEL 0xFFFFFFFF

unsigned int brightness_mapping(unsigned int level)
{
	return ERROR_BL_LEVEL;
}

static int custom_disp_bls_set_backlight(unsigned int  value)
{
        unsigned int brightness;
        unsigned int brightness_count;
        unsigned int i;
        unsigned long flags;
        static unsigned int s_bBacklightOn;
        static unsigned int g_ledCurrPulseCount;

        value = LCD_PWM_MOD_VALUE * value >> 10;

        if (value > LCD_PWM_MOD_VALUE)
                brightness = 0;

        if (value == 0) {
                mt_set_gpio_out(GPIO_LCD_PULSE_CONTRL, GPIO_OUT_ZERO);
                s_bBacklightOn = 0;
                g_ledCurrPulseCount = 0;
                mdelay(2);
        }
        else
        {
                brightness = (LCD_PWM_MOD_VALUE - value) >> 3;
		printk("brightness is %d\n", brightness);

                if (s_bBacklightOn) {
                        if (g_ledCurrPulseCount > brightness)
                                brightness_count = brightness + 32 - g_ledCurrPulseCount;
                        else if (g_ledCurrPulseCount < brightness)
                                brightness_count = brightness - g_ledCurrPulseCount;
                        else
                                return 0;

                        local_irq_save(flags);
                        for(i = 0; i < brightness_count ; i++)
                        {
                                mt_set_gpio_out(GPIO_LCD_PULSE_CONTRL, GPIO_OUT_ZERO);
                                udelay(1);
                                mt_set_gpio_out(GPIO_LCD_PULSE_CONTRL, GPIO_OUT_ONE);
                                udelay(1);
                        }
                        local_irq_restore(flags);
                }
                else
                {
                        local_irq_save(flags);
                        mt_set_gpio_out(GPIO_LCD_PULSE_CONTRL, GPIO_OUT_ONE);
                        udelay(40);
                        for(i = 0; i < brightness; i++)
                        {
                                mt_set_gpio_out(GPIO_LCD_PULSE_CONTRL, GPIO_OUT_ZERO);
                                udelay(1);
                                mt_set_gpio_out(GPIO_LCD_PULSE_CONTRL, GPIO_OUT_ONE);
                                udelay(1);
                        }
                        local_irq_restore(flags);
                }

                g_ledCurrPulseCount = brightness;
                s_bBacklightOn = 1;
        }
        return 0;

}


static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_NONE, -1, {0}},
	{"green",             MT65XX_LED_MODE_NONE, -1, {0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1, {0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1, {0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1, {0}},
	{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK2,{0}},
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (int)custom_disp_bls_set_backlight,{0}},
	//{"lcd-backlight",     MT65XX_LED_MODE_PWM, PWM2,{1,1,30,30,0}},
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

