#ifndef _CUST_BATTERY_METER_H
#define _CUST_BATTERY_METER_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
//#define SOC_BY_AUXADC
//#define SOC_BY_HW_FG
#define SOC_BY_SW_FG

/* ADC Channel Number */
#define CUST_TABT_NUMBER 17
#define VBAT_CHANNEL_NUMBER      7
#define ISENSE_CHANNEL_NUMBER	 6
#define VCHARGER_CHANNEL_NUMBER  4
#define VBATTEMP_CHANNEL_NUMBER  5

/* ADC resistor  */
#define R_BAT_SENSE 4					
#define R_I_SENSE 4						
#define R_CHARGER_1 330
#define R_CHARGER_2 39

#define TEMPERATURE_T0             110
#define TEMPERATURE_T1             0
#define TEMPERATURE_T2             25
#define TEMPERATURE_T3             50
#define TEMPERATURE_T              255 // This should be fixed, never change the value

#define FG_METER_RESISTANCE 	0

/* Qmax for battery  */
#ifdef AEON_1400MA_BAT
#define Q_MAX_POS_50	1448
#define Q_MAX_POS_25	1462
#define Q_MAX_POS_0	1476
#define Q_MAX_NEG_10	1498

#define Q_MAX_POS_50_H_CURRENT	1440
#define Q_MAX_POS_25_H_CURRENT	1440
#define Q_MAX_POS_0_H_CURRENT	1453
#define Q_MAX_NEG_10_H_CURRENT	1440
#else
#define Q_MAX_POS_50	1289
#define Q_MAX_POS_25	1301
#define Q_MAX_POS_0	1308
#define Q_MAX_NEG_10	1324

#define Q_MAX_POS_50_H_CURRENT	1277
#define Q_MAX_POS_25_H_CURRENT	1273
#define Q_MAX_POS_0_H_CURRENT	1224
#define Q_MAX_NEG_10_H_CURRENT	1293
#endif


/* Discharge Percentage */
#define OAM_D5		 0		//  1 : D5,   0: D2


/* battery meter parameter */
#define CUST_TRACKING_POINT  14
#define CUST_HW_CC 		     0
#define AGING_TUNING_VALUE   103
#define CUST_R_FG_OFFSET    0
#ifdef MTK_FAN5405_SUPPORT
#define CUST_R_SENSE         68
#else
#define CUST_R_SENSE         200
#endif

#define OCV_BOARD_COMPESATE	0 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
#define CAR_TUNE_VALUE		94 //1.00


/* HW Fuel gague  */
#define CURRENT_DETECT_R_FG	10  //1mA
#define MinErrorOffset       1000
#define FG_VBAT_AVERAGE_SIZE 18
#define R_FG_VALUE 			0 // mOhm, base is 20

#define CUST_POWERON_DELTA_CAPACITY_TOLRANCE	40
#define CUST_POWERON_LOW_CAPACITY_TOLRANCE		5
#define CUST_POWERON_MAX_VBAT_TOLRANCE			90
#define CUST_POWERON_DELTA_VBAT_TOLRANCE		30

/* Disable Battery check for HQA */
#ifdef MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
#define FIXED_TBAT_25
#endif

/* Dynamic change wake up period of battery thread when suspend*/
#define VBAT_NORMAL_WAKEUP		3600		//3.6V
#define VBAT_LOW_POWER_WAKEUP		3500		//3.5v
#define NORMAL_WAKEUP_PERIOD		5400 		//90 * 60 = 90 min
#define LOW_POWER_WAKEUP_PERIOD		300		//5 * 60 = 5 min
#define CLOSE_POWEROFF_WAKEUP_PERIOD	30	//30 s

#endif	//#ifndef _CUST_BATTERY_METER_H
