#ifndef __AWINIC_MONITOR_H__
#define __AWINIC_MONITOR_H__

//#define AW_DEBUG


#define AW882XX_MONITOR_DEFAULT_FLAG (0)
#define AW882XX_MONITOR_DEFAULT_TIMER_VAL (30000)
#define AW882XX_MONITOR_VBAT_RANGE (6025)
#define AW882XX_MONITOR_INT_10BIT (1023)
#define AW882XX_MONITOR_TEMP_SIGN_MASK (1<<9)
#define AW882XX_MONITOR_TEMP_NEG_MASK (0XFC00)
#define AW882XX_BIT_SYSCTRL2_BST_IPEAK_MASK (15<<0)
#define AW882XX_BIT_HAGCCFG4_GAIN_SHIFT (8)
#define AW882XX_BIT_HAGCCFG4_GAIN_MASK (0x00ff)

struct aw882xx_low_vol {
	uint32_t vol;
	uint8_t ipeak;
	uint8_t gain;
};

struct aw882xx_low_temp {
	int16_t temp;
	uint8_t ipeak;
	uint8_t gain;
};


/*smartpa monitor*/
enum aw882xx_ipeak {
	IPEAK_3P50_A = 0x08,
	IPEAK_3P00_A = 0x06,
	IPEAK_2P75_A = 0x05,
	IPEAK_2P50_A = 0x04,
	IPEAK_NONE   = 0xFF,
};

enum aw882xx_gain {
	GAIN_NEG_0P0_DB = 0x00,
	GAIN_NEG_0P5_DB = 0x01,
	GAIN_NEG_1P0_DB = 0x02,
	GAIN_NEG_1P5_DB = 0x03,
	GAIN_NEG_3P0_DB = 0x06,
	GAIN_NEG_4P5_DB = 0x09,
	GAIN_NEG_6P0_DB = 0x10,
	GAIN_NONE       = 0xFF,
};

/******************************************************************
* struct aw882xx monitor
*******************************************************************/
struct aw882xx_monitor {
	struct hrtimer timer;
	uint32_t timer_val;
	struct work_struct work;
	uint32_t is_enable;
	uint16_t pre_vol;
	int16_t pre_temp;
	uint8_t vol_ipeak;
	uint8_t vol_gain;
	uint8_t temp_ipeak;
	uint8_t temp_gain;
#ifdef AW_DEBUG
	uint16_t test_vol;
	int16_t test_temp;
#endif
};

/******************************************************************
* aw882xx monitor functions
*******************************************************************/
int aw882xx_monitor_start(struct aw882xx_monitor *monitor);
int aw882xx_monitor_stop(struct aw882xx_monitor *monitor);
void init_aw882xx_monitor(struct aw882xx_monitor *monitor);
void aw882xx_parse_monitor_dt(struct aw882xx_monitor *monitor);




#endif
