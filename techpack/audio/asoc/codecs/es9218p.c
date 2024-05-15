/*
 *  ES9218P Device Driver.
 *
 *  This program is device driver for ES9218P chip set.
 *  The ES9218P is a high-performance 32-bit, 2-channel audio SABRE HiFi D/A converter
 *  with headphone amplifier, analog volume control and output switch designed for
 *  audiophile-grade portable application such as mobile phones and digital music player,
 *  consumer applications such as USB DACs and A/V receivers, as well as professional
 *  such as mixer consoles and digital audio workstations.
 *
 *  Copyright (C) 2016, ESS Technology International Ltd.
 *
 */
#include    <linux/module.h>
#include    <linux/moduleparam.h>
#include    <linux/init.h>
#include    <linux/slab.h>
#include    <linux/delay.h>
#include    <linux/pm.h>
#include    <linux/platform_device.h>
#include    <linux/pm_wakeup.h>
#include    <sound/core.h>
#include    <sound/pcm.h>
#include    <sound/pcm_params.h>
#include    <sound/soc.h>
#include    <sound/initval.h>
#include    <sound/tlv.h>
#include    <trace/events/asoc.h>
#include    <linux/of_gpio.h>
#include    <linux/gpio.h>
#include    <linux/i2c.h>
#include    <linux/fs.h>
#include    <linux/string.h>
#include    <linux/clk.h>

#if defined(CONFIG_ARCH_SM8150)
#include    <linux/pm_qos.h>
#endif

//#define     USE_CONTROL_EXTERNAL_LDO_FOR_DVDD // control a external LDO drained from PMIC
#ifdef USE_CONTROL_EXTERNAL_LDO_FOR_DVDD
#include    <linux/regulator/consumer.h>
#endif

#include    "es9218p.h"
//#include    "../../../../../../kernel/msm-4.14/include/soc/qcom/lge/board_lge.h"

#define     ES9218P_SYSFS               // use this feature only for user debug, not release

//#define     USE_HPAHiQ                  // THD increased by ~2dB and Power Consumption increasded by ~2mA
//#define   ES9218P_DEBUG               // ESS pop-click debugging, define to enable step by step override sequence debug messages and time delays.  Use to pinpoint pop-click.
#define     WORKAROUND_FOR_CORNER_SAMPLES     // set ResetB high two times and send a cmd of soft reset
#define     ENABLE_DOP_AUTO_MUTE
#define     ENABLE_DOP_SOFT_MUTE

static struct es9218_priv *g_es9218_priv = NULL;
static int  es9218_write_reg(struct i2c_client *client, int reg, u8 value);
static int  es9218_read_reg(struct i2c_client *client, int reg);

static int  es9218p_sabre_hifi2lpb(void);
static int  es9218p_sabre_bypass2hifi(void);

static int  es9218p_sabre_lpb2hifione(void);
static int  es9218p_sabre_lpb2hifitwo(void);
static int  es9218p_sabre_hifione2lpb(void);
static int  es9218p_sabre_hifitwo2lpb(void);

static int  es9218p_sabre_amp_start(struct i2c_client *client, int headset);
static int  es9218p_sabre_amp_stop(struct i2c_client *client, int headset);
static int  es9218p_standby2lpb(void);
static int  es9218p_lpb2standby(void);
static int es9218p_set_volume_rate(unsigned int sample_rate, unsigned int ess_mode);
static int es9218p_set_bit_width(unsigned int bit_width, unsigned int ess_mode);
static void es9218p_initialize_registers(unsigned int ess_mode);


#ifdef CONFIG_MACH_KONA_TIMELM
extern int lge_get_board_rev_no_for_dlkm(void);
#endif /* CONFIG_MACH_KONA_TIMELM */

#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
static int get_fade_count_define(void);
static int get_fade_term_define(void);

#define VOLUME_DOWN 0
#define VOLUME_UP 1
#define FADE_INOUT_COUNT 7
#define FADE_INOUT_TERM 40
#define SHORT_FILTER 9  // first menu item

#define SHARP_FILTER 4   // second menu item
#define SLOW_FILTER 5  // third menu item
static struct workqueue_struct *mute_workqueue;
struct delayed_work *mute_work;
static int fade_direction = VOLUME_DOWN;
static int fade_count_debug_param = 99;
static int fade_term_debug_param = 99;
static int g_fade_count;
static int g_right_fade_vol_per_step = 0;
static int g_left_fade_vol_per_step = 0;
static int g_left_fade_vol = 0;
static int g_right_fade_vol = 0;
bool lge_ess_fade_inout_init = false;
#endif
struct es9218_reg {
    unsigned char   num;
    unsigned char   value;
};

struct wakeup_source *wl_sleep;
struct wakeup_source *wl_shutdown;

#if defined(CONFIG_ARCH_SM8150)
struct pm_qos_request req;
#endif

static struct clk *hifi_ext_mclk;

/*
 *  We only include the analogue supplies here; the digital supplies
 *  need to be available well before this driver can be probed.
 */

struct es9218_reg es9218_common_init_registers[] = {
//will be upadated  { ES9218P_REG_00,        0x00 },    // System Register
//will be upadated  { ES9218P_REG_01,        0x8c },    // Input selection
//default           { ES9218P_REG_02,        0x34 },    // Mixing, Serial Data and Automute Configuration
//will be upadated  { ES9218P_REG_03,        0x58 },    // Analog Volume Control
//default           { ES9218P_REG_04,        0x00 },    // Automute Time
//default           { ES9218P_REG_05,        0x68 },    // Automute Level
//will be upadated  { ES9218P_REG_06,        0x42 },    // DoP and Volmue Ramp Rate
//will be upadated  { ES9218P_REG_07,        0x80 },    // Filter Bandwidth and System Mute
//default           { ES9218P_REG_08,        0xdd },    // GPIO1-2 Confgiguratioin
//			{ ES9218P_REG_08,        0xd3 },    // GPIO1 clk debug setting

//will be upadated  { ES9218P_REG_10,        0x02 },    // Master Mode and Sync Configuration
                    { ES9218P_REG_11,        0x90 },    // Overcureent Protection
                    { ES9218P_REG_12,        0x8a },    // ASRC/DPLL Bandwidth
                    { ES9218P_REG_13,        0x00 },    // THD Compensation Bypass & Mono Mode
                    { ES9218P_REG_14,        0x25 },    // Soft Start Configuration // updated_ESS_1122
//will be upadated  { ES9218P_REG_15,        0x50 },    // Volume Control
//will be upadated  { ES9218P_REG_16,        0x50 },    // Volume Control
//will be upadated  { ES9218P_REG_17,        0xff },    // Master Trim
//will be upadated  { ES9218P_REG_18,        0xff },    // Master Trim
//will be upadated  { ES9218P_REG_19,        0xff },    // Master Trim
//will be upadated  { ES9218P_REG_20,        0x7f },    // Master Trim
                    { ES9218P_REG_21,        0x0d },    // GPIO Input Selection  // updated_ESS_1122
//will be upadated  { ES9218P_REG_22,        0x00 },    // THD Compensation C2 (left)
//will be upadated  { ES9218P_REG_23,        0x00 },    // THD Compensation C2 (left)
//will be upadated  { ES9218P_REG_24,        0x00 },    // THD Compensation C3 (left)
//will be upadated  { ES9218P_REG_25,        0x00 },    // THD Compensation C3 (left)
//will be upadated  { ES9218P_REG_26,        0x62 },    // Charge Pump Soft Start Delay
                    { ES9218P_REG_27,        0xc4 },    // Charge Pump Soft Start Delay
//will be upadated  { ES9218P_REG_29,        0x00 },    // General Confguration
                    { ES9218P_REG_30,        0x37 },    // GPIO Inversion & Automatic Clock Gearing
                    { ES9218P_REG_31,        0x30 },    // GPIO Inversion & Automatic Clock Gearing
//will be upadated  { ES9218P_REG_32,        0x00 },    // Amplifier Configuration
//default           { ES9218P_REG_34,        0x00 },    // Programmable NCO
//default           { ES9218P_REG_35,        0x00 },    // Programmable NCO
//default           { ES9218P_REG_36,        0x00 },    // Programmable NCO
//default           { ES9218P_REG_37,        0x00 },    // Programmable NCO
//default           { ES9218P_REG_40,        0x00 },    // Programmable FIR RAM Address
//default           { ES9218P_REG_41,        0x00 },    // Programmable FIR RAM Data
//default           { ES9218P_REG_42,        0x00 },    // Programmable FIR RAM Data
//default           { ES9218P_REG_43,        0x00 },    // Programmable FIR RAM Data
//will be upadated  { ES9218P_REG_44,        0x00 },    // Programmable FIR Configuration
//will be upadated  { ES9218P_REG_45,        0x00 },    // Analog Control Override
//will be upadated  { ES9218P_REG_46,        0x00 },    // dig_over_en/reserved/apdb/cp_clk_sel/reserved
//will be upadated  { ES9218P_REG_47,        0x00 },    // enfcb/encp_oe/enaux_oe/cpl_ens/cpl_enw/sel3v3_ps/ensm_ps/sel3v3_cph
//will be upadated  { ES9218P_REG_48,        0x02 },    // reserved/enhpa_out/reverved
//default           { ES9218P_REG_49,        0x62 },    // Automatic Clock Gearing Thresholds
//default           { ES9218P_REG_50,        0xc0 },    // Automatic Clock Gearing Thresholds
//default           { ES9218P_REG_51,        0x0d },    // Automatic Clock Gearing Thresholds
//will be upadated  { ES9218P_REG_53,        0x00 },    // THD Compensation C2 (Right)
//will be upadated  { ES9218P_REG_54,        0x00 },    // THD Compensation C2 (Right)
//will be upadated  { ES9218P_REG_55,        0x00 },    // THD Compensation C3 (Right)
//will be upadated  { ES9218P_REG_56,        0x00 },    // THD Compensation C3 (Right)
//default           { ES9218P_REG_60,        0x00 },    // DAC Analog Trim Control
};

struct es9218_reg   es9218_PCM_init_register[] = {
	{ ES9218P_REG_00,        0x00 },    // System Register - 0x00(default)
#ifdef ENABLE_DOP_AUTO_MUTE
	{ ES9218P_REG_02,        0x14 },    // Mixing, Serial Data and Automute Configuration // updated_ESS_1122
	{ ES9218P_REG_04,		 0x00 },	// Automute Time
#endif /* ENABLE_DOP_AUTO_MUTE */
	{ ES9218P_REG_06,        0x43 },    // DoP and Volmue Ramp Rate - 0x42 DoP disabled
	{ ES9218P_REG_10,        0x02 },    // Master Mode and Sync Configuration - 0x02 : Slave mode
//will be upadated    { ES9218P_REG_15,        0x00 },    // Volume Control - not used
//will be upadated    { ES9218P_REG_16,        0x00 },    // Volume Control - not used
    { ES9218P_REG_29,        0x00 },    // General Confguration - Max. M/2-0x05, M/4-0x06, M/8-0x07 , //es9218p 0x06 , es9219 0x00
};

struct es9218_reg   es9218_DOP_init_register[] = {
//will be upadated    { ES9218P_REG_00,        0x00 },    // System Register - 0x00 : MCLK/1 : DOP128(Default), 0x04 : MCLK/2 : DOP64
    { ES9218P_REG_01,        0x80 },    // Input selection - 0x80 : 32bit-serial only
	{ ES9218P_REG_06, 	     0x4a },	  // DoP and Volmue Ramp Rate - 0x4a : DoP(64/128) enable
    { ES9218P_REG_10,        0x82 },    // Master Mode and Sync Configuration - 0x82 : DoP64/128 Master mode enable
//        { ES9218P_REG_10,  0x02 },    // Master Mode and Sync Configuration - 0x82 : DoP64/128 Master mode enable
#ifdef ENABLE_DOP_AUTO_MUTE
    { ES9218P_REG_14,		 0x07 },	// Soft Start Configuration
#endif /* ENABLE_DOP_AUTO_MUTE */
//will be upadated    { ES9218P_REG_15,        0x00 },    // Volume Control - 0x0? : Variable Value from AP
//will be upadated    { ES9218P_REG_16,        0x00 },    // Volume Control - 0x0? : Variable Value from AP
    { ES9218P_REG_29,        0x00 },    // General Confguration - 0x00 : auto-gear disable
};


static const u32 master_trim_tbl[] = {
    /*  0   db */   0x7FFFFFFF,
    /*- 0.5 db */   0x78D6FC9D,
    /*- 1   db */   0x721482BF,
    /*- 1.1 db */   0x70C6359E,     // ESS_0104
    /*- 1.2 db */   0x6F7BBC22,     // ESS_0104
    /*- 1.3 db */   0x6E350B12,     // ESS_0104
    /*- 1.4 db */   0x6CF2175A,     // ESS_0104
    /*- 1.5 db */   0x6BB2D603,
    /*- 1.6 db */   0x6A773C38,     // ESS_0103
    /*- 1.7 db */   0x693F3F44,     // ESS_0103
    /*- 1.8 db */   0x680AD491,     // ESS_0103
    /*- 1.9 db */   0x66D9F1A7,     // ESS_0103
    /*- 2   db */   0x65AC8C2E,
    /*-2.2 dB */	  0x635C10C4,   // updated_ESS_1122
    /*- 2.5 db */   0x5FFC888F,
    /*- 3   db */   0x5A9DF7AA,
    /*- 3.5 db */   0x558C4B21,
    /*- 4   db */   0x50C335D3,
    /*- 4.5 db */   0x4C3EA838,
    /*- 5   db */   0x47FACCEF,
    /*- 5.5 db */   0x43F4057E,
    /*- 6   db */   0x4026E73C,
    /*- 6.5 db */   0x3C90386F,
    /*- 7   db */   0x392CED8D,
    /*- 7.5 db */   0x35FA26A9,
    /*- 8   db */   0x32F52CFE,
    /*- 8.5 db */   0x301B70A7,
    /*- 9   db */   0x2D6A866F,
    /*- 9.5 db */   0x2AE025C2,
    /*- 10  db */   0x287A26C4,
    /*- 10.5db */   0x26368073,
    /*- 11  db */   0x241346F5,
    /*- 11.5db */   0x220EA9F3,
    /*- 12  db */   0x2026F30F,
    /*- 12.5db */   0x1E5A8471,
    /*- 13  db */   0x1CA7D767,
    /*- 13.5db */   0x1B0D7B1B,
    /*- 14  db */   0x198A1357,
    /*- 14.5db */   0x181C5761,
    /*- 15  db */   0x16C310E3,
    /*- 15.5db */   0x157D1AE1,
    /*- 16  db */   0x144960C5,
    /*- 16.5db */   0x1326DD70,
    /*- 17  db */   0x12149A5F,
    /*- 17.5db */   0x1111AEDA,
    /*- 18  db */   0x101D3F2D,
    /*- 18.5db */   0xF367BED,
    /*- 19  db */   0xE5CA14C,
    /*- 19.5db */   0xD8EF66D,
    /*- 20  db */   0xCCCCCCC,
    /*- 20.5db */   0xC157FA9,
    /*- 21  db */   0xB687379,
    /*- 21.5db */   0xAC51566,
    /*- 22  db */   0xA2ADAD1,
    /*- 22.5db */   0x99940DB,
    /*- 23  db */   0x90FCBF7,
    /*- 23.5db */   0x88E0783,
    /*- 24  db */   0x8138561,
    /*- 24.5db */   0x79FDD9F,
    /*- 25  db */   0x732AE17,
    /*- 25.5db */   0x6CB9A26,
    /*- 26  db */   0x66A4A52,
    /*- 26.5db */   0x60E6C0B,
    /*- 27  db */   0x5B7B15A,
    /*- 27.5db */   0x565D0AA,
    /*- 28  db */   0x518847F,
    /*- 28.5db */   0x4CF8B43,
    /*- 29  db */   0x48AA70B,
    /*- 29.5db */   0x4499D60,
    /*- 30  db */   0x40C3713,
    /*- 30.5db */   0x3D2400B,
    /*- 31  db */   0x39B8718,
    /*- 31.5db */   0x367DDCB,
    /*- 32  db */   0x337184E,
    /*- 32.5db */   0x3090D3E,
    /*- 33  db */   0x2DD958A,
    /*- 33.5db */   0x2B48C4F,
    /*- 34  db */   0x28DCEBB,
    /*- 34.5db */   0x2693BF0,
    /*- 35  db */   0x246B4E3,
    /*- 35.5db */   0x2261C49,
    /*- 36  db */   0x207567A,
    /*- 36.5db */   0x1EA4958,
    /*- 37  db */   0x1CEDC3C,
    /*- 37.5db */   0x1B4F7E2,
    /*- 38  db */   0x19C8651,
    /*- 38.5db */   0x18572CA,
    /*- 39  db */   0x16FA9BA,
    /*- 39.5db */   0x15B18A4,
    /*- 40  db */   0x147AE14,
};

static const u8 avc_vol_tbl[] = {
    /*  0   db */   0x40,
    /*- 1   db */   0x41,
    /*- 2   db */   0x42,
    /*- 3   db */   0x43,
    /*- 4   db */   0x44,
    /*- 5   db */   0x45,
    /*- 6   db */   0x46,
    /*- 7   db */   0x47,
    /*- 8   db */   0x48,
    /*- 9   db */   0x49,
    /*- 10  db */   0x4A,
    /*- 11  db */   0x4B,
    /*- 12  db */   0x4C,
    /*- 13  db */   0x4D,
    /*- 14  db */   0x4E,
    /*- 15  db */   0x4F,
    /*- 16  db */   0X50,
    /*- 17  db */   0X51,
    /*- 18  db */   0X52,
    /*- 19  db */   0X53,
    /*- 20  db */   0X54,
    /*- 21  db */   0X55,
    /*- 22  db */   0X56,
    /*- 23  db */   0X57,
    /*- 24  db */   0X58,
};

static const char *power_state[] = {
    "CLOSE",
    "OPEN",
    "BYPASS",
    "HIFI",
    "IDLE",
    "ACTIVE",
};

static unsigned int es9218_start = 0;
static unsigned int es9218_power_state = ESS_PS_CLOSE;
static unsigned int es9218_is_amp_on = 0;
static unsigned int es9218_bps = 16;
static unsigned int es9218_rate = 48000;
#ifdef ES9218P_NCO
static unsigned int es9218_mclk = 49152000;
#endif
static int g_headset_type = 0;
static int g_avc_volume = 0;
static int g_volume = 0;
static int g_left_volume = 0;
static int g_right_volume = 0;
static int g_sabre_cf_num = 8; // default = 8
static int g_dop_flag = 0;

static int g_auto_mute_flag = 0;
#ifdef ES9218P_DEBUG
static int g_debug_delay = 500; // ESS pop-click debugging step time delay
#endif
#ifdef CONFIG_MACH_SM8150_ALPHA
static u8  normal_harmonic_comp_left[4] = {0x78, 0x00, 0x9a, 0xfc};
static u8  normal_harmonic_comp_right[4] = {0x1e, 0x00, 0x12, 0xfd};
static u8  advance_harmonic_comp_left[4] = {0x58, 0x02, 0x3c, 0x00};
static u8  advance_harmonic_comp_right[4] = {0x21, 0x02, 0x64, 0x00};
static u8  aux_harmonic_comp_left[4] = {0x4a, 0x01, 0xe4, 0xfd};
static u8  aux_harmonic_comp_right[4] = {0x04, 0x01, 0x0c, 0xfe};
#elif defined(CONFIG_MACH_SM8150_FLASH)
static u8  normal_harmonic_comp_left[4] = {0x78, 0x00, 0x9a, 0xfc};
static u8  normal_harmonic_comp_right[4] = {0x1e, 0x00, 0x12, 0xfd};
static u8  advance_harmonic_comp_left[4] = {0x76, 0x02, 0x3c, 0x00};
static u8  advance_harmonic_comp_right[4] = {0x53, 0x02, 0x32, 0x00};
static u8  aux_harmonic_comp_left[4] = {0x5e, 0x01, 0xd0, 0xfd};
static u8  aux_harmonic_comp_right[4] = {0x22, 0x01, 0xee, 0xfd};
#elif defined(CONFIG_MACH_KONA_TIMELM)
static u8  normal_harmonic_comp_left[4] = {0x5b, 0xff, 0xd0, 0xfd}; //ESS_0103
static u8  normal_harmonic_comp_right[4] = {0x3d, 0xff, 0x07, 0xfe}; //ESS_0103
static u8  advance_harmonic_comp_left[4] = {0xaa, 0x00, 0xa0, 0x00}; //ESS_0103
static u8  advance_harmonic_comp_right[4] = {0xa0, 0x00, 0xa0, 0x00};//ESS_0103
static u8  aux_harmonic_comp_left[4] = {0x96, 0x00, 0x0a, 0x00}; //ESS_0103
static u8  aux_harmonic_comp_right[4] = {0x9b, 0x00, 0x0a, 0x00}; //ESS_0103
#else
static u8  normal_harmonic_comp_left[4] = {0x78, 0x00, 0x9a, 0xfc};
static u8  normal_harmonic_comp_right[4] = {0x1e, 0x00, 0x12, 0xfd};
static u8  advance_harmonic_comp_left[4] = {0x30, 0x02, 0x3c, 0x00};
static u8  advance_harmonic_comp_right[4] = {0xd6, 0x01, 0x3c, 0x00};
static u8  aux_harmonic_comp_left[4] = {0x72, 0x01, 0x84, 0xfe};
static u8  aux_harmonic_comp_right[4] = {0x3b, 0x01, 0x98, 0xfe};
#endif

enum {
    ESS_MODE_INIT,   // only for common register values before each mode init.
    ESS_MODE_PCM,    // PCM mode
    ESS_MODE_DoP,    // DoP64, DoP128 mode
};

static int prev_dop_flag = 0;
static int call_common_init_registers = 0;

#define ES9219C
#ifdef ES9219C
static unsigned int clk_source = 0; // 0: X-tal( 45.1584MHz), 1: MCLK : 19.2MHz, 3: BCLK
static unsigned int pll_setting = 0;
#define ES9219C_SLAVE_ADDR_1	0x48 // Main
#define ES9219C_SLAVE_ADDR_2	0x4A // Sub , Register #192 ~ #201

#define DRE_ENABLE
#ifdef DRE_ENABLE
static int es9219c_DRE_setting( void );
static u8  normal_DRE_gain_left[2] = {0xed, 0x15}; //ESS_0103
static u8  normal_DRE_gain_right[2] = {0xe5, 0x15}; //ESS_0103
static u8  normal_DRE_on_threshold[2] = {0x50, 0x1c};//1121
static u8  normal_DRE_off_threshold[2] = {0xb9, 0x51};//1121
static u8  normal_DRE_decay_rate = 0x0a;

static u8  advance_DRE_gain_left[2] = {0x73, 0x53}; //ESS_0103
static u8  advance_DRE_gain_right[2] = {0x5f, 0x53}; //ESS_0103
static u8  advance_DRE_on_threshold[2] = {0xf5, 0x28};
static u8  advance_DRE_off_threshold[2] = {0xf5, 0x2d};
static u8  advance_DRE_decay_rate = 0x0a;

static u8  aux_DRE_gain_left[2] = {0x41, 0x53}; //ESS_0103
static u8  aux_DRE_gain_right[2] = {0x4f, 0x53}; //ESS_0103
static u8  aux_DRE_on_threshold[2] = {0xf5, 0x28};
static u8  aux_DRE_off_threshold[2] = {0xf5, 0x2d};
static u8  aux_DRE_decay_rate = 0x0a;
#endif// DRE_ENABLE

//#define X_TALK_ENHANCEMENT_ENABLE //1121
#ifdef X_TALK_ENHANCEMENT_ENABLE
static int es9219c_crosstalk_enhancement( void );
static u8  crosstalk_scale_ch1[2] = {0x03, 0x0d};
static u8  crosstalk_scale_ch2[2] = {0x03, 0x0d};
#endif // X_TALK_ENHANCEMENT_ENABLE

#endif// ES9219C

bool g_ess_rev_check = false;

enum {
    ESS_A = 1,
    ESS_B = 2,
};
int g_ess_rev = ESS_B;


#define ES9218_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |  \
        SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |   \
        SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |   \
        SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_384000 ) //|SNDRV_PCM_RATE_352800  TODO for dop128

#define ES9218_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE | \
        SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
        SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE | \
        SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE)


#ifdef ES9218P_SYSFS
struct es9218_regmap {
    const char *name;
    uint8_t reg;
    int writeable;
} es9218_regs[] = {
    { "00_REGISTER",                       ES9218P_REG_00, 1 }, //00_SYSTEM_REGISTERS
    { "01_REGISTER",                       ES9218P_REG_01, 1 }, //01_INPUT_SELECTION
    { "02_REGISTER",                       ES9218P_REG_02, 1 }, //02_MIXING_AUTOMUTE_CONFIGURATION
    { "03_REGISTER",                       ES9218P_REG_03, 1 }, //03_ANALOG_VOLUME_CONTROL
    { "04_REGISTER",                       ES9218P_REG_04, 1 }, //04_AUTOMUTE_TIME
    { "05_REGISTER",                       ES9218P_REG_05, 1 }, //05_AUTOMUTE_LEVEL
    { "06_REGISTER",                       ES9218P_REG_06, 1 }, //06_DoP_VOLUME_RAMP_RATE
    { "07_REGISTER",                       ES9218P_REG_07, 1 }, //07_FIILTER_BANDWIDTH_SYSTEM_MUTE
    { "08_REGISTER",                       ES9218P_REG_08, 1 }, //08_GPIO1_2_CONFIGURATION
    { "09_REGISTER",                       ES9218P_REG_09, 1 }, //09_RESERVED_09
    { "10_REGISTER",                       ES9218P_REG_10, 1 }, //10_MASTER_MODE_SYNC_CONFIGURATION
    { "11_REGISTER",                       ES9218P_REG_11, 1 }, //11_OVERCURRENT_PROTECTION
    { "12_REGISTER",                       ES9218P_REG_12, 1 }, //12_ASRC_DPLL_BANDWIDTH
    { "13_REGISTER",                       ES9218P_REG_13, 1 }, //13_THD_COMPENSATION_BYPASS
    { "14_REGISTER",                       ES9218P_REG_14, 1 }, //14_SOFT_START_CONFIGURATION
    { "15_REGISTER",                       ES9218P_REG_15, 1 }, //15_VOLUME_CONTROL_1
    { "16_REGISTER",                       ES9218P_REG_16, 1 }, //16_VOLUME_CONTROL_2
    { "17_REGISTER",                       ES9218P_REG_17, 1 }, //17_MASTER_TRIM_3
    { "18_REGISTER",                       ES9218P_REG_18, 1 }, //18_MASTER_TRIM_2
    { "19_REGISTER",                       ES9218P_REG_19, 1 }, //19_MASTER_TRIM_1
    { "20_REGISTER",                       ES9218P_REG_20, 1 }, //20_MASTER_TRIM_0
    { "21_REGISTER",                       ES9218P_REG_21, 1 }, //21_GPIO_INPUT_SELECTION
    { "22_REGISTER",                       ES9218P_REG_22, 1 }, //22_THD_COMPENSATION_C2_2
    { "23_REGISTER",                       ES9218P_REG_23, 1 }, //23_THD_COMPENSATION_C2_1
    { "24_REGISTER",                       ES9218P_REG_24, 1 }, //24_THD_COMPENSATION_C3_2
    { "25_REGISTER",                       ES9218P_REG_25, 1 }, //25_THD_COMPENSATION_C3_1
    { "26_REGISTER",                       ES9218P_REG_26, 1 }, //26_CHARGE_PUMP_SOFT_START_DELAY
    { "27_REGISTER",                       ES9218P_REG_27, 1 }, //27_GENERAL_CONFIGURATION
    { "28_REGISTER",                       ES9218P_REG_28, 1 }, //28_RESERVED
    { "29_REGISTER",                       ES9218P_REG_29, 1 }, //29_GIO_INVERSION_AUTO_CLOCK_GEAR
    { "30_REGISTER",                       ES9218P_REG_30, 1 }, //30_CHARGE_PUMP_CLOCK_2
    { "31_REGISTER",                       ES9218P_REG_31, 1 }, //31_CHARGE_PUMP_CLOCK_1
    { "32_REGISTER",                       ES9218P_REG_32, 1 }, //32_AMPLIFIER_CONFIGURATION
    { "33_REGISTER",                       ES9218P_REG_33, 1 }, //33_RESERVED
    { "34_REGISTER",                       ES9218P_REG_34, 1 }, //34_PROGRAMMABLE_NCO_4
    { "35_REGISTER",                       ES9218P_REG_35, 1 }, //35_PROGRAMMABLE_NCO_3
    { "36_REGISTER",                       ES9218P_REG_36, 1 }, //36_PROGRAMMABLE_NCO_2
    { "37_REGISTER",                       ES9218P_REG_37, 1 }, //37_PROGRAMMABLE_NCO_1
    { "38_REGISTER",                       ES9218P_REG_38, 1 }, //38_RESERVED_38
    { "39_REGISTER",                       ES9218P_REG_39, 1 }, //39_RESERVED_39
    { "40_REGISTER",                       ES9218P_REG_40, 1 }, //40_PROGRAMMABLE_FIR_RAM_ADDRESS
    { "41_REGISTER",                       ES9218P_REG_41, 1 }, //41_PROGRAMMABLE_FIR_RAM_DATA_3
    { "42_REGISTER",                       ES9218P_REG_42, 1 }, //42_PROGRAMMABLE_FIR_RAM_DATA_2
    { "43_REGISTER",                       ES9218P_REG_43, 1 }, //43_PROGRAMMABLE_FIR_RAM_DATA_1
    { "44_REGISTER",                       ES9218P_REG_44, 1 }, //44_PROGRAMMABLE_FIR_CONFIGURATION
    { "45_REGISTER",                       ES9218P_REG_45, 1 }, //45_ANALOG_CONTROL_OVERRIDE
    { "46_REGISTER",                       ES9218P_REG_46, 1 }, //46_DIGITAL_OVERRIDE
    { "47_REGISTER",                       ES9218P_REG_47, 1 }, //47_RESERVED
    { "48_REGISTER",                       ES9218P_REG_48, 1 }, //48_SEPERATE_CH_THD
    { "49_REGISTER",                       ES9218P_REG_49, 1 }, //49_AUTOMATIC_CLOCK_GEARING_THRESHOLDS_3
    { "50_REGISTER",                       ES9218P_REG_50, 1 }, //50_AUTOMATIC_CLOCK_GEARING_THRESHOLDS_2
    { "51_REGISTER",                       ES9218P_REG_51, 1 }, //51_AUTOMATIC_CLOCK_GEARING_THRESHOLDS_1
    { "52_REGISTER",                       ES9218P_REG_52, 1 }, //52_RESERVED
    { "53_REGISTER",                       ES9218P_REG_53, 1 }, //53_THD_COMPENSATION_C2_2
    { "54_REGISTER",                       ES9218P_REG_54, 1 }, //54_THD_COMPENSATION_C2_1
    { "55_REGISTER",                       ES9218P_REG_55, 1 }, //55_THD_COMPENSATION_C3_2
    { "56_REGISTER",                       ES9218P_REG_56, 1 }, //56_THD_COMPENSATION_C3_1
    { "57_REGISTER",                       ES9218P_REG_57, 1 }, //57_RESERVED
    { "58_REGISTER",                       ES9218P_REG_58, 1 }, //58_RESERVED
    { "59_REGISTER",                       ES9218P_REG_59, 1 }, //59_RESERVED
    { "60_REGISTER",                       ES9218P_REG_60, 1 }, //60_DAC_ANALOG_TRIM_CONTROL
    { "64_REGISTER",                       ES9218P_REG_64, 0 }, //64_CHIP_STATUS
    { "65_REGISTER",                       ES9218P_REG_65, 0 }, //65_GPIO_READBACK
    { "66_REGISTER",                       ES9218P_REG_66, 0 }, //66_DPLL_NUMBER_4
    { "67_REGISTER",                       ES9218P_REG_67, 0 }, //67_DPLL_NUMBER_3
    { "68_REGISTER",                       ES9218P_REG_68, 0 }, //68_DPLL_NUMBER_2
    { "69_REGISTER",                       ES9218P_REG_69, 0 }, //69_DPLL_NUMBER_1
    { "70_REGISTER",                       ES9218P_REG_70, 0 }, //70_RESERVED
    { "71_REGISTER",                       ES9218P_REG_71, 0 }, //71_RESERVED
    { "72_REGISTER",                       ES9218P_REG_72, 0 }, //72_INPUT_SELECTION_AND_AUTOMUTE_STATUS
    { "73_REGISTER",                       ES9218P_REG_73, 0 }, //73_RAM_COEFFEICIENT_READBACK_3
    { "74_REGISTER",                       ES9218P_REG_74, 0 }, //74_RAM_COEFFEICIENT_READBACK_2
    { "75_REGISTER",                       ES9218P_REG_75, 0 }, //75_RAM_COEFFEICIENT_READBACK_1
    { "135_REGISTER",                      ES9219C_REG_135, 1 }, //135_ANALOG_CONTROL_OVERIDES
    { "136_REGISTER",                      ES9219C_REG_136, 1 }, //136_DRE_CONFIG
    { "137_REGISTER",                      ES9219C_REG_137, 1 }, //137_DRE_GAIN_CH1
    { "138_REGISTER",                      ES9219C_REG_138, 1 }, //138_DRE_GAIN_CH1
    { "139_REGISTER",                      ES9219C_REG_139, 1 }, //139_DRE_GAIN_CH2
    { "140_REGISTER",                      ES9219C_REG_140, 1 }, //140_DRE_GAIN_CH2
    { "141_REGISTER",                      ES9219C_REG_141, 1 }, //141_DRE_ON_THRESHOLD
    { "142_REGISTER",                      ES9219C_REG_142, 1 }, //142_DRE_ON_THRESHOLD
    { "143_REGISTER",                      ES9219C_REG_143, 1 }, //143_DRE_OFF_THRESHOLD
    { "144_REGISTER",                      ES9219C_REG_144, 1 }, //144_DRE_OFF_THRESHOLD
    { "145_REGISTER",                      ES9219C_REG_145, 1 }, //145_DRE_DECAY_RATE
    { "192_REGISTER",                      ES9219C_REG_192, 1 }, //192_SOFT_RESET
    { "193_REGISTER",                      ES9219C_REG_193, 1 }, //193_PLL_CONFIG1
    { "194_REGISTER",                      ES9219C_REG_194, 1 }, //194_PLL_CONFIG2
    { "195_REGISTER",                      ES9219C_REG_195, 1 }, //195_PLL_CONFIG2
    { "196_REGISTER",                      ES9219C_REG_196, 1 }, //196_PLL_CONFIG2
    { "197_REGISTER",                      ES9219C_REG_197, 1 }, //197_PLL_CONFIG2
    { "198_REGISTER",                      ES9219C_REG_198, 1 }, //198_PLL_CONFIG3
    { "199_REGISTER",                      ES9219C_REG_199, 1 }, //199_PLL_CONFIG3
    { "200_REGISTER",                      ES9219C_REG_200, 1 }, //200_PLL_CONFIG4
    { "201_REGISTER",                      ES9219C_REG_201, 1 }, //201_PLL_CONFIG5
};
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
static ssize_t get_fade_term_param(struct device *dev,
	                    struct device_attribute *attr, char *buf)
{

	unsigned val = 1;
	sprintf(buf, "%s : current fade term %d   ",__func__, get_fade_term_define());
	pr_info("%s() : fade_term_define = %d, val %d", __func__, get_fade_term_define(), val);

        return val;
}


static ssize_t set_fade_term_param(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{

	int value;
	unsigned val = count;
	sscanf(buf, "%d", &value);
	fade_term_debug_param = value;

	pr_info("%s() : new mute term = %d", __func__, get_fade_term_define());
	return val;

}

static ssize_t get_fade_mute_param(struct device *dev,
	                    struct device_attribute *attr, char *buf)
{

	unsigned val = 1;
	sprintf(buf, "%s : current fade count %d   ",__func__, get_fade_count_define());
	pr_info("%s() : fade_count_define = %d, val %d", __func__, get_fade_count_define(), val);

        return val;
}


static ssize_t set_fade_mute_param(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{

	int value;
	unsigned val = count;
	sscanf(buf, "%d", &value);
	fade_count_debug_param = value;

	pr_info("%s() : new mute count = %d", __func__, get_fade_count_define());
	return val;

}

static DEVICE_ATTR(fade_mute_count, S_IWUSR | S_IRUGO, get_fade_mute_param, set_fade_mute_param);
static DEVICE_ATTR(fade_mute_term, S_IWUSR | S_IRUGO, get_fade_term_param, set_fade_term_param);
#endif

static ssize_t es9218_registers_show(struct device *dev,
                  struct device_attribute *attr, char *buf)
{
    unsigned i, n, reg_count;
    u8 read_buf;

    reg_count = sizeof(es9218_regs) / sizeof(es9218_regs[0]);
    for (i = 0, n = 0; i < reg_count; i++) {
        read_buf = es9218_read_reg(g_es9218_priv->i2c_client, es9218_regs[i].reg);
        n += scnprintf(buf + n, PAGE_SIZE - n,
                   "%-15s <#%02d>= 0x%02X\n",
                   es9218_regs[i].name, es9218_regs[i].reg,
                   read_buf);
    }

    return n;
}

static ssize_t es9218_registers_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf, size_t count)
{
    unsigned i, reg_count, value;
    int error = 0;
    char name[45];

    if (count >= 45) {
        pr_err("%s:input too long\n", __func__);
        return -1;
    }

    if (sscanf(buf, "%40s %x", name, &value) != 2) {
        pr_err("%s:unable to parse input\n", __func__);
        return -1;
    }

    pr_info("%s: %s %0xx",__func__,name,value);
    reg_count = sizeof(es9218_regs) / sizeof(es9218_regs[0]);
    for (i = 0; i < reg_count; i++) {
        if (!strcmp(name, es9218_regs[i].name)) {
            if (es9218_regs[i].writeable) {
                error = es9218_write_reg(g_es9218_priv->i2c_client,
                                            es9218_regs[i].reg, value);
                if (error) {
                    pr_err("%s:Failed to write %s\n", __func__, name);
                    return -1;
                }
            }
            else {
                pr_err("%s:Register %s is not writeable\n", __func__, name);
                return -1;
            }

            return count;
        }
    }

    pr_err("%s:no such register %s\n", __func__, name);
    return -1;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
        es9218_registers_show, es9218_registers_store);

static struct attribute *es9218_attrs[] = {
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
	&dev_attr_fade_mute_count.attr,
	&dev_attr_fade_mute_term.attr,
#endif
    &dev_attr_registers.attr,
    NULL
};

static const struct attribute_group es9218_attr_group = {
    .attrs = es9218_attrs,
};

#endif /* ES9218P_SYSFS*/

#ifdef ES9218P_NCO
/*******************************************************************************//**
 Enable NCO mode by sampling freq.
 \param[in]	iFSR	: Smpling freq. of audio stream such as 44100 or 48000.
 \param[in]	iMCLK  	: It is Xin/n value. Xin will be 50MHz and 'n' can be '1/2/4/8'.
 \param[in]	iMODE  	: Set NCO enable/disable

 \return	int
 \retval	0	Success
 \retval	-1	Failure

 \b Note:   Register 34-37 : Programmable NCO(0x22 - 025)
            We can calculate "nco_num = iFSR*(2^32)/iMCLK".

***********************************************************************************/
static int es9218p_sabre_set_nco_num(int iFSR, int iMCLK, int iMODE)
{
	uint32_t nco_num;
	int ret = 0;
	int i;

	if ( es9218_power_state < ESS_PS_HIFI ) {
		pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
		return -1;
	}
	pr_info("%s() : state = %s\n", __func__, power_state[es9218_power_state]);
	nco_num = (iFSR*0x100000000)/iMCLK;
	pr_info("%s nco_num:%x\n",__func__, nco_num);

	for (i = 0; i < 4; i++) {
		if(iMODE)
			ret = es9218_write_reg(g_es9218_priv->i2c_client, es9218_regs[i+34].reg, (nco_num & 0xff));
		else
			ret = es9218_write_reg(g_es9218_priv->i2c_client, es9218_regs[i+34].reg, 0x00);

		if (ret < 0) {
			pr_err("%s:Failed to write %s\n", __func__, es9218_regs[i+34].name);
			return -1;
		}
		nco_num = (nco_num >> 8);
	}

	return ret;
}
#endif /* ES9218P_NCO */


/*
 *      ES9812P's Power state / mode control signals
 *      reset_gpio;         //HIFI_RESET_N
 *      power_gpio;         //HIFI_LDO_SW
 *      hph_switch_gpio;    //HIFI_MODE2
 *      reset_gpio=H && hph_switch_gpio=L   --> HiFi mode
 *      reset_gpio=L && hph_switch_gpio=H   --> Low Power Bypass mode
 *      reset_gpio=L && hph_switch_gpio=L   --> Standby mode(Shutdown mode)
 *      reset_gpio=H && hph_switch_gpio=H   --> LowFi mode
 */


static void es9218_power_gpio_H(void)
{
    gpio_set_value(g_es9218_priv->es9218_data->power_gpio, 1);
    pr_info("%s(): pa_gpio_level = %d\n", __func__, __gpio_get_value(g_es9218_priv->es9218_data->power_gpio));
}

static void es9218_power_gpio_L(void)
{
    gpio_set_value(g_es9218_priv->es9218_data->power_gpio, 0);
    pr_info("%s(): pa_gpio_level = %d\n", __func__, __gpio_get_value(g_es9218_priv->es9218_data->power_gpio));
}

int es9218_mode_get(void) {
	pr_info("%s(): hifi power state [%d]\n", __func__, es9218_power_state);
	return es9218_power_state;
}
EXPORT_SYMBOL(es9218_mode_get);

int es9218_hifi_exception_state_get(void) {
	pr_info("%s(): hifi exception [%d]\n", __func__, es9218_power_state);
	if(es9218_power_state != ESS_PS_BYPASS)
		g_es9218_priv->es9218_data->ess_hifi_exception = true;
	return es9218_power_state;
}
EXPORT_SYMBOL(es9218_hifi_exception_state_get);
void es9218_hifi_exception_state_put(void) {
	g_es9218_priv->es9218_data->ess_hifi_exception = false;
}
EXPORT_SYMBOL(es9218_hifi_exception_state_put);

void es9218_headset_state(int detection) {
	pr_info("%s(): headset_state = %d, detection = %d [1:Insertion 0: Removal]\n", __func__, g_es9218_priv->es9218_data->headset_state, detection);
	g_es9218_priv->es9218_data->headset_state = detection;
}
EXPORT_SYMBOL(es9218_headset_state);

void es9218_hsdet_l_switch_gpio_H(void)
{
    if(g_es9218_priv->es9218_data->switch_gpio >= 0) {
        gpio_set_value(g_es9218_priv->es9218_data->switch_gpio, 1);
        pr_info("%s(): hsdet_l_switch = %d\n", __func__, __gpio_get_value(g_es9218_priv->es9218_data->switch_gpio));
    } else {
        pr_info("%s(): No hsdet_l_switch.\n", __func__);
    }
}

void es9218_hsdet_l_switch_gpio_L(void)
{
	if(g_es9218_priv->es9218_data->switch_gpio >= 0) {
        gpio_set_value(g_es9218_priv->es9218_data->switch_gpio, 0);
        pr_info("%s(): hsdet_l_switch = %d\n", __func__, __gpio_get_value(g_es9218_priv->es9218_data->switch_gpio));
    } else {
        pr_info("%s(): No hsdet_l_switch.\n", __func__);
    }
}
EXPORT_SYMBOL(es9218_hsdet_l_switch_gpio_L);

static void es9218_reset_gpio_H(void)
{
#ifdef USE_CONTROL_EXTERNAL_LDO_FOR_DVDD
    int ret;
    ret = regulator_enable(g_es9218_priv->es9218_data->vreg_dvdd);
    pr_info("%s(): turn on an external LDO connected to DVDD.[rc=%d]\n", __func__, ret);
    msleep(1);
#endif

    gpio_set_value(g_es9218_priv->es9218_data->reset_gpio, 1);

    pr_info("%s(): pa_gpio_level = %d\n", __func__, __gpio_get_value(g_es9218_priv->es9218_data->reset_gpio));

    if(g_es9218_priv->es9218_data->headset_state) {
        mdelay(1);
        es9218_hsdet_l_switch_gpio_H();
    } else {
        pr_info("%s: Headset removal status. So, don't set to high switch pin.[%d]\n", __func__, g_es9218_priv->es9218_data->headset_state);
    }
}

static void es9218_reset_gpio_L(void)
{
#ifdef USE_CONTROL_EXTERNAL_LDO_FOR_DVDD
    int ret;
#endif

    es9218_hsdet_l_switch_gpio_L();
    mdelay(1);

    gpio_set_value(g_es9218_priv->es9218_data->reset_gpio, 0);

    pr_info("%s(): pa_gpio_level = %d\n", __func__, __gpio_get_value(g_es9218_priv->es9218_data->reset_gpio));
#ifdef USE_CONTROL_EXTERNAL_LDO_FOR_DVDD
    msleep(1);
    ret = regulator_disable(g_es9218_priv->es9218_data->vreg_dvdd);
    pr_info("%s(): turn off an external LDO connected to DVDD.[rc=%d]\n", __func__, ret);
#endif
}

static void es9218_hph_switch_gpio_H(void)
{
    gpio_set_value(g_es9218_priv->es9218_data->hph_switch, 1);
    pr_info("%s(): hph_switch = %d\n", __func__, __gpio_get_value(g_es9218_priv->es9218_data->hph_switch));
}

static void es9218_hph_switch_gpio_L(void)
{
    gpio_set_value(g_es9218_priv->es9218_data->hph_switch, 0);
    pr_info("%s(): hph_switch = %d\n", __func__, __gpio_get_value(g_es9218_priv->es9218_data->hph_switch));
}

#ifdef ES9219C
static int es9219c_sabre_pll_setting( unsigned int pll_setting ){
	//I2C Addr : 0x94
	int ret = 0;
	switch (pll_setting)
		{

		/* BLCK PLL SYNC MODE : Reg #194 ~ Reg #198 */
		case 1: // BCLK : 44.1K/48K 32bit (352.8k / 384K )
			//I2C Addr : 0x94
		   // HERE PLL setting from Reg #194 ~ Reg #198
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x81);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x02);
			break;

		case 2: // BCLK : 44.1K/48K 32bit (176.4K / 192K )
			//I2C Addr : 0x94
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x41);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x04);
			break;

		case 3: // BCLK : 44.1K/48K 32bit (88.2K / 96K )
			//I2C Addr : 0x94
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x80);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x40);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x08);
			break;

		case 4: // BCLK : 44.1K/48K 32bit (44.1K / 48K )
			//I2C Addr : 0x94
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x80);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x20);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x08);
			break;

		case 5: // BCLK : 44.1K/48K 24bit (352.8k / 384K )
			//I2C Addr : 0x94
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x61);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x02);
			break;

		case 6: // BCLK : 44.1K/48K 24bit (176.4K / 192K )
			//I2C Addr : 0x94
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x31);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x04);
			break;

		case 7: // BCLK : 44.1K/48K 24bit (88.2K / 96K )
			//I2C Addr : 0x94
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x80);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x30);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x08);
			break;

		case 8: // BCLK : 44.1K/48K 24bit (44.1K / 48K )
			//I2C Addr : 0x94
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x40);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x30);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x08);
			break;

		case 9: // BCLK : 44.1K/48K 16bit (352.8k / 384K )
			//I2C Addr : 0x94
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x41);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x02);
			break;

		case 10: // BCLK : 44.1K/48K 16bit (176.4K / 192K )
			//I2C Addr : 0x94
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x21);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x04);
			break;

		case 11: // BCLK : 44.1K/48K 16bit (88.2K / 96K )
			//I2C Addr : 0x94
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x80);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x20);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x08);
			break;

		case 12: // BCLK : 44.1K/48K 16bit (44.1K / 48K )
			//I2C Addr : 0x94
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x40);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x20);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x08);
			break;

		   default :
			   pr_err("%s() : Invalid PLL = %d \n", __func__, pll_setting);
			   break;
		}

	return ret;
}

static int es9219c_sabre_clk_source_set(unsigned int clk_source, unsigned int pll_setting )
{
	int ret = 0 ;

	if( clk_source < 2 ) {

		if( clk_source == 1) { // MCLK : 19.2MHz
		   //I2C Addr : 0x90
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_192, 0x00);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_200, 0x0C);

		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_194, 0xD3);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_195, 0x88);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_196, 0x41);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_197, 0x00);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_198, 0x02);

		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_199, 0xC4);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_201, 0x01);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_193, 0xCC);
		   mdelay(1);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_201, 0x11);

		}

	}
	else{
		   //I2C Addr must be 0x94---from
		   g_es9218_priv->i2c_client->addr = ES9219C_SLAVE_ADDR_2;

		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_192, 0x00);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_200, 0x0C);

		   // HERE PLL setting from Reg #194 ~ Reg #198
		   es9219c_sabre_pll_setting(pll_setting);

		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_199, 0xC4);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_201, 0x02);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_193, 0xCC);
		   mdelay(1);
		   ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_201, 0x12);

		   g_es9218_priv->i2c_client->addr = ES9219C_SLAVE_ADDR_1;
		   //I2C Addr must be 0x94---end
	}

    return ret;
}

#ifdef DRE_ENABLE
static int es9219c_DRE_setting( void ){
	int ret = 0;
	int imp_load = g_headset_type;

    pr_info("%s(): entry\n", __func__);

	switch(imp_load)
	{

		case 1: // Normal
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_141, normal_DRE_on_threshold[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_142, normal_DRE_on_threshold[1]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_143, normal_DRE_off_threshold[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_144, normal_DRE_off_threshold[1]);

			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_137, normal_DRE_gain_left[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_138, normal_DRE_gain_left[1]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_139, normal_DRE_gain_right[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_140, normal_DRE_gain_right[1]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_145, normal_DRE_decay_rate);
			break;

		case 2: // Advanced
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_141, advance_DRE_on_threshold[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_142, advance_DRE_on_threshold[1]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_143, advance_DRE_off_threshold[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_144, advance_DRE_off_threshold[1]);

			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_137, advance_DRE_gain_left[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_138, advance_DRE_gain_left[1]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_139, advance_DRE_gain_right[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_140, advance_DRE_gain_right[1]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_145, advance_DRE_decay_rate);
			break;


		case 3: // AUX
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_141, aux_DRE_on_threshold[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_142, aux_DRE_on_threshold[1]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_143, aux_DRE_off_threshold[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_144, aux_DRE_off_threshold[1]);

			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_137, aux_DRE_gain_left[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_138, aux_DRE_gain_left[1]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_139, aux_DRE_gain_right[0]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_140, aux_DRE_gain_right[1]);
			ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_145, aux_DRE_decay_rate);
			break;

		default :
		   pr_err("%s() : Invalid DRE set(heaset_type)  \n", __func__, g_headset_type);
		   break;
	}

	if(ret == 0 )
	{
		ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_136, 0x11); //  DRE_enable, DRE_Vol CTRL
		return ret;
	}
	else
		return ret;

}
#endif /* DRE_ENABLE */

#ifdef X_TALK_ENHANCEMENT_ENABLE //1121
static int es9219c_crosstalk_enhancement( void )
{
	int ret = 0;

	int imp_load = g_headset_type;

	if (imp_load == 1)
	{
        ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_130, crosstalk_scale_ch1[0]);
        ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_131, crosstalk_scale_ch1[1]);
        ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_132, crosstalk_scale_ch2[0]);
        ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_133, crosstalk_scale_ch2[1]);

        ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_128, 0x04); // Crostalk_enable

	}
	return ret;
}
#endif // X_TALK_ENHANCEMENT_ENABLE

#endif // ES9219c

static int es9218_master_trim(struct i2c_client *client, int vol)
{
    int ret = 0;
    u32 value;

    if (vol >= sizeof(master_trim_tbl)/sizeof(master_trim_tbl[0])) {
        pr_err("%s() : Invalid vol = %d return \n", __func__, vol);
        return 0;
    }

    value = master_trim_tbl[vol];
    pr_info("%s(): MasterTrim = %08X \n", __func__, value);

    if  (es9218_power_state == ESS_PS_IDLE) {
        pr_err("%s() : Invalid vol = %d return \n", __func__, vol);
        return 0;
    }

    ret |= es9218_write_reg(g_es9218_priv->i2c_client , ES9218P_REG_17,
                        value&0xFF);

    ret |= es9218_write_reg(g_es9218_priv->i2c_client,  ES9218P_REG_18,
                        (value&0xFF00)>>8);

    ret |= es9218_write_reg(g_es9218_priv->i2c_client,  ES9218P_REG_19,
                        (value&0xFF0000)>>16);

    ret |= es9218_write_reg(g_es9218_priv->i2c_client,  ES9218P_REG_20,
                        (value&0xFF000000)>>24);
    return ret;
}

static int es9218_set_avc_volume(struct i2c_client *client, int vol)
{
    int ret = 0;
    u8  value;

    if (vol >= sizeof(avc_vol_tbl)/sizeof(avc_vol_tbl[0])) {
        pr_err("%s() : Invalid vol = %d return \n", __func__, vol);
        return 0;
    }

    value = avc_vol_tbl[vol];

    pr_info("%s(): AVC Volume = %X \n", __func__, value);

    ret |= es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_03, value);
    return ret;
}

static int es9218_set_thd(struct i2c_client *client, int headset)
{
    int ret = 0;

    switch (headset) {
         case 1: // normal
            /*  Reg #22, #23    : THD_comp2 (-16dB) */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_22, normal_harmonic_comp_left[0]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_23, normal_harmonic_comp_left[1]);

            /*  Reg #24, #25    : THD_comp3 (-16dB) */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_24, normal_harmonic_comp_left[2]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_25, normal_harmonic_comp_left[3]);

            /*  Reg #53, #54    : THD_comp2 (-16dB) */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_53, normal_harmonic_comp_right[0]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_54, normal_harmonic_comp_right[1]);

            /*  Reg #55, #56    : THD_comp3 (-16dB) */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_55, normal_harmonic_comp_right[2]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_56, normal_harmonic_comp_right[3]);
            break;

        case 2: // advanced
            /*  Reg #22, #23    : THD_comp2 (-1dB)  */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_22, advance_harmonic_comp_left[0]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_23, advance_harmonic_comp_left[1]);

            /*  Reg #24, #25    : THD_comp3 (-1dB)  */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_24, advance_harmonic_comp_left[2]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_25, advance_harmonic_comp_left[3]);

            /*  Reg #53, #54    : THD_comp2 (-16dB) */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_53, advance_harmonic_comp_right[0]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_54, advance_harmonic_comp_right[1]);

            /*  Reg #55, #56    : THD_comp3 (-16dB) */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_55, advance_harmonic_comp_right[2]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_56, advance_harmonic_comp_right[3]);
            break;

        case 3: // aux
            /*  Reg #22, #23    : THD_comp2 (-7dB)  */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_22, aux_harmonic_comp_left[0]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_23, aux_harmonic_comp_left[1]);

            /*  Reg #24, #25    : THD_comp3 (-7dB)  */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_24, aux_harmonic_comp_left[2]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_25, aux_harmonic_comp_left[3]);

            /*  Reg #53, #54    : THD_comp2 (-16dB) */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_53, aux_harmonic_comp_right[0]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_54, aux_harmonic_comp_right[1]);

            /*  Reg #55, #56    : THD_comp3 (-16dB) */
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_55, aux_harmonic_comp_right[2]);
            ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_56, aux_harmonic_comp_right[3]);
            break;

        default :
            pr_err("%s() : Invalid headset = %d \n", __func__, headset);
            break;
    }
    pr_info("%s(): Headset Type = %d \n", __func__, headset);
    return ret;
}

static int es9218p_sabre_amp_start(struct i2c_client *client, int headset)
{
    int ret = 0;

    //NOTE  GPIO2 must already be HIGH as part of standby2lpb
#ifdef CONFIG_MACH_SDM845_JUDY
    //if(!g_dop_flag)
        es9218_hph_switch_gpio_L();
#endif

    switch(headset) {
         case 1:
            //  normal
            //
            //  Low impedance 50RZ or less headphone detected
            //  Use HiFi1 amplifier mode
            //
            pr_notice("%s() : 1 valid headset = %d changing to hifi1.\n", __func__, g_headset_type);
            es9218p_sabre_lpb2hifione();
            break;

        case 2:
            //  advanced
            //
            //  High impedance >50RZ - <600RZ headphone detected (64RZ or 300RZ for example)
            //  Use HiFi2 amplifier mode
            //
            pr_notice("%s() : 2 valid headset = %d changing to hifi2.\n", __func__, g_headset_type);
            es9218p_sabre_lpb2hifitwo();
            break;

        case 3:
            //  aux
            //
            //  High impedance >600RZ line-out detected
            //  Use HiFi1 amplifier mode
            //
            pr_notice("%s() : 3 valid headset = %d changing to hifi1.\n", __func__, g_headset_type);
            es9218p_sabre_lpb2hifione();
            break;

        default :
            pr_err("%s() : Unknown headset = %d \n", __func__, headset);
            ret = 1;
            break;
    }

#ifdef CONFIG_MACH_SDM845_JUDY
    //if(!g_dop_flag)
        es9218_hph_switch_gpio_H();
#endif /* CONFIG_MACH_SDM845_JUDY */

    return ret;
}

static int es9218p_sabre_amp_stop(struct i2c_client *client, int headset)
{
    int ret = 0;

    switch(headset) {
         case 1:
            //  normal
            //
            //  Low impedance 32RZ or less headphone detected
            //  Use HiFi1 amplifier mode
            //
            pr_notice("%s() : 1 valid headset = %d changing to lbp.\n", __func__, g_headset_type);
            es9218p_sabre_hifione2lpb();
            break;

        case 2:
            //  advanced
            //
            //  High impedance >32RZ - <600RZ headphone detected (64RZ or 300RZ for example)
            //  Use HiFi2 amplifier mode
            //
            pr_notice("%s() : 2 valid headset = %d changing to lbp.\n", __func__, g_headset_type);
            es9218p_sabre_hifitwo2lpb();
            break;

        case 3:
            //  aux
            //
            //  High impedance >600RZ line-out detected
            //  Use HiFi1 amplifier mode
            //
            pr_notice("%s() : 3 valid headset = %d changing to lbp.\n", __func__, g_headset_type);
            es9218p_sabre_hifione2lpb();
            break;

        default :
            pr_err("%s() : Invalid headset = %d \n", __func__, g_headset_type);
            ret = 1;
            break;
    }

    return ret;
}


/*
 *  Program stage1 and stage2 filter coefficients
 */
static int es9218_sabre_cfg_custom_filter(struct sabre_custom_filter *sabre_filter)
{
    int rc, i, *coeff;
    int count_stage1;
    u8  rv, reg;

    reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07);
    pr_info("%s(): g_sabre_cf_num = %d, ES9218P_REG_07:%x \n", __func__, g_sabre_cf_num, reg);
    reg &= ~0xE0;

    if (g_sabre_cf_num > 3) {
        if(es9218_rate == 384000)
           rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_44, 0x01);
        else
           rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_44, 0x00);

        switch (g_sabre_cf_num) {
            case 4:
            // 3b000: linear phase fast roll-off filter
                rv = reg|0x00;
                rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, rv);
                break;
            case 5:
            // 3b001: linear phase slow roll-off filter
                rv = reg|0x20;
                rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, rv);
                break;
            case 6:
            // 3b010: minimum phase fast roll-off filter #1
                rv = reg|0x40;
                rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, rv);
                break;
            case 7:
            // 3b011: minimum phase slow roll-off filter
                rv = reg|0x60;
                rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, rv);
                break;
            case 8:
            /* 3b100: apodizing fast roll-off filter type 1 (default)
               change into Minimum Phase Fast roll-off for response of sample rate except mqa*/
                rv = reg|0x40;
                rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, rv);
                break;
            case 9:
            /* 3b101: apodizing fast roll-off filter type 2
               change into Minimum Phase Fast roll-off for response of sample rate */
                rv = reg|0x40;
                rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, rv);
                break;
            case 10:
            // 3b110: hybrid fast roll-off filter
                rv = reg|0xC0;
                rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, rv);
                break;
            case 11:
            // 3b111: brick wall filter
                rv = reg|0xE0;
                rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, rv);
                break;
            default:
                pr_info("%s(): default = %d \n", __func__, g_sabre_cf_num);
                break;
        }
        return rc;
    }
    count_stage1 = sizeof(sabre_filter->stage1_coeff)/sizeof(sabre_filter->stage1_coeff[0]);

    pr_info("%s: count_stage1 : %d",__func__,count_stage1);

    rv = (sabre_filter->symmetry << 2) | 0x02;        // set the write enable bit
    rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_44, rv);
    if (rc < 0) {
        pr_err("%s: rc = %d return ",__func__, rc);
        return rc;
    }

    rv = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_40 - 1);
    coeff = sabre_filter->stage1_coeff;
    for (i = 0; i < count_stage1 ; i++) { // Program Stage 1 filter coefficients
        u8 value[4];
        value[0] =  i;
        value[1] = (*coeff & 0xff);
        value[2] = ((*coeff>>8) & 0xff);
        value[3] = ((*coeff>>16) & 0xff);
        i2c_smbus_write_block_data(g_es9218_priv->i2c_client, ES9218P_REG_40 - 1, 4, value);
        coeff++;
    }
    coeff = sabre_filter->stage2_coeff;
    for (i = 0; i < 16; i++) { // Program Stage 2 filter coefficients
        u8 value[4];
        value[0] =  128 + i;
        value[1] = (*coeff & 0xff);
        value[2] = ((*coeff>>8) & 0xff);
        value[3] = ((*coeff>>16) & 0xff);
        i2c_smbus_write_block_data(g_es9218_priv->i2c_client, ES9218P_REG_40 - 1, 4, value);
        coeff++;
    }
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_40 - 1, rv);

    rv = (sabre_filter->shape << 5); // select the custom filter roll-off shape
    rv |= 0x80;
    rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, rv);
    if (rc < 0) {
        pr_err("%s: rc = %d return ",__func__, rc);
        return rc;
    }
    rv = (sabre_filter->symmetry << 2); // disable the write enable bit
    rv |= 0x1; // Use the custom oversampling filter.
    rc = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_44, rv);
    if (rc < 0) {
        pr_err("%s: rc = %d return ",__func__, rc);
        return rc;
    }
    return 0;
}

//Version 1.2 - December 15, 2016
//
// changes:
// corrected some differences between ess kelowna functions and ess korea implementations
// added lpb2hifitwo option1
// minor changes to hifitwo2lpb and hifione2lpb

static int  es9218p_sabre_lpb2hifione(void)
{
    //declare register starting point so we can use incremental OR / AND+1C instead of hex literals
	// x |= y; //set bits in x which are 1's in y
	// x &= ~y; //clear bits in x which are 1's in y
    int value = 0;
    int register_45_value = 0;
    int register_46_value = 0;
    int register_47_value = 0;
    int register_48_value = 0;

#ifdef ES9219C
    if (g_es9218_priv->es9218_data->is_es9219c) {
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_13, 0x00);// Enable THD comp
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_14, 0x25);// set Soft start type and time
        if(g_headset_type == 1){ // updated_ESS_1122
			es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_21, 0x09); //ESS_0103 ABG tune 2.8-->2.5V  // ABG tune 1.4 / 2.8V
        }
        else
        {
			es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_21, 0x05);// ABG tune HiFi 1 1.4V
        }
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_06, 0x47);// set vol rate to fast
    }
#endif

    pr_info("%s(): entry: state = %s\n", __func__, power_state[es9218_power_state]);
#ifdef USE_HPAHiQ   // Reg#48 = 0x0F => 2mA more and THD 2dB, Reg#48 = 0x07 => nornal mode
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_48, register_48_value = 0x0F);//HPAHiQ = 1, EN_SEPARATE_THD_COMP = 1, STATE3_CTRL_SEL = 11 for minimum state-machine delay time
#else
	es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_48, register_48_value = 0x07);//HPAHiQ = 0, EN_SEPARATE_THD_COMP = 1, STATE3_CTRL_SEL = 11 for minimum state-machine delay time
#endif
#ifdef ES9218P_DEBUG
    pr_info("%s(): R48 = %X \n", __func__, register_48_value);
    mdelay(g_debug_delay);
#endif

    if( g_es9218_priv->es9218_data->use_internal_ldo ) {
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value |= 0x80); // enable overrides, amp input shunt and output shunt both engaged
    } else {
        register_46_value |= 0x80; // enable overrides, amp input shunt and output shunt both engaged
        register_46_value |= 0x04; // SEL1V = 1, use external LDO.
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value); // enable overrides, amp input shunt and output shunt both engaged
    }

#ifdef ES9218P_DEBUG
    pr_info("%s(): R46 = %X \n", __func__, register_46_value);
    mdelay(g_debug_delay);
#endif

#ifdef ES9219C // updated_ESS_1122
    if (g_es9218_priv->es9218_data->is_es9219c) {
		if(g_headset_type == 1){
	   		 es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_32, 0x02); //ESS_0103 sets to HiFi1 // AMP_PDB_SS = 0, AMP_MODE - HiFi2 //updated_ESS_1121
		} else {
			 es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_32, 0x02); // AMP_PDB_SS = 0, AMP_MODE - HiFi1 //updated_ESS_1121
		}
    } else {
		es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_32, 0x02); // AMP_PDB_SS = 0, AMP_MODE - HiFi1 //updated_ESS_1121
    }
#else
		es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_32, 0x02); // AMP_PDB_SS = 0, AMP_MODE - HiFi1
#endif

#ifdef ES9218P_DEBUG
    pr_info("%s(): R32 = %X \n", __func__, 0x02);
    mdelay(g_debug_delay);
#endif

#ifdef ES9219C
    if (g_es9218_priv->es9218_data->is_es9219c) {
        //Do nothing
    }
    else{
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_03, 0x18); // ATC to min
    }
#else
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_03, 0x18); // ATC to min
#endif

#ifdef ES9218P_DEBUG
    pr_info("%s(): R03 = %X \n", __func__, 0x18);
    mdelay(g_debug_delay);
#endif

    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_47, register_47_value |= 0x08); // CPL_WEAK = 1 preset low voltage chargepump for weak mode
#ifdef ES9218P_DEBUG
    pr_info("%s(): R47 = %X \n", __func__, register_47_value);
    mdelay(g_debug_delay);
#endif

#ifdef ES9219C
    if (g_es9218_priv->es9218_data->is_es9219c) {
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_45, register_45_value |= 0x0C); // CPH_WEAK = 1 preset high voltage chargepump for weak mode & APDB =1
    }
    else{
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_45, register_45_value |= 0x08); // CPH_WEAK = 1 preset high voltage chargepump for weak mode
    }
#else
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_45, register_45_value |= 0x08); // CPH_WEAK = 1 preset high voltage chargepump for weak mode
#endif

#ifdef ES9218P_DEBUG
    pr_info("%s(): R45 = %X \n", __func__, register_45_value);
    mdelay(g_debug_delay);
#endif

    //0x68
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_47, register_47_value |= 0x60); // ENCP_OE = 1, ENAUX_OE = 1 enable override control of AUX switch
#ifdef ES9218P_DEBUG
    pr_info("%s(): R47 = %X \n", __func__, register_47_value);
    mdelay(g_debug_delay);
#endif

#ifdef ES9219C //updated_ESS_1121
	mdelay(5);
#endif

    //0x1c
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_45, register_45_value |= 0x14); // APDB = 1, CPH_strong = 1 set high voltage chargepump for strong mode
#ifdef ES9218P_DEBUG
    pr_info("%s(): R45 = %X \n", __func__, register_45_value);
    mdelay(g_debug_delay);
#endif

    //0x78
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_47, register_47_value |= 0x10); // CPL_strong = 1 set low voltage chargepump for strong mode
#ifdef ES9218P_DEBUG
    pr_info("%s(): R47 = %X \n", __func__, register_47_value);
    mdelay(g_debug_delay);
#endif

    mdelay(5); // required to allow Vref (APDB) voltage to settle before enabling AVDD_DAC regulator (AREG_PDB)

#ifdef ES9219C
    if (g_es9218_priv->es9218_data->is_es9219c) {
		// 0x7a
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_47, register_47_value |= 0x02); // Enable ENSM PS
    	mdelay(1); // updated_ESS_1121
	}
#endif

    //0x7c
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_45, register_45_value |= 0x60); // ENHPA = 1, AREG_PDB = 1 enable internal AVCC_DAC regulator
#ifdef ES9219C // updated_ESS_1121
	mdelay(1);
#endif
#ifdef ES9218P_DEBUG
    pr_info("%s(): R45 = %X \n", __func__, register_45_value);
    mdelay(g_debug_delay);
#endif

#ifdef ES9219C
    if (g_es9218_priv->es9218_data->is_es9219c) {
		//0x83
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value |= 0x03); // SHTINB = 1 SHTOUTB =1 disengage amplifier input shunt
		mdelay(1); //updated_ESS_1121
    }
    else{
        //0x81
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value |= 0x01); // SHTINB = 1 disengage amplifier input shunt
    }
#else
    //0x81
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value |= 0x01); // SHTINB = 1 disengage amplifier input shunt
#endif

#ifdef ES9218P_DEBUG
    pr_info("%s(): R46 = %X \n", __func__, register_46_value);
    mdelay(g_debug_delay);
#endif

#ifdef ES9219C
    if (g_es9218_priv->es9218_data->is_es9219c) {
		//0x47
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_48, register_48_value |= 0x40); // ENHPA_OUT = 1 enable amplifier output stage
        mdelay(20); // updated_ESS_1121 5m -> 20m
    }
    else{
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_48, register_48_value |= 0x40); // ENHPA_OUT = 1 enable amplifier output stage
    }
#else
    //0x47
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_48, register_48_value |= 0x40); // ENHPA_OUT = 1 enable amplifier output stage
#endif

#ifdef ES9218P_DEBUG
    pr_info("%s(): R48 = %X \n", __func__, register_48_value);
    mdelay(g_debug_delay);
#endif

    value = avc_vol_tbl[g_avc_volume];

    pr_info("%s(): AVC Volume = %X \n", __func__, value);

    //WARNING register 03 is also programmed in bypass2hifi.  Beware of conflicts.
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_03, value); // set ATC to original level
#ifdef ES9218P_DEBUG
    pr_info("%s(): R03 = %X \n", __func__, value);
    mdelay(g_debug_delay);
#endif


#ifdef ES9219C // updataed_1018
    if (g_es9218_priv->es9218_data->is_es9219c) {
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_14,  0xA5); // sfot start = 1
        es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_135,  0x17); // FB amp on + ESS_0103 enable jdet comparator
    }
#endif

    if( g_es9218_priv->es9218_data->use_internal_ldo ) {
        if (g_es9218_priv->es9218_data->is_es9219c) {
#if 0	//ESS_0103
            es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value |= 0x01); // SHTINB = 1
            mdelay(5);
            es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value |= 0x02); // SHTOUTB = 1
            mdelay(5);
            es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value &= ~0x80); // Disable overrides
            mdelay(5);
#endif //ESS_0103
        } else {
            es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value = 0x03); // SHTOUTB = 1, disable overrides.  Register 32 will take over control and hold sabre in HiFi1
        }
    } else {
        register_46_value = 0x03; // SHTOUTB = 1, disable overrides.  Register 32 will take over control and hold sabre in HiFi1
        register_46_value |= 0x04; //use external LDO, set SEL1V bit.
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value);
    }
#ifdef ES9218P_DEBUG
    pr_info("%s(): R46 = %X \n", __func__, register_46_value);
    mdelay(g_debug_delay);
#endif

    //////////////////////////////
    // Sabre is now in HiFi1 mode.
	// Override bits are not cleared but they are disabled.
    //////////////////////////////
    return  0;
}

static int  es9218p_sabre_lpb2hifitwo(void)
{
    //declare register starting point so we can use incremental OR / AND+1C instead of hex literals
    // x |= y; //set bits in x which are 1's in y
    // x &= ~y; //clear bits in x which are 1's in y
    int value = 0;
    int register_45_value = 0;
    int register_46_value = 0;
    int register_47_value = 0;
    int register_48_value = 0;

    pr_info("%s(): entry: state = %s\n", __func__, power_state[es9218_power_state]);

#ifdef USE_HPAHiQ   // Reg#48 = 0x0F => 2mA more and THD 2dB, Reg#48 = 0x07 => nornal mode
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_48, register_48_value = 0x0F);//HPAHiQ = 1, EN_SEPARATE_THD_COMP = 1, STATE3_CTRL_SEL = 11 for minimum state-machine delay time
#else
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_48, register_48_value = 0x07);//HPAHiQ = 0, EN_SEPARATE_THD_COMP = 1, STATE3_CTRL_SEL = 11 for minimum state-machine delay time
#endif
#ifdef ES9218P_DEBUG
    pr_info("%s(): R48 = %X \n", __func__, register_48_value);
    mdelay(g_debug_delay);
#endif

    if( g_es9218_priv->es9218_data->use_internal_ldo ) {
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value |= 0x80); // enable overrides, amp input shunt and output shunt both engaged
    } else {
        register_46_value |= 0x80; // enable overrides, amp input shunt and output shunt both engaged
        register_46_value |= 0x04; //use external LDO, set SEL1V bit.
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value);
    }

#ifdef ES9218P_DEBUG
    pr_info("%s(): R46 = %X \n", __func__, register_46_value);
    mdelay(g_debug_delay);
#endif

    //
    // This block is different for HiFi2
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_32, 0x03); // AMP_PDB_SS = 0, AMP_MODE - HiFi2
#ifdef ES9218P_DEBUG
    pr_info("%s(): R32 = %X \n", __func__, 0x03);
    mdelay(g_debug_delay);
#endif
    // This block is different for HiFi2
    //

    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_03, 0x18); // ATC to min
#ifdef ES9218P_DEBUG
    pr_info("%s(): R03 = %X \n", __func__, 0x18);
    mdelay(g_debug_delay);
#endif

    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_47, register_47_value |= 0x08); // CPL_WEAK = 1 preset low voltage chargepump for weak mode
#ifdef ES9218P_DEBUG
    pr_info("%s(): R47 = %X \n", __func__, register_47_value);
    mdelay(g_debug_delay);
#endif

    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_45, register_45_value |= 0x08); // CPH_WEAK = 1 preset high voltage chargepump for weak mode
#ifdef ES9218P_DEBUG
    pr_info("%s(): R45 = %X \n", __func__, register_45_value);
    mdelay(g_debug_delay);
#endif

    //0x68
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_47, register_47_value |= 0x60); // ENCP_OE = 1, ENAUX_OE = 1 enable override control of AUX switch
#ifdef ES9218P_DEBUG
    pr_info("%s(): R47 = %X \n", __func__, register_47_value);
    mdelay(g_debug_delay);
#endif

    //0x1c
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_45, register_45_value |= 0x14); // APDB = 1, CPH_strong = 1 set high voltage chargepump for strong mode
#ifdef ES9218P_DEBUG
    pr_info("%s(): R45 = %X \n", __func__, register_45_value);
    mdelay(g_debug_delay);
#endif

    //0x78
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_47, register_47_value |= 0x10); // CPL_strong = 1 set low voltage chargepump for strong mode
#ifdef ES9218P_DEBUG
    pr_info("%s(): R47 = %X \n", __func__, register_47_value);
    mdelay(g_debug_delay);
#endif

    mdelay(5); // required to allow Vref (APDB) voltage to settle before enabling AVDD_DAC regulator (AREG_PDB)

    //0x7c
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_45, register_45_value |= 0x60); // ENHPA = 1, AREG_PDB = 1 enable internal AVCC_DAC regulator
#ifdef ES9218P_DEBUG
    pr_info("%s(): R45 = %X \n", __func__, register_45_value);
    mdelay(g_debug_delay);
#endif

    //0x81
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value |= 0x01); // SHTINB = 1 disengage amplifier input shunt
#ifdef ES9218P_DEBUG
    pr_info("%s(): R46 = %X \n", __func__, register_46_value);
    mdelay(g_debug_delay);
#endif

    //0x4F
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_48, register_48_value |= 0x40); // ENHPA_OUT = 1 enable amplifier output stage
#ifdef ES9218P_DEBUG
    pr_info("%s(): R48 = %X \n", __func__, register_48_value);
    mdelay(g_debug_delay);
#endif

    //
    // This block is extra for HiFi2
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value &= ~0x01); // SHTINB = 0 engage input shunt
#ifdef ES9218P_DEBUG
    pr_info("%s(): R46 = %X \n", __func__, register_46_value);
    mdelay(g_debug_delay);
#endif

    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_47, register_47_value |= 0x05); // SEL3V3 = 1, SEL3V3_PS = 1 set the amplifier power switch for HiFi2 mode
#ifdef ES9218P_DEBUG
    pr_info("%s(): R46 = %X \n", __func__, register_47_value);
    mdelay(g_debug_delay);
#endif

    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value |= 0x01); // SHTINB = 1 disengage input shunt
#ifdef ES9218P_DEBUG
    pr_info("%s(): R46 = %X \n", __func__, register_46_value);
    mdelay(g_debug_delay);
#endif

    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_47, register_47_value |= 0x02); // ENSMPS = 1 change the power switch to strong mode
#ifdef ES9218P_DEBUG
    pr_info("%s(): R46 = %X \n", __func__, register_47_value);
    mdelay(g_debug_delay);
#endif
    // This block is extra for HiFi2
    //

    value = avc_vol_tbl[g_avc_volume];
    pr_info("%s(): AVC Volume = %X \n", __func__, value);

    //WARNING register 03 is also programmed in bypass2hifi.  Beware of conflicts.
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_03, value); // set ATC to original level
#ifdef ES9218P_DEBUG
    pr_info("%s(): R03 = %X \n", __func__, value);
    mdelay(g_debug_delay);
#endif

    if( g_es9218_priv->es9218_data->use_internal_ldo ) {
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value = 0x03); // SHTOUTB = 1, disable overrides, amp input shunt and output shunt both disengaged
    } else {
        register_46_value = 0x03; // SHTOUTB = 1, disable overrides.  Register 32 will take over control and hold sabre in HiFi2.
        register_46_value |= 0x04; // use external LDO, set SEL1V bit.
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, register_46_value);
    }

#ifdef ES9218P_DEBUG
    pr_info("%s(): R46 = %X \n", __func__, register_46_value);
    mdelay(g_debug_delay);
#endif
    //////////////////////////////
    // Sabre is now in HiFi2 mode.
    // Override bits are not cleared but they are disabled.
    //////////////////////////////
    return  0;
}

static int  es9218p_standby2lpb(void)
{
    /////////////////////////////////////////////////////////////////////////////////
    // ESS recommended sequence for transition from Standby to Low Power Bypass (LPB)
	/////////////////////////////////////////////////////////////////////////////////

    pr_info("%s(): entry: state = %s\n", __func__, power_state[es9218_power_state]);

	es9218_hph_switch_gpio_H(); // GPIO2 HIGH to activate LPB mode

    return  0;
}


static int  es9218p_lpb2standby(void)
{
    es9218_hph_switch_gpio_L(); //GPIO2 low to move into standby mode
    mdelay(10); // lots of extra time for chargepumps to completely discharge before power off, may not be necessary.  Tune lower as needed.
    return  0;
}

static int  es9218p_sabre_hifione2lpb(void)
{
    pr_info("%s()\n", __func__);

    // if hifi to lpb pops, set R7[0] = 1 manually trigger mute before change amp mode
    // int rv = 0;
    // es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07); //WARNING this register read will slow down the transition! Would be faster to know what R7 value is already.
    // es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, rv |= 0x01);
    //

#ifdef ES9219C

    if (g_es9218_priv->es9218_data->is_es9219c) {
#if 1 //ESS_0103
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_14, 0x25); // soft start disable (ramp down)
        //  mdelay(50);
        //	mdelay(20);
        mdelay(40);

        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_02, 0x34);
        es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_135, 0x10);  // disable ATR
        mdelay(2);

        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, 0x81);  //engage output shunt
        mdelay(2);

        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_48, 0x07);  //disable enhpa out
        mdelay(3);

        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_32, 0x00);  // switch to CoreOn

        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, 0x81);  //engage output shunt

        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, 0x80);  //engage input shunt
        mdelay(2);

        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_32, 0x01);  // switch to LowFi
        mdelay(5);

#else // ESS_0103
//ESS_1213_2
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_15, 0xff);      // Digital volume down
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_16, 0xff);      // Digital volume down
        mdelay(5);
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_14, 0x00);      // soft start = 0

        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_03, 0x18);      // Set AVC to min

        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_48, 0x40);      // ENHPA_OUT = 0 disable amplifier output stage
        mdelay(5);
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_32, 0x00);      // AMP_PDB_SS = 0, AMP_MODE - Core On

        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, 0x00);      // Disable overries
        mdelay(10);

        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_21, 0x0D);      // ABG Tune
        es9218_write_reg(g_es9218_priv->i2c_client, ES9219C_REG_135, 0x10);     // FB
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_45, 0x00);
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_46, 0x00);
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_47, 0x00);
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_48, 0x01);
//ESS_1213_2
#endif // ESS_0103
    }
    else{
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_32, 0x00);      // amp mode core on, amp mode gpio set to trigger Core On
    }
#else
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_32, 0x00);      // amp mode core on, amp mode gpio set to trigger Core On
#endif

    ///////////////////////////////
    // sabre is now in LowFi mode controlled by GPIO2
    ///////////////////////////////

    if(g_dop_flag) {
        pr_debug("%s: add 300ms delay before reset gpio set for dop playback case.\n",__func__);
        msleep(300);
    }  else {
        msleep(100);
   }

    es9218_reset_gpio_L();  // RESETb LOW move to Low Power Bypass mode

    if (g_es9218_priv->es9218_data->is_es9219c && clk_source == 1) {
        pr_info("%s() hifi_ext_mclk disable\n", __func__);
        clk_disable_unprepare(hifi_ext_mclk);
    }

    ///////////////////////////////
    // sabre is now in low power bypass mode
    ///////////////////////////////

    //no longer required, lpb2hifione does not change hph_switch_gpio state
    //es9218p_standby2lpb();  // Quiet transition from Standby mode to Low Power Bypass mode

    return  0;
}


static int  es9218p_sabre_hifitwo2lpb(void)
{
    pr_info("%s()\n", __func__);

    // if hifi to lpb pops, set R7[0] = 1 manually trigger mute before change amp mode
    // u8 rv = 0;
    // rv = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07); //WARNING this register read will slow down the transition! Would be faster to know what R7 value is already.
    // es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, rv |= 0x01);
    //
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_32, 0x00);      // amp mode core on, amp mode gpio set to trigger Core On

    ///////////////////////////////
    // sabre is now in LowFi mode controlled by GPIO2
    ///////////////////////////////

    if(g_dop_flag) {
        pr_debug("%s: add 300ms delay before reset gpio set for dop playback case.\n",__func__);
        msleep(300);
    }
    es9218_reset_gpio_L();  // RESETb LOW move to Low Power Bypass mode

    if (g_es9218_priv->es9218_data->is_es9219c && clk_source == 1) {
        pr_info("%s() hifi_ext_mclk disable\n", __func__);
        clk_disable_unprepare(hifi_ext_mclk);
    }

    ///////////////////////////////
    // sabre is now in low power bypass mode
    ///////////////////////////////

    //no longer required, lpb2hifione does not change hph_switch_gpio state
    //es9218p_standby2lpb();  // Quiet transition from Standby mode to Low Power Bypass mode

    return  0;
}

static int es9218p_sabre_bypass2hifi(void)
{
    u8 reg;
    int ret = 0;

    pr_info("%s() : enter. state = %s\n", __func__, power_state[es9218_power_state]);

    if ( es9218_power_state != ESS_PS_BYPASS ) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }

    if (g_es9218_priv->es9218_data->is_es9219c && clk_source == 1) {
        pr_info("%s() hifi_ext_mclk enable\n", __func__);
        ret = clk_prepare_enable(hifi_ext_mclk);
        if (ret)
            pr_err("%s() : ext clk enable failed\n", __func__);
    }

#ifdef WORKAROUND_FOR_CORNER_SAMPLES
    es9218_reset_gpio_H();
    mdelay(1);

    /*
     * workaround 1 : set RESETB high twice
     * improvement for auto power on sequence inside ES921P chip
     * it's expected to prevent i2c failure
     */
    es9218_reset_gpio_L();
    mdelay(1);
    es9218_reset_gpio_H();
    mdelay(2);

    /*
     * workaround 2 : send a I2C cmd of soft_reset
     * improvement for auto power on sequence inside ES921P chip
     * it's expected to read default values of registers WELL
     */
#ifdef ES9219C

    if (g_es9218_priv->es9218_data->is_es9219c) {
        if(clk_source > 1){

            // I2S BCLK should be driven before executing the following function
            es9219c_sabre_clk_source_set(clk_source, pll_setting);

            i2c_smbus_write_byte_data(g_es9218_priv->i2c_client, ES9218P_REG_00, 0x01);
            mdelay(1);

        }
        else{
            i2c_smbus_write_byte_data(g_es9218_priv->i2c_client, ES9218P_REG_00, 0x01);
            mdelay(1);

            es9219c_sabre_clk_source_set(clk_source, pll_setting);
        }
    }
    else{
        i2c_smbus_write_byte_data(g_es9218_priv->i2c_client, ES9218P_REG_00, 0x01);
        mdelay(1);
    }
#else /* ES9218P */
    i2c_smbus_write_byte_data(g_es9218_priv->i2c_client, ES9218P_REG_00, 0x01);
    mdelay(1);
#endif /* ES9219C */

#else /* Original code. Finally, we MUST use code below if ESS confirms that chips have no problems */
    es9218_reset_gpio_H();
    mdelay(2);
#endif /* WORKAROUND_FOR_CORNER_SAMPLES */

    ///////////////////////////////////////////////////////
	// sabre is now in LowFi mode  (RESETb HIGH, GPIO2 LOW)
	///////////////////////////////////////////////////////

    if( call_common_init_registers == 1 ) {
        call_common_init_registers = 0;

        es9218p_initialize_registers(ESS_MODE_INIT);
        pr_info("%s(): call es9218_common_init_registers.\n", __func__);
    }

    if (g_dop_flag == 0) { //  PCM
        pr_info("%s(): PCM Format Reg Initial in es9218p_sabre_bypass2hifi() \n", __func__);
        // re-init. PCM reg.
        es9218p_initialize_registers(ESS_MODE_PCM);

        // set bit width to ESS
        es9218p_set_bit_width(es9218_bps, ESS_MODE_PCM);
    }
    else if (g_dop_flag > 0) { //  DOP
        pr_info("%s(): DOP Format Reg Initial in es9218p_sabre_bypass2hifi() \n", __func__);

#ifdef ENABLE_DOP_SOFT_MUTE
        reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07);
        reg = reg|0x01;
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, reg);
        mdelay(2);
#endif /* ENABLE_DOP_SOFT_MUTE */

        // re-init. DoP reg.
        es9218p_initialize_registers(ESS_MODE_DoP);

        // set bit width to ESS
        es9218p_set_bit_width(g_dop_flag, ESS_MODE_DoP);
    }

#ifdef DRE_ENABLE
     if (g_es9218_priv->es9218_data->is_es9219c) {
         es9219c_DRE_setting();
     }
#endif

    es9218_set_thd(g_es9218_priv->i2c_client, g_headset_type);
#if 0 /* CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER*/
    es9218_sabre_cfg_custom_filter(&es9218_sabre_custom_ft[g_sabre_cf_num]);
#endif
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_15, g_left_volume);     // set left channel digital volume level
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_16, g_right_volume);    // set right channel digital volume level

    pr_info("%s() : g_left_volume = %d, g_right_volume = %d \n", __func__, g_left_volume, g_right_volume);

    es9218_master_trim(g_es9218_priv->i2c_client, g_volume);                        // set master trim level
#ifdef ES9219C
#ifdef X_TALK_ENHANCEMENT_ENABLE
	es9219c_crosstalk_enhancement();
#endif
	if (g_es9218_priv->es9218_data->is_es9219c) {
	    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_03, 0x58);  // AVC to Min
	} else {
	    es9218_set_avc_volume(g_es9218_priv->i2c_client, g_avc_volume);                 // set analog volume control, must happen before amp start // 1113
	}
#else
	es9218_set_avc_volume(g_es9218_priv->i2c_client, g_avc_volume);                 // set analog volume control, must happen before amp start // 1113
#endif

    es9218p_sabre_amp_start(g_es9218_priv->i2c_client, g_headset_type);             // move to HiFi mode

    es9218_power_state = ESS_PS_HIFI;

    pr_info("%s() : exit. state = %s\n", __func__, power_state[es9218_power_state]);

    return 0;
}

static int es9218p_sabre_hifi2lpb(void)
{
    if ( es9218_power_state < ESS_PS_HIFI ) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }
    pr_info("%s() : state = %s\n", __func__, power_state[es9218_power_state]);

    es9218p_sabre_amp_stop(g_es9218_priv->i2c_client, g_headset_type);              // moves from either HiFi1 or HiFi2 to Low Power Bypass Mode

    es9218_power_state = ESS_PS_BYPASS;

    return 0;
}

/*
 *  HiFi mode, playback is stopped or paused
 */
static int es9218_sabre_audio_idle(void)
{
    if ( es9218_power_state != ESS_PS_HIFI ) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }
    pr_info("%s() : state = %s\n", __func__, power_state[es9218_power_state]);
    /*  Auto Mute disable   */
    //es9218_write_reg(g_es9218_priv->i2c_client, ESS9218_02, 0x34);

    es9218_power_state = ESS_PS_IDLE;
    return 0;
}

static int es9218_sabre_audio_active(void)
{
    if ( es9218_power_state != ESS_PS_IDLE ) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }
    pr_info("%s() : state = %s\n", __func__, power_state[es9218_power_state]);

    es9218_power_state = ESS_PS_HIFI;
    return 0;
}

/*
 *  Power up to bypass mode when headphone is plugged in
 *  Sabre DAC (ES9218) is still in power-down mode, but switch is on and sw position is at AUX/QC_CODEC.
 *  Thus, i2c link will not function as HiFi_RESET_N is still '0'.
 */
static int __es9218_sabre_headphone_on(void)
{
    pr_info("%s(): entry: state = %s\n", __func__, power_state[es9218_power_state]);

    if (es9218_power_state == ESS_PS_CLOSE) {
		// If sabre has been powered off, power it on
        es9218_hph_switch_gpio_L();     // GPIO2 LOW - ESS recommended for quiet power on
		es9218_reset_gpio_L();          //confirm RESETb LOW - ESS recommended for quiet power on
        es9218_power_gpio_H();          // Power on, GPIO2
        // Verify power on sequence?

        //////////////////////////////////////////
		//sabre is now powered on in Standby mode.
		//////////////////////////////////////////

        es9218p_standby2lpb();

        es9218_power_state = ESS_PS_BYPASS;
        return 0;
    } else if (es9218_power_state == ESS_PS_BYPASS && es9218_is_amp_on) {
		// if sabre is already powered on and waiting in LPB mode, transition from LPB to HiFi depending on load detected
        pr_info("%s() : state = %s , is_amp_on = %d \n",    __func__, power_state[es9218_power_state], es9218_is_amp_on);
        // defesive code for usecase not calling es9218_startup
        // calling es9218_startup depends on kinds of sound. for example,
        // hifi has 1sec of standby time, after that es9218_startup is invoked.
        // offload mp3 has 10sec of standby time, after that es9218_startup is invoked.
        call_common_init_registers = 1;

        cancel_delayed_work_sync(&g_es9218_priv->sleep_work);
#if defined(CONFIG_ARCH_SM8150)
        if(pm_qos_request_active(&req)) {
            pr_info("%s(): pm qos active state. so, remove pm qos request", __func__);
            pm_qos_remove_request(&req);
        }
#endif
        // guanrantee engough time to check impedance of headphone before entering to hifi mode, which means
        // that es9218p_sabre_bypass2hifi() is invoked after some delay like 250ms.
        schedule_delayed_work(&g_es9218_priv->hifi_in_standby_work, msecs_to_jiffies(250));
        pr_info("%s(): end calling es9218p_sabre_bypass2hifi() after 250ms \n", __func__);
        return 1;
    } else {
        pr_err("%s() : state = %s , skip enabling EDO.\n",  __func__, power_state[es9218_power_state]);
        return 0;
    }

    pr_info("%s(): end \n", __func__);

    return 0;
}

/*
 *  Power down when headphone is plugged out. This state is the same as system power-up state.
 */
static int __es9218_sabre_headphone_off(void)
{
    if ( es9218_power_state == ESS_PS_CLOSE) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }

    cancel_delayed_work_sync(&g_es9218_priv->hifi_in_standby_work);

#ifdef ES9219C
    if (g_es9218_priv->es9218_data->is_es9219c){
        es9218_reset_gpio_L();
    }
    else{
        if ( es9218_power_state != ESS_PS_BYPASS ||
            es9218_power_state != ESS_PS_IDLE) {
            es9218p_sabre_hifi2lpb(); // if power state indicates chip is in HiFi mode, move to Low Power Bypass
        }
    }
#else
    if ( es9218_power_state != ESS_PS_BYPASS ||
        es9218_power_state != ESS_PS_IDLE) {
        es9218p_sabre_hifi2lpb(); // if power state indicates chip is in HiFi mode, move to Low Power Bypass
    }
#endif

    pr_info("%s() : state = %d\n", __func__, es9218_power_state);

    es9218p_lpb2standby();

    // power off when ess chis becomes shutdown
    if( g_es9218_priv->es9218_data->always_power_on == false )
        es9218_power_gpio_L();

    // set reset pin Low if reset gpio is high
    if( __gpio_get_value(g_es9218_priv->es9218_data->reset_gpio) == 1 )
        es9218_reset_gpio_L();

    es9218_power_state = ESS_PS_CLOSE;
    return 0;
}

/*
 *  Power up to bypass mode when headphone is plugged in
 *  Sabre DAC (ES9018) is still in power-down mode, but switch is on and sw position is at AUX/QC_CODEC.
 *  Thus, i2c link will not function as HiFi_RESET_N is still '0'.
 */
int es9218_sabre_headphone_on(void)
{
    pr_info("%s() Called !! \n", __func__);

    mutex_lock(&g_es9218_priv->power_lock);
    __es9218_sabre_headphone_on();
    mutex_unlock(&g_es9218_priv->power_lock);
    return 0;
}
EXPORT_SYMBOL(es9218_sabre_headphone_on);

/*
 *  Power down when headphone is plugged out. This state is the same as system power-up state.
 */
int es9218_sabre_headphone_off(void)
{
    pr_info("%s() Called !! \n", __func__);

    mutex_lock(&g_es9218_priv->power_lock);
    __es9218_sabre_headphone_off();
    mutex_unlock(&g_es9218_priv->power_lock);
    return 0;
}
EXPORT_SYMBOL(es9218_sabre_headphone_off);

int es9218_get_power_state(void){
    return es9218_power_state;
}

static int es9218p_set_volume_rate(unsigned int sample_rate, unsigned int ess_mode)
{
    int ret = -1;

    if( ess_mode == ESS_MODE_PCM ) {

        /* as per sample rate, set volume rate. MAY add setting soft time start(REG#14) */
        switch(sample_rate)
        {
            case 48000:
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_06, 0x44);
                break;

            default:
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_06, 0x43);
                break;
        }
    } else if( ess_mode == ESS_MODE_DoP ) {
    }

    pr_info("%s() exit - setting volume rate for %d Hz is %s\n", __func__, sample_rate, !ret?"done":"failed");

    return ret;

}
static int es9218p_set_sample_rate(unsigned int bit_width, unsigned int sample_rate, unsigned int ess_mode)
{
	int ret = -1;
	u8 reg = 0;

	if( ess_mode == ESS_MODE_PCM ) {
		reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07);
		if( (g_sabre_cf_num == 4  || g_sabre_cf_num == 5) && (reg & 0x80 )){
			pr_info("%s set custom filter when g_sabre_cf_num is %d\n", __func__, g_sabre_cf_num);
			es9218_sabre_cfg_custom_filter(&es9218_sabre_custom_ft[g_sabre_cf_num]);
		}
		switch (sample_rate) {
			case 384000:
				reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07);
				if(g_sabre_cf_num == 8 || g_sabre_cf_num == 9){
					reg &= ~0xE0;
					reg = reg|0x48;
				} else {
					reg = reg|0x8;
				}
				pr_info("%s register_07:%x, sample_rate:%d\n", __func__,reg, sample_rate);
				ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, reg);
				reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_44);
				reg = reg|0x01;
				ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_44, reg);
				break;
			default :
				reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07);
				if(g_sabre_cf_num == 8 || g_sabre_cf_num == 9){
					reg &= ~0xE8;
					reg = reg|0x40;
				} else {
					reg &= ~0x8;
				}
				ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, reg);
				pr_info("%s register_07:%x, sample_rate:%d\n", __func__, reg, sample_rate);
				reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_44);
				reg &= ~0x01;
				ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_44, reg);
				pr_info("%s register_44:%x\n", __func__, reg);
				break;
		}
	}
	return ret;
}

static int es9218p_set_bit_width(unsigned int bit_width, unsigned int ess_mode)
{
    int ret = -1;
    u8  i2c_len_reg = 0;
    u8  in_cfg_reg = 0;

    if( ess_mode == ESS_MODE_PCM ) {
        /*  Bit width(depth) Per Sec. Setting     */
        switch (bit_width) {
            case 16 :
                i2c_len_reg = 0x0;
                in_cfg_reg |= i2c_len_reg;
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_01, in_cfg_reg);
                break;

            case 24 :
            case 32 :
            default :
                i2c_len_reg = 0x80;
                in_cfg_reg |= i2c_len_reg;
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_01, in_cfg_reg);
                break;
        }
    } else if( ess_mode == ESS_MODE_DoP ) {
        /*  BPS Setting     */
        switch (bit_width) {
            case 64 :
#ifdef ENABLE_DOP_AUTO_MUTE
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_04, 0x05);	   // 0x01 : 170msec, 0x05 : 34msec
                /*	Volume Rate */
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_06, 0x4E);
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_02, 0xF4);
#endif /* ENABLE_DOP_AUTO_MUTE */
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_00, 0x04);   //  Reg #0
                break;

            case 128 :
            default :
#ifdef ENABLE_DOP_AUTO_MUTE
                /*	Automute Time					*/
                /*	 0xFF : 0 msec ,	0x04 : 50 msec ,	0x02 : 100 msec ,	0x01 : 170 msec */
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_04, 0x05);	   // 0x01 : 170msec, 0x05 : 34msec
                /*	Volume Rate */
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_06, 0x4E);
                /*	Auto Mute enable	*/
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_02, 0xF4);
#endif /* ENABLE_DOP_AUTO_MUTE */
                ret = es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_00, 0x00);   //  Reg #0 : HW Default Value
                break;
        }
    }
    pr_info("%s() exit - setting %d bits width is %s\n", __func__, bit_width, !ret?"done":"failed");

    return ret;

}

static void es9218p_initialize_registers(unsigned int ess_mode)
{
    int i;

    if( ess_mode == ESS_MODE_PCM ) {
        for (i = 0 ; i < sizeof(es9218_PCM_init_register)/sizeof(es9218_PCM_init_register[0]) ; i++) {
            es9218_write_reg(g_es9218_priv->i2c_client,
                            es9218_PCM_init_register[i].num,
                            es9218_PCM_init_register[i].value);
        }
    } else if( ess_mode == ESS_MODE_DoP ) {
        for (i = 0 ; i < sizeof(es9218_DOP_init_register)/sizeof(es9218_DOP_init_register[0]) ; i++) {
            es9218_write_reg(g_es9218_priv->i2c_client,
                            es9218_DOP_init_register[i].num,
                            es9218_DOP_init_register[i].value);
        }
    } else if( ess_mode == ESS_MODE_INIT ) {
        for (i = 0 ; i < sizeof(es9218_common_init_registers)/sizeof(es9218_common_init_registers[0]) ; i++) {
            es9218_write_reg(g_es9218_priv->i2c_client,
                            es9218_common_init_registers[i].num,
                            es9218_common_init_registers[i].value);

            if (!g_es9218_priv->es9218_data->is_es9219c) { // for ES9218P
                // ES9218P must set to : ES9218P_REG_21 = 0x0f
                es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_21, 0x0f); //GPIO Input Selection
            }
        }
    }
}
static void es9218_sabre_hifi_in_standby_work(struct work_struct *work)
{
    mutex_lock(&g_es9218_priv->power_lock);
    pr_info("%s() enter - go to hifi mode from standy mode status:%s, bps : %d , rate : %d\n",
    __func__,
    power_state[es9218_power_state],
    es9218_bps,
    es9218_rate);

    es9218p_sabre_bypass2hifi();

    if( g_dop_flag == 0 ) { // PCM
        // set bit width to ESS
        es9218p_set_bit_width(es9218_bps, ESS_MODE_PCM);
        es9218p_set_sample_rate(es9218_bps, es9218_rate, ESS_MODE_PCM);
        // set volume rate to ESS
        es9218p_set_volume_rate(es9218_rate, ESS_MODE_PCM);
    } else if (g_dop_flag > 0){ // DoP
        // set bit width to ESS
        es9218p_set_bit_width(g_dop_flag, ESS_MODE_DoP);

        // DoP has teardown mode in AudioFWK, so shutdown() is already invoked.
        // workaournd - forced change mode to idle in DoP mode
        es9218_sabre_audio_idle();
        pr_info("%s() force change a mode status:%s\n", __func__, power_state[es9218_power_state]);

    }

    pr_info("%s() exit - go to hifi mode from standy mode status:%s\n", __func__, power_state[es9218_power_state]);
    mutex_unlock(&g_es9218_priv->power_lock);
    return;
}

static void es9218_sabre_sleep_work (struct work_struct *work)
{
	__pm_wakeup_event(wl_sleep, jiffies_to_msecs(2000));
    mutex_lock(&g_es9218_priv->power_lock);
    if (es9218_power_state == ESS_PS_IDLE) {
        pr_info("%s(): sleep_work state is %s running \n", __func__, power_state[es9218_power_state]);

        es9218p_sabre_hifi2lpb();

    }
    else {
        pr_info("%s(): sleep_work state is %s skip operation \n", __func__, power_state[es9218_power_state]);
    }

    es9218_is_amp_on = 0;
#if defined(CONFIG_ARCH_SM8150)
    if(pm_qos_request_active(&req)) {
        pr_info("%s(): pm qos active state. so, remove pm qos request", __func__);
        pm_qos_remove_request(&req);
    }
#endif
    mutex_unlock(&g_es9218_priv->power_lock);
    return;
}

static int es9218_power_state_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    pr_debug("%s(): power state = %d\n", __func__, es9218_power_state);

    ucontrol->value.enumerated.item[0] = es9218_power_state;

    pr_debug("%s(): ucontrol = %d\n", __func__, ucontrol->value.enumerated.item[0]);

    return 0;
}

static int es9218_power_state_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int ret=0;

    pr_debug("%s():ucontrol = %d, power state=%d\n", __func__, ucontrol->value.enumerated.item[0], es9218_power_state);

    if (es9218_power_state == ucontrol->value.enumerated.item[0]) {
        pr_info("%s():no power state change\n", __func__);
    }

    //"Open", "Close","Bypass","Hifi","Idle","Active","PowerHigh","PowerLow","HphHigh","HphLow"
    switch(ucontrol->value.enumerated.item[0]) {
        case 0:
            __es9218_sabre_headphone_on();
            break;
        case 1:
            __es9218_sabre_headphone_off();
            break;
        case 2:
            es9218p_sabre_hifi2lpb();
            break;
        case 3:
            es9218p_sabre_bypass2hifi();
            break;
        case 4:
            es9218_sabre_audio_idle();
            break;
        case 5:
            es9218_sabre_audio_active();
            break;
        case 6:
            es9218_reset_gpio_H();
            break;
        case 7:
            es9218_reset_gpio_L();
            break;
        case 8:
            es9218_hph_switch_gpio_H();
            break;
        case 9:
            es9218_hph_switch_gpio_L();
            break;
        default:
            break;

    }
    return ret;
}


static int es9218_normal_harmonic_comp_put_left(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol) {
    int i =0 ;

    for( i =0  ;  i < 4  ; i ++ ) {
        normal_harmonic_comp_left[i] = (u8) ucontrol->value.integer.value[i];
    }
    return 0 ;
}
static int es9218_advance_harmonic_comp_put_left(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol) {
    int i =0;

    for( i =0  ;  i < 4  ; i ++ ) {
        advance_harmonic_comp_left[i] = (u8) ucontrol->value.integer.value[i];
    }
    return 0;
}
static int es9218_aux_harmonic_comp_put_left(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol) {
    int i =0;

    for( i =0  ;  i < 4  ; i ++ ) {
        aux_harmonic_comp_left[i] = (u8) ucontrol->value.integer.value[i];
    }
    return 0;
}
static int es9218_normal_harmonic_comp_put_right(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol) {
    int i =0 ;

    for( i =0  ;  i < 4  ; i ++ ) {
        normal_harmonic_comp_right[i] = (u8) ucontrol->value.integer.value[i];
    }
    return 0 ;
}
static int es9218_advance_harmonic_comp_put_right(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol) {
    int i =0;

    for( i =0  ;  i < 4  ; i ++ ) {
        advance_harmonic_comp_right[i] = (u8) ucontrol->value.integer.value[i];
    }
    return 0;
}
static int es9218_aux_harmonic_comp_put_right(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol) {
    int i =0;

    for( i =0  ;  i < 4  ; i ++ ) {
        aux_harmonic_comp_right[i] = (u8) ucontrol->value.integer.value[i];
    }
    return 0;
}

static int es9218_headset_type_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = g_headset_type;

    pr_info("%s(): type = %d \n", __func__, g_headset_type);

    return 0;
}

static int es9218_headset_type_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int value = 0;

    value = (int)ucontrol->value.integer.value[0];

    if(value != 0) {
        g_headset_type = value;
        pr_info("%s(): type = %d, state = %s\n ", __func__, value, power_state[es9218_power_state]);
    } else {
        /*
         * In mixer_paths.xml, 0 stands for no headset.
         ** init ** : <ctl name="Es9018 HEADSET TYPE" value="0" />
         * normal   : <ctl name="Es9018 HEADSET TYPE" value="1" />
         * advanced : <ctl name="Es9018 HEADSET TYPE" value="2" />
         * aux      : <ctl name="Es9018 HEADSET TYPE" value="3" />
        */
        pr_err("%s() : invalid headset type = %d, state = %s\n", __func__, value, power_state[es9218_power_state]);
        return 0;
	}


    if (es9218_power_state < ESS_PS_HIFI) {
        pr_debug("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }

    es9218_set_thd(g_es9218_priv->i2c_client, g_headset_type);
    // es9218p_sabre_bypass2hifi();  // unnecessary code, but remain it as histroy.

    return 0;
}

static int es9218_auto_mute_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = g_auto_mute_flag;

    pr_debug("%s(): type = %d \n", __func__, g_auto_mute_flag);

    return 0;
}

static int es9218_auto_mute_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int ret = 0 ;

    g_auto_mute_flag = (int)ucontrol->value.integer.value[0];

    pr_debug("%s(): g_auto_mute_flag = %d \n", __func__, g_auto_mute_flag);

    if (es9218_power_state < ESS_PS_HIFI) {
        pr_err("%s() : return = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }

    if(g_auto_mute_flag) {
        pr_debug("%s(): Disable g_auto_mute_flag = %d \n", __func__, g_auto_mute_flag);
        //es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_02,0x34); // #02  : Automute Config Disable
    } else {
        /*  BPS Setting     */
        switch (es9218_bps) {
            case 24 :
            case 32 :
                switch  (es9218_rate) {
                    case 96000  :
                    case 192000 :
                        /*  Auto Mute enable    */
                        pr_debug("%s(): Enable g_auto_mute_flag = %d \n", __func__, g_auto_mute_flag);
                        //es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_02,0xF4); // #02  : Automute Config Enable
                    break;
                }
        }
    }
    return ret;
}
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
static int lge_ess_digital_filter_setting_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = g_sabre_cf_num;
    pr_info("%s(): ucontrol = %d\n", __func__, g_sabre_cf_num);
    return 0;
}

static int lge_ess_digital_filter_setting_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int ret = 0;

    g_sabre_cf_num = (int)ucontrol->value.integer.value[0];
    pr_info("%s():filter num= %d\n", __func__, g_sabre_cf_num);
    return ret;
}




static int get_fade_count_define(void)
{
    if(fade_count_debug_param == 99) {
	return FADE_INOUT_COUNT;
    } else {
	return fade_count_debug_param;
    }

}
static int get_fade_term_define(void)
{
    if(fade_term_debug_param == 99) {
	return FADE_INOUT_TERM;
    } else {
	return fade_term_debug_param;
    }

}

static void mute_work_function(struct work_struct *work)
{

	int result = 0;

        result = cancel_delayed_work(mute_work);
	pr_info("%s(): called volume direction %d, g_fade_count %d, cancle_delayed_work result %d , g_left_fade_vol %d, g_right_fade_vol %d ",
		       	__func__, fade_direction, g_fade_count, result, g_left_fade_vol, g_right_fade_vol);

	if(fade_direction == VOLUME_DOWN) {
		if( g_fade_count != 0 ) {
			g_fade_count--;
//			pr_info("%s(): case 1 called volume direction %d, g_fade_count %d\n", __func__, fade_direction, g_fade_count);
	                g_left_fade_vol = g_left_fade_vol + g_left_fade_vol_per_step;
		        g_right_fade_vol = g_right_fade_vol + g_right_fade_vol_per_step;

			if(g_left_fade_vol > 0xff)
			    g_left_fade_vol = 0xff;
		        if(g_right_fade_vol > 0xff)
			    g_right_fade_vol = 0xff;

			es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_15, g_left_fade_vol);
                        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_16, g_right_fade_vol);
		} else {
//			pr_info("%s(): case 2 called volume g_fade_count %d\n", __func__, g_fade_count);
			if(g_sabre_cf_num == SHORT_FILTER)
			    g_volume = 0;
			else if(g_sabre_cf_num == SHARP_FILTER)
			    g_volume = 4;
			else
			    g_volume = 2;

                        es9218_master_trim(g_es9218_priv->i2c_client, g_volume);
			fade_direction = VOLUME_UP;
		}

                queue_delayed_work(mute_workqueue , mute_work, msecs_to_jiffies(get_fade_term_define()) );
	} else {
		if( g_fade_count < (get_fade_count_define() ) ) {
//			pr_info("%s(): case 3 called volume g_fade_count %d g_left_fade_vol %d\n", __func__, g_fade_count, g_left_fade_vol);
			g_fade_count++;
			g_left_fade_vol = g_left_fade_vol - g_left_fade_vol_per_step;
			g_right_fade_vol = g_right_fade_vol - g_right_fade_vol_per_step;

       			if(g_left_fade_vol < g_left_volume)
			    g_left_fade_vol = g_left_volume;
		        if(g_right_fade_vol < g_right_volume)
			    g_right_fade_vol = g_right_volume;

			es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_15, g_left_fade_vol);
                        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_16, g_right_fade_vol);
                        queue_delayed_work(mute_workqueue , mute_work, msecs_to_jiffies(get_fade_term_define()) ); // 100 msec
		} else {
            es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_15, g_left_volume);
            es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_16, g_right_volume);

			pr_info("%s(): case 4 called volume direction %d, g_fade_count %d\n", __func__, fade_direction, g_fade_count);
		}
	}


	return;
}

static int lge_ess_fade_inout_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

    if (es9218_power_state < ESS_PS_HIFI) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }
    if(lge_ess_fade_inout_init != true) {
        lge_ess_fade_inout_init = true;
        pr_info("%s(): fade in out work queue initialize. \n", __func__);

        mute_workqueue = create_workqueue("mute_workqueue");
        mute_work = devm_kzalloc(&g_es9218_priv->i2c_client->dev, sizeof(struct delayed_work), GFP_KERNEL);
        if(!mute_work) {
            lge_ess_fade_inout_init = false;
            pr_err("%s() : devm_kzalloc failed!!\n", __func__);
            return 0;
        }

        INIT_DELAYED_WORK(mute_work, mute_work_function);
    }

    if(work_pending( &mute_work->work )) {
        pr_info("%s(): previous fade in/out is not yet finished. \n", __func__);
//        return 0;
        g_fade_count = get_fade_count_define();
        fade_direction = VOLUME_DOWN;
    } else {
        g_fade_count = get_fade_count_define();
        g_left_fade_vol = g_left_volume;
        g_right_fade_vol = g_right_volume;
        g_left_fade_vol_per_step = (0xff - g_left_volume) / g_fade_count;
        g_right_fade_vol_per_step = (0xff - g_right_volume) / g_fade_count;
        fade_direction = VOLUME_DOWN;
        pr_info("%s(): start fade in/out g_left_volume %d, g_right_volume %d \n", __func__, g_left_volume, g_right_volume);
    }
    queue_delayed_work(mute_workqueue , mute_work, msecs_to_jiffies(get_fade_term_define()) ); // ?? msec

	return 0;
}

static int lge_ess_fade_inout_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
#endif
static int es9218_avc_volume_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = g_avc_volume;

    pr_debug("%s(): AVC Volume= -%d db\n", __func__, g_avc_volume);

    return 0;
}

static int es9218_avc_volume_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int ret = 0;
    int vol = 25;

    /* A range of g_avc_volume is from 0 to 24. */
    vol = (int)ucontrol->value.integer.value[0];

    if (vol >= sizeof(avc_vol_tbl)/sizeof(avc_vol_tbl[0])) {
        pr_err("%s() : Invalid vol = %d return \n", __func__, vol);
        return 0;
    }

    g_avc_volume = vol;

    pr_debug("%s(): AVC Volume= -%d db  state = %s\n", __func__, g_avc_volume , power_state[es9218_power_state]);

    if (es9218_power_state < ESS_PS_HIFI) {
        pr_debug("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }

    es9218_set_avc_volume(g_es9218_priv->i2c_client, g_avc_volume);
    return ret;
}

static int es9218_master_volume_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = g_volume;
    pr_debug("%s(): Master Volume= -%d db\n", __func__, g_volume/2);

    return 0;
}

static int es9218_master_volume_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int ret = 0;

    g_volume = (int)ucontrol->value.integer.value[0];
    pr_debug("%s(): Master Volume= -%d db\n", __func__, g_volume/2);


    if (es9218_power_state < ESS_PS_HIFI) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }

    es9218_master_trim(g_es9218_priv->i2c_client, g_volume);
    return ret;
}

static int es9218_left_volume_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = g_left_volume;
    pr_debug("%s(): Left Volume= -%d db\n", __func__, g_left_volume/2);

    return 0;
}

static int es9218_left_volume_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int ret = 0;

    g_left_volume = (int)ucontrol->value.integer.value[0];
    pr_debug("%s(): Left Volume= -%d db\n", __func__, g_left_volume/2);

    if (es9218_power_state < ESS_PS_HIFI) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }

    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_15, g_left_volume);
    return ret;
}

static int es9218_right_volume_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = g_right_volume;
    pr_debug("%s(): Right Volume= -%d db\n", __func__, g_right_volume/2);

    return 0;
}

static int es9218_right_volume_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int ret = 0;

    g_right_volume = (int)ucontrol->value.integer.value[0];
    pr_debug("%s(): Right Volume= -%d db\n", __func__, g_right_volume/2);

    if (es9218_power_state < ESS_PS_HIFI) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }

    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_16, g_right_volume);
    return ret;
}

static int es9218_filter_enum_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = g_sabre_cf_num;
    pr_debug("%s(): ucontrol = %d\n", __func__, g_sabre_cf_num);
    return 0;
}

static int es9218_filter_enum_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int ret = 0;
    u8 reg;

#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
    int new_filter = 0;

    new_filter = (int)ucontrol->value.integer.value[0];
    pr_info("%s(): g_sabre_cf_num %d, new filter num = %d \n", __func__, g_sabre_cf_num, new_filter);

    if (es9218_power_state < ESS_PS_HIFI) {
        pr_info("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }
#endif
    g_sabre_cf_num = (int)ucontrol->value.integer.value[0];
    pr_debug("%s():filter num= %d\n", __func__, g_sabre_cf_num);
    reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07);
    reg = reg|0x01;
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, reg);
    mdelay(2);
    es9218_sabre_cfg_custom_filter(&es9218_sabre_custom_ft[g_sabre_cf_num]);
    reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07);
    reg &= ~0x01;
    es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, reg);
    mdelay(2);
    return ret;
}

static int es9218_dop_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = g_dop_flag;
    pr_debug("%s() %d\n", __func__, g_dop_flag);
    return 0;
}

static int mode_changed = 0;
static int es9218_dop_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    /* g_dop_flag is for checking a format.
      0   : PCM format
      64  : DoP format(DSD64)
      128 : DoP format(DSD128)
    */

    prev_dop_flag = g_dop_flag; // stored a prevoius format

    g_dop_flag = (int)ucontrol->value.integer.value[0];
    pr_debug("%s() dop_enable:%d, state:%d\n", __func__, g_dop_flag, es9218_power_state);
    if( !(g_dop_flag == 0 || g_dop_flag == 64 || g_dop_flag == 128 ) ) {
        pr_err("%s() dop_enable error:%d. invalid arg.\n", __func__, g_dop_flag);
    }

    if( prev_dop_flag != g_dop_flag )
        mode_changed = 1;

    return 0;
}

static int es9218_clk_source_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = clk_source;
    pr_info("%s() clk_source = %d ,,,, 0: X-tal( 45.1584MHz), 1: MCLK : 19.2MHz, 3: BCLK \n", __func__, clk_source);

    return 0;
}

static int es9218_clk_source_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    clk_source = (int)ucontrol->value.integer.value[0];
    pr_info("%s() clk_source = %d ,,,, 0: X-tal( 45.1584MHz), 1: MCLK : 19.2MHz, 3: BCLK \n", __func__, clk_source);

    return 0;
}



static void es9218_chip_revision_get(void)
{
    int ret ;
    u8  readChipStatus;
    unsigned char   chipId = 0x00;
    int readCnt;

    pr_debug("%s(): enter, es9218_power_state=%s.\n", __func__, power_state[es9218_power_state]);


    if (g_es9218_priv->es9218_data->is_es9219c && clk_source == 1) {
        pr_info("%s() hifi_ext_mclk enable\n", __func__);
        ret = clk_prepare_enable(hifi_ext_mclk);
        if (ret)
            pr_err("%s() : ext clk enable failed\n", __func__);
    }

    mutex_lock(&g_es9218_priv->power_lock);
    es9218_power_gpio_H();
    mdelay(1);
    es9218_reset_gpio_H();
    mdelay(1);

    if(!g_ess_rev_check) // ESS Revision check is one time during the booting.
    {
        for (readCnt = 0; readCnt < 3; readCnt++) {
            readChipStatus = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_CHIPSTATUS);
            chipId = readChipStatus & 0xFC;

            pr_err("%s: chipId:0x%x readCnt : %d \n", __func__, chipId , readCnt);

            if (chipId == 0xd0){
                pr_err("%s: ESS revsion = ESS_9218p\n", __func__);
                g_ess_rev_check = true;
                break;
            }
            else if (chipId == 0x8c){
                pr_err("%s: ESS revsion = ESS_9219\n", __func__);
                g_ess_rev_check = true;
                break;
            }
            else if (chipId == 0xe0){
                pr_err("%s: ESS revsion = ESS_9228\n", __func__);
                g_ess_rev_check = true;
                break;
            }
        }
    }

    if(es9218_power_state < ESS_PS_HIFI){
        es9218_reset_gpio_L();

        if (g_es9218_priv->es9218_data->is_es9219c && clk_source == 1) {
            pr_info("%s() hifi_ext_mclk disable\n", __func__);
            clk_disable_unprepare(hifi_ext_mclk);

            if (chipId == 0xd0){ // es9218p for HW revision 1.0
                g_es9218_priv->es9218_data->is_es9219c = false;
            }
        }

        // power off when ess chis becomes shutdown
        if( g_es9218_priv->es9218_data->always_power_on == false ) {
            if( es9218_power_state == ESS_PS_BYPASS ) {
                pr_info("%s(): headphones is already inserted from booting up. call es9218p_standby2lpb() to re-enter to lpb. \n", __func__);
                es9218p_standby2lpb();
                //////////////////////////////////////////////////////////////////////////////////////
                // Sabre is now powered on in Low Power Bypass mode[ResetB Low and GPIO2(switch) High]
                //////////////////////////////////////////////////////////////////////////////////////
                pr_info("%s(): re-enter to lpb by calling es9218p_standby2lpb(). \n", __func__);
            } else {
                es9218_power_gpio_L();
            }
        }
    }

    mutex_unlock(&g_es9218_priv->power_lock);

    pr_debug("%s(): leave, es9218_power_state=%s.\n", __func__, power_state[es9218_power_state]);

}


static int es9218_chip_state_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int ret ;
    u8  readChipStatus;
    unsigned char   chipId = 0x00;
    int readCnt;

    pr_debug("%s(): enter, es9218_power_state=%s.\n", __func__, power_state[es9218_power_state]);


    if (g_es9218_priv->es9218_data->is_es9219c && clk_source == 1) {
        pr_info("%s() hifi_ext_mclk enable\n", __func__);
        ret = clk_prepare_enable(hifi_ext_mclk);
        if (ret)
            pr_err("%s() : ext clk enable failed\n", __func__);
    }

    mutex_lock(&g_es9218_priv->power_lock);
    es9218_power_gpio_H();
    mdelay(1);
    es9218_reset_gpio_H();
    mdelay(1);

    if(!g_ess_rev_check) // ESS Revision check is one time during the booting.
    {
        for (readCnt = 0; readCnt < 3; readCnt++) {
            readChipStatus = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_CHIPSTATUS);
            chipId = readChipStatus & 0xFC;

            pr_err("%s: chipId:0x%x readCnt : %d \n", __func__, chipId , readCnt);

            if (chipId == 0xd0){
                pr_err("%s: ESS revsion = ESS_9218p\n", __func__);
                g_ess_rev_check = true;
                break;
            }
            else if (chipId == 0x8c){
                pr_err("%s: ESS revsion = ESS_9219\n", __func__);
                g_ess_rev_check = true;
                break;
            }
            else if (chipId == 0xe0){
                pr_err("%s: ESS revsion = ESS_9228\n", __func__);
                g_ess_rev_check = true;
                break;
            }
        }
    }

    ret = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_00);
    if(ret<0){
        pr_err("%s : i2_read fail : %d\n",__func__ ,ret);
        ucontrol->value.enumerated.item[0] = 0; // fail
    } else {
        pr_notice("%s : i2_read success : %d\n",__func__ ,ret);
        ucontrol->value.enumerated.item[0] = 1; // true
    }

    if(es9218_power_state < ESS_PS_HIFI){
        es9218_reset_gpio_L();

        if (g_es9218_priv->es9218_data->is_es9219c && clk_source == 1) {
            pr_info("%s() hifi_ext_mclk disable\n", __func__);
            clk_disable_unprepare(hifi_ext_mclk);
        }

        // power off when ess chis becomes shutdown
        if( g_es9218_priv->es9218_data->always_power_on == false ) {
            if( es9218_power_state == ESS_PS_BYPASS ) {
                pr_info("%s(): headphones is already inserted from booting up. call es9218p_standby2lpb() to re-enter to lpb. \n", __func__);
                es9218p_standby2lpb();
                //////////////////////////////////////////////////////////////////////////////////////
                // Sabre is now powered on in Low Power Bypass mode[ResetB Low and GPIO2(switch) High]
                //////////////////////////////////////////////////////////////////////////////////////
                pr_info("%s(): re-enter to lpb by calling es9218p_standby2lpb(). \n", __func__);
            } else {
                es9218_power_gpio_L();
            }
        }
    }

    mutex_unlock(&g_es9218_priv->power_lock);

    pr_debug("%s(): leave, es9218_power_state=%s.\n", __func__, power_state[es9218_power_state]);
    return 0;
}

static int es9218_chip_state_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int ret = 0;

    //ret= ucontrol->value.enumerated.item[0];
    pr_debug("%s():ret = %d\n", __func__ ,ret );
    return 0;
}
#if 0
static int chargerlogo_chipstate_get(void)
{
    int ret ;
    u8  readChipStatus;
    unsigned char   chipId = 0x00;
    int readCnt;

    pr_debug("%s(): enter, es9218_power_state=%s.\n", __func__, power_state[es9218_power_state]);

    mutex_lock(&g_es9218_priv->power_lock);
    es9218_power_gpio_H();
    mdelay(1);
    es9218_reset_gpio_H();
    mdelay(1);

    if(!g_ess_rev_check) // ESS Revision check is one time during the booting.
    {
        for (readCnt = 0; readCnt < 3; readCnt++) {
            readChipStatus = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_CHIPSTATUS);
            chipId = readChipStatus & 0xF0;

            pr_info("%s: chipId:0x%x readCnt : %d \n", __func__, chipId , readCnt);

            if (chipId == 0xd0){
                pr_info("%s: ESS revsion = ESS_9218p\n", __func__);
                g_ess_rev_check = true;
                break;
            }
            else if (chipId == 0xe0){
                pr_info("%s: ESS revsion = ESS_9228\n", __func__);
                g_ess_rev_check = true;
                break;
            }
        }
    }

    ret = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_00);
    if(ret<0){
        pr_err("%s : i2_read fail : %d\n",__func__ ,ret);
    } else {
        pr_notice("%s : i2_read success : %d\n",__func__ ,ret);
    }

    if(es9218_power_state < ESS_PS_HIFI){
        es9218_reset_gpio_L();

        // power off when ess chis becomes shutdown
        if( g_es9218_priv->es9218_data->always_power_on == false ) {
            if( es9218_power_state == ESS_PS_BYPASS ) {
                pr_info("%s(): headphones is already inserted from booting up. call es9218p_standby2lpb() to re-enter to lpb. \n", __func__);
                es9218p_standby2lpb();
                //////////////////////////////////////////////////////////////////////////////////////
                // Sabre is now powered on in Low Power Bypass mode[ResetB Low and GPIO2(switch) High]
                //////////////////////////////////////////////////////////////////////////////////////
                pr_info("%s(): re-enter to lpb by calling es9218p_standby2lpb(). \n", __func__);
            } else {
                es9218_power_gpio_L();
            }
        }
    }

    mutex_unlock(&g_es9218_priv->power_lock);

    pr_debug("%s(): leave, es9218_power_state=%s.\n", __func__, power_state[es9218_power_state]);
    return 0;
}
#endif
static int es9218_sabre_wcdon2bypass_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    pr_err("%s(): power state = %d\n", __func__, es9218_power_state);

    ucontrol->value.enumerated.item[0] = es9218_power_state;
    pr_err("%s(): ucontrol = %d\n", __func__, ucontrol->value.enumerated.item[0]);

    return 0;
}

static int es9218_sabre_wcdon2bypass_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int ret = 0 ;

    mutex_lock(&g_es9218_priv->power_lock);

    if ( es9218_power_state == ESS_PS_CLOSE) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        pr_debug("%s(): exit\n", __func__);
        mutex_unlock(&g_es9218_priv->power_lock);
        return 0;
    }

    ret = (int)ucontrol->value.integer.value[0];

    pr_err("%s(): entry wcd on : %d \n ", __func__ , ret);

    if(ret == 0) {
        if(es9218_start && !g_es9218_priv->es9218_data->ess_hifi_exception) {
            if( __es9218_sabre_headphone_on() == 0 )
                es9218p_sabre_bypass2hifi();
            es9218_is_amp_on = 1;
            pr_info("%s() : state = %s : WCD On State ByPass -> HiFi !!\n", __func__, power_state[es9218_power_state]);
        } else {
            pr_info("%s() : state = %s : don't change\n", __func__, power_state[es9218_power_state]);
        }
    } else {
        if ( es9218_power_state > ESS_PS_BYPASS ) {
    //  if ( es9218_power_state == ESS_PS_IDLE ) {
            pr_info("%s() : state = %s : WCD On State HiFi -> ByPass !!\n", __func__, power_state[es9218_power_state]);
            cancel_delayed_work_sync(&g_es9218_priv->sleep_work);
#if defined(CONFIG_ARCH_SM8150)
            if(pm_qos_request_active(&req)) {
                pr_info("%s(): pm qos active state. so, remove pm qos request", __func__);
                pm_qos_remove_request(&req);
            }
#endif
            es9218p_sabre_hifi2lpb();
        }  else {
            pr_info("%s() : Invalid state = %s !!\n", __func__, power_state[es9218_power_state]);
        }
        es9218_is_amp_on = 0;
    }
    pr_debug("%s(): exit\n", __func__);

    mutex_unlock(&g_es9218_priv->power_lock);

    return 0;
}

static int es9218_clk_divider_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int err_check = -1;
    u8 reg_val;

    if (es9218_power_state < ESS_PS_HIFI) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }

    err_check = es9218_read_reg(g_es9218_priv->i2c_client,
                MASTER_MODE_CONTROL);
    if (err_check >= 0) {
        reg_val = err_check;
    } else {
        return -1;
    }

    reg_val = reg_val >> 5;
    ucontrol->value.integer.value[0] = reg_val;

    pr_debug("%s: i2s_length = 0x%x\n", __func__, reg_val);

    return 0;
}

static int es9218_clk_divider_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    int err_check = -1;
    u8 reg_val;

    if (es9218_power_state < ESS_PS_HIFI) {
        pr_err("%s() : invalid state = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }

    pr_debug("%s: ucontrol->value.integer.value[0]  = %ld\n", __func__, ucontrol->value.integer.value[0]);

    err_check = es9218_read_reg(g_es9218_priv->i2c_client,
                MASTER_MODE_CONTROL);
    if (err_check >= 0) {
        reg_val = err_check;
    } else {
        return -1;
    }

    reg_val &= ~(I2S_CLK_DIVID_MASK);
    reg_val |=  ucontrol->value.integer.value[0] << 5;

    es9218_write_reg(g_es9218_priv->i2c_client,
                MASTER_MODE_CONTROL, reg_val);
    return 0;
}


static const char * const es9218_power_state_texts[] = {
    "Close",
    "Open",
    "Bypass",
    "Hifi",
    "Idle",
    "Active",
    "ResetHigh",
    "ResetLow",
    "HphHigh",
    "HphLow"
};

static const char * const es9218_clk_divider_texts[] = {
    "DIV4",
    "DIV8",
    "DIV16",
    "DIV16"
};

static const struct soc_enum es9218_power_state_enum =
SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
        ARRAY_SIZE(es9218_power_state_texts),
        es9218_power_state_texts);

static const struct soc_enum es9218_clk_divider_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9218_clk_divider_texts),
        es9218_clk_divider_texts);

static struct snd_kcontrol_new es9218_digital_ext_snd_controls[] = {
    SOC_SINGLE_EXT("Es9018 Left Volume", SND_SOC_NOPM, 0, 256, 0,
                    es9218_left_volume_get,
                    es9218_left_volume_put),
    SOC_SINGLE_EXT("Es9018 Right Volume", SND_SOC_NOPM, 0, 256, 0,
                    es9218_right_volume_get,
                    es9218_right_volume_put),
    SOC_SINGLE_EXT("Es9018 Master Volume", SND_SOC_NOPM, 0, 100, 0,
                    es9218_master_volume_get,
                    es9218_master_volume_put),
    SOC_SINGLE_EXT("HIFI Custom Filter", SND_SOC_NOPM, 0, 12, 0, //current 0~3 custom filter num 4~11 built-in filter
                    es9218_filter_enum_get,
                    es9218_filter_enum_put),
    //SOC_SINGLE_EXT("HIFI THD Value", SND_SOC_NOPM, 0, 0xFFFFFF, 0, //current 0~3 filter num
    //                es9018_set_filter_enum,
    //                es9218_filter_enum_put),
#ifdef CONFIG_SND_SOC_LGE_ESS_DIGITAL_FILTER
    SOC_SINGLE_EXT("LGE ESS FADE INOUT", SND_SOC_NOPM, 0, 12, 0,
                    lge_ess_fade_inout_get,
                    lge_ess_fade_inout_put),
    SOC_SINGLE_EXT("LGE ESS DIGITAL FILTER SETTING", SND_SOC_NOPM, 0, 12, 0,
                    lge_ess_digital_filter_setting_get,
                    lge_ess_digital_filter_setting_put),
#endif
    SOC_ENUM_EXT("Es9018 State", es9218_power_state_enum,
                    es9218_power_state_get,
                    es9218_power_state_put),
    SOC_ENUM_EXT("Es9018 CLK Divider", es9218_clk_divider_enum,
                    es9218_clk_divider_get,
                    es9218_clk_divider_put),
    SOC_SINGLE_EXT("Es9018 Chip State", SND_SOC_NOPM, 0, 1, 0,
                    es9218_chip_state_get,
                    es9218_chip_state_put),
    SOC_SINGLE_EXT("Es9018 AVC Volume", SND_SOC_NOPM, 0, 25, 0,
                    es9218_avc_volume_get,
                    es9218_avc_volume_put),
    SOC_SINGLE_EXT("Es9018 HEADSET TYPE", SND_SOC_NOPM, 0, 4, 0,
                    es9218_headset_type_get,
                    es9218_headset_type_put),
    SOC_SINGLE_EXT("Es9018 Dop", SND_SOC_NOPM, 0, 128, 0,
                    es9218_dop_get,
                    es9218_dop_put),
    SOC_SINGLE_EXT("Es9218 AUTO_MUTE", SND_SOC_NOPM, 0, 1, 0,
                    es9218_auto_mute_get,
                    es9218_auto_mute_put),
    SOC_SINGLE_EXT("Es9218 Bypass", SND_SOC_NOPM, 0, 1, 0,
                    es9218_sabre_wcdon2bypass_get,
                    es9218_sabre_wcdon2bypass_put),
    SOC_SINGLE_EXT("Es9219 CLK_SOURCE", SND_SOC_NOPM, 0, 1, 0,
                    es9218_clk_source_get,
                    es9218_clk_source_put),
    SOC_SINGLE_MULTI_EXT("Es9218 NORMAL_HARMONIC LEFT", SND_SOC_NOPM, 0, 256,
                    0, 4, NULL, es9218_normal_harmonic_comp_put_left),
    SOC_SINGLE_MULTI_EXT("Es9218 ADVANCE_HARMONIC LEFT", SND_SOC_NOPM, 0, 256,
                    0, 4, NULL, es9218_advance_harmonic_comp_put_left),
    SOC_SINGLE_MULTI_EXT("ES9218 AUX_HARMONIC LEFT", SND_SOC_NOPM, 0, 256,
                    0, 4, NULL, es9218_aux_harmonic_comp_put_left),
    SOC_SINGLE_MULTI_EXT("Es9218 NORMAL_HARMONIC RIGHT", SND_SOC_NOPM, 0, 256,
                    0, 4, NULL, es9218_normal_harmonic_comp_put_right),
    SOC_SINGLE_MULTI_EXT("Es9218 ADVANCE_HARMONIC RIGHT", SND_SOC_NOPM, 0, 256,
                    0, 4, NULL, es9218_advance_harmonic_comp_put_right),
    SOC_SINGLE_MULTI_EXT("Es9218 AUX_HARMONIC RIGHT", SND_SOC_NOPM, 0, 256,
                    0, 4, NULL, es9218_aux_harmonic_comp_put_right),
};

static int es9218_read_reg(struct i2c_client *client, int reg)
{
    int ret;

    ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0) {
        pr_err("%s: err %d , reg = %d \n", __func__, ret, reg);
    }

    return ret;
}

static int es9218_write_reg(struct i2c_client *client, int reg, u8 value)
{
    int ret, i;

    // pr_notice("%s(): %03d=0x%x\n", __func__, reg, value);
    for (i = 0; i < 3; i++) {
        ret = i2c_smbus_write_byte_data(client, reg, value);
        if (ret < 0) {
            pr_err("%s: err %d,, reg = %d , and try again\n", __func__, ret , reg);
            mdelay(50);
        }
        else {
            break;
        }
    }

    if (ret < 0) {
        pr_err("%s: err %d\n", __func__, ret);
    }

    return ret;
}
static int es9218_populate_get_pdata(struct device *dev,
        struct es9218_data *pdata)
{
#ifdef USE_CONTROL_EXTERNAL_LDO_FOR_DVDD
    u32 vol_suply[2];
    int ret;
#endif

    pdata->reset_gpio = of_get_named_gpio(dev->of_node,
            "dac,reset-gpio", 0);

    if (pdata->reset_gpio < 0) {
        pr_err("Looking up %s property in node %s failed %d\n", "dac,reset-gpio", dev->of_node->full_name, pdata->reset_gpio);
        goto err;
    }

    pr_info("%s: reset gpio %d", __func__, pdata->reset_gpio);


    pdata->switch_gpio = of_get_named_gpio(dev->of_node,
            "dac,switch-gpio", 0);

    if (pdata->switch_gpio < 0) {
        pr_err("Looking up %s property in node %s failed %d\n", "dac,switch-gpio", dev->of_node->full_name, pdata->switch_gpio);
        //goto err;
    }
    pr_info("%s: switch gpio %d", __func__, pdata->switch_gpio);


    pdata->hph_switch = of_get_named_gpio(dev->of_node,
            "dac,hph-sw", 0);
    if (pdata->hph_switch < 0) {
        pr_err("Looking up %s property in node %s failed %d\n", "dac,hph-sw", dev->of_node->full_name, pdata->hph_switch);
        goto err;
    }

    pr_info("%s: hph switch %d", __func__, pdata->hph_switch);

#ifdef DEDICATED_I2C
    pdata->i2c_scl_gpio= of_get_named_gpio(dev->of_node,
            "dac,i2c-scl-gpio", 0);
    if (pdata->i2c_scl_gpio < 0) {
        pr_err("Looking up %s property in node %s failed %d\n", "dac,i2c-scl-gpio", dev->of_node->full_name, pdata->i2c_scl_gpio);
        goto err;
    }
    dev_dbg(dev, "%s: i2c_scl_gpio %d", __func__, pdata->i2c_scl_gpio);

    pdata->i2c_sda_gpio= of_get_named_gpio(dev->of_node,
            "dac,i2c-sda-gpio", 0);
    if (pdata->i2c_sda_gpio < 0) {
        pr_err("Looking up %s property in node %s failed %d\n", "dac,i2c-sda-gpio", dev->of_node->full_name, pdata->i2c_sda_gpio);
        goto err;
    }
    pr_info("%s: i2c_sda_gpio %d", __func__, pdata->i2c_sda_gpio);
#endif  //  End of  #ifdef  DEDICATED_I2C

    pdata->power_gpio= of_get_named_gpio(dev->of_node,
            "dac,power-gpio", 0);
    if (pdata->power_gpio < 0) {
        pr_err("Looking up %s property in node %s failed %d\n", "dac,power-gpio", dev->of_node->full_name, pdata->power_gpio);
        goto err;
    }

    pr_info("%s: power gpio %d\n", __func__, pdata->power_gpio);

#if 0
    pdata->ear_dbg = of_get_named_gpio(dev->of_node,
            "dac,ear-dbg", 0);
    if (pdata->ear_dbg < 0) {
        pr_err("Looking up %s property in node %s failed %d\n", "dac,ear-dbg", dev->of_node->full_name, pdata->ear_dbg);
        goto err;
    }
    pr_info("%s: ear_dbg gpio %d\n", __func__, pdata->ear_dbg);
#endif

#ifdef USE_CONTROL_EXTERNAL_LDO_FOR_DVDD
    pdata->vreg_dvdd = regulator_get(dev, "dac,dvdd");
    if (IS_ERR(pdata->vreg_dvdd)) {
		/*
		 * if analog voltage regulator, VA is not ready yet, return
		 * -EPROBE_DEFER to kernel so that probe will be called at
		 * later point of time.
		 */
		if (PTR_ERR(pdata->vreg_dvdd) == -EPROBE_DEFER) {
			pr_err("In %s, vreg_dvdd probe defer\n", __func__);
			devm_kfree(dev, pdata);
            return PTR_ERR(pdata->vreg_dvdd);
		}
	}

    pr_info("%s: DVDD ldo GET\n", __func__);

    ret = of_property_read_u32_array(dev->of_node, "dac,va-supply-voltage", vol_suply, 2);
    if (ret < 0) {
        pr_err("%s Invalid property name\n",__func__);
        regulator_put(pdata->vreg_dvdd);
        devm_kfree(dev, pdata);
        return -EINVAL;
    } else {
        pdata->low_vol_level = vol_suply[0];
        pdata->high_vol_level = vol_suply[1];
        pr_info("%s: MIN uV=%d, MAX uV=%d. \n",
            __func__, pdata->low_vol_level, pdata->high_vol_level);
    }
#endif

    /*
     * pre-condition to use dynamic power on/off
     * 1. Two LDOs are dedicated for each 1.8V, 3.3V(to handle LDOs on/off)
     * 2. DVDD line is connected to GND(to avoid leakage current at sleep mode)
    */
    if( of_property_read_bool(dev->of_node, "dac,disable-always-power-on") ) {
        pdata->always_power_on = false;
    } else {
        pdata->always_power_on = true;
    }

    pr_info("%s: always-power-on is [%s]\n", __func__, pdata->always_power_on?"enabled":"disabled");

    /*
     * if DVDD pin is connected to GND, the internal LDO must be used.
     * if DVDD pin is connected to 1.2V, we have to choose internal/external LDO through a method below.
     * At first, MUST measure current at active/sleep,
     * and figure out which one is better
     * with power conpsumtion(at active) including leakage current(at sleep).
    */
    if( of_property_read_bool(dev->of_node, "dac,use-internal-ldo") ) {
        pdata->use_internal_ldo = true;
    } else {
        pdata->use_internal_ldo = false;
    }

    pr_info("%s: use-internal-ldo is [%s]\n", __func__, pdata->use_internal_ldo?"enabled":"disabled");

    if( of_property_read_bool(dev->of_node, "dac,es9219") ) {
        pdata->is_es9219c = true;
    } else {
        pdata->is_es9219c = false;
    }
    pr_info("%s: HIFI Dac is es9219 [%d]\n", __func__, pdata->is_es9219c);

    // for use MCLK ( clk_source = 1 )
#if defined(CONFIG_MACH_KONA_TIMELM)
    if (lge_get_board_rev_no_for_dlkm() < 9)
    {
        clk_source = 1;
        pr_info("%s: HW Revision is [%d] , clk_source = %d \n", __func__, lge_get_board_rev_no_for_dlkm(), clk_source);
    }
#endif

    if (pdata->is_es9219c && clk_source == 1) {
        hifi_ext_mclk = clk_get(dev, "hifi_mclk");
        if (IS_ERR(hifi_ext_mclk)) {
            pr_err("%s MCLK get failed\n", __func__);
        }
    }

    return 0;

err:
    devm_kfree(dev, pdata);
    return -1;
}

static unsigned int es9218_component_read(struct snd_soc_component *component,
        unsigned int reg)
{
    //struct es9218_priv *priv = codec->control_data;

    return 0;
}

static int es9218_component_write(struct snd_soc_component *component, unsigned int reg,
        unsigned int value)
{
    //struct es9218_priv *priv = codec->control_data;

    return 0;
}

static int es9218_set_bias_level(struct snd_soc_component *component,
        enum snd_soc_bias_level level)
{
    int ret = 0;

	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

    /* dev_dbg(codec->dev, "%s(codec, level = 0x%04x): entry\n", __func__, level); */

    switch (level) {
        case SND_SOC_BIAS_ON:
            break;

        case SND_SOC_BIAS_PREPARE:
            break;

        case SND_SOC_BIAS_STANDBY:
            break;

        case SND_SOC_BIAS_OFF:
            break;
    }
#if 0 //LGE_AudioBSP, temp code for fix build error // org code
        codec->dapm.bias_level = level;
#else
        dapm->bias_level = level;
#endif

    /* dev_dbg(codec->dev, "%s(): exit\n", __func__); */
    return ret;
}

static int es9218_pcm_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params,
        struct snd_soc_dai *codec_dai)
{
    //struct snd_soc_codec    *codec = codec_dai->codec;
    //struct es9218_priv      *priv  = codec->control_data;
    int ret = -1;
    es9218_bps  = params_width(params);
    es9218_rate = params_rate(params);

    pr_info("%s(): entry , bps : %d , rate : %d\n", __func__, es9218_bps, es9218_rate);

    if (g_dop_flag == 0) { //  PCM
        pr_info("%s(): PCM Format Running \n", __func__);
        // set bit width to ESS
        ret = es9218p_set_bit_width(es9218_bps, ESS_MODE_PCM);
        ret = es9218p_set_sample_rate(es9218_bps, es9218_rate, ESS_MODE_PCM);
        // set volume rate to ESS
        ret = es9218p_set_volume_rate(es9218_rate, ESS_MODE_PCM);

#ifdef ENABLE_DOP_AUTO_MUTE
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_04, 0x00);
        es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_02, 0x34);
#endif /* ENABLE_DOP_AUTO_MUTE */
    } else if (g_dop_flag > 0) { // DoP
        pr_info("%s(): DOP Format Running \n", __func__);

        // set bit width to ESS
        ret = es9218p_set_bit_width(g_dop_flag, ESS_MODE_DoP);
    }

    // changing a mode is done here, so reset mode_changed
	mode_changed = 0;

	pr_info("%s(): exit, ret=%d\n", __func__, ret);

    return ret;
}

static int es9218_mute(struct snd_soc_dai *dai, int mute)
{
    //struct snd_soc_component *component = dai->codec;
    //struct es9218_priv *priv = component->control_data;
    u8 reg;
#ifdef ENABLE_DOP_SOFT_MUTE
    pr_info("%s(): entry, mute_state = %d , power_state = %s\n", __func__, mute ,power_state[es9218_power_state]);

    if (es9218_power_state < ESS_PS_HIFI) {
        pr_info("%s() : return = %s\n", __func__, power_state[es9218_power_state]);
        return 0;
    }

	if(mute){
		reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07);
		reg = reg|0x01;
		es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, reg);
	} else {
		if(g_dop_flag){
			pr_info("%s(): dop add delay\n", __func__);
			mdelay(10);
		}
		reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07);
		reg &= ~0x01;
		es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, reg);
	}
	mdelay(5);
#endif /* ENABLE_DOP_SOFT_MUTE */
    return 0;

}

static int es9218_set_dai_sysclk(struct snd_soc_dai *codec_dai,
        int clk_id, unsigned int freq, int dir)
{
    //struct snd_soc_component *component = codec_dai->component;
    //struct es9218_priv *priv = codec->control_data;
    //pr_info("%s(): entry\n", __func__);

    return 0;
}


static int es9218_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
    //struct snd_soc_component *component = codec_dai->component;
    //struct es9218_priv *priv = component->control_data;
    //pr_info("%s(): entry\n", __func__);

    return 0;
}

static int es9218_startup(struct snd_pcm_substream *substream,
               struct snd_soc_dai *dai)
{
//  struct snd_soc_component *component = dai->component;
    u8 reg;
    pr_info("%s(): entry\n", __func__);

    mutex_lock(&g_es9218_priv->power_lock);
    call_common_init_registers = 1;

    cancel_delayed_work_sync(&g_es9218_priv->sleep_work);
#if defined(CONFIG_ARCH_SM8150)
    if(pm_qos_request_active(&req)) {
        pr_info("%s(): pm qos active state. so, remove pm qos request", __func__);
        pm_qos_remove_request(&req);
    }
#endif
    if ( es9218_power_state == ESS_PS_IDLE ) {
        pr_info("%s() : state = %s : Audio Active !!\n", __func__, power_state[es9218_power_state]);
        // check if DoP64 <-> DoP128
        if( (prev_dop_flag != g_dop_flag) && (prev_dop_flag*g_dop_flag > 0) ) {
            // do nothing here, after that, will update bit width in es9218_pcm_hw_params().
            pr_info("%s() : And, changed DoP's bit width from/to DoP64/DoP128(%d -> %d), so don't need to re-initialize registers !!\n",
                __func__, prev_dop_flag, g_dop_flag);
        }
        // check if PCM <-> DoP
        else if( (prev_dop_flag != g_dop_flag) && (prev_dop_flag*g_dop_flag == 0) ) {
            pr_info("%s() : However, changed a format from/to PCM/DoP(%d -> %d), so need to re-initialize registers !!\n",
                __func__, prev_dop_flag, g_dop_flag);
            if (g_dop_flag == 0) { //  DoP -> PCM
                // re-init. PCM reg.
                es9218p_initialize_registers(ESS_MODE_PCM);
            } else if (g_dop_flag > 0) { //  PCM -> DOP
#ifdef ENABLE_DOP_SOFT_MUTE
                reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07);
                reg = reg|0x01;
                es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, reg);
                mdelay(2);
#endif /* ENABLE_DOP_SOFT_MUTE */
                // re-init. DoP reg.
                es9218p_initialize_registers(ESS_MODE_DoP);
            }
        }
        // check if PCM <-> DoP with mode_changed
        else if( mode_changed ) {
            pr_info("%s() : However, changed a format to PCM/DoP(%d) with mode_changed %d, so need to re-initialize registers !!\n",
                __func__, g_dop_flag, mode_changed);
            if (g_dop_flag == 0) { //  DoP -> PCM
                // re-init. PCM reg.
                es9218p_initialize_registers(ESS_MODE_PCM);
            } else if (g_dop_flag > 0) { //  PCM -> DOP
#ifdef ENABLE_DOP_SOFT_MUTE
                reg = es9218_read_reg(g_es9218_priv->i2c_client, ES9218P_REG_07);
                reg = reg|0x01;
                es9218_write_reg(g_es9218_priv->i2c_client, ES9218P_REG_07, reg);
                mdelay(2);
#endif /* ENABLE_DOP_SOFT_MUTE */
                // re-init. DoP reg.
                es9218p_initialize_registers(ESS_MODE_DoP);
            }
        }

        es9218_sabre_audio_active();
    } else {
        if(!g_es9218_priv->es9218_data->ess_hifi_exception) {
            pr_info("%s() : state = %s : goto HIFI !!\n", __func__, power_state[es9218_power_state]);
            if( __es9218_sabre_headphone_on() == 0 )
                es9218p_sabre_bypass2hifi();
        } else {
            pr_info("%s() : ess hifi exception status = %d. state = %s. \n", __func__, g_es9218_priv->es9218_data->ess_hifi_exception, power_state[es9218_power_state]);
        }
    }
    es9218_is_amp_on = 1;
    es9218_start = 1;

    pr_info("%s(): exit\n", __func__);

    mutex_unlock(&g_es9218_priv->power_lock);
    return 0;
}

static void es9218_shutdown(struct snd_pcm_substream *substream,
               struct snd_soc_dai *dai)
{
    struct snd_soc_component *component = dai->component;

    mutex_lock(&g_es9218_priv->power_lock);

    dev_info(component->dev, "%s(): entry\n", __func__);

    es9218_sabre_audio_idle();

#if defined(CONFIG_ARCH_SM8150)
    req.type = PM_QOS_REQ_AFFINE_CORES;
    req.irq = -1;
    cpumask_copy(&req.cpus_affine, cpu_present_mask);
#endif

#ifdef ES9218P_DEBUG
	__pm_wakeup_event(wl_shutdown, jiffies_to_msecs(10));
    schedule_delayed_work(&g_es9218_priv->sleep_work, msecs_to_jiffies(10));      //  3 Sec
#else
    __pm_wakeup_event(wl_shutdown, jiffies_to_msecs(5000));
#if defined(CONFIG_ARCH_SM8150)
    if(!pm_qos_request_active(&req)) {
        pr_info("%s(): pm qos nonactive state. so, pm_qos_add_request", __func__);
        pm_qos_add_request(&req, PM_QOS_CPU_DMA_LATENCY, 0);
    }
#endif
    schedule_delayed_work(&g_es9218_priv->sleep_work, msecs_to_jiffies(2000));      //  2 Sec
#endif

    es9218_start = 0;
    mutex_unlock(&g_es9218_priv->power_lock);
}

static int es9218_hw_free(struct snd_pcm_substream *substream,
               struct snd_soc_dai *dai)
{
    struct snd_soc_component *component = dai->component;

    dev_info(component->dev, "%s(): entry\n", __func__);

    return 0;
}


static const struct snd_soc_dai_ops es9218_dai_ops = {
    .hw_params      = es9218_pcm_hw_params,  //soc_dai_hw_params
    .digital_mute   = es9218_mute,
    .set_fmt        = es9218_set_dai_fmt,
    .set_sysclk     = es9218_set_dai_sysclk,
    .startup        = es9218_startup,
    .shutdown       = es9218_shutdown,
    .hw_free        = es9218_hw_free,
};

static struct snd_soc_dai_driver es9218_dai[] = {
    {
        .name   = "es9218-hifi",
        .playback = {
            .stream_name    = "Playback",
            .channels_min   = 2,
            .channels_max   = 2,
            .rates          = ES9218_RATES,
            .formats        = ES9218_FORMATS,
        },
        .capture = {
            .stream_name    = "Capture",
            .channels_min   = 2,
            .channels_max   = 2,
            .rates          = ES9218_RATES,
            .formats        = ES9218_FORMATS,
        },
        .ops = &es9218_dai_ops,
    },
};

static  int es9218_component_probe(struct snd_soc_component *component)
{
    struct es9218_priv *priv = snd_soc_component_get_drvdata(component);

    pr_notice("%s(): entry\n", __func__);

    if (priv)
        priv->component = component;
    else
        pr_err("%s(): fail !!!!!!!!!!\n", __func__);

//    component->control_data = snd_soc_component_get_drvdata(component);
	wl_sleep = wakeup_source_register(NULL, "sleep_lock");
	wl_shutdown = wakeup_source_register(NULL, "shutdown_lock");
    es9218_set_bias_level(component, SND_SOC_BIAS_STANDBY);

    pr_notice("%s(): exit \n", __func__);
    return 0;
}

static void  es9218_component_remove(struct snd_soc_component *component)
{
	es9218_set_bias_level(component, SND_SOC_BIAS_OFF);
    dev_info(component->dev, "%s(): entry\n", __func__);

}

static struct snd_soc_component_driver soc_component_dev_es9218 = {
    .probe          = es9218_component_probe,
    .remove         = es9218_component_remove,
    .read           = es9218_component_read,
    .write          = es9218_component_write,
  	.controls       = es9218_digital_ext_snd_controls,
   	.num_controls   = ARRAY_SIZE(es9218_digital_ext_snd_controls),
    .idle_bias_on   = 1, // if 0, that makes WARN() on soc-core and It can cause sec wdt.
};


static int es9218_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    struct es9218_priv  *priv;
    struct es9218_data  *pdata;
    int ret = 0;

    pr_info("%s: enter.\n", __func__);

    if (!i2c_check_functionality(client->adapter,
                I2C_FUNC_SMBUS_BYTE_DATA)) {
        pr_err("%s: no support for i2c read/write byte data\n", __func__);
        return -EIO;
    }

    if (client->dev.of_node) {
        pdata = devm_kzalloc(&client->dev,
                sizeof(struct es9218_data), GFP_KERNEL);
        if (!pdata) {
            pr_err("Failed to allocate memory\n");
            return -ENOMEM;
        }

        ret = es9218_populate_get_pdata(&client->dev, pdata);
        if (ret) {
            pr_err("Parsing DT failed(%d)", ret);
            return ret;
        }
    } else {
        pdata = client->dev.platform_data;
    }

    if (!pdata) {
        pr_err("%s: no platform data\n", __func__);
        return -EINVAL;
    }

    priv = devm_kzalloc(&client->dev, sizeof(struct es9218_priv),
            GFP_KERNEL);
    if (priv == NULL) {
        return -ENOMEM;
    }

    priv->i2c_client = client;
    priv->es9218_data = pdata;
    i2c_set_clientdata(client, priv);

    g_es9218_priv = priv;
    INIT_DELAYED_WORK(&g_es9218_priv->hifi_in_standby_work, es9218_sabre_hifi_in_standby_work);
    INIT_DELAYED_WORK(&g_es9218_priv->sleep_work, es9218_sabre_sleep_work);

    mutex_init(&g_es9218_priv->power_lock);
#if 0
    ret = gpio_request_one(pdata->ear_dbg, GPIOF_IN,"gpio_earjack_debugger");
    if (ret < 0) {
        pr_err("%s(): debugger gpio request failed\n", __func__);
        goto ear_dbg_gpio_request_error;
    }
    pr_info("%s: ear dbg. gpio num : %d, value : %d!\n",
        __func__, pdata->ear_dbg, gpio_get_value(pdata->ear_dbg));
#endif

    ret = gpio_request(pdata->power_gpio, "ess_power");
    if (ret < 0) {
        pr_err("%s(): ess power request failed\n", __func__);
        goto power_gpio_request_error;
    }
    ret = gpio_direction_output(pdata->power_gpio, 1);
    if (ret < 0) {
        pr_err("%s: ess power set failed\n", __func__);
        goto power_gpio_request_error;
    }
    gpio_set_value(pdata->power_gpio, 0);

    ret = gpio_request(pdata->hph_switch, "ess_switch");
    if (ret < 0) {
        pr_err("%s(): ess switch request failed\n", __func__);
        goto switch_gpio_request_error;
    }
    ret = gpio_direction_output(pdata->hph_switch, 1);
    if (ret < 0) {
        pr_err("%s: ess switch set failed\n", __func__);
        goto switch_gpio_request_error;
    }
    gpio_set_value(pdata->hph_switch, 0);

    ret = gpio_request(pdata->reset_gpio, "ess_reset");
    if (ret < 0) {
        pr_err("%s(): ess reset request failed\n", __func__);
        goto reset_gpio_request_error;
    }
    ret = gpio_direction_output(pdata->reset_gpio, 1);
    if (ret < 0) {
        pr_err("%s: ess reset set failed\n", __func__);
        goto reset_gpio_request_error;
    }
    gpio_set_value(pdata->reset_gpio, 0);

    if (pdata->switch_gpio >= 0) {
        pr_info("%s: request msm switch gpio [%d] \n", __func__, pdata->switch_gpio);

        ret = gpio_request(pdata->switch_gpio, "hsdet_l_switch");
        if (ret < 0) {
            pr_err("%s(): msm switch request failed\n", __func__);
            goto hsdet_l_switch_gpio_request_error;
        }
        ret = gpio_direction_output(pdata->switch_gpio, 1);
        if (ret < 0) {
            pr_err("%s: msm switch set failed\n", __func__);
            goto hsdet_l_switch_gpio_request_error;
        }
        gpio_set_value(pdata->switch_gpio, 0);
    }
    pdata->ess_hifi_exception = false;

    if (g_es9218_priv->es9218_data->is_es9219c) {
        es9218_chip_revision_get();
    }

    ret = snd_soc_register_component(&client->dev,
                      &soc_component_dev_es9218,
                      es9218_dai, ARRAY_SIZE(es9218_dai));

#ifdef ES9218P_SYSFS
    ret = sysfs_create_group(&client->dev.kobj, &es9218_attr_group);
#endif
#if 0
    if(lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO){
       pr_info("%s chargerlogo mode call chargerlogo_chipstate_get function\n",__func__);
       chargerlogo_chipstate_get();
    }
#endif
    pr_info("%s: snd_soc_register_component ret = %d\n",__func__, ret);
    return ret;

switch_gpio_request_error:
    gpio_free(pdata->hph_switch);
reset_gpio_request_error:
    gpio_free(pdata->reset_gpio);
power_gpio_request_error:
    gpio_free(pdata->power_gpio);
hsdet_l_switch_gpio_request_error:
    gpio_free(pdata->switch_gpio);
#if 0
ear_dbg_gpio_request_error:
    gpio_free(pdata->ear_dbg);
#endif
    return ret;

}

static int es9218_remove(struct i2c_client *client)
{
#ifdef USE_CONTROL_EXTERNAL_LDO_FOR_DVDD
    struct es9218_data  *pdata;
    pdata = (struct es9218_data*)i2c_get_clientdata(client);    //pdata = (struct es9218_data*)client->dev.driver_data;
    if( pdata->vreg_dvdd != NULL )
        regulator_put(pdata->vreg_dvdd);
#endif
    snd_soc_unregister_component(&client->dev);
    mutex_destroy(&g_es9218_priv->power_lock);
    return 0;
}

static struct of_device_id es9218_match_table[] = {
    { .compatible = "dac,es9218-codec", },
    {}
};

static const struct i2c_device_id es9218_id[] = {
    { "es9218-codec", 0 },
    { },
};

//MODULE_DEVICE_TABLE(i2c, isa1200_id);
MODULE_DEVICE_TABLE(i2c, es9218_id);

static struct i2c_driver es9218_i2c_driver = {
    .driver = {
        .name           = "es9218-codec",
        .owner          = THIS_MODULE,
        .of_match_table = es9218_match_table,
    },
    .probe      = es9218_probe,
    .remove     = es9218_remove,
    //.suspend  = es9218_suspend,
    //.resume   = es9218_resume,
    .id_table   = es9218_id,
};



static int __init es9218_init(void)
{
    pr_notice("%s()\n", __func__);
    return i2c_add_driver(&es9218_i2c_driver);
}

static void __exit es9218_exit(void)
{
    i2c_del_driver(&es9218_i2c_driver);
}

module_init(es9218_init);
module_exit(es9218_exit);

MODULE_DESCRIPTION("ASoC ES9218 driver");
MODULE_AUTHOR("ESS-LINshaodong");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:es9218-codec");
