/*
 * tfa98xx.c   tfa98xx codec module
 *
 * Copyright (C) 2014-2020 NXP Semiconductors, All Rights Reserved.
 * Copyright 2020 GOODIX
 */

#define pr_fmt(fmt) "%s(): " fmt, __func__

#if !defined(DEBUG)
#define DEBUG
#endif

#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include "config.h"
#include "tfa98xx.h"
#include "tfa.h"
#include "tfa_internal.h"

 /* required for enum tfa9912_irq */
#include "tfa98xx_tfafieldnames.h"

#define TFA98XX_VERSION	TFA98XX_API_REV_STR

#define I2C_RETRIES 50
#define I2C_RETRY_DELAY 5 /* ms */

#ifdef N1A
#include "inc/tfa98xx_genregs_N1A12.h"
#else
#include "inc/tfa98xx_genregs_N1C.h"
#endif

#define TFA_READ_BATTERY_TEMP
#if defined(TFA_READ_BATTERY_TEMP)
#include <linux/power_supply.h>
#endif

#ifdef MPLATFORM
#include <mtk-sp-spk-amp.h>
#endif

#define TFA98XX_VERSION	TFA98XX_API_REV_STR

#if defined(TFA_NO_SND_FORMAT_CHECK)
#define TFA_FULL_RATE_SUPPORT_WITH_POST_CONVERSION
#endif
#define TFA_SET_DAPM_IGNORE_SUSPEND

/* Change volume selection behavior:
 * Uncomment following line to generate a profile change when updating
 * a volume control (also changes to the profile of the modified volume
 * control)
 */
/*#define TFA98XX_ALSA_CTRL_PROF_CHG_ON_VOL	1*/

/* Supported rates and data formats */
#if !defined(TFA_FULL_RATE_SUPPORT_WITH_POST_CONVERSION)
#if defined(USE_TFA9874) || defined(USE_TFA9878)
#define TFA98XX_RATES (SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_32000 | \
SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)
#elif defined(USE_TFA9872)
#define TFA98XX_RATES (SNDRV_PCM_RATE_16000 | \
SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)
#else
#define TFA98XX_RATES SNDRV_PCM_RATE_8000_48000
#endif
#else
#define TFA98XX_RATES SNDRV_PCM_RATE_8000_192000
static unsigned int sr_converted = 48000;
#endif
#define TFA98XX_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#define TF98XX_MAX_DSP_START_TRY_COUNT	10

#define TFA_DBGFS_CHECK_MTPEX

/* data accessible by all instances */
/* Memory pool used for DSP messages */
static struct kmem_cache *tfa98xx_cache = NULL;
/* Mutex protected data */
static DEFINE_MUTEX(tfa98xx_mutex);
static DEFINE_MUTEX(probe_lock);
static LIST_HEAD(tfa98xx_device_list);
static int tfa98xx_device_count = 0;
static int tfa98xx_sync_count = 0;
static int tfa98xx_monitor_count = 0;
#define MONITOR_COUNT_MAX 5

static LIST_HEAD(profile_list);        /* list of user selectable profiles */
static int tfa98xx_mixer_profiles = 0; /* number of user selectable profiles */
static int tfa98xx_mixer_profile = 0;  /* current mixer profile */
static struct snd_kcontrol_new *tfa98xx_controls;
static struct tfa_container *tfa98xx_container = NULL;

#if defined(TFADSP_DSP_BUFFER_POOL)
static int buf_pool_size[POOL_MAX_INDEX] = {
	64*1024,
	64*1024,
	64*1024,
	64*1024,
	64*1024,
	8*1024
};
#endif

static int tfa98xx_kmsg_regs = 0;
static int tfa98xx_ftrace_regs = 0;

static char *fw_name = "tfa98xx.cnt";
module_param(fw_name, charp, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(fw_name, "TFA98xx DSP firmware (container file) name.");

static int trace_level = 0;
module_param(trace_level, int, S_IRUGO);
MODULE_PARM_DESC(trace_level, "TFA98xx debug trace level (0=off, bits:1=verbose,2=regdmesg,3=regftrace,4=timing).");

static char *dflt_prof_name = "";
module_param(dflt_prof_name, charp, S_IRUGO);

static int no_start;
module_param(no_start, int, S_IRUGO);
MODULE_PARM_DESC(no_start, "do not start the work queue; for debugging via user\n");

static int no_reset;
module_param(no_reset, int, S_IRUGO);
MODULE_PARM_DESC(no_reset, "do not use the reset line; for debugging via user\n");

static int pcm_sample_format;
module_param(pcm_sample_format, int, S_IRUGO);
MODULE_PARM_DESC(pcm_sample_format, "PCM sample format: 0=S16_LE, 1=S24_LE, 2=S32_LE\n");

#if defined(TFA_NO_SND_FORMAT_CHECK)
static int pcm_no_constraint = 1;
#else
static int pcm_no_constraint;
#endif
module_param(pcm_no_constraint, int, S_IRUGO);
MODULE_PARM_DESC(pcm_no_constraint, "do not use constraints for PCM parameters\n");

#if defined(USE_TFA9891)
static void tfa98xx_tapdet_check_update(struct tfa98xx *tfa98xx);
#endif
static int tfa98xx_get_fssel(unsigned int rate);
static void tfa98xx_interrupt_enable(struct tfa98xx *tfa98xx, bool enable);

static int get_profile_from_list(char *buf, int id);
static int get_profile_id_for_sr(int id, unsigned int rate);

static int _tfa98xx_mute(struct tfa98xx *tfa98xx, int mute, int stream);
static int _tfa98xx_stop(struct tfa98xx *tfa98xx);

#if defined(CONFIG_DEBUG_FS) && defined(TFA_READ_BATTERY_TEMP)
static enum tfa98xx_error tfa98xx_read_battery_temp(short *value);
#endif

#ifdef CONFIG_MACH_KONA_TIMELM
extern int lge_get_board_rev_no_for_dlkm(void);
#endif /* CONFIG_MACH_LGE */

struct tfa98xx_rate {
	unsigned int rate;
	unsigned int fssel;
};

static const struct tfa98xx_rate rate_to_fssel[] = {
	{8000, 0},
	{11025, 1},
	{12000, 2},
	{16000, 3},
	{22050, 4},
	{24000, 5},
	{32000, 6},
	{44100, 7},
	{48000, 8},
#if defined(TFA_NO_SND_FORMAT_CHECK)
/* out of range */
	{64000, 9},
	{88200, 10},
	{96000, 11},
	{176400, 12},
	{192000, 13},
#endif
};

static inline char *_tfa_cont_profile_name
(struct tfa98xx *tfa98xx, int prof_idx)
{
	if (tfa98xx->tfa->cnt == NULL)
		return NULL;

	return tfa_cont_profile_name(tfa98xx->tfa->cnt,
		tfa98xx->tfa->dev_idx, prof_idx);
}

static enum tfa_error tfa98xx_write_re25(struct tfa_device *tfa, int value)
{
	enum tfa_error err;

	/* clear MTPEX */
	err = tfa_dev_mtp_set(tfa, TFA_MTP_EX, 0);
	if (err == tfa_error_ok) {
		/* set RE25 in shadow regiser */
		err = tfa_dev_mtp_set(tfa, TFA_MTP_RE25_PRIM, value);
	}
	if (err == tfa_error_ok) {
		/* set MTPEX to copy RE25 into MTP  */
		err = tfa_dev_mtp_set(tfa, TFA_MTP_EX, 2);
	}

	return err;
}

/* Wrapper for tfa start */
static enum tfa_error
tfa98xx_tfa_start(struct tfa98xx *tfa98xx, int next_profile, int vstep)
{
	enum tfa_error err;
	ktime_t start_time, stop_time;
	u64 delta_time;

	if (trace_level & 8)
		start_time = ktime_get_boottime();

	err = tfa_dev_start(tfa98xx->tfa, next_profile, vstep);

	if (trace_level & 8) {
		stop_time = ktime_get_boottime();
		delta_time = ktime_to_ns(ktime_sub(stop_time, start_time));
		do_div(delta_time, 1000);
		dev_dbg(&tfa98xx->i2c->dev, "tfa_dev_start(%d,%d) time = %lld us\n",
			next_profile, vstep, delta_time);
	}

	if ((err == tfa_error_ok) && (tfa98xx->set_mtp_cal)) {
		enum tfa_error err_cal;

		err_cal = tfa98xx_write_re25(tfa98xx->tfa, tfa98xx->cal_data);
		if (err_cal != tfa_error_ok) {
			pr_err("Error, setting calibration value in mtp, err=%d\n", err_cal);
		} else {
			tfa98xx->set_mtp_cal = false;
			pr_info("Calibration value (%d) set in mtp\n",
				tfa98xx->cal_data);
		}
	}

#if defined(USE_TFA9891)
	/* Check and update tap-detection state (in case of profile change) */
	tfa98xx_tapdet_check_update(tfa98xx);
#endif

	/* Remove sticky bit by reading it once */
	tfa_get_noclk(tfa98xx->tfa);

	/* A cold start erases the configuration, including interrupts setting.
	 * Restore it if required
	 */
	tfa98xx_interrupt_enable(tfa98xx, true);

	return err;
}

static int tfa98xx_input_open(struct input_dev *dev)
{
	struct tfa98xx *tfa98xx = input_get_drvdata(dev);
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	dev_dbg(tfa98xx->component->dev, "opening device file\n");
#else
	dev_dbg(tfa98xx->codec->dev, "opening device file\n");
#endif

	/* note: open function is called only once by the framework.
	 * No need to count number of open file instances.
	 */
	if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK) {
		dev_dbg(&tfa98xx->i2c->dev,
			"DSP not loaded, cannot start tap-detection\n");
		return -EIO;
	}

#if defined(USE_TFA9891)
	/* enable tap-detection service */
	tfa98xx->tapdet_open = true;
	tfa98xx_tapdet_check_update(tfa98xx);
#endif

	return 0;
}

static void tfa98xx_input_close(struct input_dev *dev)
{
	struct tfa98xx *tfa98xx = input_get_drvdata(dev);

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	dev_dbg(tfa98xx->component->dev, "closing device file\n");
#else
	dev_dbg(tfa98xx->codec->dev, "closing device file\n");
#endif

	/* Note: close function is called if the device is unregistered */

#if defined(USE_TFA9891)
	/* disable tap-detection service */
	tfa98xx->tapdet_open = false;
	tfa98xx_tapdet_check_update(tfa98xx);
#endif
}

static int tfa98xx_register_inputdev(struct tfa98xx *tfa98xx)
{
	int err;
	struct input_dev *input;

	input = input_allocate_device();

	if (!input) {
	#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
		dev_err(tfa98xx->component->dev,
			"Unable to allocate input device\n");
	#else
		dev_err(tfa98xx->codec->dev,
			"Unable to allocate input device\n");
	#endif
		return -ENOMEM;
	}

	input->evbit[0] = BIT_MASK(EV_KEY);
	input->keybit[BIT_WORD(BTN_0)] |= BIT_MASK(BTN_0);
	input->keybit[BIT_WORD(BTN_1)] |= BIT_MASK(BTN_1);
	input->keybit[BIT_WORD(BTN_2)] |= BIT_MASK(BTN_2);
	input->keybit[BIT_WORD(BTN_3)] |= BIT_MASK(BTN_3);
	input->keybit[BIT_WORD(BTN_4)] |= BIT_MASK(BTN_4);
	input->keybit[BIT_WORD(BTN_5)] |= BIT_MASK(BTN_5);
	input->keybit[BIT_WORD(BTN_6)] |= BIT_MASK(BTN_6);
	input->keybit[BIT_WORD(BTN_7)] |= BIT_MASK(BTN_7);
	input->keybit[BIT_WORD(BTN_8)] |= BIT_MASK(BTN_8);
	input->keybit[BIT_WORD(BTN_9)] |= BIT_MASK(BTN_9);

	input->open = tfa98xx_input_open;
	input->close = tfa98xx_input_close;

	input->name = "tfa98xx-tapdetect";

	input->id.bustype = BUS_I2C;
	input_set_drvdata(input, tfa98xx);

	err = input_register_device(input);
	if (err) {
	#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
		dev_err(tfa98xx->component->dev,
			"Unable to register input device\n");
	#else
		dev_err(tfa98xx->codec->dev,
			"Unable to register input device\n");
	#endif
		goto err_free_dev;
	}

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	dev_dbg(tfa98xx->component->dev,
		"Input device for tap-detection registered: %s\n",
		input->name);
#else
	dev_dbg(tfa98xx->codec->dev,
		"Input device for tap-detection registered: %s\n",
		input->name);
#endif
	tfa98xx->input = input;
	return 0;

err_free_dev:
	input_free_device(input);
	return err;
}

/*
 * Check if an input device for tap-detection can and shall be registered.
 * Register it if appropriate.
 * If already registered, check if still relevant and remove it if necessary.
 * unregister: true to request inputdev unregistration.
 */
static void
__tfa98xx_inputdev_check_register(struct tfa98xx *tfa98xx,
	bool unregister)
{
	bool tap_profile = false;
	unsigned int i;
	for (i = 0; i < tfa_cnt_get_dev_nprof(tfa98xx->tfa); i++) {
		if (strnstr(_tfa_cont_profile_name(tfa98xx, i), ".tap",
			strlen(_tfa_cont_profile_name(tfa98xx, i)))
			!= NULL) {
			tap_profile = true;
			tfa98xx->tapdet_profiles |= 1 << i;
		#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
			dev_info(tfa98xx->component->dev,
				"found a tap-detection profile (%d - %s)\n",
				i, _tfa_cont_profile_name(tfa98xx, i));
		#else
			dev_info(tfa98xx->codec->dev,
				"found a tap-detection profile (%d - %s)\n",
				i, _tfa_cont_profile_name(tfa98xx, i));
		#endif
		}
	}

	/* Check for device support:
	 *  - at device level
	 *  - at container (profile) level
	 */
	if (!(tfa98xx->flags & TFA98XX_FLAG_TAPDET_AVAILABLE) ||
		!tap_profile ||
		unregister) {
		/* No input device supported or required */
		if (tfa98xx->input) {
			input_unregister_device(tfa98xx->input);
			tfa98xx->input = NULL;
		}
		return;
	}

	/* input device required */
	if (tfa98xx->input)
	#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
		dev_info(tfa98xx->component->dev,
			"Input device already registered, skipping\n");
	#else
		dev_info(tfa98xx->codec->dev,
			"Input device already registered, skipping\n");
	#endif
	else
		tfa98xx_register_inputdev(tfa98xx);
}

static void tfa98xx_inputdev_check_register(struct tfa98xx *tfa98xx)
{
	__tfa98xx_inputdev_check_register(tfa98xx, false);
}

static void tfa98xx_inputdev_unregister(struct tfa98xx *tfa98xx)
{
	__tfa98xx_inputdev_check_register(tfa98xx, true);
}

#ifdef CONFIG_DEBUG_FS
/* OTC reporting
 * Returns the MTP0 OTC bit value
 */
static int tfa98xx_dbgfs_otc_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int value;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	value = tfa_dev_mtp_get(tfa98xx->tfa, TFA_MTP_OTC);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (value < 0) {
		pr_err("[0x%x] Unable to check DSP access: %d\n",
			tfa98xx->i2c->addr, value);
		return -EIO;
	}

	*val = value;
	pr_debug("[0x%x] OTC : %d\n", tfa98xx->i2c->addr, value);

	return 0;
}

static int tfa98xx_dbgfs_otc_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum tfa_error err;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	if (val != 0 && val != 1) {
		pr_err("[0x%x] Unexpected value %llu\n",
			tfa98xx->i2c->addr, val);
		return -EINVAL;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	err = tfa_dev_mtp_set(tfa98xx->tfa, TFA_MTP_OTC, val);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (err != tfa_error_ok) {
		pr_err("[0x%x] Unable to check DSP access: %d\n",
			tfa98xx->i2c->addr, err);
		return -EIO;
	}

	pr_debug("[0x%x] OTC < %llu\n", tfa98xx->i2c->addr, val);

	return 0;
}

static int tfa98xx_dbgfs_mtpex_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int value;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	value = tfa_dev_mtp_get(tfa98xx->tfa, TFA_MTP_EX);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (value < 0) {
		pr_err("[0x%x] Unable to check DSP access: %d\n",
			tfa98xx->i2c->addr, value);
		return -EIO;
	}


	*val = value;
	pr_debug("[0x%x] MTPEX : %d\n", tfa98xx->i2c->addr, value);

	return 0;
}

static int tfa98xx_dbgfs_mtpex_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum tfa_error err;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	if (val != 0) {
		pr_err("[0x%x] Can only clear MTPEX (0 value expected)\n",
			tfa98xx->i2c->addr);
		return -EINVAL;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	err = tfa_dev_mtp_set(tfa98xx->tfa, TFA_MTP_EX, val);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (err != tfa_error_ok) {
		pr_err("[0x%x] Unable to check DSP access: %d\n",
			tfa98xx->i2c->addr, err);
		tfa98xx->tfa->reset_mtpex = 1; /* suspend until TFA98xx is active */
		return -EIO;
	}

	pr_debug("[0x%x] MTPEX < 0\n", tfa98xx->i2c->addr);

	return 0;
}

static int tfa98xx_dbgfs_temp_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	*val = tfa98xx_get_exttemp(tfa98xx->tfa);
	mutex_unlock(&tfa98xx->dsp_lock);

	pr_debug("[0x%x] TEMP : %llu\n", tfa98xx->i2c->addr, *val);

	return 0;
}

static int tfa98xx_dbgfs_temp_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	tfa98xx_set_exttemp(tfa98xx->tfa, (short)val);
	tfa98xx->tfa->temp = (short)val;
	mutex_unlock(&tfa98xx->dsp_lock);

	pr_debug("[0x%x] TEMP < %llu\n", tfa98xx->i2c->addr, val);

	return 0;
}

static ssize_t tfa98xx_dbgfs_start_get(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	char *str;
	int ret = 0;
#if defined(TFA_DBGFS_CHECK_MTPEX)
	unsigned short value;
#endif

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	str = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

#if defined(TFA_DBGFS_CHECK_MTPEX)
	mutex_lock(&tfa98xx->dsp_lock);
	value = tfa_dev_mtp_get(tfa98xx->tfa, TFA_MTP_EX);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (value > 0) {
		tfa98xx->calibrate_done =
			(value) ? 1 : 0;
		pr_info("[0x%x] calibrate_done = MTPEX (%d)\n",
			tfa98xx->i2c->addr, tfa98xx->calibrate_done);

	} else {
		pr_info("[0x%x] error in reading MTPEX\n",
			tfa98xx->i2c->addr);
		tfa98xx->calibrate_done = 0;
	}
#endif

	if (tfa98xx->calibrate_done) {
		pr_info("[0x%x] Calibration Success\n", tfa98xx->i2c->addr);
		snprintf(str, PAGE_SIZE, "Success\n");
		ret = sizeof("Success");
	} else {
		pr_info("[0x%x] Calibration Fail\n", tfa98xx->i2c->addr);
		snprintf(str, PAGE_SIZE, "Fail\n");
		ret = sizeof("Fail");
	}
	//ret = sizeof(str);

	ret = simple_read_from_buffer(user_buf, count, ppos, str, ret);
	kfree(str);

	return ret;
}

static ssize_t tfa98xx_dbgfs_start_set(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum tfa_error ret;
	char buf[32];
	const char ref[] = "1"; /* "please calibrate now" */
	int buf_size, cal_profile = 0;
#if defined(TFA_DBGFS_CHECK_MTPEX)
	unsigned short value;
#endif
	u64 otc_val = 1;
	u16 temp_val = 25;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	pr_info("%s: begin\n", __func__);

	if (tfa98xx->pstream == 0) {
		pr_info("%s: Playback Fail. speaker init calibration Fail\n",
			__func__);
#if !defined(TFA_DBGFS_CHECK_MTPEX)
		tfa98xx->calibrate_done = 0;
#endif
		return count;
	}

#if defined(TFA_DBGFS_CHECK_MTPEX)
	if (!tfa98xx->calibrate_done) {
		mutex_lock(&tfa98xx->dsp_lock);
		value = tfa_dev_mtp_get(tfa98xx->tfa, TFA_MTP_EX);
		mutex_unlock(&tfa98xx->dsp_lock);

		if (value > 0) {
			tfa98xx->calibrate_done =
				(value) ? 1 : 0;
			pr_debug("[0x%x] calibrate_done = MTPEX (%d)\n",
				tfa98xx->i2c->addr, tfa98xx->calibrate_done);

		} else {
			pr_debug("[0x%x] error in reading MTPEX\n",
				tfa98xx->i2c->addr);
			tfa98xx->calibrate_done = 0;
		}
	}

	if (tfa98xx->calibrate_done) {
		pr_info("[0x%x] Calibration is already done (Success)\n",
			tfa98xx->i2c->addr);
		return count;
	}
#endif

	/* check string length, and account for eol */
	if (count > sizeof(ref) + 1 || count < (sizeof(ref) - 1))
		return -EINVAL;

	buf_size = min(count, (size_t)(sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	/* Compare string, excluding the trailing \0 and the potentials eol */
	if (strncmp(buf, ref, sizeof(ref) - 1))
		return -EINVAL;

	mutex_lock(&tfa98xx->dsp_lock);

	// OTC <0:always 1:once>
	ret = tfa_dev_mtp_set(tfa98xx->tfa, TFA_MTP_OTC, otc_val);
	if (ret)
		pr_info("setting OTC failed (%d)\n", ret);
	// MTPEX <reset to force to calibrate>
	ret = tfa_dev_mtp_set(tfa98xx->tfa, TFA_MTP_EX, 0);
	if (ret) {
		pr_info("resetting MTPEX failed (%d)\n", ret);
		tfa98xx->tfa->reset_mtpex = 1; /* suspend until TFA98xx is active */
	}
	// EXT_TEMP
#if defined(TFA_READ_BATTERY_TEMP)
	ret = tfa98xx_read_battery_temp(&temp_val);
	if (ret)
		pr_err("error in reading battery temp\n");
#endif
	tfa98xx_set_exttemp(tfa98xx->tfa, temp_val);
	tfa98xx->tfa->temp = temp_val;

	ret = tfa_calibrate(tfa98xx->tfa);
	if (ret == tfa_error_ok) {
		cal_profile = tfa_cont_get_cal_profile(tfa98xx->tfa);
		if (cal_profile < 0)
			pr_warn("[0x%x] Calibration profile not found\n",
				tfa98xx->i2c->addr);

		ret = tfa98xx_tfa_start(tfa98xx, cal_profile, tfa98xx->vstep);
	}

	if (ret == tfa_error_ok)
		tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_UNMUTE, 0);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (ret) {
		pr_info("[0x%x] Calibration start failed (%d)\n",
			tfa98xx->i2c->addr, ret);
		return -EIO;
	} else {
		pr_info("[0x%x] Calibration started\n",
			tfa98xx->i2c->addr);
	}

	pr_info("%s: end\n", __func__);

	return count;
}

static ssize_t tfa98xx_dbgfs_r_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	char *str;
	uint16_t status;
	int ret;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	mutex_lock(&tfa98xx->dsp_lock);

	/* Need to ensure DSP is access-able, use mtp read access for this
	 * purpose
	 */
	ret = tfa98xx_get_mtp(tfa98xx->tfa, &status);
	if (ret) {
		ret = -EIO;
		pr_err("[0x%x] MTP read failed\n", tfa98xx->i2c->addr);
		goto r_c_err;
	}

	ret = tfa_run_speaker_calibration(tfa98xx->tfa);
	if (ret) {
		ret = -EIO;
		pr_err("[0x%x] calibration failed\n", tfa98xx->i2c->addr);
		goto r_c_err;
	}

	str = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!str) {
		ret = -ENOMEM;
		pr_err("[0x%x] memory allocation failed\n", tfa98xx->i2c->addr);
		goto r_c_err;
	}

	if (tfa98xx->tfa->spkr_count > 1) {
		ret = snprintf(str, PAGE_SIZE,
			"Prim:%d mOhms, Sec:%d mOhms\n",
			tfa98xx->tfa->mohm[0],
			tfa98xx->tfa->mohm[1]);
	}
	else {
		ret = snprintf(str, PAGE_SIZE,
			"Prim:%d mOhms\n",
			tfa98xx->tfa->mohm[0]);
	}

	pr_debug("[0x%x] calib_done: %s", tfa98xx->i2c->addr, str);

	if (ret < 0)
		goto r_err;

	ret = simple_read_from_buffer(user_buf, count, ppos, str, ret);

r_err:
	kfree(str);
r_c_err:
	mutex_unlock(&tfa98xx->dsp_lock);
	return ret;
}

#if defined(TFA_READ_BATTERY_TEMP)
static enum tfa98xx_error tfa98xx_read_battery_temp(short *value)
{
	struct power_supply *psy;
	union power_supply_propval prop_read = {0};
	int ret = 0;

	/* get power supply of "battery" */
	/* value is preserved with default when error happens */
	psy = power_supply_get_by_name("battery");
	if (!psy) {
		pr_err("%s: failed to get power supply\n", __func__);
		return TFA98XX_ERROR_FAIL;
	}
#if KERNEL_VERSION(4, 1, 0) > LINUX_VERSION_CODE
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &prop_read);
#else
	if (!psy->desc) {
		pr_err("%s: failed to get desc of power supply\n", __func__);
		return TFA98XX_ERROR_FAIL;
	}

	ret = psy->desc->get_property(psy, POWER_SUPPLY_PROP_TEMP, &prop_read);
#endif
	if (!ret) {
		*value = (short)(prop_read.intval / 10); /* in degC */
		pr_info("%s: read battery temp (%d)\n", __func__, *value);
	} else {
		pr_err("%s: failed to get temp property\n", __func__);
		return TFA98XX_ERROR_FAIL;
	}

	return TFA98XX_ERROR_OK;
}
#endif /* TFA_READ_BATTERY_TEMP */

static ssize_t tfa98xx_dbgfs_version_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	char str[] = TFA98XX_VERSION "\n";
	int ret;

	ret = simple_read_from_buffer(user_buf, count, ppos, str, sizeof(str));

	return ret;
}

static ssize_t tfa98xx_dbgfs_dsp_state_get(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int ret = 0;
	char *str;

	switch (tfa98xx->dsp_init) {
	case TFA98XX_DSP_INIT_STOPPED:
		str = "Stopped\n";
		break;
	case TFA98XX_DSP_INIT_RECOVER:
		str = "Recover requested\n";
		break;
	case TFA98XX_DSP_INIT_FAIL:
		str = "Failed init\n";
		break;
	case TFA98XX_DSP_INIT_PENDING:
		str = "Pending init\n";
		break;
	case TFA98XX_DSP_INIT_DONE:
		str = "Init complete\n";
		break;
	default:
		str = "Invalid\n";
		break;
	}

	pr_debug("[0x%x] dsp_state : %s\n", tfa98xx->i2c->addr, str);

	ret = simple_read_from_buffer(user_buf, count, ppos, str, strlen(str));
	return ret;
}

static ssize_t tfa98xx_dbgfs_dsp_state_set(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum tfa_error ret;
	char buf[32];
	const char start_cmd[] = "start";
	const char stop_cmd[] = "stop";
	const char mon_start_cmd[] = "monitor start";
	const char mon_stop_cmd[] = "monitor stop";
	int buf_size;

	buf_size = min(count, (size_t)(sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	/* Compare strings, excluding the trailing \0 */
	if (!strncmp(buf, start_cmd, sizeof(start_cmd) - 1)) {
		pr_info("[0x%x] Manual triggering of dsp start...\n",
			tfa98xx->i2c->addr);
		mutex_lock(&tfa98xx->dsp_lock);
		ret = tfa98xx_tfa_start(tfa98xx,
			tfa98xx->profile, tfa98xx->vstep);
		mutex_unlock(&tfa98xx->dsp_lock);
		pr_debug("[0x%x] tfa_dev_start complete: %d\n",
			tfa98xx->i2c->addr, ret);
	} else if (!strncmp(buf, stop_cmd, sizeof(stop_cmd) - 1)) {
		pr_info("[0x%x] Manual triggering of dsp stop...\n",
			tfa98xx->i2c->addr);
		mutex_lock(&tfa98xx->dsp_lock);
		ret = tfa_dev_stop(tfa98xx->tfa);
		mutex_unlock(&tfa98xx->dsp_lock);
		pr_debug("[0x%x] tfa_dev_stop complete: %d\n",
			tfa98xx->i2c->addr, ret);
	} else if (!strncmp(buf, mon_start_cmd,
		sizeof(mon_start_cmd) - 1)) {
		pr_info("[0x%x] Manual start of monitor thread...\n",
			tfa98xx->i2c->addr);
		tfa98xx_monitor_count = -1;
		queue_delayed_work(tfa98xx->tfa98xx_wq,
			&tfa98xx->monitor_work, HZ);
	} else if (!strncmp(buf, mon_stop_cmd,
		sizeof(mon_stop_cmd) - 1)) {
		pr_info("[0x%x] Manual stop of monitor thread...\n",
			tfa98xx->i2c->addr);
		cancel_delayed_work_sync(&tfa98xx->monitor_work);
	} else {
		return -EINVAL;
	}

	return count;
}

static ssize_t tfa98xx_dbgfs_fw_state_get(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	char *str;

	switch (tfa98xx->dsp_fw_state) {
	case TFA98XX_DSP_FW_NONE:
		str = "None\n";
		break;
	case TFA98XX_DSP_FW_PENDING:
		str = "Pending\n";
		break;
	case TFA98XX_DSP_FW_FAIL:
		str = "Fail\n";
		break;
	case TFA98XX_DSP_FW_OK:
		str = "Ok\n";
		break;
	default:
		str = "Invalid\n";
		break;
	}

	pr_debug("[0x%x] fw_state : %s", tfa98xx->i2c->addr, str);

	return simple_read_from_buffer(user_buf, count, ppos, str, strlen(str));
}

static ssize_t tfa98xx_dbgfs_rpc_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int ret = 0;
	uint8_t *buffer;
	enum tfa98xx_error error;

	if (tfa98xx->tfa == NULL) {
		pr_debug("[0x%x] dsp is not available\n", tfa98xx->i2c->addr);
		return -ENODEV;
	}

	if (count == 0)
		return 0;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	buffer = kmalloc(count, GFP_KERNEL);
	if (buffer == NULL) {
		ret = -ENOMEM;
		pr_debug("[0x%x] can not allocate memory\n",
			tfa98xx->i2c->addr);
		return ret;
	}

	pr_info("%s called\n", __func__);
	mutex_lock(&tfa98xx->dsp_lock);
	tfa98xx->tfa->individual_msg = 1;
	error = dsp_msg_read(tfa98xx->tfa, count, buffer);
	tfa98xx->tfa->individual_msg = 0;
	mutex_unlock(&tfa98xx->dsp_lock);
	if (error != TFA98XX_ERROR_OK) {
		pr_debug("[0x%x] dsp_msg_read error: %d\n",
			tfa98xx->i2c->addr, error);
		kfree(buffer);
		return -EFAULT;
	}

	ret = copy_to_user(user_buf, buffer, count);
	kfree(buffer);
	if (ret)
		return -EFAULT;

	*ppos += count;
	return count;
}

static ssize_t tfa98xx_dbgfs_rpc_send(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	struct tfa_file_dsc *msg_file;
	enum tfa98xx_error error;
	int ret = 0;

	if (tfa98xx->tfa == NULL) {
		pr_debug("[0x%x] dsp is not available\n", tfa98xx->i2c->addr);
		return -ENODEV;
	}

	if (count == 0)
		return 0;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	/* msg_file.name is not used */
	msg_file = kmalloc(count + sizeof(struct tfa_file_dsc), GFP_KERNEL);
	if (msg_file == NULL) {
		ret = -ENOMEM;
		pr_debug("[0x%x] can not allocate memory\n",
			tfa98xx->i2c->addr);
		return ret;
	}
	msg_file->size = (uint32_t)count;

	if (copy_from_user(msg_file->data, user_buf, count)) {
		kfree(msg_file);
		return -EFAULT;
	}

	pr_info("%s called\n", __func__);
	mutex_lock(&tfa98xx->dsp_lock);
	tfa98xx->tfa->individual_msg = 1;
	if ((msg_file->data[0] == 'M') && (msg_file->data[1] == 'G')) {
		/* int vstep_idx, int vstep_msg_idx both 0 */
		error = tfa_cont_write_file(tfa98xx->tfa,
			msg_file, 0, 0);
		if (error != TFA98XX_ERROR_OK) {
			pr_debug("[0x%x] tfa_cont_write_file error: %d\n",
				tfa98xx->i2c->addr, error);
			ret = -EIO;
		}
	} else {
		error = dsp_msg(tfa98xx->tfa, msg_file->size, msg_file->data);
		if (error != TFA98XX_ERROR_OK) {
			pr_debug("[0x%x] dsp_msg error: %d\n",
				tfa98xx->i2c->addr, error);
			ret = -EIO;
		}
	}
	tfa98xx->tfa->individual_msg = 0;
	mutex_unlock(&tfa98xx->dsp_lock);

	kfree(msg_file);

	if (ret)
		return ret;

	return count;
}
/* -- RPC */

/* ++ DSP message fops */
static ssize_t tfa98xx_dbgfs_dsp_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum tfa98xx_error error;
	int ret = 0;
	uint8_t *buffer;

	if (tfa98xx->tfa == NULL) {
		pr_debug("[0x%x] dsp is not available\n", tfa98xx->i2c->addr);
		return -ENODEV;
	}

	if (count == 0)
		return 0;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	buffer = kmalloc(count, GFP_KERNEL);
	if (buffer == NULL) {
		ret = -ENOMEM;
		pr_debug("[0x%x] can not allocate memory\n",
			tfa98xx->i2c->addr);
		return ret;
	}

	pr_info("%s called\n", __func__);
	mutex_lock(&tfa98xx->dsp_lock);
	tfa98xx->tfa->individual_msg = 1;
	error = dsp_msg_read(tfa98xx->tfa, count, buffer);
	tfa98xx->tfa->individual_msg = 0;
	mutex_unlock(&tfa98xx->dsp_lock);
	if (error) {
		pr_debug("[0x%x] dsp_msg_read error: %d\n",
			tfa98xx->i2c->addr, error);
		kfree(buffer);
		return -EFAULT;
	}

	/* ret = simple_read_from_buffer
	 * (user_buf, count, ppos, buffer, count);
	 */
	ret = copy_to_user(user_buf, buffer, count);
	if (ret) {
		pr_debug("[0x%x] cannot copy buffer to user: %d\n",
			tfa98xx->i2c->addr, ret);
		kfree(buffer);
		return -EFAULT;
	}

	kfree(buffer);

	return count;
}

static ssize_t tfa98xx_dbgfs_dsp_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum tfa98xx_error error;
	int ret = 0;
	uint8_t *buffer;

	if (tfa98xx->tfa == NULL) {
		pr_debug("[0x%x] dsp is not available\n", tfa98xx->i2c->addr);
		return -ENODEV;
	}

	if (count == 0)
		return 0;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	buffer = kmalloc(count, GFP_KERNEL);
	if (buffer == NULL) {
		ret = -ENOMEM;
		pr_debug("[0x%x] can not allocate memory\n",
			tfa98xx->i2c->addr);
		return  ret;
	}

	ret = copy_from_user(buffer, user_buf, count);
	if (ret) {
		pr_debug("[0x%x] cannot copy buffer from user: %d\n",
			tfa98xx->i2c->addr, ret);
		kfree(buffer);
		return -EFAULT;
	}

	pr_info("%s called\n", __func__);
	mutex_lock(&tfa98xx->dsp_lock);
	tfa98xx->tfa->individual_msg = 1;
	error = dsp_msg(tfa98xx->tfa, count, buffer);
	tfa98xx->tfa->individual_msg = 0;
	mutex_unlock(&tfa98xx->dsp_lock);
	if (error) {
		pr_debug("[0x%x] dsp_msg error: %d\n",
			tfa98xx->i2c->addr, error);
		kfree(buffer);
		return -EFAULT;
	}

	kfree(buffer);

	return count;
}
/* -- DSP */

static ssize_t tfa98xx_dbgfs_spkr_damaged_get(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int ret = 0;
	char *str;

	str = kmalloc(count, GFP_KERNEL);
	if (str == NULL) {
		ret = -ENOMEM;
		pr_debug("[0x%x] can not allocate memory\n",
			tfa98xx->i2c->addr);
		return ret;
	}

	scnprintf(str, PAGE_SIZE, "%s\n",
		(tfa98xx->tfa->spkr_damaged == 1)
		? "damaged" : "ready");

	ret = simple_read_from_buffer(user_buf, count, ppos, str, strlen(str));

	kfree(str);

	return ret;
}

static int tfa98xx_dbgfs_pga_gain_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	unsigned int value;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	value = tfa_get_pga_gain(tfa98xx->tfa);
	if (value < 0)
		return -EINVAL;
	*val = value;

	return 0;
}

static int tfa98xx_dbgfs_pga_gain_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	uint16_t value;
	int err;

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	value = val & 0xffff;
	if (value > 7)
		return -EINVAL;

	err = tfa_set_pga_gain(tfa98xx->tfa, value);
	if (err < 0)
		return -EINVAL;

	return 0;
}

static ssize_t tfa98xx_dbgfs_trace_level_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	char out_buf[4] = {0};

	if (count < 1) {
		pr_err("%s: read size exceeds buf size %zd\n", __func__, count);
		return 0;
	}
	snprintf(out_buf, 4, "%d\n", trace_level);

	return simple_read_from_buffer(user_buf, count, ppos, out_buf, sizeof(out_buf));
}

static ssize_t tfa98xx_dbgfs_trace_level_write(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int tl = 0;
	char buf[2] = {0};

	if (copy_from_user(buf, user_buf, 1))
		return -EFAULT;

	tl = buf[0] - 48;
	pr_info("%s: trace_level = %d\n", __func__, tl);
	if (tl < 0 || tl > 15)
		return -EFAULT;

	trace_level = tl;

	// 3 : reset MTP and MTPEX
	// 5 : disable dummy calibration
	// 7 : do not write MTP/MTPEX
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		if (tfa98xx && tfa98xx->tfa) {
			tfa98xx->tfa->verbose = trace_level;
			if (trace_level == 3) { // reset MTP and MTPEX
				tfa_dev_mtp_set(tfa98xx->tfa, TFA_MTP_RE25, 0);
				tfa_dev_mtp_set(tfa98xx->tfa, TFA_MTP_EX, 0);
				tfa98xx->tfa->verbose = 5;
			}
		}
	}

	return count;
}

static ssize_t tfa98xx_dbgfs_show_cal_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);

	int mtp = 0;
	int mtpex = 0;
	char out_buf[64] = {0};

	if (tfa98xx->tfa->tfa_family == 0) {
		pr_err("[0x%x] %s: system is not initialized: not probed yet!\n",
			tfa98xx->i2c->addr, __func__);
		return -EIO;
	}

	if (count < 1) {
		pr_err("%s: read size exceeds buf size %zd\n", __func__, count);
		return 0;
	}

	mtp = tfa_dev_mtp_get(tfa98xx->tfa, TFA_MTP_RE25);
	mtpex = TFA_GET_BF(tfa98xx->tfa, MTPEX);

	snprintf(out_buf, 64, "[%s] MTPEX: %d, MTP: %d mOhm\n",
		tfa_cont_device_name(tfa98xx->tfa->cnt, tfa98xx->tfa->dev_idx),
		mtpex, mtp);

	return  simple_read_from_buffer(user_buf,
		count, ppos, out_buf, sizeof(out_buf));
}

/* Direct registers access - provide register address in hex */
#define TFA98XX_DEBUGFS_REG_SET(__reg)	\
static int tfa98xx_dbgfs_reg_##__reg##_set(void *data, u64 val) \
{ \
	struct i2c_client *i2c = (struct i2c_client *)data; \
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c); \
	unsigned int ret, value; \
 \
	ret = regmap_write(tfa98xx->regmap, 0x##__reg, (val & 0xffff)); \
	value = val & 0xffff; \
	return 0; \
} \
static int tfa98xx_dbgfs_reg_##__reg##_get(void *data, u64 *val) \
{ \
	struct i2c_client *i2c = (struct i2c_client *)data; \
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c); \
	unsigned int value; \
	int ret; \
 \
	ret = regmap_read(tfa98xx->regmap, 0x##__reg, &value); \
	*val = value; \
	return 0; \
} \
DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_reg_##__reg##_fops, \
	tfa98xx_dbgfs_reg_##__reg##_get, \
	tfa98xx_dbgfs_reg_##__reg##_set, "0x%llx\n")

#define VAL(str) #str
#define TOSTRING(str) VAL(str)
#define TFA98XX_DEBUGFS_REG_CREATE_FILE(__reg, __name)	\
	debugfs_create_file(TOSTRING(__reg) "-" TOSTRING(__name), \
		S_IRUGO|S_IWUSR|S_IWGRP, dbg_reg_dir, \
		i2c, &tfa98xx_dbgfs_reg_##__reg##_fops);

TFA98XX_DEBUGFS_REG_SET(00);
TFA98XX_DEBUGFS_REG_SET(01);
TFA98XX_DEBUGFS_REG_SET(02);
TFA98XX_DEBUGFS_REG_SET(03);
TFA98XX_DEBUGFS_REG_SET(04);
TFA98XX_DEBUGFS_REG_SET(05);
TFA98XX_DEBUGFS_REG_SET(06);
TFA98XX_DEBUGFS_REG_SET(07);
TFA98XX_DEBUGFS_REG_SET(08);
TFA98XX_DEBUGFS_REG_SET(09);
TFA98XX_DEBUGFS_REG_SET(0A);
TFA98XX_DEBUGFS_REG_SET(0B);
TFA98XX_DEBUGFS_REG_SET(0F);
TFA98XX_DEBUGFS_REG_SET(10);
TFA98XX_DEBUGFS_REG_SET(11);
TFA98XX_DEBUGFS_REG_SET(12);
TFA98XX_DEBUGFS_REG_SET(13);
TFA98XX_DEBUGFS_REG_SET(22);
TFA98XX_DEBUGFS_REG_SET(25);

DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_calib_otc_fops,
	tfa98xx_dbgfs_otc_get,
	tfa98xx_dbgfs_otc_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_calib_mtpex_fops,
	tfa98xx_dbgfs_mtpex_get,
	tfa98xx_dbgfs_mtpex_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_calib_temp_fops,
	tfa98xx_dbgfs_temp_get,
	tfa98xx_dbgfs_temp_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_pga_gain_fops,
	tfa98xx_dbgfs_pga_gain_get,
	tfa98xx_dbgfs_pga_gain_set, "%llu\n");

static const struct file_operations tfa98xx_dbgfs_calib_start_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_start_get,
	.write = tfa98xx_dbgfs_start_set,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_r_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_r_read,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_version_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_version_read,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_dsp_state_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_dsp_state_get,
	.write = tfa98xx_dbgfs_dsp_state_set,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_fw_state_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_fw_state_get,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_rpc_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_rpc_read,
	.write = tfa98xx_dbgfs_rpc_send,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_dsp_fops = {
	.open = simple_open,
	.read = tfa98xx_dbgfs_dsp_read,
	.write = tfa98xx_dbgfs_dsp_write,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_spkr_damaged_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_spkr_damaged_get,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_trace_level_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_trace_level_read,
	.write = tfa98xx_dbgfs_trace_level_write,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_show_cal_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_show_cal_read,
	.llseek = default_llseek,
};

static void tfa98xx_debug_init(struct tfa98xx *tfa98xx, struct i2c_client *i2c)
{
	char name[50];

	scnprintf(name, MAX_CONTROL_NAME, "%s-%x", i2c->name, i2c->addr);
	tfa98xx->dbg_dir = debugfs_create_dir(name, NULL);
	debugfs_create_file("OTC", S_IRUGO|S_IWUSR|S_IWGRP, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_calib_otc_fops);
	debugfs_create_file("MTPEX", S_IRUGO|S_IWUSR|S_IWGRP, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_calib_mtpex_fops);
	debugfs_create_file("TEMP", S_IRUGO|S_IWUSR|S_IWGRP, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_calib_temp_fops);
	debugfs_create_file("calibrate", S_IRUGO|S_IWUSR|S_IWGRP, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_calib_start_fops);
	debugfs_create_file("R", S_IRUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_r_fops);
	debugfs_create_file("version", S_IRUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_version_fops);
	debugfs_create_file("dsp-state", S_IRUGO|S_IWUSR|S_IWGRP, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_dsp_state_fops);
	debugfs_create_file("fw-state", S_IRUGO|S_IWUSR|S_IWGRP, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_fw_state_fops);
	debugfs_create_file("rpc", S_IRUGO|S_IWUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_rpc_fops);
	debugfs_create_file("dsp", S_IRUGO|S_IWUSR, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_dsp_fops);

	if (tfa98xx->flags & TFA98XX_FLAG_SAAM_AVAILABLE) {
		dev_dbg(tfa98xx->dev, "Adding pga_gain debug interface\n");
		debugfs_create_file("pga_gain", S_IRUGO, tfa98xx->dbg_dir,
			tfa98xx->i2c,
			&tfa98xx_dbgfs_pga_gain_fops);
	}

	debugfs_create_file("trace-level", S_IRUGO|S_IWUSR,
		tfa98xx->dbg_dir,
		tfa98xx->i2c,
		&tfa98xx_dbgfs_trace_level_fops);

	debugfs_create_file("mtp", S_IRUGO|S_IWUSR,
		tfa98xx->dbg_dir,
		tfa98xx->i2c,
		&tfa98xx_dbgfs_show_cal_fops);
}

static void tfa98xx_debug_remove(struct tfa98xx *tfa98xx)
{
	debugfs_remove_recursive(tfa98xx->dbg_dir);
}
#endif

static void tfa98xx_set_dsp_configured(struct tfa98xx *tfa98xx)
{
	static int is_configured; /* reset by default */

	switch (tfa98xx->dsp_init) {
	case TFA98XX_DSP_INIT_DONE:
	case TFA98XX_DSP_INIT_INVALIDATED:
	case TFA98XX_DSP_INIT_RECOVER:
		/* set working if already running */
		is_configured = 1;
		break;
	case TFA98XX_DSP_INIT_STOPPED:
		if (tfa98xx->tfa->is_probus_device) {
			/* preserve state except pstream off */
			if (tfa98xx->pstream == 0)
				is_configured = 0;
		} else {
			/* reset for embedded DSP case */
			is_configured = 0;
		}
		break;
	case TFA98XX_DSP_INIT_FAIL:
	case TFA98XX_DSP_INIT_PENDING:
	default:
		is_configured = 0;
		break;
	}

	pr_debug("%s: dsp_init %d, is_configured %d\n", __func__,
		tfa98xx->dsp_init, is_configured);

	tfa98xx->tfa->is_configured = is_configured;

	return;
}

static int tfa98xx_get_vstep(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *component
		= snd_soc_kcontrol_component(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(component);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	int mixer_profile = kcontrol->private_value;
	int ret = 0;
	int profile;

	profile = get_profile_id_for_sr(mixer_profile, tfa98xx->rate);
	if (profile < 0) {
		pr_err("%s: invalid profile %d (mixer_profile=%d, rate=%d)\n",
			__func__, profile, mixer_profile, tfa98xx->rate);
		return -EINVAL;
	}

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		int vstep = tfa98xx->prof_vsteps[profile];

		ucontrol->value.integer.value[tfa98xx->tfa->dev_idx]
			= tfa_cont_get_max_vstep(tfa98xx->tfa, profile)
			- vstep - 1;
	}
	mutex_unlock(&tfa98xx_mutex);

	return ret;
}

static int tfa98xx_set_vstep(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *component
		= snd_soc_kcontrol_component(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(component);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	int mixer_profile = kcontrol->private_value;
	int profile;
	int err = 0;
	int change = 0;
#if defined(TFA_MUTE_DURING_SWITCHING_PROFILE)
#if defined(TFA_RAMPDOWN_BEFORE_MUTE)
	int i = 0;
#endif
#endif

	if (no_start != 0)
		return 0;

	profile = get_profile_id_for_sr(mixer_profile, tfa98xx->rate);
	if (profile < 0) {
		pr_err("%s: invalid profile %d (mixer_profile=%d, rate=%d)\n",
			__func__, profile, mixer_profile, tfa98xx->rate);
		return -EINVAL;
	}

	/* wait until when DSP is ready for initialization */
	if (tfa98xx->pstream == 0 && tfa98xx->samstream == 0) {
		pr_info("%s: tfa_start is suspended when only cstream is on\n",
			__func__);
		return 0;
	}

	mutex_lock(&tfa98xx_mutex);

#if defined(TFA_MUTE_DURING_SWITCHING_PROFILE)
	change = 1;
#if defined(TFA_RAMPDOWN_BEFORE_MUTE)
	for (i = 0; i < RAMPDOWN_MAX; i++) {
		err = 0;
		list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
			mutex_lock(&tfa98xx->dsp_lock);
			err += tfa_gain_rampdown(tfa98xx->tfa, i, RAMPDOWN_MAX);
			mutex_unlock(&tfa98xx->dsp_lock);
		}
		if (err == tfa98xx_device_count * TFA98XX_ERROR_OTHER)
			break;
		msleep_interruptible(1);
	}
#endif /* TFA_RAMPDOWN_BEFORE_MUTE */
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		mutex_lock(&tfa98xx->dsp_lock);
		show_current_state(tfa98xx->tfa);
		tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_MUTE, 0);
		mutex_unlock(&tfa98xx->dsp_lock);
	}
#endif /* TFA_MUTE_DURING_SWITCHING_PROFILE */

	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		int vstep, vsteps;
		int ready = 0;
		int new_vstep;
		int value = ucontrol->value
			.integer.value[tfa98xx->tfa->dev_idx];

		vstep = tfa98xx->prof_vsteps[profile];
		vsteps = tfa_cont_get_max_vstep(tfa98xx->tfa, profile);

		if (vstep == vsteps - value - 1)
			continue;

		new_vstep = vsteps - value - 1;

		if (new_vstep < 0)
			new_vstep = 0;

		tfa98xx->prof_vsteps[profile] = new_vstep;

#ifndef TFA98XX_ALSA_CTRL_PROF_CHG_ON_VOL
		if (profile == tfa98xx->profile) {
#endif
			/* this is the active profile, program the new vstep */
			tfa98xx->vstep = new_vstep;
			mutex_lock(&tfa98xx->dsp_lock);
			tfa98xx_dsp_system_stable(tfa98xx->tfa, &ready);

			if (ready) {
				err = tfa98xx_tfa_start(tfa98xx,
					tfa98xx->profile, tfa98xx->vstep);
				if (err) {
					pr_err("Write vstep error: %d\n", err);
				} else {
					pr_debug("Succesfully changed vstep index!\n");
					change = 1;
				}
			}
			mutex_unlock(&tfa98xx->dsp_lock);
#ifndef TFA98XX_ALSA_CTRL_PROF_CHG_ON_VOL
		}
#endif
		pr_debug("%d: vstep:%d, (control value: %d) - profile %d\n",
			tfa98xx->tfa->dev_idx, new_vstep, value, profile);
	}

	if (!change) {
		mutex_unlock(&tfa98xx_mutex);
		return change;
	}

	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		mutex_lock(&tfa98xx->dsp_lock);
		tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_UNMUTE, 0);
		mutex_unlock(&tfa98xx->dsp_lock);
	}

	mutex_unlock(&tfa98xx_mutex);

	return change;
}

static int tfa98xx_info_vstep(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *component
		= snd_soc_kcontrol_component(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(component);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	int mixer_profile = tfa98xx_mixer_profile;
	int profile = get_profile_id_for_sr(mixer_profile, tfa98xx->rate);

	if (profile < 0) {
		pr_err("%s: invalid profile %d (mixer_profile=%d, rate=%d)\n",
			__func__, profile, mixer_profile, tfa98xx->rate);
		return -EINVAL;
	}

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	mutex_lock(&tfa98xx_mutex);
	uinfo->count = tfa98xx_device_count;
	mutex_unlock(&tfa98xx_mutex);
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max
		= max(0, tfa_cont_get_max_vstep(tfa98xx->tfa, profile) - 1);
	pr_debug("vsteps count: %d [prof=%d]\n",
		tfa_cont_get_max_vstep(tfa98xx->tfa, profile),
		profile);
	return 0;
}

static int tfa98xx_get_profile(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&tfa98xx_mutex);
	ucontrol->value.integer.value[0] = tfa98xx_mixer_profile;
	mutex_unlock(&tfa98xx_mutex);

	return 0;
}

static int tfa98xx_set_profile(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *component
		= snd_soc_kcontrol_component(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(component);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	int err = 0;
	int change = 0;
	int new_profile;
	int prof_idx;
	int profile_count = tfa98xx_mixer_profiles;
	int profile = tfa98xx_mixer_profile;
#if defined(TFA_MUTE_DURING_SWITCHING_PROFILE)
#if defined(TFA_RAMPDOWN_BEFORE_MUTE)
	int i = 0;
#endif
#endif

	if (no_start != 0)
		return 0;

	new_profile = ucontrol->value.integer.value[0];
	if (new_profile == profile)
		return 0;

	if ((new_profile < 0) || (new_profile >= profile_count)) {
		pr_err("not existing profile (%d)\n", new_profile);
		return -EINVAL;
	}

	/* get the container profile for the requested sample rate */
	prof_idx = get_profile_id_for_sr(new_profile, tfa98xx->rate);
	if (prof_idx < 0) {
		pr_err("tfa98xx: sample rate [%d] not supported for this mixer profile [%d].\n",
			tfa98xx->rate, new_profile);
		return 0;
	}
	pr_debug("selected container profile [%d]\n", prof_idx);

	/* update mixer profile */
	tfa98xx_mixer_profile = new_profile;

	/* wait until when DSP is ready for initialization */
	if (tfa98xx->pstream == 0 && tfa98xx->samstream == 0) {
		pr_info("%s: tfa_start is suspended when only cstream is on\n",
			__func__);
		return 0;
	}

	mutex_lock(&tfa98xx_mutex);

#if defined(TFA_MUTE_DURING_SWITCHING_PROFILE)
	change = 1;
#if defined(TFA_RAMPDOWN_BEFORE_MUTE)
	for (i = 0; i < RAMPDOWN_MAX; i++) {
		err = 0;
		list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
			mutex_lock(&tfa98xx->dsp_lock);
			err += tfa_gain_rampdown(tfa98xx->tfa, i, RAMPDOWN_MAX);
			mutex_unlock(&tfa98xx->dsp_lock);
		}
		if (err == tfa98xx_device_count * TFA98XX_ERROR_OTHER)
			break;
		msleep_interruptible(1);
	}
#endif /* TFA_RAMPDOWN_BEFORE_MUTE */
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		mutex_lock(&tfa98xx->dsp_lock);
		show_current_state(tfa98xx->tfa);
		tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_MUTE, 0);
		mutex_unlock(&tfa98xx->dsp_lock);
	}
#endif /* TFA_MUTE_DURING_SWITCHING_PROFILE */

	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		int ready = 0;

		/* update 'real' profile (container profile) */
		tfa98xx->profile = prof_idx;
		tfa98xx->vstep = tfa98xx->prof_vsteps[prof_idx];

		mutex_lock(&tfa98xx->dsp_lock);
#if defined(TFA_TRIGGER_AT_SET_PROFILE_WITH_CLK)
		/* Don't call tfa_dev_start() if there is no clock */
		tfa98xx_dsp_system_stable(tfa98xx->tfa, &ready);
		pr_debug("%s: device is %sready\n", __func__,
			ready ? "" : "not ");
#else
		/* Set ready by force, for selective channel control */
		ready = 1;
#endif
		if (ready) {
			/* Also re-enables the interrupts */
			pr_info("%s: trigger [dev %d - prof %d]\n", __func__,
				tfa98xx->tfa->dev_idx, prof_idx);
			err = tfa98xx_tfa_start(tfa98xx, prof_idx, tfa98xx->vstep);
			if (err) {
				pr_info("Write profile error: %d\n", err);
			} else {
				pr_debug("Changed to profile %d (vstep = %d)\n",
					prof_idx, tfa98xx->vstep);
				change = 1;
			}
		}

		/* Flag DSP as invalidated as the profile change may invalidate the
		 * current DSP configuration. That way, further stream start can
		 * trigger a tfa_dev_start.
		 */
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_INVALIDATED;
		tfa98xx_set_dsp_configured(tfa98xx);
		mutex_unlock(&tfa98xx->dsp_lock);
	}

	if (!change) {
		mutex_unlock(&tfa98xx_mutex);
		return change;
	}

	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		mutex_lock(&tfa98xx->dsp_lock);
		tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_UNMUTE, 0);
		mutex_unlock(&tfa98xx->dsp_lock);
	}

	mutex_unlock(&tfa98xx_mutex);

	return change;
}

/* copies the profile basename (i.e. part until .) into buf */
static void get_profile_basename(char *buf, char *profile)
{
	int cp_len = 0, idx = 0;
	char *pch;

	pch = strnchr(profile, strlen(profile), '.');
	idx = pch - profile;
	cp_len = (pch != NULL) ? idx : (int)strlen(profile);
	memcpy(buf, profile, cp_len);
	buf[cp_len] = 0;
}

/* return the profile name accociated with id from the profile list */
static int get_profile_from_list(char *buf, int id)
{
	struct tfa98xx_baseprofile *bprof;

	list_for_each_entry(bprof, &profile_list, list) {
		if (bprof->item_id == id) {
			strlcpy(buf, bprof->basename, MAX_CONTROL_NAME);
			return 0;
		}
	}

	return TFA_ERROR;
}

/* search for the profile in the profile list */
static int is_profile_in_list(char *profile, int len)
{
	struct tfa98xx_baseprofile *bprof;

	list_for_each_entry(bprof, &profile_list, list) {
		if ((len == bprof->len)
			&& (strncmp(bprof->basename, profile, len) == 0))
			return 1;
	}

	return 0;
}

/*
 * for the profile with id, look if the requested samplerate is
 * supported, if found return the (container)profile for this
 * samplerate, on error or if not found return -1
 */
static int get_profile_id_for_sr(int id, unsigned int rate)
{
	int idx = 0;
	struct tfa98xx_baseprofile *bprof;

	list_for_each_entry(bprof, &profile_list, list) {
		if (id == bprof->item_id) {
			idx = tfa98xx_get_fssel(rate);
			if (idx < 0) {
				/* samplerate not supported */
				return TFA_ERROR;
			}

			return bprof->sr_rate_sup[idx];
		}
	}

	/* profile not found */
	return TFA_ERROR;
}

/* check if this profile is a calibration profile */
static int is_calibration_profile(char *profile)
{
	if (strnstr(profile, ".cal", strlen(profile)) != NULL)
		return 1;
	return 0;
}

/*
 * adds the (container)profile index of the samplerate found in
 * the (container)profile to a fixed samplerate table in the (mixer)profile
 */
static int add_sr_to_profile(struct tfa98xx *tfa98xx,
	char *basename, int len, int profile)
{
	struct tfa98xx_baseprofile *bprof;
	int idx = 0;
	unsigned int sr = 0;

	list_for_each_entry(bprof, &profile_list, list) {
		if ((len == bprof->len)
			&& (strncmp(bprof->basename, basename, len) == 0)) {
			/* add supported samplerate for this profile */
			sr = tfa98xx_get_profile_sr(tfa98xx->tfa, profile);
			if (!sr) {
				pr_err("unable to identify supported sample rate for %s\n",
					bprof->basename);
				return TFA_ERROR;
			}
#if defined(TFA_FULL_RATE_SUPPORT_WITH_POST_CONVERSION)
			if (sr_converted != sr) {
				pr_info("sr_converted: %d to %d\n",
					sr_converted, sr);
				sr_converted = sr;
			}
#endif

			/* get the index for this samplerate */
			idx = tfa98xx_get_fssel(sr);
			if (idx < 0 || idx >= TFA98XX_NUM_RATES) {
				pr_err("invalid index for samplerate %d\n",
					idx);
				return TFA_ERROR;
			}

			/* enter the (container)profile for this samplerate
			 * at the corresponding index
			 */
			bprof->sr_rate_sup[idx] = profile;

			pr_debug("added profile:samplerate = [%d:%d] for mixer profile: %s\n",
				profile, sr, bprof->basename);
		}
	}

	return 0;
}

#if KERNEL_VERSION(3, 16, 0) > LINUX_VERSION_CODE
static struct snd_soc_codec *snd_soc_kcontrol_codec(struct snd_kcontrol *kcontrol)
{
	return snd_kcontrol_chip(kcontrol);
}
#endif

static int tfa98xx_info_profile(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	char profile_name[MAX_CONTROL_NAME] = {0};
	int count = tfa98xx_mixer_profiles, err = -1;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	err = get_profile_from_list(profile_name,
		uinfo->value.enumerated.item);
	if (err != 0)
		return -EINVAL;

	strlcpy(uinfo->value.enumerated.name,
		profile_name, MAX_CONTROL_NAME);

	return 0;
}

static int tfa98xx_info_stop_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	mutex_lock(&tfa98xx_mutex);
	uinfo->count = tfa98xx_device_count;
	mutex_unlock(&tfa98xx_mutex);
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	return 0;
}

static int tfa98xx_get_stop_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tfa98xx *tfa98xx;

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		ucontrol->value.integer.value[tfa98xx->tfa->dev_idx] = 0;
	}
	mutex_unlock(&tfa98xx_mutex);

	return 0;
}

static int tfa98xx_set_stop_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tfa98xx *tfa98xx;

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		int ready = 0;
		int i = tfa98xx->tfa->dev_idx;

		pr_debug("%d: %ld\n", i, ucontrol->value.integer.value[i]);

		tfa98xx_dsp_system_stable(tfa98xx->tfa, &ready);

		if ((ucontrol->value.integer.value[i] != 0) && ready) {
			cancel_delayed_work_sync(&tfa98xx->monitor_work);

			cancel_delayed_work_sync(&tfa98xx->init_work);
			if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK)
				continue;

			mutex_lock(&tfa98xx->dsp_lock);
			tfa_dev_stop(tfa98xx->tfa);
			tfa98xx->dsp_init = TFA98XX_DSP_INIT_STOPPED;
			tfa98xx_set_dsp_configured(tfa98xx);
			mutex_unlock(&tfa98xx->dsp_lock);
		}

		ucontrol->value.integer.value[i] = 0;
	}
	mutex_unlock(&tfa98xx_mutex);

	return 1;
}

static int tfa98xx_info_cal_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	mutex_lock(&tfa98xx_mutex);
	uinfo->count = tfa98xx_device_count;
	mutex_unlock(&tfa98xx_mutex);
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xffff; /* 16 bit value */

	return 0;
}

static int tfa98xx_set_cal_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tfa98xx *tfa98xx;

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		enum tfa_error err;
		int i = tfa98xx->tfa->dev_idx;

		tfa98xx->cal_data = (uint16_t)ucontrol->value.integer.value[i];

		mutex_lock(&tfa98xx->dsp_lock);
		err = tfa98xx_write_re25(tfa98xx->tfa, tfa98xx->cal_data);
		tfa98xx->set_mtp_cal = (err != tfa_error_ok);
		if (tfa98xx->set_mtp_cal == false)
			pr_info("Calibration value (%d) set in mtp\n",
				tfa98xx->cal_data);
		mutex_unlock(&tfa98xx->dsp_lock);
	}
	mutex_unlock(&tfa98xx_mutex);

	return 1;
}

static int tfa98xx_get_cal_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tfa98xx *tfa98xx;

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		mutex_lock(&tfa98xx->dsp_lock);
		ucontrol->value.integer.value[tfa98xx->tfa->dev_idx]
			= tfa_dev_mtp_get(tfa98xx->tfa, TFA_MTP_RE25_PRIM);
		mutex_unlock(&tfa98xx->dsp_lock);
	}
	mutex_unlock(&tfa98xx_mutex);

	return 0;
}

static int tfa98xx_create_controls(struct tfa98xx *tfa98xx)
{
	int prof, nprof, mix_index = 0;
	int nr_controls = 0, id = 0;
	char *name;
	struct tfa98xx_baseprofile *bprofile;

	/* Create the following controls:
	 *  - enum control to select the active profile
	 *  - one volume control for each profile hosting a vstep
	 *  - Stop control on TFA1 devices
	 */

	nr_controls = 2; /* Profile and stop control */

	if (tfa98xx->flags & TFA98XX_FLAG_CALIBRATION_CTL)
		nr_controls += 1; /* calibration */

	/* allocate the tfa98xx_controls base on the nr of profiles */
	nprof = tfa_cnt_get_dev_nprof(tfa98xx->tfa);
	for (prof = 0; prof < nprof; prof++) {
		if (tfa_cont_get_max_vstep(tfa98xx->tfa, prof))
			nr_controls++; /* Playback Volume control */
	}

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	tfa98xx_controls = devm_kzalloc(tfa98xx->component->dev,
		nr_controls * sizeof(tfa98xx_controls[0]), GFP_KERNEL);
#else
	tfa98xx_controls = devm_kzalloc(tfa98xx->codec->dev,
		nr_controls * sizeof(tfa98xx_controls[0]), GFP_KERNEL);
#endif
	if (!tfa98xx_controls)
		return -ENOMEM;

	/* Create a mixer item for selecting the active profile */
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	name = devm_kzalloc(tfa98xx->component->dev,
		MAX_CONTROL_NAME, GFP_KERNEL);
#else
	name = devm_kzalloc(tfa98xx->codec->dev,
		MAX_CONTROL_NAME, GFP_KERNEL);
#endif
	if (!name)
		return -ENOMEM;
	scnprintf(name, MAX_CONTROL_NAME, "%s Profile", tfa98xx->fw.name);
	pr_info("%s: Mixer Control Name = %s\n", __func__, name);
	tfa98xx_controls[mix_index].name = name;
	tfa98xx_controls[mix_index].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	tfa98xx_controls[mix_index].info = tfa98xx_info_profile;
	tfa98xx_controls[mix_index].get = tfa98xx_get_profile;
	tfa98xx_controls[mix_index].put = tfa98xx_set_profile;
	/* tfa98xx_controls[mix_index].private_value = profs; */
	/* save number of profiles */
	mix_index++;

	/* create mixer items for each profile that has volume */
	for (prof = 0; prof < nprof; prof++) {
		/* create an new empty profile */
	#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
		bprofile = devm_kzalloc(tfa98xx->component->dev,
			sizeof(*bprofile), GFP_KERNEL);
	#else
		bprofile = devm_kzalloc(tfa98xx->codec->dev,
			sizeof(*bprofile), GFP_KERNEL);
	#endif
		if (!bprofile)
			return -ENOMEM;

		bprofile->len = 0;
		bprofile->item_id = -1;
		INIT_LIST_HEAD(&bprofile->list);

		/* copy profile name into basename until the . */
		get_profile_basename(bprofile->basename,
			_tfa_cont_profile_name(tfa98xx, prof));
		bprofile->len = strlen(bprofile->basename);

		/*
		 * search the profile list for a profile with basename, if it is not found then
		 * add it to the list and add a new mixer control (if it has vsteps)
		 * also, if it is a calibration profile, do not add it to the list
		 */
		if ((is_profile_in_list(bprofile->basename, bprofile->len) == 0)
			&& is_calibration_profile
			(_tfa_cont_profile_name(tfa98xx, prof)) == 0) {
			/* the profile is not present, add it to the list */
			list_add(&bprofile->list, &profile_list);
			bprofile->item_id = id++;

			pr_debug("profile added [%d]: %s\n",
				bprofile->item_id, bprofile->basename);

			if (tfa_cont_get_max_vstep(tfa98xx->tfa, prof)) {
			#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
				name = devm_kzalloc(tfa98xx->component->dev,
					MAX_CONTROL_NAME, GFP_KERNEL);
			#else
				name = devm_kzalloc(tfa98xx->codec->dev,
					MAX_CONTROL_NAME, GFP_KERNEL);
			#endif
				if (!name)
					return -ENOMEM;

				scnprintf(name, MAX_CONTROL_NAME,
					"%s %s Playback Volume",
					tfa98xx->fw.name, bprofile->basename);

				tfa98xx_controls[mix_index].name = name;
				tfa98xx_controls[mix_index].iface
					= SNDRV_CTL_ELEM_IFACE_MIXER;
				tfa98xx_controls[mix_index].info
					= tfa98xx_info_vstep;
				tfa98xx_controls[mix_index].get
					= tfa98xx_get_vstep;
				tfa98xx_controls[mix_index].put
					= tfa98xx_set_vstep;
				tfa98xx_controls[mix_index].private_value
					= bprofile->item_id;
				/* save profile index */
				mix_index++;
			}
		}

		/* look for the basename profile in the list of mixer profiles
		 * and add the container profile index
		 * to the supported samplerates of this mixer profile
		 */
		add_sr_to_profile(tfa98xx, bprofile->basename,
			bprofile->len, prof);
	}

	/* set the number of user selectable profiles in the mixer */
	tfa98xx_mixer_profiles = id;

	/* Create a mixer item for stop control on TFA1 */
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	name = devm_kzalloc(tfa98xx->component->dev,
 		MAX_CONTROL_NAME, GFP_KERNEL);
#else
	name = devm_kzalloc(tfa98xx->codec->dev,
		MAX_CONTROL_NAME, GFP_KERNEL);
#endif
	if (!name)
		return -ENOMEM;

	scnprintf(name, MAX_CONTROL_NAME, "%s Stop", tfa98xx->fw.name);
	tfa98xx_controls[mix_index].name = name;
	tfa98xx_controls[mix_index].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	tfa98xx_controls[mix_index].info = tfa98xx_info_stop_ctl;
	tfa98xx_controls[mix_index].get = tfa98xx_get_stop_ctl;
	tfa98xx_controls[mix_index].put = tfa98xx_set_stop_ctl;
	mix_index++;

	if (tfa98xx->flags & TFA98XX_FLAG_CALIBRATION_CTL) {
	#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
		name = devm_kzalloc(tfa98xx->component->dev,
			MAX_CONTROL_NAME, GFP_KERNEL);
	#else
		name = devm_kzalloc(tfa98xx->codec->dev,
			MAX_CONTROL_NAME, GFP_KERNEL);
	#endif
		if (!name)
			return -ENOMEM;

		scnprintf(name, MAX_CONTROL_NAME,
			"%s Calibration", tfa98xx->fw.name);
		tfa98xx_controls[mix_index].name = name;
		tfa98xx_controls[mix_index].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
		tfa98xx_controls[mix_index].info = tfa98xx_info_cal_ctl;
		tfa98xx_controls[mix_index].get = tfa98xx_get_cal_ctl;
		tfa98xx_controls[mix_index].put = tfa98xx_set_cal_ctl;
		mix_index++;
	}

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	return snd_soc_add_component_controls(tfa98xx->component,
		tfa98xx_controls, mix_index);
#else
	return snd_soc_add_codec_controls(tfa98xx->codec,
		tfa98xx_controls, mix_index);
#endif
}

static void *tfa98xx_devm_kstrdup(struct device *dev, char *buf)
{
	char *str = devm_kzalloc(dev, strlen(buf) + 1, GFP_KERNEL);

	if (!str)
		return str;
	memcpy(str, buf, strlen(buf));

	return str;
}

static int tfa98xx_append_i2c_address(struct device *dev,
	struct i2c_client *i2c,
	struct snd_soc_dapm_widget *widgets,
	int num_widgets,
	struct snd_soc_dai_driver *dai_drv,
	int num_dai)
{
	char buf[50];
	int i;
	int addr = i2c->addr;

	if (dai_drv && num_dai > 0)
		for (i = 0; i < num_dai; i++) {
			snprintf(buf, 50, "%s-%x", dai_drv[i].name,
				addr);
			dai_drv[i].name = tfa98xx_devm_kstrdup(dev, buf);
			pr_info("dai_drv[%d].name=%s\n", i, dai_drv[i].name);

			snprintf(buf, 50, "%s-%x",
				dai_drv[i].playback.stream_name,
				addr);
			dai_drv[i].playback.stream_name
				= tfa98xx_devm_kstrdup(dev, buf);
			pr_info("dai_drv[%d].playback.stream_name=%s\n",
				i, dai_drv[i].playback.stream_name);

			snprintf(buf, 50, "%s-%x",
				dai_drv[i].capture.stream_name,
				addr);
			dai_drv[i].capture.stream_name
				= tfa98xx_devm_kstrdup(dev, buf);
			pr_info("dai_drv[%d].capture.stream_name=%s\n",
				i, dai_drv[i].capture.stream_name);
		}

	/* the idea behind this is convert:
	 * SND_SOC_DAPM_AIF_IN
	 *   ("AIF IN","AIF Playback",0,SND_SOC_NOPM,0,0),
	 * into:
	 * SND_SOC_DAPM_AIF_IN
	 *   ("AIF IN","AIF Playback-2-36",0,SND_SOC_NOPM,0,0),
	 */
	if (widgets && num_widgets > 0)
		for (i = 0; i < num_widgets; i++) {
			if (!widgets[i].sname)
				continue;
			if ((widgets[i].id == snd_soc_dapm_aif_in)
				|| (widgets[i].id == snd_soc_dapm_aif_out)) {
				snprintf(buf, 50, "%s-%x",
					widgets[i].sname, addr);
				widgets[i].sname
					= tfa98xx_devm_kstrdup(dev, buf);
				pr_info("widgets[%d].sname=%s\n",
					i, widgets[i].sname);
			}
		}

	return 0;
}

static struct snd_soc_dapm_widget tfa98xx_dapm_widgets_common[] = {
	/* Stream widgets */
	SND_SOC_DAPM_AIF_IN("AIF IN", "AIF Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF OUT", "AIF Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUTPUT("OUTL"),
	SND_SOC_DAPM_INPUT("AEC Loopback"),
};

static struct snd_soc_dapm_widget tfa98xx_dapm_widgets_stereo[] = {
	SND_SOC_DAPM_OUTPUT("OUTR"),
};

#if 0 //temp
static struct snd_soc_dapm_widget tfa98xx_dapm_widgets_saam[] = {
	SND_SOC_DAPM_INPUT("SAAM MIC"),
};

static struct snd_soc_dapm_widget tfa9888_dapm_inputs[] = {
	SND_SOC_DAPM_INPUT("DMIC1"),
	SND_SOC_DAPM_INPUT("DMIC2"),
	SND_SOC_DAPM_INPUT("DMIC3"),
	SND_SOC_DAPM_INPUT("DMIC4"),
};
#endif

static const struct snd_soc_dapm_route tfa98xx_dapm_routes_common[] = {
	{"OUTL", NULL, "AIF IN"},
	{"AIF OUT", NULL, "AEC Loopback"},
};

static const struct snd_soc_dapm_route tfa98xx_dapm_routes_saam[] = {
	{"AIF OUT", NULL, "SAAM MIC"},
};

static const struct snd_soc_dapm_route tfa98xx_dapm_routes_stereo[] = {
	{"OUTR", NULL, "AIF IN"},
};

static const struct snd_soc_dapm_route tfa9888_input_dapm_routes[] = {
	{"AIF OUT", NULL, "DMIC1"},
	{"AIF OUT", NULL, "DMIC2"},
	{"AIF OUT", NULL, "DMIC3"},
	{"AIF OUT", NULL, "DMIC4"},
};

#if KERNEL_VERSION(4, 2, 0) > LINUX_VERSION_CODE
static struct snd_soc_dapm_context *
snd_soc_codec_get_dapm(struct snd_soc_codec *codec)
{
	return &codec->dapm;
}
#endif

static void tfa98xx_add_widgets(struct tfa98xx *tfa98xx)
{
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	struct snd_soc_dapm_context *dapm
		= snd_soc_component_get_dapm(tfa98xx->component);
#elif KERNEL_VERSION(4, 2, 0) > LINUX_VERSION_CODE
	struct snd_soc_dapm_context *dapm
		= &tfa98xx->codec->dapm;
#else
	struct snd_soc_dapm_context *dapm
		= snd_soc_codec_get_dapm(tfa98xx->codec);
#endif
	struct snd_soc_dapm_widget *widgets;
	unsigned int num_dapm_widgets
		= ARRAY_SIZE(tfa98xx_dapm_widgets_common);
	int i;

	widgets = devm_kzalloc(&tfa98xx->i2c->dev,
		sizeof(struct snd_soc_dapm_widget)
		* ARRAY_SIZE(tfa98xx_dapm_widgets_common),
		GFP_KERNEL);
	if (!widgets)
		return;
	memcpy(widgets, tfa98xx_dapm_widgets_common,
		sizeof(struct snd_soc_dapm_widget)
		* ARRAY_SIZE(tfa98xx_dapm_widgets_common));

	tfa98xx_append_i2c_address(&tfa98xx->i2c->dev,
		tfa98xx->i2c,
		widgets,
		num_dapm_widgets,
		NULL,
		0);

	snd_soc_dapm_new_controls(dapm, widgets,
		ARRAY_SIZE(tfa98xx_dapm_widgets_common));
	snd_soc_dapm_add_routes(dapm, tfa98xx_dapm_routes_common,
		ARRAY_SIZE(tfa98xx_dapm_routes_common));

	if (tfa98xx->flags & TFA98XX_FLAG_STEREO_DEVICE) {
		snd_soc_dapm_new_controls
			(dapm, tfa98xx_dapm_widgets_stereo,
			ARRAY_SIZE(tfa98xx_dapm_widgets_stereo));
		snd_soc_dapm_add_routes
			(dapm, tfa98xx_dapm_routes_stereo,
			ARRAY_SIZE(tfa98xx_dapm_routes_stereo));
	}

#if defined(TFA_SET_DAPM_IGNORE_SUSPEND)
	if (num_dapm_widgets > 0) {
		for (i = 0; i < num_dapm_widgets; i++) {
			if (!widgets[i].sname)
				continue;
			if ((widgets[i].id == snd_soc_dapm_aif_in)
				|| (widgets[i].id == snd_soc_dapm_aif_out)) {
				pr_info("dapm_ignore_suspend widgets[%d].sname=%s\n",
					i, widgets[i].sname);
				snd_soc_dapm_ignore_suspend(dapm, widgets[i].sname);
			}
		}
	}

	snd_soc_dapm_ignore_suspend(dapm, "AIF IN");
	snd_soc_dapm_ignore_suspend(dapm, "OUTL");
#endif

#if 0 // temp
	if (tfa98xx->flags & TFA98XX_FLAG_MULTI_MIC_INPUTS) {
		snd_soc_dapm_new_controls
			(dapm, tfa9888_dapm_inputs,
			ARRAY_SIZE(tfa9888_dapm_inputs));
		snd_soc_dapm_add_routes
			(dapm, tfa9888_input_dapm_routes,
			ARRAY_SIZE(tfa9888_input_dapm_routes));
	}

	if (tfa98xx->flags & TFA98XX_FLAG_SAAM_AVAILABLE) {
		snd_soc_dapm_new_controls
			(dapm, tfa98xx_dapm_widgets_saam,
			ARRAY_SIZE(tfa98xx_dapm_widgets_saam));
		snd_soc_dapm_add_routes
			(dapm, tfa98xx_dapm_routes_saam,
			ARRAY_SIZE(tfa98xx_dapm_routes_saam));
	}
#endif
}

/* I2C wrapper functions */
enum tfa98xx_error tfa98xx_write_register16(struct tfa_device *tfa,
	unsigned char subaddress,
	unsigned short value)
{
	enum tfa98xx_error error = TFA98XX_ERROR_OK;
	struct tfa98xx *tfa98xx;
	int ret;
	int retries = I2C_RETRIES;

	if (tfa == NULL) {
		pr_err("No device available\n");
		return TFA98XX_ERROR_FAIL;
	}

	tfa98xx = (struct tfa98xx *)tfa->data;
	if (!tfa98xx || !tfa98xx->regmap) {
		pr_err("No tfa98xx regmap available\n");
		return TFA98XX_ERROR_BAD_PARAMETER;
	}

retry:
	ret = regmap_write(tfa98xx->regmap, subaddress, value);
	if (ret < 0) {
		pr_warn("i2c error, retries left: %d\n", retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
		return TFA98XX_ERROR_FAIL;
	}

#if 0 // temp
	if (tfa98xx_kmsg_regs)
		dev_dbg(&tfa98xx->i2c->dev,
			"WR reg=0x%02x, val=0x%04x %s\n",
			subaddress, value,
			ret < 0 ? "Error!!" : "");

	if (tfa98xx_ftrace_regs)
		tfa98xx_trace_printk
			("\tWR reg=0x%02x, val=0x%04x %s\n",
			subaddress, value,
			ret < 0 ? "Error!!" : "");
#endif

	return error;
}

enum tfa98xx_error tfa98xx_read_register16(struct tfa_device *tfa,
	unsigned char subaddress,
	unsigned short *val)
{
	enum tfa98xx_error error = TFA98XX_ERROR_OK;
	struct tfa98xx *tfa98xx;
	unsigned int value;
	int retries = I2C_RETRIES;
	int ret;

	if (tfa == NULL) {
		pr_err("No device available\n");
		return TFA98XX_ERROR_FAIL;
	}

	tfa98xx = (struct tfa98xx *)tfa->data;
	if (!tfa98xx || !tfa98xx->regmap) {
		pr_err("No tfa98xx regmap available\n");
		return TFA98XX_ERROR_BAD_PARAMETER;
	}

retry:
	ret = regmap_read(tfa98xx->regmap, subaddress, &value);
	if (ret < 0) {
		pr_warn("i2c error at subaddress 0x%x, retries left: %d\n",
			subaddress, retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
		return TFA98XX_ERROR_FAIL;
	}
	*val = value & 0xffff;

#if 0 // temp
	if (tfa98xx_kmsg_regs)
		dev_dbg(&tfa98xx->i2c->dev,
			"RD reg=0x%02x, val=0x%04x %s\n",
			subaddress, *val,
			ret < 0 ? "Error!!" : "");
	if (tfa98xx_ftrace_regs)
		tfa98xx_trace_printk
			("\tRD reg=0x%02x, val=0x%04x %s\n",
			subaddress, *val,
			ret < 0 ? "Error!!" : "");
#endif

	return error;
}

/*
 * init external dsp
 */
enum tfa98xx_error tfa98xx_init_dsp(struct tfa_device *tfa)
{
	return TFA98XX_ERROR_NOT_SUPPORTED;
}

int tfa98xx_get_dsp_status(struct tfa_device *tfa)
{
	return 0;
}

/*
 * write external dsp message
 */
enum tfa98xx_error tfa98xx_write_dsp(struct tfa_device *tfa,
	int num_bytes, const char *command_buffer)
{
	return TFA98XX_ERROR_NOT_SUPPORTED;
}

/*
 * read external dsp message
 */
enum tfa98xx_error tfa98xx_read_dsp(struct tfa_device *tfa,
	int num_bytes, unsigned char *result_buffer)
{
	return TFA98XX_ERROR_NOT_SUPPORTED;
}
/*
 * write/read external dsp message
 */
enum tfa98xx_error tfa98xx_writeread_dsp(struct tfa_device *tfa,
	int command_length, void *command_buffer,
	int result_length, void *result_buffer)
{
	return TFA98XX_ERROR_NOT_SUPPORTED;
}

enum tfa98xx_error tfa98xx_read_data(struct tfa_device *tfa,
	unsigned char reg,
	int len, unsigned char value[])
{
	enum tfa98xx_error error = TFA98XX_ERROR_OK;
	struct tfa98xx *tfa98xx;
	struct i2c_client *tfa98xx_client;
	int err;
	int tries = 0;
	unsigned char *reg_buf = NULL;
	struct i2c_msg msgs[] = {
		{
			.flags = 0,
			.len = 1,
			.buf = NULL,
		}, {
			.flags = I2C_M_RD,
			.len = len,
			.buf = value,
		},
	};

	reg_buf = (unsigned char *)
		kmalloc(sizeof(reg), GFP_DMA); // GRP_KERNEL  also works,
	if (!reg_buf)
		return -ENOMEM;;

	*reg_buf = reg;
	msgs[0].buf = reg_buf;

	if (tfa == NULL) {
		pr_err("No device available\n");
		return TFA98XX_ERROR_FAIL;
	}

	tfa98xx = (struct tfa98xx *)tfa->data;
	if (tfa98xx->i2c) {
		tfa98xx_client = tfa98xx->i2c;
		msgs[0].addr = tfa98xx_client->addr;
		msgs[1].addr = tfa98xx_client->addr;

		do {
			err = i2c_transfer(tfa98xx_client->adapter, msgs,
				ARRAY_SIZE(msgs));
			if (err != ARRAY_SIZE(msgs))
				msleep_interruptible(I2C_RETRY_DELAY);
		} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

		if (err != ARRAY_SIZE(msgs)) {
			dev_err(&tfa98xx_client->dev,
				"read transfer error %d\n", err);
			error = TFA98XX_ERROR_FAIL;
		}

		if (tfa98xx_kmsg_regs)
			dev_dbg(&tfa98xx_client->dev,
				"RD-DAT reg=0x%02x, len=%d\n",
				reg, len);
		if (tfa98xx_ftrace_regs)
			tfa98xx_trace_printk
				("\tRD-DAT reg=0x%02x, len=%d\n",
				reg, len);
	} else {
		pr_err("No device available\n");
		error = TFA98XX_ERROR_FAIL;
	}

	kfree(reg_buf);

	return error;
}

enum tfa98xx_error tfa98xx_write_raw(struct tfa_device *tfa,
	int len,
	const unsigned char data[])
{
	enum tfa98xx_error error = TFA98XX_ERROR_OK;
	struct tfa98xx *tfa98xx;
	int ret;
	int retries = I2C_RETRIES;

	if (tfa == NULL) {
		pr_err("No device available\n");
		return TFA98XX_ERROR_FAIL;
	}

	tfa98xx = (struct tfa98xx *)tfa->data;

retry:
	ret = i2c_master_send(tfa98xx->i2c, data, len);
	if (ret < 0) {
		pr_warn("i2c error, retries left: %d\n", retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
	}

	if (ret == len) {
		if (tfa98xx_kmsg_regs)
			dev_dbg(&tfa98xx->i2c->dev,
				"WR-RAW len=%d\n", len);
		if (tfa98xx_ftrace_regs)
			tfa98xx_trace_printk
				("\tWR-RAW len=%d\n", len);
		return TFA98XX_ERROR_OK;
	}
	pr_err("WR-RAW (len=%d) Error I2C send size mismatch %d\n",
		len, ret);
	error = TFA98XX_ERROR_FAIL;

	return error;
}

int tfa_ext_register(dsp_write_reg_t tfa_write_reg,
	dsp_send_message_t tfa_send_message,
	dsp_read_message_t tfa_read_message,
	tfa_event_handler_t *tfa_event_handler)
{
	struct tfa98xx *tfa98xx;

	mutex_lock(&tfa98xx_mutex);

	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		tfa98xx->tfa->ext_dsp = 1;
		tfa98xx->tfa->is_probus_device = 1;
		tfa98xx->tfa->is_cold = 1;

		if (tfa_write_reg != NULL)
			tfa98xx->tfa->dev_ops.reg_write = tfa_write_reg;
		if (tfa_send_message != NULL)
			tfa98xx->tfa->dev_ops.dsp_msg = tfa_send_message;
		if (tfa_read_message != NULL)
			tfa98xx->tfa->dev_ops.dsp_msg_read = tfa_read_message;
	}

	if (tfa_event_handler != NULL)
		tfa_event_handler = (tfa_event_handler_t *)tfa_ext_event_handler;

	mutex_unlock(&tfa98xx_mutex);

	return 0;
}
#if !defined(TFA_SET_EXT_INTERNALLY)
EXPORT_SYMBOL(tfa_ext_register);
#endif

#if defined(TFA_SET_EXT_INTERNALLY)
#ifdef MPLATFORM
enum tfa98xx_error ipi_tfadsp_write(struct tfa_device *tfa,
	int length, const char *buf)
{
	enum tfa98xx_error err = TFA98XX_ERROR_OK;
	int ret = 0;

	if (buf == NULL) {
		pr_err("%s: error with NULL buffer\n", __func__);
		return TFA98XX_ERROR_BAD_PARAMETER;
	}

#if defined(TFADSP_DSP_MSG_PACKET_STRATEGY)
	if (length > 6) {
		pr_debug("%s: [0]:0x%02x-[1]:0x%02x-[2]:0x%02x-[3]:0x%02x, packet_id:%d, packet_size:%d\n",
			__func__, buf[0], buf[1], buf[2], buf[3],
			(buf[0] << 8) | buf[1], (buf[2] << 8) | buf[3]);
		pr_debug("%s: [4]:0x%02x-[5]:0x%02x-[6]:0x%02x, length:%d\n",
			__func__, buf[4], buf[5], buf[6], length);
	}
#else
	if (length >= 3)
		pr_debug("%s: [0]:0x%02x-[1]:0x%02x-[2]:0x%02x, length:%d\n",
			__func__, buf[0], buf[1], buf[2], length);
#endif /* TFADSP_DSP_MSG_PACKET_STRATEGY */

	ret = mtk_spk_send_ipi_buf_to_dsp((void *)buf, (uint32_t)length);
	if (ret != 0) {
		pr_err("%s: error in sending message to DSP (err %d)\n",
			__func__, ret);
		err = TFA98XX_ERROR_DSP_NOT_RUNNING;
	}

	return err;
}

enum tfa98xx_error ipi_tfadsp_read(struct tfa_device *tfa,
	int length, unsigned char *bytes)
{
	enum tfa98xx_error err = TFA98XX_ERROR_OK;
	int ret = 0;
	uint32_t buf_len;

	if (bytes == NULL) {
		pr_err("%s: error with NULL buffer\n", __func__);
		return TFA98XX_ERROR_BAD_PARAMETER;
	}

	ret = mtk_spk_recv_ipi_buf_from_dsp((int8_t *)bytes,
		(int16_t)length, &buf_len);
	if (ret != 0) {
		pr_err("%s: error in receiving message to DSP (err %d)\n",
			__func__, ret);
		err = TFA98XX_ERROR_DSP_NOT_RUNNING;
	}

	if (length >= 3)
		pr_debug("%s: [0]:0x%02x-[1]:0x%02x-[2]:0x%02x, length:%d, buf_len:%d\n",
			__func__, bytes[0], bytes[1], bytes[2],
			length, buf_len);

	return err;
}
#endif
#endif /* TFA_SET_EXT_INTERNALLY */

/* Interrupts management */

static void tfa98xx_interrupt_enable_tfa2(struct tfa98xx *tfa98xx, bool enable)
{
#if 0
	/* Only for 0x72 we need to enable NOCLK interrupts */
	if (tfa98xx->flags & TFA98XX_FLAG_REMOVE_PLOP_NOISE)
		tfa_irq_ena(tfa98xx->tfa,
			tfa9912_irq_stnoclk, enable);

	if (tfa98xx->flags & TFA98XX_FLAG_LP_MODES) {
		tfa_irq_ena(tfa98xx->tfa,
			36, enable); /* FIXME: IELP0 does not excist for 9912 */
		tfa_irq_ena(tfa98xx->tfa,
			tfa9912_irq_stclpr, enable);
	}
#endif
}

#if defined(USE_TFA9891)
/* Check if tap-detection can and shall be enabled.
 * Configure SPK interrupt accordingly or setup polling mode
 * Tap-detection shall be active if:
 *  - the service is enabled (tapdet_open), AND
 *  - the current profile is a tap-detection profile
 * On TFA1 familiy of devices, activating tap-detection means enabling the SPK
 * interrupt if available.
 * We also update the tapdet_enabled and tapdet_poll variables.
 */
static void tfa98xx_tapdet_check_update(struct tfa98xx *tfa98xx)
{
	unsigned int enable = false;

	/* Support tap-detection on TFA1 family of devices */
	if ((tfa98xx->flags & TFA98XX_FLAG_TAPDET_AVAILABLE) == 0)
		return;

	if (tfa98xx->tapdet_open &&
		(tfa98xx->tapdet_profiles & (1 << tfa98xx->profile)))
		enable = true;

	if (!gpio_is_valid(tfa98xx->irq_gpio)) {
		/* interrupt not available, setup polling mode */
		tfa98xx->tapdet_poll = true;
		if (enable)
			queue_delayed_work(tfa98xx->tfa98xx_wq,
				&tfa98xx->tapdet_work, HZ / 10);
		else
			cancel_delayed_work_sync(&tfa98xx->tapdet_work);
	#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
		dev_dbg(tfa98xx->component->dev,
			"Polling for tap-detection: %s (%d; 0x%x, %d)\n",
			enable ? "enabled" : "disabled",
			tfa98xx->tapdet_open, tfa98xx->tapdet_profiles,
			tfa98xx->profile);
	#else
		dev_dbg(tfa98xx->codec->dev,
			"Polling for tap-detection: %s (%d; 0x%x, %d)\n",
			enable ? "enabled" : "disabled",
			tfa98xx->tapdet_open, tfa98xx->tapdet_profiles,
			tfa98xx->profile);
	#endif

	} else {
	#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
		dev_dbg(tfa98xx->component->dev,
			"Interrupt for tap-detection: %s (%d; 0x%x, %d)\n",
			enable ? "enabled" : "disabled",
			tfa98xx->tapdet_open, tfa98xx->tapdet_profiles,
			tfa98xx->profile);
	#else
		dev_dbg(tfa98xx->codec->dev,
			"Interrupt for tap-detection: %s (%d; 0x%x, %d)\n",
			enable ? "enabled" : "disabled",
			tfa98xx->tapdet_open, tfa98xx->tapdet_profiles,
			tfa98xx->profile);
	#endif
		/*  enabled interrupt */
		tfa_irq_ena(tfa98xx->tfa, tfa9912_irq_sttapdet, enable);
	}

	/* check disabled => enabled transition to clear pending events */
	if (!tfa98xx->tapdet_enabled && enable)
		/* clear pending event if any */
		tfa_irq_clear(tfa98xx->tfa,
			tfa9912_irq_sttapdet);

	if (!tfa98xx->tapdet_poll)
		tfa_irq_ena(tfa98xx->tfa,
			tfa9912_irq_sttapdet, 1); /* enable again */
}
#endif

/* global enable / disable interrupts */
static void tfa98xx_interrupt_enable(struct tfa98xx *tfa98xx, bool enable)
{
	if (tfa98xx->flags & TFA98XX_FLAG_SKIP_INTERRUPTS)
		return;

	if (tfa98xx->tfa->tfa_family == 2)
		tfa98xx_interrupt_enable_tfa2(tfa98xx, enable);
}

/* Firmware management */
static void
tfa98xx_container_loaded(const struct firmware *cont, void *context)
{
	struct tfa_container *container;
	struct tfa98xx *tfa98xx = context;
	enum tfa_error tfa_err;
	int container_size;
#if defined(PRELOAD_FIRMWARE_BY_STARTING_AT_PROBING)
	int ret;
#endif
#if defined(TFA_DBGFS_CHECK_MTPEX)
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	unsigned int value;
#else
	unsigned short value;
#endif
#endif

	mutex_lock(&probe_lock);

	tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_FAIL;

	if (!cont) {
		pr_err("Failed to read %s\n", fw_name);
		mutex_unlock(&probe_lock);
		return;
	}

	pr_debug("loaded %s - size: %zu\n", fw_name, cont->size);

	mutex_lock(&tfa98xx_mutex);
	if (tfa98xx_container == NULL) {
		container = kzalloc(cont->size, GFP_KERNEL);
		if (container == NULL) {
			mutex_unlock(&tfa98xx_mutex);
			release_firmware(cont);
			pr_err("Error allocating memory\n");
			mutex_unlock(&probe_lock);
			return;
		}

		container_size = cont->size;
		memcpy(container, cont->data, container_size);
		release_firmware(cont);

		pr_debug("%.2s%.2s\n", container->version, container->subversion);
		pr_debug("%.8s\n", container->customer);
		pr_debug("%.8s\n", container->application);
		pr_debug("%.8s\n", container->type);
		pr_debug("%d ndev\n", container->ndev);
		pr_debug("%d nprof\n", container->nprof);

		tfa_err = tfa_load_cnt(container, container_size);
		if (tfa_err != tfa_error_ok) {
			mutex_unlock(&tfa98xx_mutex);
			kfree(container);
			dev_err(tfa98xx->dev, "Cannot load container file, aborting\n");
			mutex_unlock(&probe_lock);
			return;
		}

		tfa98xx_container = container;
	}
	else {
		pr_debug("container file already loaded...\n");
		container = tfa98xx_container;
		release_firmware(cont);
	}
	mutex_unlock(&tfa98xx_mutex);

	tfa98xx->tfa->cnt = container;

	/*
		i2c transaction limited to 64k
		(Documentation/i2c/writing-clients)
	*/
	tfa98xx->tfa->buffer_size = 65536;

	/* DSP messages via i2c */
	tfa98xx->tfa->has_msg = 0;

	if (tfa_dev_probe(tfa98xx->i2c->addr, tfa98xx->tfa) != 0) {
		dev_err(tfa98xx->dev,
			"Failed to probe TFA98xx @ 0x%.2x\n",
			tfa98xx->i2c->addr);
		mutex_unlock(&probe_lock);
		return;
	}

/* TEMPORARY, until TFA device is probed before tfa_ext is called */
#if defined(TFA_SET_EXT_INTERNALLY)
#ifdef QPLATFORM
	tfa98xx->tfa->dev_ops.dsp_msg = (dsp_send_message_t)afe_tfadsp_write;
	tfa98xx->tfa->dev_ops.dsp_msg_read = (dsp_read_message_t)afe_tfadsp_read;
#endif
#ifdef MPLATFORM
	tfa98xx->tfa->dev_ops.dsp_msg = (dsp_send_message_t)ipi_tfadsp_write;
	tfa98xx->tfa->dev_ops.dsp_msg_read = (dsp_read_message_t)ipi_tfadsp_read;
#endif
#endif

	tfa98xx->tfa->dev_idx = tfa_cont_get_idx(tfa98xx->tfa);
	if (tfa98xx->tfa->dev_idx < 0) {
		dev_err(tfa98xx->dev,
			"Failed to find TFA98xx @ 0x%.2x in container file\n",
			tfa98xx->i2c->addr);
		mutex_unlock(&probe_lock);
		return;
	}

	/* Enable debug traces */
	tfa98xx->tfa->verbose = trace_level & 1;

	/* prefix is the application name from the cnt */
	tfa_cont_get_app_name(tfa98xx->tfa, tfa98xx->fw.name);

	/* set default profile/vstep */
	tfa98xx->profile = 0;
	tfa98xx->vstep = 0;

	/* Override default profile if requested */
	if (strcmp(dflt_prof_name, "")) {
		unsigned int i;
		int nprof = tfa_cnt_get_dev_nprof(tfa98xx->tfa);
		for (i = 0; i < nprof; i++) {
			if (strcmp(_tfa_cont_profile_name(tfa98xx, i),
				dflt_prof_name) == 0) {
				tfa98xx->profile = i;
				dev_info(tfa98xx->dev,
					"changing default profile to %s (%d)\n",
					dflt_prof_name, tfa98xx->profile);
				break;
			}
		}
		if (i >= nprof)
			dev_info(tfa98xx->dev,
				"Default profile override failed (%s profile not found)\n",
				dflt_prof_name);
	}

	tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_OK;

#if defined(TFA_DBGFS_CHECK_MTPEX)
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	value = snd_soc_component_read32(tfa98xx->component,
		TFA98XX_KEY2_PROTECTED_MTP0);
#else
	value = snd_soc_read(tfa98xx->codec, TFA98XX_KEY2_PROTECTED_MTP0);
#endif
	if (value != -1) {
		tfa98xx->calibrate_done =
			(value & TFA98XX_KEY2_PROTECTED_MTP0_MTPEX_MSK) ? 1 : 0;
		pr_info("[0x%x] calibrate_done = MTPEX (%d) 0x%04x\n",
			tfa98xx->i2c->addr, tfa98xx->calibrate_done, value);
	} else {
		pr_info("[0x%x] error in reading MTPEX\n", tfa98xx->i2c->addr);
		tfa98xx->calibrate_done = 0;
	}
#else
	tfa98xx->calibrate_done = 0;
#endif

	pr_debug("Firmware init complete\n");

#if defined(TFADSP_DSP_BUFFER_POOL)
	/* allocate buffer_pool */
	if (tfa98xx->tfa->dev_idx == 0) {
		int index = 0;

		pr_info("Allocate buffer_pool\n");
		for (index = 0; index < POOL_MAX_INDEX; index++)
			tfa_buffer_pool(tfa98xx->tfa, index,
				buf_pool_size[index], POOL_ALLOC);
	}
#endif

	if (no_start != 0) {
		mutex_unlock(&probe_lock);
		return;
	}

	/* Only controls for master device */
	if (tfa98xx->tfa->dev_idx == 0)
		tfa98xx_create_controls(tfa98xx);

	tfa98xx_inputdev_check_register(tfa98xx);

	if (tfa_is_cold(tfa98xx->tfa) == 0) {
		pr_debug("Warning: device 0x%.2x is still warm\n",
			tfa98xx->i2c->addr);
		tfa_reset(tfa98xx->tfa);
	}

	/* Preload settings using internal clock on TFA2 */
#if defined(PRELOAD_FIRMWARE_BY_STARTING_AT_PROBING)
	if (tfa98xx->tfa->tfa_family == 2) {
		mutex_lock(&tfa98xx->dsp_lock);
		ret = tfa98xx_tfa_start(tfa98xx,
			tfa98xx->profile, tfa98xx->vstep);
		if (ret == TFA98XX_ERROR_NOT_SUPPORTED)
			tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_FAIL;
		mutex_unlock(&tfa98xx->dsp_lock);
	}
#endif

	tfa98xx_interrupt_enable(tfa98xx, true);

	mutex_unlock(&probe_lock);
}

static int tfa98xx_load_container(struct tfa98xx *tfa98xx)
{
	mutex_lock(&probe_lock);
	tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_PENDING;
	mutex_unlock(&probe_lock);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		fw_name, tfa98xx->dev, GFP_KERNEL,
		tfa98xx, tfa98xx_container_loaded);
}

#if defined(USE_TFA9891)
static void tfa98xx_tapdet(struct tfa98xx *tfa98xx)
{
	unsigned int tap_pattern;
	int btn;

	/* check tap pattern (BTN_0 is "error" wrong tap indication */
	tap_pattern = tfa_get_tap_pattern(tfa98xx->tfa);
	switch (tap_pattern) {
	case 0xffffffff:
		pr_info("More than 4 taps detected! (flagTapPattern = -1)\n");
		btn = BTN_0;
		break;
	case 0xfffffffe:
	case 0xfe:
		pr_info("Illegal tap detected!\n");
		btn = BTN_0;
		break;
	case 0:
		pr_info("Unrecognized pattern! (flagTapPattern = 0)\n");
		btn = BTN_0;
		break;
	default:
		pr_info("Detected pattern: %d\n", tap_pattern);
		btn = BTN_0 + tap_pattern;
		break;
	}

	input_report_key(tfa98xx->input, btn, 1);
	input_report_key(tfa98xx->input, btn, 0);
	input_sync(tfa98xx->input);

	/* acknowledge event done by clearing interrupt */
}

static void tfa98xx_tapdet_work(struct work_struct *work)
{
	struct tfa98xx *tfa98xx;

	/* TODO check is this is still needed for tap polling */
	tfa98xx = container_of(work, struct tfa98xx, tapdet_work.work);

	if (tfa_irq_get(tfa98xx->tfa, tfa9912_irq_sttapdet))
		tfa98xx_tapdet(tfa98xx);

	queue_delayed_work(tfa98xx->tfa98xx_wq,
		&tfa98xx->tapdet_work, HZ / 10);
}
#endif

static void tfa98xx_monitor(struct work_struct *work)
{
	struct tfa98xx *tfa98xx;
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	unsigned int val;
#else
	u16 val;
#endif
	enum tfa98xx_error error = TFA98XX_ERROR_OK;
	int active_handle = -1;
#if defined(TFA_USE_DEVICE_SPECIFIC_CONTROL)
#if defined(TFA_PROFILE_ON_DEVICE)
	int dev;
#endif
#endif

	mutex_lock(&probe_lock);

	tfa98xx = container_of(work, struct tfa98xx, monitor_work.work);

	pr_info("%s: [%d] - profile = %d: %s\n", __func__,
		tfa98xx->tfa->dev_idx, tfa98xx->profile,
		tfa_cont_profile_name(tfa98xx->tfa->cnt,
			tfa98xx->tfa->dev_idx, tfa98xx->profile));

#if defined(TFA_USE_DEVICE_SPECIFIC_CONTROL)
#if defined(TFA_PROFILE_ON_DEVICE)
	/* check active handle with profile */
	for (dev = 0; dev < tfa98xx_device_count; dev++) {
		if (tfa_cont_is_dev_specific_profile
			(tfa98xx->tfa->cnt, dev,
			tfa98xx->profile) != 0)
			active_handle = dev;
	}
#endif

	if (active_handle != -1) {
		pr_info("%s: profile = %d, active handle [%s]\n",
			__func__, tfa98xx->profile,
			tfa_cont_device_name(tfa98xx->tfa->cnt,
			active_handle));
		if (active_handle != tfa98xx->tfa->dev_idx)
			goto tfa_monitor_exit;
	} else {
		pr_info("%s: profile = %d, all active\n",
			__func__, tfa98xx->profile);
		active_handle = tfa98xx->tfa->dev_idx;
	}
#else
	active_handle = tfa98xx->tfa->dev_idx;
#endif /* TFA_USE_DEVICE_SPECIFIC_CONTROL */

	/* Check for tap-detection - bypass monitor if it is active */
	if (!tfa98xx->input) {
		mutex_lock(&tfa98xx->dsp_lock);
#if defined(USE_TFA9874) || defined(USE_TFA9878)
		error = tfa7478_status(tfa98xx->tfa);
#else
		error = tfa_status(tfa98xx->tfa);
#endif
		mutex_unlock(&tfa98xx->dsp_lock);
		if (error == TFA98XX_ERROR_DSP_NOT_RUNNING) {
			if (tfa98xx->dsp_init == TFA98XX_DSP_INIT_DONE) {
				tfa98xx->dsp_init = TFA98XX_DSP_INIT_RECOVER;
				tfa98xx_set_dsp_configured(tfa98xx);
				queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->init_work, 0);
			}
		}

		// temporal debugging
		mutex_lock(&tfa98xx->dsp_lock);
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
		val = snd_soc_component_read32(tfa98xx->component,
			TFA98XX_SYS_CONTROL0);
		pr_debug("[%d] SYS_CONTROL0: 0x%04x\n",
			active_handle, val);
		val = snd_soc_component_read32(tfa98xx->component,
			TFA98XX_SYS_CONTROL1);
		pr_debug("[%d] SYS_CONTROL1: 0x%04x\n",
			active_handle, val);
		val = snd_soc_component_read32(tfa98xx->component,
			TFA98XX_SYS_CONTROL2);
		pr_debug("[%d] SYS_CONTROL2: 0x%04x\n",
			active_handle, val);
		val = snd_soc_component_read32(tfa98xx->component,
			TFA98XX_CLOCK_CONTROL);
		pr_debug("[%d] CLOCK_CONTROL: 0x%04x\n",
			active_handle, val);
		val = snd_soc_component_read32(tfa98xx->component,
			TFA98XX_STATUS_FLAGS0);
		pr_debug("[%d] STATUS_FLAG0: 0x%04x\n",
			active_handle, val);
		val = snd_soc_component_read32(tfa98xx->component,
			TFA98XX_STATUS_FLAGS1);
		pr_debug("[%d] STATUS_FLAG1: 0x%04x\n",
			active_handle, val);
		val = snd_soc_component_read32(tfa98xx->component,
			TFA98XX_STATUS_FLAGS3);
		pr_debug("[%d] STATUS_FLAG3: 0x%04x\n",
			active_handle, val);
		val = snd_soc_component_read32(tfa98xx->component,
			TFA98XX_STATUS_FLAGS4);
		pr_debug("[%d] STATUS_FLAG4: 0x%04x\n",
			active_handle, val);
		val = snd_soc_component_read32(tfa98xx->component,
			TFA98XX_TDM_CONFIG0);
		pr_debug("[%d] TDM_CONFIG0: 0x%04x\n",
			active_handle, val);
#else
		val = snd_soc_read(tfa98xx->codec, TFA98XX_SYS_CONTROL0);
		pr_debug("[%d] SYS_CONTROL0: 0x%04x\n",
			active_handle, val);
		val = snd_soc_read(tfa98xx->codec, TFA98XX_SYS_CONTROL1);
		pr_debug("[%d] SYS_CONTROL1: 0x%04x\n",
			active_handle, val);
		val = snd_soc_read(tfa98xx->codec, TFA98XX_SYS_CONTROL2);
		pr_debug("[%d] SYS_CONTROL2: 0x%04x\n",
			active_handle, val);
		val = snd_soc_read(tfa98xx->codec, TFA98XX_CLOCK_CONTROL);
		pr_debug("[%d] CLOCK_CONTROL: 0x%04x\n",
			active_handle, val);
		val = snd_soc_read(tfa98xx->codec, TFA98XX_STATUS_FLAGS0);
		pr_debug("[%d] STATUS_FLAG0: 0x%04x\n",
			active_handle, val);
		val = snd_soc_read(tfa98xx->codec, TFA98XX_STATUS_FLAGS1);
		pr_debug("[%d] STATUS_FLAG1: 0x%04x\n",
			active_handle, val);
		val = snd_soc_read(tfa98xx->codec, TFA98XX_STATUS_FLAGS3);
		pr_debug("[%d] STATUS_FLAG3: 0x%04x\n",
			active_handle, val);
		val = snd_soc_read(tfa98xx->codec, TFA98XX_STATUS_FLAGS4);
		pr_debug("[%d] STATUS_FLAG4: 0x%04x\n",
			active_handle, val);
		val = snd_soc_read(tfa98xx->codec, TFA98XX_TDM_CONFIG0);
		pr_debug("[%d] TDM_CONFIG0: 0x%04x\n",
			active_handle, val);
#endif
		mutex_unlock(&tfa98xx->dsp_lock);
	}

tfa_monitor_exit:
	pr_info("%s: exit\n", __func__);

	mutex_unlock(&probe_lock);

	if (!tfa98xx->tfa->verbose)
		return;

	if (tfa98xx_monitor_count != -1)
		if (++tfa98xx_monitor_count > MONITOR_COUNT_MAX)
			return;

	/* reschedule */
	queue_delayed_work(tfa98xx->tfa98xx_wq,
		&tfa98xx->monitor_work, 5 * HZ);
}

static void tfa98xx_dsp_init(struct tfa98xx *tfa98xx)
{
	int ret;
	bool failed = false;
	bool reschedule = false;
	bool sync = false;
#if defined(TFA_USE_DEVICE_SPECIFIC_CONTROL)
	int active_handle = -1;
#if defined(TFA_PROFILE_ON_DEVICE)
	int dev;
#endif
#endif

	if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK) {
		pr_debug("Skipping tfa_dev_start (no FW: %d)\n",
			tfa98xx->dsp_fw_state);
		return;
	}

	if (tfa98xx->dsp_init == TFA98XX_DSP_INIT_DONE) {
		pr_debug("Stream already started, skipping DSP power-on\n");
		return;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	pr_info("%s: ...\n", __func__);

	tfa98xx->dsp_init = TFA98XX_DSP_INIT_PENDING;

	if (tfa98xx->init_count < TF98XX_MAX_DSP_START_TRY_COUNT) {
		/* directly try to start DSP */
		ret = tfa98xx_tfa_start(tfa98xx,
			tfa98xx->profile, tfa98xx->vstep);
		if (ret == TFA98XX_ERROR_NOT_SUPPORTED) {
			tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_FAIL;
			dev_err(&tfa98xx->i2c->dev, "Failed starting device\n");
			failed = true;
		} else if (ret != TFA98XX_ERROR_OK) {
			/* It may fail as we may not have a valid clock at that
			 * time, so re-schedule and re-try later.
			 */
			dev_err(&tfa98xx->i2c->dev,
				"tfa_dev_start failed! (err %d) - %d\n",
				ret, tfa98xx->init_count);
			reschedule = true;
		} else {
			sync = true;

			/* Subsystem ready, tfa init complete */
			tfa98xx->dsp_init = TFA98XX_DSP_INIT_DONE;
			tfa98xx_set_dsp_configured(tfa98xx);
			dev_dbg(&tfa98xx->i2c->dev,
				"tfa_dev_start success (%d)\n",
				tfa98xx->init_count);
			/* cancel other pending init works */
			cancel_delayed_work(&tfa98xx->init_work);
			tfa98xx->init_count = 0;
		}
	} else {
		/* exceeded max number ot start tentatives, cancel start */
		dev_err(&tfa98xx->i2c->dev,
			"Failed starting device (%d)\n",
			tfa98xx->init_count);
		failed = true;
	}
	mutex_unlock(&tfa98xx->dsp_lock);

	if (reschedule) {
		/* reschedule this init work for later */
		queue_delayed_work(tfa98xx->tfa98xx_wq,
			&tfa98xx->init_work,
			msecs_to_jiffies(5));
		tfa98xx->init_count++;
	}
	if (failed) {
		mutex_lock(&tfa98xx->dsp_lock);
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_FAIL;
		tfa98xx_set_dsp_configured(tfa98xx);
		mutex_unlock(&tfa98xx->dsp_lock);

		/* cancel other pending init works */
		cancel_delayed_work(&tfa98xx->init_work);
		tfa98xx->init_count = 0;
	}

#if defined(TFA_USE_DEVICE_SPECIFIC_CONTROL)
#if defined(TFA_PROFILE_ON_DEVICE)
	/* check active handle with profile */
	for (dev = 0; dev < tfa98xx_device_count; dev++) {
		if (tfa_cont_is_dev_specific_profile
			(tfa98xx->tfa->cnt, dev,
			tfa98xx->profile) != 0)
			active_handle = dev;
	}
#endif

	if (active_handle != -1) {
		pr_info("%s: profile = %d, active handle [%s]\n",
			__func__, tfa98xx->profile,
			tfa_cont_device_name(tfa98xx->tfa->cnt,
			active_handle));

		tfa98xx_sync_count = 0;

		if (active_handle == tfa98xx->tfa->dev_idx) {
			mutex_lock(&tfa98xx->dsp_lock);
			pr_info("%s: UNMUTE dev %d\n",
				__func__, tfa98xx->tfa->dev_idx);
			tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_UNMUTE, 0);

			/*
			 * start monitor thread to check IC status bit
			 * periodically, and re-init IC to recover if
			 * needed.
			 */
			tfa98xx_monitor_count = 0;
			queue_delayed_work(tfa98xx->tfa98xx_wq,
				&tfa98xx->monitor_work,
				1 * HZ);
			mutex_unlock(&tfa98xx->dsp_lock);
		}

		return;
	}
#endif /* TFA_USE_DEVICE_SPECIFIC_CONTROL */

	if (sync) {
		/* check if all devices have started */
		bool do_sync;

		mutex_lock(&tfa98xx_mutex);
		if (tfa98xx_sync_count < tfa98xx_device_count)
			tfa98xx_sync_count++;

		do_sync = (tfa98xx_sync_count >= tfa98xx_device_count);

		/* when all devices have started then unmute */
		if (do_sync) {
			tfa98xx_sync_count = 0;
			list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
				mutex_lock(&tfa98xx->dsp_lock);
				pr_info("%s: UNMUTE dev %d\n",
					__func__, tfa98xx->tfa->dev_idx);
				tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_UNMUTE, 0);

				/*
				 * start monitor thread to check IC status bit
				 * periodically, and re-init IC to recover if
				 * needed.
				 */
				tfa98xx_monitor_count = 0;
				queue_delayed_work(tfa98xx->tfa98xx_wq,
					&tfa98xx->monitor_work,
					1 * HZ);
				mutex_unlock(&tfa98xx->dsp_lock);
			}
		}
		mutex_unlock(&tfa98xx_mutex);
	}

	return;
}


static void tfa98xx_dsp_init_work(struct work_struct *work)
{
	struct tfa98xx *tfa98xx
		= container_of(work, struct tfa98xx, init_work.work);

	pr_debug("%s: enter with profile %d\n", __func__, tfa98xx->profile);

	/* Only do dsp init for master device */
/*
	if (tfa98xx->tfa->dev_idx != 0) {
		pr_debug("%s: no first handle\n", __func__);
		return;
	}
*/

	tfa98xx_dsp_init(tfa98xx);
}

static void tfa98xx_interrupt(struct work_struct *work)
{
	struct tfa98xx *tfa98xx
		= container_of(work, struct tfa98xx, interrupt_work.work);

	pr_info("\n");

#if defined(USE_TFA9891)
	if (tfa98xx->flags & TFA98XX_FLAG_TAPDET_AVAILABLE) {
		/* check for tap interrupt */
		if (tfa_irq_get(tfa98xx->tfa, tfa9912_irq_sttapdet)) {
			tfa98xx_tapdet(tfa98xx);

			/* clear interrupt */
			tfa_irq_clear(tfa98xx->tfa, tfa9912_irq_sttapdet);
		}
	} /* TFA98XX_FLAG_TAPDET_AVAILABLE */
#endif

	if (tfa98xx->flags & TFA98XX_FLAG_REMOVE_PLOP_NOISE) {
		int start_triggered;

		mutex_lock(&tfa98xx->dsp_lock);
		start_triggered = tfa_plop_noise_interrupt
			(tfa98xx->tfa, tfa98xx->profile, tfa98xx->vstep);
		/* Only enable when the return value is 1,
		 * otherwise the interrupt is triggered twice
		 */
		if (start_triggered)
			tfa98xx_interrupt_enable(tfa98xx, true);
		mutex_unlock(&tfa98xx->dsp_lock);
	} /* TFA98XX_FLAG_REMOVE_PLOP_NOISE */

	if (tfa98xx->flags & TFA98XX_FLAG_LP_MODES) {
		tfa_lp_mode_interrupt(tfa98xx->tfa);
	} /* TFA98XX_FLAG_LP_MODES */

	/* unmask interrupts masked in IRQ handler */
	tfa_irq_unmask(tfa98xx->tfa);
}

static int tfa98xx_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *component = dai->component;
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(component);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	int idx = 0;
#if !defined(TFA_FULL_RATE_SUPPORT_WITH_POST_CONVERSION)
	unsigned int sr;
	int len, prof, nprof;
#if defined(TFADSP_DSP_BUFFER_POOL)
	char basename[MAX_CONTROL_NAME] = {0};
#else
	char *basename;
#endif
#endif /* TFA_FULL_RATE_SUPPORT_WITH_POST_CONVERSION */
	u64 formats;
	int err;

	/*
	 * Support CODEC to CODEC links,
	 * these are called with a NULL runtime pointer.
	 */
	if (!substream->runtime)
		return 0;

	if (pcm_no_constraint != 0)
		return 0;

	switch (pcm_sample_format) {
	case 1:
		formats = SNDRV_PCM_FMTBIT_S24_LE;
		break;
	case 2:
		formats = SNDRV_PCM_FMTBIT_S32_LE;
		break;
	default:
		formats = SNDRV_PCM_FMTBIT_S16_LE;
		break;
	}

	err = snd_pcm_hw_constraint_mask64(substream->runtime,
		SNDRV_PCM_HW_PARAM_FORMAT, formats);
	if (err < 0)
		return err;

	if (no_start != 0)
		return 0;

	if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK) {
	#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
		dev_info(component->dev, "Container file not loaded\n");
	#else
		dev_info(codec->dev, "Container file not loaded\n");
	#endif
		return -EINVAL;
	}

#if !defined(TFA_FULL_RATE_SUPPORT_WITH_POST_CONVERSION)
#if !defined(TFADSP_DSP_BUFFER_POOL)
	basename = kzalloc(MAX_CONTROL_NAME, GFP_KERNEL);
	if (!basename)
		return -ENOMEM;
#endif

	/* copy profile name into basename until the . */
	get_profile_basename(basename,
		_tfa_cont_profile_name(tfa98xx, tfa98xx->profile));
	len = strlen(basename);
#endif /* TFA_FULL_RATE_SUPPORT_WITH_POST_CONVERSION */

	/* loop over all profiles and get the supported samples rate(s) from
	 * the profiles with the same basename
	 */
	tfa98xx->rate_constraint.list = &tfa98xx->rate_constraint_list[0];
	tfa98xx->rate_constraint.count = 0;

#if !defined(TFA_FULL_RATE_SUPPORT_WITH_POST_CONVERSION)
	nprof = tfa_cnt_get_dev_nprof(tfa98xx->tfa);
	for (prof = 0; prof < nprof; prof++) {
		if (strncmp(basename,
			_tfa_cont_profile_name(tfa98xx, prof), len)
			== 0) {
			/* Check which sample rate is supported
			 * with current profile, and enforce this.
			 */
			sr = tfa98xx_get_profile_sr(tfa98xx->tfa, prof);
			if (!sr)
			#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
				dev_info(component->dev,
					"Unable to identify supported sample rate\n");
			#else
				dev_info(codec->dev,
					"Unable to identify supported sample rate\n");
			#endif

			if (tfa98xx->rate_constraint.count
				>= TFA98XX_NUM_RATES) {
			#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
				dev_err(component->dev, "too many sample rates\n");
			#else
				dev_err(codec->dev, "too many sample rates\n");
			#endif
			} else {
				tfa98xx->rate_constraint_list[idx++] = sr;
				tfa98xx->rate_constraint.count += 1;
			}
		}
	}

#if !defined(TFADSP_DSP_BUFFER_POOL)
	kfree(basename);
#endif

	return snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE,
		&tfa98xx->rate_constraint);
#else
	pr_info("%s: add all the rates in the list\n", __func__);
	for (idx = 0; idx < (int)ARRAY_SIZE(rate_to_fssel); idx++) {
		tfa98xx->rate_constraint_list[idx] = rate_to_fssel[idx].rate;
		tfa98xx->rate_constraint.count += 1;
	}

	pr_info("%s: skip setting constraint, assuming fixed format\n",
		__func__);

	return 0;
#endif /* TFA_FULL_RATE_SUPPORT_WITH_POST_CONVERSION */
}

static int tfa98xx_set_dai_sysclk(struct snd_soc_dai *codec_dai,
	int clk_id, unsigned int freq, int dir)
{
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	struct tfa98xx *tfa98xx
		= snd_soc_component_get_drvdata(codec_dai->component);
#else
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec_dai->codec);
#endif

	tfa98xx->sysclk = freq;
	return 0;
}

static int tfa98xx_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
	unsigned int rx_mask, int slots, int slot_width)
{
	pr_debug("\n");
	return 0;
}

static int tfa98xx_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	struct tfa98xx *tfa98xx
		= snd_soc_component_get_drvdata(dai->component);
	struct snd_soc_component *component = dai->component;
#else
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(dai->codec);
	struct snd_soc_codec *codec = dai->codec;
#endif

	pr_debug("fmt=0x%x\n", fmt);

	/* Supported mode: regular I2S, slave, or PDM */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		if ((fmt & SND_SOC_DAIFMT_MASTER_MASK)
			!= SND_SOC_DAIFMT_CBS_CFS) {
		#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
			dev_err(component->dev, "Invalid Codec master mode\n");
		#else
			dev_err(codec->dev, "Invalid Codec master mode\n");
		#endif
			return -EINVAL;
		}
		break;
	case SND_SOC_DAIFMT_PDM:
		break;
	default:
	#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
		dev_err(component->dev, "Unsupported DAI format %d\n",
			fmt & SND_SOC_DAIFMT_FORMAT_MASK);
	#else
		dev_err(codec->dev, "Unsupported DAI format %d\n",
			fmt & SND_SOC_DAIFMT_FORMAT_MASK);
	#endif
		return -EINVAL;
	}

	tfa98xx->audio_mode = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	return 0;
}

static int tfa98xx_get_fssel(unsigned int rate)
{
	int i;

	for (i = 0; i < (int)ARRAY_SIZE(rate_to_fssel); i++)
		if (rate_to_fssel[i].rate == rate)
			return rate_to_fssel[i].fssel;

	return -EINVAL;
}

static int tfa98xx_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *component = dai->component;
	struct tfa98xx *tfa98xx
		= snd_soc_component_get_drvdata(component);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	unsigned int rate;
	int prof_idx;
	int sample_size;
	int slot_size;

	/* Supported */
	rate = params_rate(params);
	sample_size = snd_pcm_format_width(params_format(params));
	slot_size = snd_pcm_format_physical_width(params_format(params));
	pr_debug("Requested rate: %d, sample size: %d, physical size: %d\n",
		rate, sample_size, slot_size);
#if defined(TFA_FULL_RATE_SUPPORT_WITH_POST_CONVERSION)
	pr_info("forced to change rate: %d to %d\n", rate, sr_converted);
	rate = sr_converted;
#endif

	if (no_start != 0)
		return 0;

	/* check if samplerate is supported for this mixer profile */
	prof_idx = get_profile_id_for_sr(tfa98xx_mixer_profile, rate);
	if (prof_idx < 0) {
		pr_err("tfa98xx: invalid sample rate %d.\n", rate);
		return -EINVAL;
	}
	pr_debug("mixer profile:container profile = [%d:%d]\n",
		tfa98xx_mixer_profile, prof_idx);

	/* update 'real' profile (container profile) */
	tfa98xx->profile = prof_idx;

	pr_info("%s: tfa98xx_profile %d\n", __func__, tfa98xx->profile);

	/* update to new rate */
	tfa98xx->rate = rate;

	return 0;
}

static int tfa98xx_mute(struct snd_soc_dai *dai, int mute, int stream)
{
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *component = dai->component;
	struct tfa98xx *tfa98xx
		= snd_soc_component_get_drvdata(component);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif

	dev_dbg(&tfa98xx->i2c->dev,
		"%s: state: %d (stream = %d)\n", __func__, mute, stream);

	if (no_start) {
		pr_debug("no_start parameter set no tfa_dev_start or tfa_dev_stop, returning\n");
		return 0;
	}

	_tfa98xx_mute(tfa98xx, mute, stream);

	return 0;
}

static int _tfa98xx_mute(struct tfa98xx *tfa98xx, int mute, int stream)
{
	if (mute) {
		/* stop DSP only when both playback and capture streams
		 * are deactivated
		 */
		if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (tfa98xx->pstream == 0) {
				pr_debug("mute:%d [pstream duplicated]\n",
					mute);
				return 0;
			}
			tfa98xx->pstream = 0;
		} else if (stream == SNDRV_PCM_STREAM_CAPTURE) {
			if (tfa98xx->cstream == 0) {
				pr_debug("mute:%d [cstream duplicated]\n",
					mute);
				return 0;
			}
			tfa98xx->cstream = 0;
		}
		mutex_lock(&tfa98xx->dsp_lock);
		pr_info("mute:%d [pstream %d, cstream %d, samstream %d]\n",
			mute,
			tfa98xx->pstream, tfa98xx->cstream, tfa98xx->samstream);
		tfa98xx_set_stream_state(tfa98xx->tfa,
			(tfa98xx->pstream & BIT_PSTREAM)
			|((tfa98xx->cstream<<1) & BIT_CSTREAM)
			|((tfa98xx->samstream<<2) & BIT_SAMSTREAM));
		mutex_unlock(&tfa98xx->dsp_lock);

		/* case: both p/cstream (either) and samstream are off
		 * if (!(tfa98xx->pstream == 0 || tfa98xx->cstream == 0)
		 *  || (tfa98xx->samstream != 0)) {
		 *  pr_info("mute is suspended until playback/saam are off\n");
		 *  return 0;
		 * }
		 */
		/* wait until both main streams (pstream / samstream) are off */
		if ((tfa98xx->pstream == 0)
			&& (tfa98xx->samstream == 0)) {
			pr_info("mute is triggered\n");
		} else {
			pr_info("mute is suspended when only cstream is off\n");
			return 0;
		}

		mutex_lock(&tfa98xx_mutex);
		tfa98xx_sync_count = 0;
		mutex_unlock(&tfa98xx_mutex);

		_tfa98xx_stop(tfa98xx);
	} else {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			tfa98xx->pstream = 1;
		else if (stream == SNDRV_PCM_STREAM_CAPTURE)
			tfa98xx->cstream = 1;
		mutex_lock(&tfa98xx->dsp_lock);
		pr_info("mute:%d [pstream %d, cstream %d, samstream %d]\n", mute,
			tfa98xx->pstream, tfa98xx->cstream, tfa98xx->samstream);
		tfa98xx_set_stream_state(tfa98xx->tfa,
			(tfa98xx->pstream & BIT_PSTREAM)
			|((tfa98xx->cstream<<1) & BIT_CSTREAM)
			|((tfa98xx->samstream<<2) & BIT_SAMSTREAM));
		mutex_unlock(&tfa98xx->dsp_lock);

		/* case: either p/cstream (both) or samstream is on
		 * if ((tfa98xx->pstream != 0 && tfa98xx->cstream != 0)
		 *  || tfa98xx->samstream != 0) {
		 */
		/* wait until DSP is ready for initialization */
		if (stream == SNDRV_PCM_STREAM_PLAYBACK
			|| stream == SNDRV_PCM_STREAM_SAAM) {
			pr_info("unmute is triggered\n");
		} else {
			pr_info("unmute is suspended when only cstream is on\n");
			return 0;
		}

		pr_debug("%s: unmute with profile %d\n",
			__func__, tfa98xx->profile);

		/* Only do dsp init for master device */
/*
		if (tfa98xx->tfa->dev_idx != 0) {
			pr_err("%s: no first handle\n", __func__);
			return 0;
		}
*/

		/* Start DSP */
		pr_info("%s: start tfa amp\n", __func__);
#if 1
		if (tfa98xx->dsp_init != TFA98XX_DSP_INIT_PENDING)
			queue_delayed_work(tfa98xx->tfa98xx_wq,
				&tfa98xx->init_work, 0);
#else
		tfa98xx_dsp_init(tfa98xx);
#endif
	}

	return 0;
}

static int _tfa98xx_stop(struct tfa98xx *tfa98xx)
{
	cancel_delayed_work_sync(&tfa98xx->monitor_work);

	cancel_delayed_work_sync(&tfa98xx->init_work);
	if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK)
		return 0;

	mutex_lock(&tfa98xx->dsp_lock);
	tfa_dev_stop(tfa98xx->tfa);
	tfa98xx->dsp_init = TFA98XX_DSP_INIT_STOPPED;
	tfa98xx_set_dsp_configured(tfa98xx);
	mutex_unlock(&tfa98xx->dsp_lock);

	return 0;
}

static const struct snd_soc_dai_ops tfa98xx_dai_ops = {
	.startup = tfa98xx_startup,
	.set_fmt = tfa98xx_set_fmt,
	.set_sysclk = tfa98xx_set_dai_sysclk,
	.set_tdm_slot = tfa98xx_set_tdm_slot,
	.hw_params = tfa98xx_hw_params,
	.mute_stream = tfa98xx_mute,
};

static struct snd_soc_dai_driver tfa98xx_dai[] = {
	{
		.name = "tfa98xx-aif",
		.id = 1,
		.playback = {
			.stream_name = "AIF Playback",
			.channels_min = 1,
			.channels_max = MAX_HANDLES,
			.rates = TFA98XX_RATES,
			.formats = TFA98XX_FORMATS,
		},
		.capture = {
			.stream_name = "AIF Capture",
			.channels_min = 1,
			.channels_max = MAX_HANDLES,
			.rates = TFA98XX_RATES,
			.formats = TFA98XX_FORMATS,
		},
		.ops = &tfa98xx_dai_ops,
		.symmetric_rates = 1,
#if KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE
		.symmetric_channels = 1,
		.symmetric_samplebits = 0,
#endif
	},
};

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
static int tfa98xx_probe(struct snd_soc_component *component)
{
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(component);
#else
static int tfa98xx_probe(struct snd_soc_codec *codec)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	int ret;

	pr_debug("\n");

	/* setup work queue, will be used to initial DSP on first boot up */
	tfa98xx->tfa98xx_wq = create_singlethread_workqueue("tfa98xx");
	if (!tfa98xx->tfa98xx_wq)
		return -ENOMEM;
#if defined(MPLATFORM)
	tfa98xx->tfa->tfacal_wq = create_singlethread_workqueue("tfacal");
	if (!tfa98xx->tfa->tfacal_wq)
		return -ENOMEM;
#endif

	INIT_DELAYED_WORK(&tfa98xx->init_work, tfa98xx_dsp_init_work);
	INIT_DELAYED_WORK(&tfa98xx->monitor_work, tfa98xx_monitor);
	INIT_DELAYED_WORK(&tfa98xx->interrupt_work, tfa98xx_interrupt);
#if defined(USE_TFA9891)
	INIT_DELAYED_WORK(&tfa98xx->tapdet_work, tfa98xx_tapdet_work);
#endif
#if defined(TFA_USE_WAITQUEUE_SEQ)
	init_waitqueue_head(&tfa98xx->tfa->waitq_seq);
#endif

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	tfa98xx->component = component;
#else
	tfa98xx->codec = codec;
#endif

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	snd_soc_component_init_regmap(component, tfa98xx->regmap);
#endif

#if defined(CONFIG_MACH_KONA_TIMELM)
    if (lge_get_board_rev_no_for_dlkm() <= 7)
        fw_name = "tfa98xx_reva.cnt";

    dev_info(component->dev, "use cnt file rev_no %d, fw_name %s\n", lge_get_board_rev_no_for_dlkm(), fw_name);
#endif

	ret = tfa98xx_load_container(tfa98xx);
	pr_debug("Container loading requested: %d\n", ret);

#if KERNEL_VERSION(3, 16, 0) > LINUX_VERSION_CODE
	codec->control_data = tfa98xx->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 8, 16, SND_SOC_REGMAP);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
#endif
	tfa98xx_add_widgets(tfa98xx);

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	dev_info(component->dev, "tfa98xx codec registered (%s)",
		tfa98xx->fw.name);
#else
	dev_info(codec->dev, "tfa98xx codec registered (%s)",
		tfa98xx->fw.name);
#endif

	return ret;
}

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
static void tfa98xx_remove(struct snd_soc_component *component)
{
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(component);
#else
static int tfa98xx_remove(struct snd_soc_codec *codec)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif

	pr_debug("\n");

	tfa98xx_interrupt_enable(tfa98xx, false);

	tfa98xx_inputdev_unregister(tfa98xx);

	cancel_delayed_work_sync(&tfa98xx->interrupt_work);
	cancel_delayed_work_sync(&tfa98xx->monitor_work);
	cancel_delayed_work_sync(&tfa98xx->init_work);
#if defined(USE_TFA9891)
	cancel_delayed_work_sync(&tfa98xx->tapdet_work);
#endif

	if (tfa98xx->tfa98xx_wq)
		destroy_workqueue(tfa98xx->tfa98xx_wq);
#if defined(MPLATFORM)
	if (tfa98xx->tfa->tfacal_wq)
		destroy_workqueue(tfa98xx->tfa->tfacal_wq);
#endif

#if defined(TFADSP_DSP_BUFFER_POOL)
	/* deallocate buffer_pool */
	if (tfa98xx->tfa->dev_idx == 0) {
		int index = 0;

		pr_info("Deallocate buffer_pool\n");
		for (index = 0; index < POOL_MAX_INDEX; index++)
			tfa_buffer_pool(tfa98xx->tfa,
				index, 0, POOL_FREE);
	}
#endif

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	return;
#else
	return 0;
#endif
}

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	/* Do nothing about REGMAP */
#elif KERNEL_VERSION(3, 16, 0) < LINUX_VERSION_CODE
static struct regmap *tfa98xx_get_regmap(struct device *dev)
{
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);

	return tfa98xx->regmap;
}
#endif

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
static struct snd_soc_component_driver soc_component_dev_tfa98xx = {
#else
static struct snd_soc_codec_driver soc_codec_dev_tfa98xx = {
#endif
	.probe = tfa98xx_probe,
	.remove = tfa98xx_remove,
#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	/* Do nothing about REGMAP */
#elif KERNEL_VERSION(3, 16, 0) < LINUX_VERSION_CODE
	.get_regmap = tfa98xx_get_regmap,
#endif
};

static bool tfa98xx_writeable_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return 1;
}

static bool tfa98xx_readable_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return 1;
}

static bool tfa98xx_volatile_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return 1;
}

static const struct regmap_config tfa98xx_regmap = {
	.reg_bits = 8,
	.val_bits = 16,

	.max_register = TFA98XX_MAX_REGISTER,
	.writeable_reg = tfa98xx_writeable_register,
	.readable_reg = tfa98xx_readable_register,
	.volatile_reg = tfa98xx_volatile_register,
	.cache_type = REGCACHE_NONE,
};

static void tfa98xx_irq_tfa2(struct tfa98xx *tfa98xx)
{
	pr_info("\n");

	/*
	 * mask interrupts
	 * will be unmasked after handling interrupts in workqueue
	 */
	tfa_irq_mask(tfa98xx->tfa);
	queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->interrupt_work, 0);
}


static irqreturn_t tfa98xx_irq(int irq, void *data)
{
	struct tfa98xx *tfa98xx = data;

	if (tfa98xx->tfa->tfa_family == 2)
		tfa98xx_irq_tfa2(tfa98xx);

	return IRQ_HANDLED;
}

static int tfa98xx_ext_reset(struct tfa98xx *tfa98xx)
{
	if (tfa98xx && gpio_is_valid(tfa98xx->reset_gpio)) {
		int reset = tfa98xx->reset_polarity;
		gpio_set_value_cansleep(tfa98xx->reset_gpio, reset);
		mdelay(1);
		gpio_set_value_cansleep(tfa98xx->reset_gpio, !reset);
		mdelay(1);
	}
	return 0;
}

static int tfa98xx_parse_dt(struct device *dev,
	struct tfa98xx *tfa98xx, struct device_node *np)
{
	u32 value;
	int ret;

	tfa98xx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (tfa98xx->reset_gpio < 0)
		dev_dbg(dev, "No reset GPIO provided, will not HW reset device\n");

	tfa98xx->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (tfa98xx->irq_gpio < 0)
		dev_dbg(dev, "No IRQ GPIO provided.\n");
	ret = of_property_read_u32(np, "reset-polarity",&value);
	if (ret< 0)
#if defined(USE_TFA9878)
		tfa98xx->reset_polarity = LOW; /* RSTN */
#else
		tfa98xx->reset_polarity = HIGH; /* RST */
#endif
	else
		tfa98xx->reset_polarity = (value == 0) ? LOW : HIGH;

	dev_info(dev, "reset-polarity:%d\n", tfa98xx->reset_polarity);

	return 0;
}

static ssize_t tfa98xx_reg_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);

	if (count != 1) {
		pr_debug("invalid register address");
		return -EINVAL;
	}

	pr_info("i2c set reg: 0x%x\n", tfa98xx->reg);

	tfa98xx->reg = buf[0];

	return 1;
}

static ssize_t tfa98xx_rw_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);
	u8 *data;
	int ret = 0;
	int retries = I2C_RETRIES;

	data = kmalloc(count + 1, GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		pr_debug("can not allocate memory\n");
		return ret;
	}

	data[0] = tfa98xx->reg;
	memcpy(&data[1], buf, count);

retry:
	ret = i2c_master_send(tfa98xx->i2c, data, count + 1);
	if (ret < 0) {
		pr_warn("i2c error, retries left: %d\n", retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
	}

	kfree(data);

	/* the number of data bytes written without the register address */
	return ((ret > 1) ? count : -EIO);
}

static ssize_t tfa98xx_rw_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);
	struct i2c_msg msgs[] = {
		{
			.addr = tfa98xx->i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = &tfa98xx->reg,
		},
		{
			.addr = tfa98xx->i2c->addr,
			.flags = I2C_M_RD,
			.len = count,
			.buf = buf,
		},
	};
	int ret;
	int retries = I2C_RETRIES;

retry:
	ret = i2c_transfer(tfa98xx->i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		pr_warn("i2c error, retries left: %d\n", retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
		return ret;
	}

	/* ret contains the number of i2c transaction */
	/* return the number of bytes read */
	return ((ret > 1) ? count : -EIO);
}

static struct bin_attribute dev_attr_rw = {
	.attr = {
		.name = "rw",
		.mode = S_IRUSR | S_IWUSR,
	},
	.size = 0,
	.read = tfa98xx_rw_read,
	.write = tfa98xx_rw_write,
};

static struct bin_attribute dev_attr_reg = {
	.attr = {
		.name = "reg",
		.mode = S_IWUSR,
	},
	.size = 0,
	.read = NULL,
	.write = tfa98xx_reg_write,
};

struct tfa_device *tfa98xx_get_tfa_device_from_index(int index)
{
	struct tfa98xx *tfa98xx;
	struct tfa_device *tfa0 = NULL;

	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		if (tfa98xx->tfa->dev_idx == index) {
			tfa0 = tfa98xx->tfa;
			break;
		}
	}

	return tfa0;
}

int tfa98xx_count_active_stream(int stream_flag)
{
	struct tfa98xx *tfa98xx;
	int stream_counter = 0;

	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		if (tfa98xx->tfa->stream_state & stream_flag)
			stream_counter++;
	}

	return stream_counter;
}

static int tfa98xx_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct snd_soc_dai_driver *dai;
	struct tfa98xx *tfa98xx;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags;
	unsigned int reg;
	int ret;

	pr_info("%s: start probing\n", __func__);
	pr_info("addr=0x%x\n", i2c->addr);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	tfa98xx = devm_kzalloc(&i2c->dev,
		sizeof(struct tfa98xx), GFP_KERNEL);
	if (tfa98xx == NULL)
		return -ENOMEM;

	tfa98xx->dev = &i2c->dev;
	tfa98xx->i2c = i2c;
	tfa98xx->dsp_init = TFA98XX_DSP_INIT_STOPPED;
	tfa98xx->rate = 48000; /* init to the default sample rate (48kHz) */
	tfa98xx->tfa = NULL;

	tfa98xx->regmap = devm_regmap_init_i2c(i2c, &tfa98xx_regmap);
	if (IS_ERR(tfa98xx->regmap)) {
		ret = PTR_ERR(tfa98xx->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	i2c_set_clientdata(i2c, tfa98xx);
	mutex_init(&tfa98xx->dsp_lock);
	init_waitqueue_head(&tfa98xx->wq);

	if (np) {
		ret = tfa98xx_parse_dt(&i2c->dev, tfa98xx, np);
		if (ret) {
			dev_err(&i2c->dev, "Failed to parse DT node\n");
			return ret;
		}
		if (no_start)
			tfa98xx->irq_gpio = -1;
		if (no_reset)
			tfa98xx->reset_gpio = -1;
	} else {
		tfa98xx->reset_gpio = -1;
		tfa98xx->irq_gpio = -1;
	}

	if (gpio_is_valid(tfa98xx->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, tfa98xx->reset_gpio,
			GPIOF_OUT_INIT_LOW, "TFA98XX_RST");
		if (ret)
			return ret;
	}

	if (gpio_is_valid(tfa98xx->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, tfa98xx->irq_gpio,
			GPIOF_DIR_IN, "TFA98XX_INT");
		if (ret)
			return ret;
	}

	/* Power up! */
	tfa98xx_ext_reset(tfa98xx);

	if ((no_start == 0) && (no_reset == 0)) {
		ret = regmap_read(tfa98xx->regmap, 0x03, &reg);
		if (ret < 0) {
			dev_err(&i2c->dev, "Failed to read Revision register: %d\n",
				ret);
			return -EIO;
		}

		switch (reg & 0xff) {
		case 0x72: /* tfa9872 */
			pr_info("TFA9872 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_CALIBRATION_CTL;
			tfa98xx->flags |= TFA98XX_FLAG_REMOVE_PLOP_NOISE;
			/* tfa98xx->flags |= TFA98XX_FLAG_LP_MODES; */
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x74: /* tfa9874 */
			pr_info("TFA9874 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_CALIBRATION_CTL;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x78: /* tfa9878 */
			pr_info("TFA9878 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_CALIBRATION_CTL;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x88: /* tfa9888 */
			pr_info("TFA9888 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_STEREO_DEVICE;
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x13: /* tfa9912 */
			pr_info("TFA9912 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			/* tfa98xx->flags |= TFA98XX_FLAG_TAPDET_AVAILABLE; */
			break;
		case 0x94: /* tfa9894 */
			pr_info("TFA9894 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x80: /* tfa9890 */
		case 0x81: /* tfa9890 */
			pr_info("TFA9890 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			break;
		case 0x92: /* tfa9891 */
			pr_info("TFA9891 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_SAAM_AVAILABLE;
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			break;
		case 0x12: /* tfa9895 */
			pr_info("TFA9895 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			break;
		case 0x97:
			pr_info("TFA9897 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x96:
			pr_info("TFA9896 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		default:
			pr_info("Unsupported device revision (0x%x)\n", reg & 0xff);
			return -EINVAL;
		}
	}

	tfa98xx->tfa = devm_kzalloc(&i2c->dev, sizeof(struct tfa_device), GFP_KERNEL);
	if (tfa98xx->tfa == NULL)
		return -ENOMEM;

	tfa98xx->tfa->data = (void *)tfa98xx;
	tfa98xx->tfa->cachep = tfa98xx_cache;

	/* Modify the stream names, by appending the i2c device address.
	 * This is used with multicodec, in order to discriminate devices.
	 * Stream names appear in the dai definition and in the stream.
	 * We create copies of original structures because each device will
	 * have its own instance of this structure, with its own address.
	 */
	dai = devm_kzalloc(&i2c->dev, sizeof(tfa98xx_dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;
	memcpy(dai, tfa98xx_dai, sizeof(tfa98xx_dai));

	tfa98xx_append_i2c_address(&i2c->dev,
		i2c,
		NULL,
		0,
		dai,
		ARRAY_SIZE(tfa98xx_dai));

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	ret = snd_soc_register_component(&i2c->dev,
		&soc_component_dev_tfa98xx, dai,
		ARRAY_SIZE(tfa98xx_dai));
#else
	ret = snd_soc_register_codec(&i2c->dev,
		&soc_codec_dev_tfa98xx, dai,
		ARRAY_SIZE(tfa98xx_dai));
#endif

	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to register TFA98xx: %d\n", ret);
		return ret;
	}

	if (gpio_is_valid(tfa98xx->irq_gpio) &&
		!(tfa98xx->flags & TFA98XX_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
			gpio_to_irq(tfa98xx->irq_gpio),
			NULL, tfa98xx_irq, irq_flags,
			"tfa98xx", tfa98xx);
		if (ret != 0) {
			dev_err(&i2c->dev, "Failed to request IRQ %d: %d\n",
				gpio_to_irq(tfa98xx->irq_gpio), ret);
			return ret;
		}
	} else {
		dev_info(&i2c->dev, "Skipping IRQ registration\n");
		/* disable feature support if gpio was invalid */
		tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
	}

#ifdef CONFIG_DEBUG_FS
	if (no_start == 0)
		tfa98xx_debug_init(tfa98xx, i2c);
#endif
	/* Register the sysfs files for climax backdoor access */
	ret = device_create_bin_file(&i2c->dev, &dev_attr_rw);
	if (ret)
		dev_info(&i2c->dev, "error creating sysfs files\n");
	ret = device_create_bin_file(&i2c->dev, &dev_attr_reg);
	if (ret)
		dev_info(&i2c->dev, "error creating sysfs files\n");

	pr_info("%s Probe completed successfully!\n", __func__);

	INIT_LIST_HEAD(&tfa98xx->list);

	mutex_lock(&tfa98xx_mutex);
	tfa98xx_device_count++;
	list_add(&tfa98xx->list, &tfa98xx_device_list);
	mutex_unlock(&tfa98xx_mutex);

	return 0;
}

static int tfa98xx_i2c_remove(struct i2c_client *i2c)
{
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);

	pr_debug("addr=0x%x\n", i2c->addr);

	tfa98xx_interrupt_enable(tfa98xx, false);

	cancel_delayed_work_sync(&tfa98xx->interrupt_work);
	cancel_delayed_work_sync(&tfa98xx->monitor_work);
	cancel_delayed_work_sync(&tfa98xx->init_work);
#if defined(USE_TFA9891)
	cancel_delayed_work_sync(&tfa98xx->tapdet_work);
#endif

	device_remove_bin_file(&i2c->dev, &dev_attr_reg);
	device_remove_bin_file(&i2c->dev, &dev_attr_rw);
#ifdef CONFIG_DEBUG_FS
	tfa98xx_debug_remove(tfa98xx);
#endif

#if KERNEL_VERSION(4, 18, 0) <= LINUX_VERSION_CODE
	snd_soc_unregister_component(&i2c->dev);
#else
	snd_soc_unregister_codec(&i2c->dev);
#endif

	if (gpio_is_valid(tfa98xx->irq_gpio))
		devm_gpio_free(&i2c->dev, tfa98xx->irq_gpio);
	if (gpio_is_valid(tfa98xx->reset_gpio))
		devm_gpio_free(&i2c->dev, tfa98xx->reset_gpio);

	mutex_lock(&tfa98xx_mutex);
	list_del(&tfa98xx->list);
	tfa98xx_device_count--;
	if (tfa98xx_device_count == 0) {
		kfree(tfa98xx_container);
		tfa98xx_container = NULL;
	}
	mutex_unlock(&tfa98xx_mutex);

	return 0;
}

static const struct i2c_device_id tfa98xx_i2c_id[] = {
	{"tfa98xx", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tfa98xx_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id tfa98xx_dt_match[] = {
	{.compatible = "nxp,tfa98xx"},
	{.compatible = "nxp,tfa9872"},
	{.compatible = "nxp,tfa9874"},
	{.compatible = "nxp,tfa9878"},
	{.compatible = "nxp,tfa9888"},
	{.compatible = "nxp,tfa9890"},
	{.compatible = "nxp,tfa9891"},
	{.compatible = "nxp,tfa9894"},
	{.compatible = "nxp,tfa9895"},
	{.compatible = "nxp,tfa9896"},
	{.compatible = "nxp,tfa9897"},
	{.compatible = "nxp,tfa9912"},
	{},
};
#endif

static struct i2c_driver tfa98xx_i2c_driver = {
	.driver = {
		.name = "tfa98xx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tfa98xx_dt_match),
	},
	.probe = tfa98xx_i2c_probe,
	.remove = tfa98xx_i2c_remove,
	.id_table = tfa98xx_i2c_id,
};

static int __init tfa98xx_i2c_init(void)
{
	int ret = 0;

	pr_info("TFA98XX driver version %s\n", TFA98XX_VERSION);

	/* Enable debug traces */
	tfa98xx_kmsg_regs = trace_level & 2;
	tfa98xx_ftrace_regs = trace_level & 4;

	/* Initialize kmem_cache */
	tfa98xx_cache = kmem_cache_create("tfa98xx_cache", /* Cache name /proc/slabinfo */
		PAGE_SIZE, /* Structure size, we should fit in single page */
		0, /* Structure alignment */
		(SLAB_HWCACHE_ALIGN | SLAB_RECLAIM_ACCOUNT |
		SLAB_MEM_SPREAD), /* Cache property */
		NULL); /* Object constructor */
	if (!tfa98xx_cache) {
		pr_err("tfa98xx can't create memory pool\n");
		ret = -ENOMEM;
	}

	ret = i2c_add_driver(&tfa98xx_i2c_driver);

	return ret;
}
module_init(tfa98xx_i2c_init);

static void __exit tfa98xx_i2c_exit(void)
{
	i2c_del_driver(&tfa98xx_i2c_driver);
	kmem_cache_destroy(tfa98xx_cache);
}
module_exit(tfa98xx_i2c_exit);

MODULE_DESCRIPTION("ASoC TFA98XX driver");
MODULE_LICENSE("GPL");

