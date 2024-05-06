/*
 * Copyright (C) 2014-2020 NXP Semiconductors, All Rights Reserved.
 * Copyright 2020 GOODIX
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 */

/**\file
 *
 * The tfa_device interface controls a single I2C device instance by
 * referencing to the device specific context provided by means of the
 * tfa_device structure pointer.
 * Multiple instances of tfa_device structures will be created and maintained
 * by the caller.
 *
 * The API is functionally grouped as:
 *  - tfa_dev basic codec interface to probe, start/stop and control the device state
 *  - access to internal MTP storage
 *  - abstraction for interrupt bits and handling
 *  - container reading support
 */
#ifndef __TFA_DEVICE_H__
#define __TFA_DEVICE_H__

#include "config.h"

struct tfa_device;

/*
 * hw/sw feature bit settings in MTP
 */
enum feature_support {
	SUPPORT_NOT_SET, /**< default means not set yet */
	SUPPORT_NO,      /**< no support */
	SUPPORT_YES      /**< supported */
};
/*
 * supported Digital Audio Interfaces bitmap
 */
enum tfa98xx_dai_bitmap {
	TFA98XX_DAI_I2S = 0x01, /**< I2S only */
	TFA98XX_DAI_TDM = 0x02, /**< TDM, I2S */
	TFA98XX_DAI_PDM = 0x04, /**< PDM  */
};

/*
 * device ops function structure
 */
struct tfa_device_ops {
#ifdef QPLATFORM
	int(*dsp_msg)(struct tfa_device *tfa, int length, const char *buf);
	int(*dsp_msg_read)(struct tfa_device *tfa, int length, unsigned char *bytes);
#else
	enum tfa98xx_error(*dsp_msg)(struct tfa_device *tfa, int length, const char *buf);
	enum tfa98xx_error(*dsp_msg_read)(struct tfa_device *tfa, int length, unsigned char *bytes);
#endif
	enum tfa98xx_error(*reg_read)(struct tfa_device *tfa, unsigned char subaddress, unsigned short *value);
	enum tfa98xx_error(*reg_write)(struct tfa_device *tfa, unsigned char subaddress, unsigned short value);
	enum tfa98xx_error(*mem_read)(struct tfa_device *tfa, unsigned int start_offset, int num_words, int *p_values);
	enum tfa98xx_error(*mem_write)(struct tfa_device *tfa, unsigned short address, int value, int memtype);

	enum tfa98xx_error (*tfa_init)(struct tfa_device *tfa); /**< init typically for loading optimal settings */
	enum tfa98xx_error (*dsp_reset)(struct tfa_device *tfa, int state); /**< reset the coolflux dsp */
	enum tfa98xx_error (*dsp_system_stable)(struct tfa_device *tfa, int *ready); /**< ready when clocks are stable to allow DSP subsystem access */
	enum tfa98xx_error (*dsp_write_tables)(struct tfa_device *tfa, int sample_rate); /**< write the device/type specific delaytables */
	enum tfa98xx_error (*auto_copy_mtp_to_iic)(struct tfa_device *tfa); /**< Set auto_copy_mtp_to_iic */
	enum tfa98xx_error (*factory_trimmer)(struct tfa_device *tfa); /**< Factory trimming for the Boost converter */
	int (*set_swprof)(struct tfa_device *tfa, unsigned short new_value); /**< Set the sw profile in the struct and the hw register */
	int (*get_swprof)(struct tfa_device *tfa); /**< Get the sw profile from the hw register */
	int(*set_swvstep)(struct tfa_device *tfa, unsigned short new_value); /**< Set the sw vstep in the struct and the hw register */
	int(*get_swvstep)(struct tfa_device *tfa); /**< Get the sw vstep from the hw register */
	int(*get_mtpb)(struct tfa_device *tfa); /**< get status of MTB busy bit*/
	enum tfa98xx_error (*set_mute)(struct tfa_device *tfa, int mute); /**< set mute */
	enum tfa98xx_error (*faim_protect)(struct tfa_device *tfa, int state); /**< Protect FAIM from being corrupted  */
	enum tfa98xx_error(*set_osc_powerdown)(struct tfa_device *tfa, int state); /**< Allow to change internal osc. gating settings */
	enum tfa98xx_error(*update_lpm)(struct tfa_device *tfa, int state); /**< Allow to change lowpowermode settings */
};

/**
 * Device states and modifier flags to allow a device/type independent fine
 * grained control of the internal state.\n
 * Values below 0x10 are referred to as base states which can be or-ed with
 * state modifiers, from 0x10 and higher.
 *
 */
enum tfa_state {
	TFA_STATE_UNKNOWN,      /**< unknown or invalid */
	TFA_STATE_POWERDOWN,    /**< PLL in powerdown, Algo is up/warm */
	TFA_STATE_INIT_HW,      /**< load I2C/PLL hardware setting (~wait2srcsettings) */
	TFA_STATE_INIT_CF,      /**< coolflux HW access possible (~initcf) */
	TFA_STATE_INIT_FW,      /**< DSP framework active (~patch loaded) */
	TFA_STATE_OPERATING,    /**< Amp and Algo running */
	TFA_STATE_FAULT,        /**< An alarm or error occurred */
	TFA_STATE_RESET,        /**< I2C reset and ACS set */
	/* --sticky state modifiers-- */
	TFA_STATE_MUTE=0x10,         /**< Algo & Amp mute */
	TFA_STATE_UNMUTE=0x20,       /**< Algo & Amp unmute */
	TFA_STATE_CLOCK_ALWAYS=0x40, /**< PLL connect to internal oscillator */
	TFA_STATE_CLOCK_AUDIO=0x80,  /**< PLL connect to audio clock (BCK/FS) */
	TFA_STATE_LOW_POWER=0x100,   /**< lowest possible power state */
};

#if defined(TFADSP_DSP_BUFFER_POOL)
enum pool_control {
	POOL_NOT_SUPPORT,
	POOL_ALLOC,
	POOL_FREE,
	POOL_GET,
	POOL_RETURN,
	POOL_MAX_CONTROL
};

#define POOL_MAX_INDEX 6

struct tfa98xx_buffer_pool {
	int size;
	unsigned char in_use;
	void *pool;
};
#endif // TFADSP_DSP_BUFFER_POOL

/**
 * This is the main tfa device context structure, it will carry all information
 * that is needed to handle a single I2C device instance.
 * All functions dealing with the device will need access to the fields herein.
 */
struct tfa_device {
	int dev_idx;			/**< device container index */
	int in_use;
	int buffer_size;		/**< lowest level max buffer size */
	int has_msg; 			/**< support direct dsp messaging */
	unsigned char slave_address; /**< I2C slave address (not shifted) */
	unsigned short rev;     /**< full revid of this device */
	unsigned char tfa_family; /**< tfa1/tfa2 */
	enum feature_support support_drc;
	enum feature_support support_framework;
	enum feature_support support_saam;
	int sw_feature_bits[2]; /**< cached copy of sw feature bits */
	int hw_feature_bits;    /**< cached copy of hw feature bits */
	int profile;            /**< active profile */
	int vstep;              /**< active vstep */
	unsigned char spkr_count;
	unsigned char spkr_select;
	unsigned char support_tcoef;/**< legacy tfa9887, will be removed */
	enum tfa98xx_dai_bitmap daimap; /**< supported audio interface types */
	int mohm[3]; /**< speaker calibration values in milli ohms -1 is error */
	struct tfa_device_ops dev_ops;
	uint16_t interrupt_enable[3];
	uint16_t interrupt_status[3];
	int ext_dsp; /**< respond to external DSP: -1:none, 0:no_dsp, 1:cold, 2:warm */
	int bus; /* TODO fix ext_dsp and bus handling */
	int tfadsp_event; /**< enum tfadsp_event_en is for external registry */
	int verbose; /**< verbosity level for debug print output */
	enum tfa_state state;  /**< last known state or-ed with optional state_modifier */
	struct tfa_container *cnt;/**< the loaded container file */
	struct tfa_volume_step_register_info *p_reg_info; /**< remember vstep for partial updates */
	int partial_enable; /**< enable partial updates */
	void *data; /**< typically pointing to Linux driver structure owning this device */
	int convert_dsp32; /**< convert 24 bit DSP messages to 32 bit */
	int sync_iv_delay; /**< synchronize I/V delay at cold start */
	int is_probus_device; /**< probus device: device without internal DSP */
	int advance_keys_handling;
	int needs_reset; /**< add the reset trigger for SetAlgoParams and SetMBDrc commands */
	struct kmem_cache *cachep;	/**< Memory allocator handle */
	char fw_itf_ver[4];          /* Firmware ITF version */

	int temp; /* calibration temperature */
	int spkr_damaged; /* check if speaker is damaged */
	int is_cold; /* respond to MANSTATE, before tfa_run_speaker_boost */
	int is_bypass; /* respond to vstep in profile, before sending calibration data */
	int is_configured; /* respond to DSP state, whether TFADSP is loaded */
	int reset_mtpex; /* suspended resetting MTPEX, if it's called when not active */
	int stream_state; /* b0: pstream (Rx), b1: cstream (Tx), b2: samstream (SaaM) */
	int prev_samstream;
	int first_after_boot;
#if defined(TFA_USE_DEVICE_SPECIFIC_CONTROL)
	int active_handle;
#endif
	int ampgain;
	int individual_msg;
#if defined(TFA_USE_OVERRIDING_PROFILE)
	int skip_dsp_msg;
#endif
#if defined(TFADSP_DSP_BUFFER_POOL)
	struct tfa98xx_buffer_pool buf_pool[POOL_MAX_INDEX];
#endif
#if defined(TFA_USE_WAITQUEUE_SEQ)
	wait_queue_head_t waitq_seq;
#endif
#if defined(MPLATFORM)
	struct workqueue_struct *tfacal_wq;
	struct delayed_work wait_cal_work;
#endif
};

/**
 * The tfa_dev_probe is called before accessing any device accessing functions.
 * Access to the tfa device register 3 is attempted and will record the
 * returned id for further use. If no device responds the function will abort.
 * The recorded id will by used by the query functions to fill the remaining
 * relevant data fields of the device structure.
 * Data such as MTP features that requires device access will only be read when
 * explicitly called and the result will be then cached in the struct.
 *
 * A structure pointer passed to this device needs to refer to existing memory
 * space allocated by the caller.
 *
 *  @param slave = I2C slave address of the target device (not shifted)
 *  @param tfa struct = points to memory that holds the context for this device
 *  instance
 *
 *  @return
 *   - 0 if the I2C device responded to a read of register address 3\n
 *       when the device responds but with an unknown id a warning will be printed
 *   - -1 if no response from the I2C device
 *
 */
int tfa_dev_probe(int slave, struct tfa_device *tfa);

/**
 * Start this instance at the profile and vstep as provided.
 * The profile and vstep will be loaded first in case the current value differs
 * from the requested values.
 * Note that this call will not change the mute state of the tfa, which means
 * that of this instance was called in muted state the caller will have to
 * unmute in order to get audio.
 *
 *  @param tfa struct = pointer to context of this device instance
 *  @param profile the selected profile to run
 *  @param vstep the selected vstep to use
 *  @return tfa_error enum
 */
enum tfa_error tfa_dev_start(struct tfa_device *tfa, int profile, int vstep);

enum tfa_error tfa_dev_switch_profile(struct tfa_device *tfa, int profile, int vstep);

/**
 * Stop audio for this instance as gracefully as possible.
 * Audio will be muted and the PLL will be shutdown together with any other
 * device/type specific settings needed to prevent audio artifacts or
 * workarounds.
 *
 * Note that this call will change state of the tfa to mute and powered down.
 *
 *  @param tfa struct = pointer to context of this device instance
 *  @return tfa_error enum
 */
enum tfa_error tfa_dev_stop(struct tfa_device *tfa);

/**
 * This interface allows a device/type independent fine grained control of the
 * internal state of the instance.
 * Whenever a base state is requested an attempt is made to actively bring the device
 * into this state. However this may depend on external conditions beyond control of
 * this software layer. Therefore in case the state cannot be set an erro will
 * be returned and the current state remains unchanged.
 * The base states, lower values below 0x10, are all mutually exclusive, they higher ones
 * can also function as a sticky modifier which means for example that operating
 * state could be in either muted or unmuted state. Or in case of init_cf it can be
 * internal clock (always) or external audio clock.
 * This function is intended to be used for device mute/unmute synchronization
 * when called from higher layers. Mostly internal calls will use this to control
 * the startup and profile transitions in a device/type independent way.
 *
 *  @param tfa struct = pointer to context of this device instance
 *  @param state struct = desired device state after function return
 *  @return tfa_error enum
 */
enum tfa_error tfa_dev_set_state(struct tfa_device *tfa, enum tfa_state state,int is_calibration);

/**
 * Retrieve the current state of this instance in an active way.
 * The state field in tfa structure will reflect the result unless an error is
 * returned.
 * Note that the hardware state may change on external events an as such this
 * field should be treated as volatile.
 *
 *  @param tfa struct = pointer to context of this device instance
 *  @return tfa_error enum
 *
 */
enum tfa_state tfa_dev_get_state(struct tfa_device *tfa);

/*****************************************************************************/
/*****************************************************************************/
/**
 *  MTP support functions
 */
enum tfa_mtp {
	TFA_MTP_OTC,		/**< */
	TFA_MTP_EX,		/**< */
	TFA_MTP_RE25,		/**< */
	TFA_MTP_RE25_PRIM,	/**< */
	TFA_MTP_RE25_SEC,	/**< */
	TFA_MTP_LOCK,		/**< */
};

/**
 *
 */
int tfa_dev_mtp_get(struct tfa_device *tfa, enum tfa_mtp item);

/**
 *
 */
enum tfa_error tfa_dev_mtp_set(struct tfa_device *tfa, enum tfa_mtp item, int value);


//irq
/* tfa2 interrupt support
 *    !!! enum tfa9912_irq !!!*/
/*
 * interrupt bit function to clear
 */
int tfa_irq_clear(struct tfa_device *tfa, int bit);
/*
 * return state of irq or -1 if illegal bit
 */
int tfa_irq_get(struct tfa_device *tfa, int bit);
/*
 * interrupt bit function that operates on the shadow regs in the handle
 */
int tfa_irq_ena(struct tfa_device *tfa, int bit, int state);
/*
 * interrupt bit function that sets the polarity
 */
int tfa_irq_set_pol(struct tfa_device *tfa, int bit, int state);

/*
 * mask interrupts by disabling them
 */
int tfa_irq_mask(struct tfa_device *tfa);
/*
 * unmask interrupts by enabling them again
 */
int tfa_irq_unmask(struct tfa_device *tfa);
//cnt read
//debug?

enum tfa98xx_error tfa_get_fw_api_version(struct tfa_device *tfa,
	unsigned char *pfw_version);

#if defined(TFA_RAMPDOWN_BEFORE_MUTE)
#define RAMPDOWN_MAX 2 /* 5 or higher if usleep_range works */
enum tfa98xx_error tfa_gain_rampdown(struct tfa_device *tfa,
	int step, int count);
enum tfa98xx_error tfa_gain_restore(struct tfa_device *tfa,
	int step, int count);
#endif

#if defined(MPLATFORM)
enum tfa98xx_error ipi_tfadsp_write(struct tfa_device *tfa,
	int length, const char *buf);
enum tfa98xx_error ipi_tfadsp_read(struct tfa_device *tfa,
	int length, unsigned char *bytes);
#endif

#endif /* __TFA_DEVICE_H__ */

