/*
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

// Topology ID, Module ID, Parameter ID
#define AUDIO_TOPOLOGY_LVACFQ    		0x1000BFFA
#define AUDIO_MODULE_AC          		0x1000BA01
#define AUDIO_PARAM_AC_OEM_CONTROL		0x1000BA22
#define AUDIO_PARAM_AC_OEM_VOICEFOCUS	0x1000BA23
#define AUDIO_PARAM_AC_OEM_DEVICEINFO	0x1000BA24

#define CAPI_V2_MODULE_ID_LGE_MIXER 0x10003000
#define CAPI_V2_PARAM_LGE_MIXER_MIXLEVEL 0x10003002

#define ADM_RAM_STATUS 0x00010D01
#define LGE_SWITCH_RAM_STATUS "ram_status"

#define AC_INPUTGAIN_MIN    (-72)
#define AC_OUTPUTGAIN_MIN   (-96)
#define AC_VOL_VIDEO_MUTE   (-800)
#define AC_VOL_DOWN_WITHDRC (-17)
#define AC_HPF_ENHANCED     2
#define AC_HPF_MIN          50
#define AC_HPF_MAX          350

#define AC_OEM_DEVICE_FACING_BACK 	(1)	///< The device is faced in opposite direction as main screen
#define AC_OEM_DEVICE_FACING_FRONT	(0) ///< The device is faced in same direction as main screen


struct tx_control_param_t {

	int32_t AC_OperatingMode;

	// AC_LVAC_Gain_ControlParams_st Gain;
	int16_t InputGain[4];
	int16_t EqGain;
	int16_t WnsGain;
	int16_t OutputGain;
	uint16_t Gain_reserved;

	// AC_LVAC_WNS_ControlParams_st WNS; 
	int32_t WNS_OperatingMode; 
	int16_t WindNoiseSuppression[2];

	// AC_LVAC_Hpf_ControlParams_st Hpf;
	int32_t HPF_OperatingMode; 
	int16_t HighpassFrequency;
	uint16_t HPF_reserved;
	
	// AC_LVAC_Avl_ControlParams_st Avl; 
	int32_t AVL_OperatingMode; 
	int16_t TargetGain; 
	uint16_t AttackTime;
	uint16_t ReleaseTime;
	uint16_t AVL_reserved;

	// AC_LVAC_Limiter_ControlParams_st Limiter; 
	int32_t Limiter_OperatingMode;
	int16_t LimiterThreshold;
	uint16_t Limiter_reserved;

	// AC_LVAC_OutputCompressor_ControlParams_st OutputCompressor; 
	int32_t OutComp_OperatingMode;  
	uint16_t MdrcBandCount;
	uint16_t Mdrc_0_LowFrequency; 
	uint16_t Mdrc_0_KneeCount;
	uint16_t Mdrc_0_AttackTime; 
	uint16_t Mdrc_0_ReleaseTime;
	int16_t Mdrc_0_InputLevels[5]; 
	int16_t Mdrc_0_OutputLevels[5];
	uint16_t Mdrc_1_LowFrequency; 
	uint16_t Mdrc_1_KneeCount;
	uint16_t Mdrc_1_AttackTime; 
	uint16_t Mdrc_1_ReleaseTime;
	int16_t Mdrc_1_InputLevels[5]; 
	int16_t Mdrc_1_OutputLevels[5];
	uint16_t Mdrc_2_LowFrequency; 
	uint16_t Mdrc_2_KneeCount;
	uint16_t Mdrc_2_AttackTime; 
	uint16_t Mdrc_2_ReleaseTime;
	int16_t Mdrc_2_InputLevels[5]; 
	int16_t Mdrc_2_OutputLevels[5];
	uint16_t Mdrc_3_LowFrequency; 
	uint16_t Mdrc_3_KneeCount;
	uint16_t Mdrc_3_AttackTime; 
	uint16_t Mdrc_3_ReleaseTime;
	int16_t Mdrc_3_InputLevels[5]; 
	int16_t Mdrc_3_OutputLevels[5];
	uint16_t Mdrc_4_LowFrequency; 
	uint16_t Mdrc_4_KneeCount;
	uint16_t Mdrc_4_AttackTime; 
	uint16_t Mdrc_4_ReleaseTime;
	int16_t Mdrc_4_InputLevels[5]; 
	int16_t Mdrc_4_OutputLevels[5];
	uint16_t Mdrc_reserved;

	// AC_LVAC_HighSpl_ControlParams_st HighSpl; 
	int32_t HighSpl_OperatingMode;  
	int16_t HighSpl_AlgoChoice;
	int16_t HighSpl_SaturationCheckType;
	int16_t HighSpl_SaturationThreshold;  
	int16_t HighSpl_SaturationStrength;  
	int16_t HighSpl_SaturationMax; 
	int16_t HighSpl_SaturationSlowRelease; 
	int16_t HighSpl_SaturationSlowAttack;
	int16_t HighSpl_SaturationFastThreshold;  
	int16_t HighSpl_SaturationThreshold2; 
	int16_t HighSpl_SaturationStrength2; 
	int16_t HighSpl_SaturationMax2; 
	int16_t HighSpl_SaturationSlowRelease2; 
	int16_t HighSpl_SaturationSlowAttack2;
	int16_t HighSpl_SaturationFastThreshold2; 
	int16_t HighSpl_MixALowFreq;	
	int16_t HighSpl_MixAHighFreq; 
	int16_t HighSpl_MixAInBandMixingCoeff; 
	int16_t HighSpl_AlgoCoreStrength; 
	int16_t HighSpl_AlgoCoreClippingReduction; 
	int16_t HighSpl_SamBandPassHighFrequency; 
	int16_t HighSpl_CurrentSaturationThreshold;  
	int16_t HighSpl_SamLevelThreshold;  
	int16_t HighSpl_MixCLowFreq;	 
	int16_t HighSpl_MixCHighFreq;	 
	int16_t HighSpl_SamCalibCoeffFreqStart; 
	int16_t HighSpl_SamCalibCoeffMax; 
	int16_t HighSpl_SamCalibCoeffMid;	  
	int16_t HighSpl_SamCalibCoeffMin; 
	int16_t HighSpl_MixCoherence; 
	int16_t HighSpl_MixCoherenceRelease;
	int16_t HighSpl_MixCoherenceAttack;
	uint16_t HighSpl_reserved;
} __packed;

struct tx_ac_voicefocus_param_t {
	uint16_t EffectLevel;
	int16_t AudioFocusAngle;  
	uint16_t AudioFocusWidth;  
	int16_t Gain; 
	uint16_t VoiceFocusEffectLevel;  
} __packed;

struct tx_ac_deviceinfo_param_t {
	int32_t DeviceFacing;
} __packed;

struct lgemixer_mixinglevel_t {
	int mixinglevel;
};

int q6adm_set_tx_cfg_parms(int port_id, int param_id, struct tx_control_param_t *tx_control_param);
int q6adm_set_tx_voice_focus_parms(int port_id, int param_id, struct tx_ac_voicefocus_param_t *tx_control_param);
int q6adm_set_tx_device_info_parms(int port_id, int param_id, struct tx_ac_deviceinfo_param_t *tx_control_param);
int q6adm_set_tx_mix_parms(int port_id, int param_id, struct lgemixer_mixinglevel_t *tx_control_param);
