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

enum {
    AIS_VIRT_ON = 0,
	AIS_VIRT_MODE,
	AIS_GEQ_ON,
	AIS_GEQ_BAND1,
	AIS_GEQ_BAND2,
	AIS_GEQ_BAND3,
	AIS_GEQ_BAND4,
	AIS_GEQ_BAND5,
	AIS_GEQ_BAND6,
	AIS_GEQ_BAND7,
	AIS_GEQ_BAND8,
	AIS_GEQ_BAND9,
	AIS_GEQ_BAND10,
	AIS_BASSBOOST_ON,
	AIS_VOCALBOOST_ON,
	AIS_LOUDNESS_LEVEL_ON,
	AIS_GEQ_PRESET,
	AIS_BASSBOOST_LEVEL,
	AIS_VOCALBOOST_LEVEL,
	AIS_VOLUME_LEVEL,
	AIS_TURN_ON,
	AIS_SPEAKER_ENABLE,
	AIS_FADE_ENABLE,
	LGE_AIS_PARAM_MAX
};

struct asm_lge_ais_param {
	uint32_t                  value;
} __packed;
#if 0
struct asm_lge_ais_param {
    struct apr_hdr  hdr;
    struct asm_stream_cmd_set_pp_params_v2 param;
    struct asm_stream_param_data_v2 data;
    int32_t                  value;
} __packed;
#endif
int q6asm_set_lge_ais_param(struct audio_client *ac, int param_id, int value);
