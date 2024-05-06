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
#define LGE_CH_SWAPPER_MODULE_ID        0x10078001
#define LGE_CH_SWAPPER_PARAM_ID_ENABLE  0x10078000

int q6adm_set_ch_swapper_parms(int port_id, int param_id, int param_data);
