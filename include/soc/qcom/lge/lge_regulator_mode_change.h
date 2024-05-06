/* Copyright (c) 2014 LG Electronics, Inc.
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
#ifndef LGE_REGULATOR_MODE_CHANGE_H
#define LGE_REGULATOR_MODE_CHANGE_H

extern void bob_mode_enable(void);
extern void bob_mode_disable(void);
#ifdef CONFIG_MACH_KONA_TIMELM
bool isEnable;
#endif

#endif
