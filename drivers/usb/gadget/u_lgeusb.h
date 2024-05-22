/* linux/drivers/usb/gadget/u_lgeusb.h
 *
 * Copyright (C) 2011, 2012 LG Electronics Inc.
 * Author : Hyeon H. Park <hyunhui.park@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __U_LGEUSB_H__
#define __U_LGEUSB_H__

#define LGE_USB_VID		0x1004
#define LGE_USB_FACTORY_PID	0x6000

#ifdef CONFIG_LGE_USB_GADGET_AUTORUN
int lgeusb_get_model_name(char *, size_t);
int lgeusb_get_phone_id(char *, size_t);
int lgeusb_get_sw_ver(char *, size_t);
int lgeusb_get_sub_ver(char *, size_t);
#endif

#endif /* __U_LGEUSB_H__ */
