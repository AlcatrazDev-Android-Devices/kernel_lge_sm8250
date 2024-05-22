/*
 * Copyright (C) 2014-2020 NXP Semiconductors, All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 */

#ifndef _DBGPRINT_H
#   define _DBGPRINT_H

/* Debugging macro's. */
#   ifndef DEBUG
#      define DEBUG
#   endif

#   ifndef ASSERT
//#define ASSERT
#   endif
 //TODO wwwim
#   ifndef _ASSERT
		#define _ASSERT(e)
#   endif

#   ifndef PREFIX
#     define PREFIX "tfa98xx: "
#	define DRIVER_NAME "tfa98xx"
#   endif

#ifdef __KERNEL__

#   ifdef DEBUG
#      define _DEBUG(level,fmt,va...) do {\
                if (unlikely(debug >= (level))) \
                        printk(KERN_INFO PREFIX "%s:%d: "fmt,__func__,__LINE__,##va); \
       } while (0)

#   else
#      define _DEBUG(level,fmt,va...) do {} while(0)
#   endif

#   define MSG(fmt,va...) printk(KERN_INFO PREFIX "%s:%d: "fmt,__func__,__LINE__,##va)
#   define _ERRORMSG(fmt,va...) pr_err(PREFIX "ERROR %s:%d: "fmt,__func__,__LINE__, ##va)


#   define DEBUG0(x...) MSG(x)
#   define DEBUG1(x...) _DEBUG(1,x)
#   define DEBUG2(x...) _DEBUG(2,x)
#   define DEBUG3(x...) _DEBUG(3,x)
#   define ERRORMSG(x...) _ERRORMSG(x)
/*
#   define pr_debug(x...)	printk(x)
#   define pr_err(x...)	printk(KERN_INFO PREFIX " **ERROR** " x)
#   define PRINT_ASSERT(e)	if ((e)) pr_err("PrintAssert:%s (%s:%d) error code:%d\n",__func__,__FILE__,__line__, e)
*/

#   define PRINT_ENTRY DEBUG2("+[%s]\n", __func__)
#   define PRINT_EXIT  DEBUG2("-[%s]\n", __func__)

#   ifdef ASSERT
#      define assert(cond,action) do {if (unlikely(!(cond))) {DEBUG0("Assert: %s\n",#cond); action;}} while(0)
#   else
#      define assert(cond,action) do {} while (0)
#   endif

#else /* __KERNEL__ */
#if defined(WIN32) || defined(_X64)
#include <stdio.h>
/* user mode */
#   ifdef DEBUG
#      define _DEBUGMSG(level,fmt,...)  printf(PREFIX "%s:%d: "fmt,__func__,__line__,__VA_ARGS__);
#   else
#      define _DEBUGMSG(level,fmt,...) do {} while(0)
#   endif

#   define _ERRORMSG(fmt,...) printf(PREFIX "%s:%s:%d: "fmt,__FILE__,__func__,__line__,__VA_ARGS__)

#   define DEBUG0(...) MSG(__VA_ARGS__)
#   define DEBUG1(...) _DEBUGMSG(1,__VA_ARGS__)
#   define DEBUG2(...) _DEBUGMSG(2,__VA_ARGS__)
#   define DEBUG3(...) _DEBUGMSG(3,__VA_ARGS__)
#   define ERRORMSG(fmt,...) _ERRORMSG(fmt,__VA_ARGS__)
#	define pr_debug(...)	printf(__VA_ARGS__)
/*
#	define pr_debug(...) {	FILE *stream;														\
							if ((stream = freopen("tfa_debug.txt", "ab+", stdout)) == NULL) exit(-1);	\
							printf(__VA_ARGS__);												\
							freopen("CON", "ab+", stdout);										\
						}
*/
#	define pr_err(...)	fprintf(stderr,__VA_ARGS__)
#	define PRINT_FILE(file,...)	fprintf(file,__VA_ARGS__)
#	define PRINT_ASSERT(e)	if ((e)) fprintf(stderr, "PrintAssert:%s (%s:%d) error code:%d\n",__func__,__FILE__,__line__, e)
//#	define PRINT_ASSERT(e)	if ((e)) fprintf(stderr, "PrintAssert:%s (%s:%d) %s\n",__func__,__FILE__,__line__, Tfa98xx_GetErrorString(e))

#elif defined(__CODE_RED)
#include "app_global.h"
#   ifdef DEBUG
#      define _DEBUG(level,fmt,va...) TB_TRACE_INF(TbTracePfx2("tfa",TB_FUNC,va))
//printf(PREFIX "%s:%d: "fmt,__func__,__line__,##va);
#   else
#      define _DEBUG(level,fmt,va...) do {} while(0)
#   endif

#   define MSG(fmt,...) TB_TRACE_INF(TbTracePfx2("tfa",TB_FUNC,__VA_ARGS__))
//printf(PREFIX "%s:%s:%d: "fmt,__FILE__,__func__,__line__,##va)
//TB_TRACE_INF(TbTracePfx2(APP_PFX,TB_FUNC, "path=%s, chan=%u, muted=%s, vol=%d\n",
//                                              path->isRecording ? "recording" : "playback",
//                                              i,
//                                              channelVol.currentMuteValue ? "YES" : "NO",
//                                              channelVol.currentVolumeValue
//                                             ));
//#   define _ERRORMSG(fmt,va...) TB_TRACE_INF(TbTracePfx2("tfa",TB_FUNC,va))
#   define ERRORMSG(...) TB_TRACE_INF(TbTracePfx2("tfa",TB_FUNC,__VA_ARGS__))
//fprintf(stderr, PREFIX "ERROR %s:%s:%d: "fmt,__FILE__,__func__,__line__, ##va)

#   define DEBUG0(x...) MSG(x)
#   define DEBUG1(x...) _DEBUG(1,x)
#   define DEBUG2(x...) _DEBUG(2,x)
#   define DEBUG3(x...) _DEBUG(3,x)
//#   define ERRORMSG(x...) _ERRORMSG(x)
#	define pr_debug(x...)	TB_TRACE_INF(TbTracePfx2("tfa",TB_FUNC,x))
//printf(x)
#	define pr_err(x...) TB_TRACE_INF(TbTracePfx2("tfa",TB_FUNC,x))
//fprintf(stderr,__VA_ARGS__)
#	define PRINT_FILE(file,x...) TB_TRACE_INF(TbTracePfx2("tfa",TB_FUNC,x))
//fprintf(file,__VA_ARGS__)
#	define PRINT_ASSERT(e)
//TB_TRACE_INF(TbTracePfx2("tfa",TB_FUNC,Tfa98xx_GetErrorString(e)))
//if ((e)) fprintf(stderr, "PrintAssert:%s (%s:%d) %s\n",__func__,__FILE__,__line__, Tfa98xx_GetErrorString(e))
#else
#include <stdio.h>
/* user mode */
#   ifdef DEBUG
#      define _DEBUG(level,fmt,va...)  printf(PREFIX "%s:%d: "fmt,__func__,__line__,##va);
#   else
#      define _DEBUG(level,fmt,va...) do {} while(0)
#   endif

#   define MSG(fmt,va...) printf(PREFIX "%s:%s:%d: "fmt,__FILE__,__func__,__line__,##va)
#   define _ERRORMSG(fmt,va...) fprintf(stderr, PREFIX "ERROR %s:%s:%d: "fmt,__FILE__,__func__,__line__, ##va)

#   define DEBUG0(x...)	MSG(x)
#   define DEBUG1(x...)	_DEBUG(1,x)
#   define DEBUG2(x...)	_DEBUG(2,x)
#   define DEBUG3(x...)	_DEBUG(3,x)
#   define ERRORMSG(x...)	_ERRORMSG(x)
#	define pr_debug(x...)	printf(x)
#	define pr_err(...)	fprintf(stderr,__VA_ARGS__)
#	define PRINT_FILE(file,...)	fprintf(file,__VA_ARGS__)
#	define PRINT_ASSERT(e)	if ((e)) fprintf(stderr, "PrintAssert:%s (%s:%d) error code:%d\n",__func__,__FILE__,__line__, e)
//#	define PRINT_ASSERT(e)	if ((e)) fprintf(stderr, "PrintAssert:%s (%s:%d) %s\n",__func__,__FILE__,__line__, Tfa98xx_GetErrorString(e))


#endif	 /* WIN32 */

#endif	 /* user */

#endif				/* _DBGPRINT_H --------------- */
