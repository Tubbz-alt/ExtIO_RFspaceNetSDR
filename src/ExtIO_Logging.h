#pragma once

#include "LC_ExtIO_Types.h"

#ifdef _MSC_VER
	#pragma warning(disable : 4996)
	#define snprintf  _snprintf
#endif

/* ExtIO Callback */
extern pfnExtIOCallback gpfnExtIOCallbackPtr;

// error message, with "const char*" in IQdata,
//   intended for a log file  AND  a message box
// for messages extHw_MSG_*

extern char acExtioMsg[1024];
extern bool SDRsupportsLogging;


#define SDRLOGTXT( A, TEXT )	\
	do {												\
		if ( gpfnExtIOCallbackPtr && SDRsupportsLogging )		\
			gpfnExtIOCallbackPtr(-1, A, 0, TEXT );			\
  	} while (0)

// optional after calling gpfnExtIOCallbackPtr() ..
// 		if (extHw_MSG_ERRDLG == A)	\
//			setStatusCB(TEXT, true);	\


#define SDRLOG( A, TEXTFMT, ... )	\
  do {                            \
    if ( gpfnExtIOCallbackPtr && SDRsupportsLogging )     \
    {                                                     \
      snprintf(acExtioMsg, 1023, TEXTFMT, ##__VA_ARGS__); \
      gpfnExtIOCallbackPtr(-1, A, 0, acExtioMsg );        \
    }                                                     \
  } while (0)


// for compatibility with PROCITEC code ..
// translate log level macros and implement LOG_PRO() macro .. similar to SDRLOG

#define LOG_ERROR     extHw_MSG_ERROR
#define LOG_PROTOCOL  extHw_MSG_WARNING
#define LOG_NORMAL    extHw_MSG_LOG
#define LOG_DEBUG     extHw_MSG_DEBUG

#define LOG_PRO( A, TEXTFMT, ... )	\
  do {                              \
    if ( gpfnExtIOCallbackPtr && SDRsupportsLogging )     \
    {                                                     \
      snprintf(acExtioMsg, 1023, TEXTFMT, ##__VA_ARGS__); \
      gpfnExtIOCallbackPtr(-1, A, 0, acExtioMsg );        \
    }                                                     \
  } while (0)

