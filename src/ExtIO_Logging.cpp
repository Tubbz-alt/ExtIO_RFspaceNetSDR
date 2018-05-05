
#include "ExtIO_Logging.h"

// error message, with "const char*" in IQdata,
//   intended for a log file  AND  a message box
// for messages extHw_MSG_*

pfnExtIOCallback gpfnExtIOCallbackPtr = nullptr;

char acExtioMsg[1024] = { 0 };
bool SDRsupportsLogging = false;

