
#include "procitec_replacements.h"

#ifdef WIN32
  #ifndef _WINSOCK2API_
  #define _WINSOCK2API_
  #define _WINSOCKAPI_
  #endif
  #include <Windows.h>

uint64_t currentMSecsSinceEpoch()
{
  // mostly from http://www.frenk.com/2009/12/convert-filetime-to-unix-timestamp/

  SYSTEMTIME st;
  GetSystemTime(&st);
  FILETIME ft;
  SystemTimeToFileTime(&st, &ft);

  LARGE_INTEGER date;
  date.HighPart = ft.dwHighDateTime;
  date.LowPart = ft.dwLowDateTime;

  // Between Jan 1, 1601 and Jan 1, 1970 there are 11644473600 seconds
  // 100-nanoseconds = milliseconds * 10000
  LARGE_INTEGER adjust;
  adjust.QuadPart = 11644473600000LL * 10000LL;

  // removes the diff between 1970 and 1601
  date.QuadPart -= adjust.QuadPart;

  // converts back from 100-nanoseconds to milliseconds
  uint64_t ret = uint64_t( date.QuadPart / 10000 );
  return ret;
}

#else

#include <sys/time.h>

uint64_t currentMSecsSinceEpoch()
{
    struct timeval te;
    gettimeofday(&te, 0); // get current time
    uint64_t milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    // printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}

#endif
