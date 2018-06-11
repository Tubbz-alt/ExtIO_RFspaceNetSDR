#pragma once

// missing definitions, macros and functions from following include files
//#include "common/base/baselib/base/ProLogging.h"
//#include "common/base/baselib/base/ByteOrder.h"
//#include "common/base/baselib/base/ProMakros.h"
//#include "common/base/baselib/base/ProStd.h"

// hacked down all missing things
//  - strictly assuming little endian platform!

#include <stdint.h>
#include <assert.h>

#define NUMEL( ARR )  ( sizeof(ARR) / sizeof(ARR[0]) )
#define ProAssert( COND )   assert( COND )
#define PROASSERT( COND )   assert( COND )

#define STRINGIZE(s) STRINGIZEL(s)
#define STRINGIZEL(s) #s

#define EXTIO_IFC_VER 2018
#define EXTIO_VER_REV 3
#define SCM_TAGNAME ""
#define SCM_DATE    ""

#define LITTLE_INT16_T  int16_t


#pragma pack(push, 1) // exact fit - no padding
struct LITTLE_INT24_T
{
  uint8_t lo;
  uint8_t mi;
  int8_t  hi;
};
#pragma pack(pop) //back to whatever the previous packing mode was


extern uint64_t currentMSecsSinceEpoch();

static inline
const char * ProVersion()
{
  return "?";
}

static inline
void WRITE_LITTLE_INT8(uint8_t v, void * vp)
{
  uint8_t * tp = (uint8_t *)vp;
  *tp = v;
}

static inline
void WRITE_BIG_INT8(uint8_t v, void * vp)
{
  uint8_t * tp = (uint8_t *)vp;
  *tp = v;
}

static inline
void WRITE_LITTLE_INT16(uint16_t v, void * vp)
{
  uint16_t * tp = (uint16_t *)vp;
  *tp = v;
}

static inline
void WRITE_BIG_INT32(uint32_t v, void * vp)
{
  uint32_t * tp = (uint32_t *)vp;
  uint32_t flipped = ((v & 0xFFU) << 24)
    | ((v & 0xFF00U) << 8)
    | ((v & 0xFF0000U) >> 8)
    | (((v & 0xFF000000U) >> 24) & 0xFFU);
  *tp = flipped;
}

static inline
void WRITE_LITTLE_INT32(uint32_t v, void * vp)
{
  uint32_t * tp = (uint32_t *)vp;
  *tp = v;
}

static inline
void WRITE_LITTLE_INT64(uint64_t v, void * vp)
{
  uint64_t * tp = (uint64_t *)vp;
  *tp = v;
}


static inline
uint8_t READ_LITTLE_INT8( const void * vp )
{
  const uint8_t * tp = (const uint8_t *)vp;
  return *tp;
}

static inline
uint8_t READ_BIG_INT8(const void * vp)
{
  const uint8_t * tp = (const uint8_t *)vp;
  return *tp;
}

static inline
uint16_t READ_LITTLE_INT16(const void * vp)
{
  const uint16_t * tp = (const uint16_t *)vp;
  return *tp;
}

static inline
uint32_t READ_LITTLE_INT32(const void * vp)
{
  const uint32_t * tp = (const uint32_t *)vp;
  return *tp;
}

static inline
uint64_t READ_LITTLE_INT64(const void * vp)
{
  const uint64_t * tp = (const uint64_t *)vp;
  return *tp;
}

static inline
uint32_t READ_BIG_INT32(const void * vp)
{
  const uint32_t * tp = (const uint32_t *)vp;
  const uint32_t v = *tp;
  uint32_t flipped = ((v & 0xFFU) << 24)
    | ((v & 0xFF00U) << 8)
    | ((v & 0xFF0000U) >> 8)
    | (((v & 0xFF000000U) >> 24) & 0xFFU);
  return flipped;
}


template <class T>
T PROCLIP(T val, T minVal, T maxVal)
{
  if (val < minVal)
    return minVal;
  else if (val > maxVal)
    return maxVal;
  else
    return val;
}

inline float PRO_INTN_TO_FLOAT(int16_t v, int)
{
  return v * (1.0F / 32768.0F);
}

inline float PRO_INTN_TO_FLOAT(const LITTLE_INT24_T & r, int)
{
  const void *vp = &r.lo - 1;
  const int32_t *p = (const int32_t *)vp;
  int32_t v = (*p) & 0xFFFFFF00;
  return v * (1.0F / 2147483648.0F);
}

