#ifndef EXTIO_RFSPACE_NETSDR_H
#define EXTIO_RFSPACE_NETSDR_H

// Define extern "C" only if this file is included in C++ code.

#ifdef __cplusplus
  #define C_FUNC extern "C"
#endif

//
// WINDOWS
//
#if defined(WIN64) || defined(_WIN64) || defined(__WIN64__) || defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)

  #define EXTIO_CALL     __stdcall

  #ifdef EXTIO_EXPORTS
    // dllexport is done by .def file, which is necessary for __stdcall with non-decorated function names!
    //#define EXTIO_API C_FUNC __declspec(dllexport)
    #define EXTIO_API C_FUNC
  #else
    #define EXTIO_API C_FUNC __declspec(dllimport)
  #endif

//
// LINUX
//
#elif defined(__linux) || defined(__linux__) || defined(linux) || defined(__CYGWIN__)  || defined(__CYGWIN)

  #define EXTIO_CALL
  #define EXTIO_API C_FUNC __attribute__ ((visibility ("default")))

//
// Unknown platform
//
#else

  /*! \brief Calling convention that is used for the interface functions.
   *
   *  This defines the calling convention that is used by the shared library interface.
   *  On \c WINDOWS platform <tt>__stdcall</tt> is used, while on LINUX platforms this define
   *  is left empty in order to use default callig convention. For gcc this is
   *  <tt>__attribute__((cdecl))</tt>.
   */
  #define EXTIO_CALL     __stdcall

  /*! \brief Ensure C-style linkage
   *
   *  This define ensures external C-style linkage for the interface functions. This
   *  prevents name mangling on the function declarations during linkage.
   */
  #define EXTIO_API extern "C"

#endif



#include "common/functions/receiverlib/LC_ExtIO_Types.h"

EXTIO_API const char * EXTIO_CALL GetBuildInfo();
EXTIO_API const char * EXTIO_CALL GetExtIOVersion();
EXTIO_API bool EXTIO_CALL InitHW( char* name, char* model, int& type );
EXTIO_API int64_t EXTIO_CALL StartHW64( int64_t freq );
EXTIO_API bool EXTIO_CALL OpenHW( void );
EXTIO_API int EXTIO_CALL StartHW( long freq );
EXTIO_API void EXTIO_CALL StopHW( void );
EXTIO_API void EXTIO_CALL CloseHW( void );
EXTIO_API int EXTIO_CALL SetHWLO( long LOfreq );
EXTIO_API int64_t EXTIO_CALL SetHWLO64( int64_t LOfreq );
EXTIO_API int EXTIO_CALL GetStatus( void );
EXTIO_API void EXTIO_CALL SetCallback( pfnExtIOCallback funcptr );
// void extIOCallback(int cnt, int status, float IQoffs, short IQdata[]);

EXTIO_API long EXTIO_CALL GetHWLO( void );
EXTIO_API int64_t EXTIO_CALL GetHWLO64( void );
EXTIO_API long EXTIO_CALL GetHWSR( void );

EXTIO_API int EXTIO_CALL ExtIoGetFreqRanges(int idx, int64_t * freq_low, int64_t * freq_high );

// EXTIO_API long EXTIO_CALL GetTune(void);
// EXTIO_API void EXTIO_CALL GetFilters(int& loCut, int& hiCut, int& pitch);
// EXTIO_API char EXTIO_CALL GetMode(void);
// EXTIO_API void EXTIO_CALL ModeChanged(char mode);
// EXTIO_API void EXTIO_CALL IFLimitsChanged(long low, long high);
// EXTIO_API void EXTIO_CALL TuneChanged(long freq);

// EXTIO_API void    EXTIO_CALL TuneChanged64(int64_t freq);
// EXTIO_API int64_t EXTIO_CALL GetTune64(void);
// EXTIO_API void    EXTIO_CALL IFLimitsChanged64(int64_t low, int64_t high);

// EXTIO_API void EXTIO_CALL RawDataReady(long samprate, int *Ldata, int *Rdata, int numsamples);

EXTIO_API void EXTIO_CALL VersionInfo( const char* progname, int ver_major, int ver_minor );

EXTIO_API int EXTIO_CALL GetAttenuators( int idx, float* attenuation ); // fill in attenuation
                                                              // use positive attenuation levels if signal is amplified (LNA)
                                                              // use negative attenuation levels if signal is attenuated
                                                              // sort by attenuation: use idx 0 for highest attenuation / most damping
                                                              // this functions is called with incrementing idx
                                                              //    - until this functions return != 0 for no more attenuator setting
EXTIO_API int EXTIO_CALL GetActualAttIdx( void );                       // returns -1 on error
EXTIO_API int EXTIO_CALL SetAttenuator( int idx );                      // returns != 0 on error

//EXTIO_API int EXTIO_CALL ExtIoGetAGCs( int agc_idx, char* text );
//EXTIO_API int EXTIO_CALL ExtIoGetActualAGCidx( void );
//EXTIO_API int EXTIO_CALL ExtIoSetAGC( int agc_idx );
//EXTIO_API int EXTIO_CALL ExtIoShowMGC( int agc_idx );

//EXTIO_API int EXTIO_CALL ExtIoGetMGCs( int mgc_idx, float* gain );
//EXTIO_API int EXTIO_CALL ExtIoGetActualMgcIdx( void );
//EXTIO_API int EXTIO_CALL ExtIoSetMGC( int mgc_idx );

EXTIO_API int EXTIO_CALL ExtIoGetSrates( int idx, double* samplerate ); // fill in possible samplerates
EXTIO_API int EXTIO_CALL ExtIoGetActualSrateIdx( void );                // returns -1 on error
EXTIO_API int EXTIO_CALL ExtIoSetSrate( int idx );                      // returns != 0 on error
EXTIO_API long EXTIO_CALL ExtIoGetBandwidth( int srate_idx );           // returns != 0 on error

EXTIO_API int EXTIO_CALL ExtIoGetSetting( int idx, char* description, char* value ); // will be called (at least) before exiting application
EXTIO_API void EXTIO_CALL ExtIoSetSetting( int idx, const char* value );             // before calling InitHW() !!!

EXTIO_API void EXTIO_CALL ExtIoSDRInfo( int extSDRInfo, int additionalValue, void * additionalPtr );

EXTIO_API int  EXTIO_CALL MetaInfoByteOffset( int * piOutMetaHeaderVersion );
EXTIO_API void EXTIO_CALL PROCITECgetZeroDBFSInfo( float *dBm, float * currentHeadroom_in_dB, float * hdrMaxLevel_in_dBm, int * numOvershoots );

#endif
