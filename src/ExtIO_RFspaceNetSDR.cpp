
#define HWNAME "RFspace"
#define HWMODEL "RFspace NetSDR"
#define SETTINGS_IDENTIFIER "RFspace NetSDR-0.1"

#define IGNORE_CONF_DATA_DEST 0

#if IGNORE_CONF_DATA_DEST
// for test with "dump_tcp_forwarding.exe -v 58666"
#define DATA_DEST_IP    "10.10.11.2"
#define DATA_DEST_PORT  50000
#else
#define DATA_DEST_IP    gacDataIP
#define DATA_DEST_PORT  guDataPortNo
#endif

#include "ExtIO_RFspaceNetSDR.h"
#include "rfspace_netsdr_receiver.h"

#include "common/base/baselib/base/ProLogging.h"
#include "foreign/extio/common/ExtIO_ProLogImpl.h"


//---------------------------------------------------------------------------

#include "foreign/tinythread/tinythread.h"

#include "common/base/baselib/base/TimeMeasuring.h"
#include "common/base/baselib/base/ProStd.h"
#include "common/base/baselib/base/ProVersion.h"
#include "common/base/baselib/base/ProMakros.h"

#if defined( WIN32 ) || defined( WIN64 )
// #define WIN32_LEAN_AND_MEAN             // Selten verwendete Teile der Windows-Header nicht einbinden.
#include <windows.h>
#endif

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>

#include <queue>
#include <tuple>
#include <string>
#include <vector>

#include <type_traits>
#include <typeinfo>

//---------------------------------------------------------------------------

#define LO_MIN 9000LL
#define LO_MAX 30000000LL

#define DATA_TIMEOUT_MSECS  5000

//---------------------------------------------------------------------------

#ifdef _MSC_VER
#pragma warning( disable : 4996 )
#define snprintf _snprintf
#endif


extern pfnExtIOCallback gpfnExtIOCallbackPtr;

static tthread::thread  * gpoReceiverControlThread = nullptr;
static RFspaceNetReceiver * gpoReceiver = nullptr;
static RFspaceNetReceiver::Settings * gpoSettings = nullptr;


/// all the settings
static int giRcvMGC = 0;

static bool gbHaveLastRcvFreqAndBW = false;
static int64_t giLastRcvFreq = 0;
static int32_t giLastRcvBW = 0;

static bool SDR_settings_valid    = false; // assume settings are for some other ExtIO


bool             gbInitHW        = false;
int              giAttIdx        = 0; // max atten
int              giAgcIdx        = 0;
int              giDefaultAgcIdx = 1; // Auto

volatile bool   gbDataStreamActivated = false;
volatile bool   gbBoundTcpDataPort    = false;

volatile bool   gbDataStreamTimeout   = false;

volatile bool   gbExitControlThread     = false;
volatile bool   gbControlThreadRunning  = false;


//---------------------------------------------------------------------------

static void ReceiverThreadProc( void* lpParameter )
{
  // @todo: move everything to gpoReceiver->ThreadProc() - including gbExitControlThread ...

  const uint16_t uiWaitMs = 5;
  while ( !gbExitControlThread )
  {
    tthread::this_thread::sleep_for( tthread::chrono::milliseconds( uiWaitMs ) );
    gpoReceiver->TimerProc(uiWaitMs);
  }
  gbExitControlThread    = false;
  gbControlThreadRunning = false;
  return;

  bool bProtocolOrTimeoutError = false;
  while ( !gbExitControlThread )
  {
    //if ( !gbBoundTcpDataPort )
    {
      // wait until TCP data port is bound, when using TCP protocol
      tthread::this_thread::sleep_for( tthread::chrono::milliseconds( 10 ) );
      gpoReceiver->TimerProc(uiWaitMs);
      continue;
    }
    if ( bProtocolOrTimeoutError )
    {
      // wait with reconnect .. until NetSDR might get ready!
      LOG_PRO( LOG_PROTOCOL, "ControlThreadProc(): waiting 10 seconds before reconnecting to RFspace NetSDR");
      tthread::this_thread::sleep_for( tthread::chrono::milliseconds( 10000 ) );
      bProtocolOrTimeoutError = false;
    }

    uint64_t lastStrmCheck = currentMSecsSinceEpoch();
    uint64_t lastRcvCheck = currentMSecsSinceEpoch();

    // pre-processing
    if ( true )
    {
      LOG_PRO( LOG_NORMAL, "ControlThreadProc(): established = OK" );

      while ( 1 )
      {
        LOG_PRO( LOG_NORMAL, "ControlThreadProc()" );

        LOG_PRO( LOG_NORMAL, "ControlThreadProc(): controlling to frq %.1f kHz with BW %.1f kHz with ATT %d dB"
                 , gpoSettings->iFrequency / 1000.0
                 , gpoSettings->uiBandwidth / 1000.0
                 , giRcvMGC
                 );
      }

      // restart
      gbDataStreamTimeout = false;

      break;
    }

    if ( !bProtocolOrTimeoutError )
    {
      LOG_PRO( LOG_NORMAL, "ControlThreadProc(): was STREAM ACTIVATED! Data reception can start. DataPort is %s"
                 , ( gbBoundTcpDataPort ? "bound" : "unbound" ) );
        gbDataStreamActivated = true;

      giLastRcvFreq = 0;  // oConn.getLastReceiverFreq();
      giLastRcvBW = 0;    // oConn.getLastReceiverBW();
      gbHaveLastRcvFreqAndBW = ( giLastRcvBW > 0 );
      if ( giLastRcvBW <= 0 )
      {
        LOG_PRO( LOG_PROTOCOL, "ControlThreadProc(): received ReceiverBandWidth <= 0!" );
      }
    }

    // processing
    {
      bool bExitProcLoop = bProtocolOrTimeoutError;
      while ( !gbExitControlThread && !bExitProcLoop )
      {
        uint64_t uiCurr = currentMSecsSinceEpoch();
        if ( ( uiCurr - lastStrmCheck ) >= 2000 )
        {
          //oConn.requestStreamState();
          lastStrmCheck = uiCurr;
        }
        if ( ( uiCurr - lastRcvCheck ) >= 5000 )
        {

          lastRcvCheck = uiCurr;
        }
        if ( gbDataStreamTimeout )
        {
          gbDataStreamTimeout = false;
          LOG_PRO( LOG_NORMAL, "ControlThreadProc(): STREAM TIMEOUT! Restarting .. " );
          lastStrmCheck = uiCurr;
        }
      }
    }

    // post-processing

    if ( gbExitControlThread )
      break;
    tthread::this_thread::sleep_for( tthread::chrono::milliseconds( 1000 ) );
  }

  gbExitControlThread    = false;
  gbControlThreadRunning = false;
}


//---------------------------------------------------------------------------

#if 0
static void ReceptionStreamThreadProc( void* lpParameter )
{
  (void)lpParameter;

  MonotonicClock oClock;

  uint64_t uTicksNew           = 0;
  uint64_t uLastReceivedPacket = oClock.msecs();

  while ( 1 )
  {
    if ( !gbDataStreamActivated )
    {
      // wait for control - before binding UDP port, when using UDP protocol
      tthread::this_thread::sleep_for( tthread::chrono::milliseconds( 1 ) );
      continue;
    }

    gbBoundTcpDataPort = true;
    while ( 1 )
    {
      uLastReceivedPacket = oClock.msecs();
    }
  }
}
#endif

//---------------------------------------------------------------------------

static void stopThread()
{
  // signal "stop" to reception & control threads
  if ( gbControlThreadRunning )
    gbExitControlThread = true;

  // stop
  {
    if ( gbControlThreadRunning )
    {
      gbExitControlThread = true;
      while ( gbControlThreadRunning )
      {
        tthread::this_thread::sleep_for( tthread::chrono::milliseconds( 20 ) );
      }
    }

    if ( gpoReceiverControlThread )
    {
      if ( gpoReceiverControlThread->joinable() )
        gpoReceiverControlThread->join();
      delete gpoReceiverControlThread;
    }
    gpoReceiverControlThread = nullptr;
  }
}

static void startThread()
{
  stopThread();

  // @todo: nur noch lokal im ReceiverThreadProc ?
  {
    gbDataStreamActivated = false;
    gbBoundTcpDataPort  = false;
    gbDataStreamTimeout = false;
  }

  gbExitControlThread    = false;
  gbControlThreadRunning = true;
  gpoReceiverControlThread  = new tthread::thread( ReceiverThreadProc, 0 );
}

//---------------------------------------------------------------------------

static const char acProductVersion[] = "ExtIO version " STRINGIZE(EXTIO_IFC_VER) "." STRINGIZE(EXTIO_VER_REV) " " SCM_TAGNAME " " SCM_DATE;

const char * EXTIO_CALL GetBuildInfo()
{
  return ProVersion();
}

const char * EXTIO_CALL GetExtIOVersion()
{
  return acProductVersion;
}

//---------------------------------------------------------------------------
bool EXTIO_CALL InitHW( char* name, char* model, int& type )
{
  type = RFspaceNetReceiver::getDefaultHWType();
  strcpy( name, HWNAME );
  strcpy( model, HWMODEL );

  if ( !gbInitHW )
  {
    // read default values from Settings: giRcvFreq, giAgcIdx, ..
    // do not overwrite them
    if (!gpoSettings)
      gpoSettings = new RFspaceNetReceiver::Settings();

    if ( gpoSettings->iAttenuationIdx < 0 || gpoSettings->iAttenuationIdx >= RFspaceNetReceiver::miNumAttenuations  )
      gpoSettings->iAttenuationIdx = 0;

    gbInitHW = true;
  }


  return gbInitHW;
}

//---------------------------------------------------------------------------
bool EXTIO_CALL OpenHW( void )
{
  LOG_PRO( LOG_DEBUG, "OpenHW() called" );

  bool bOpenOK = true;
  if ( gbInitHW )
  {
    if ( !gpoReceiver )
    {
      if (!gpoSettings)
        gpoSettings = new RFspaceNetReceiver::Settings();

      gpoReceiver = new RFspaceNetReceiver();
      bOpenOK = false;
      if ( gpoReceiver )
      {
        gpoReceiver->setExtIoCallback(gpfnExtIOCallbackPtr);
        stopThread();
        bOpenOK = gpoReceiver->openHW(gpoSettings);
        if (bOpenOK)
          startThread();
        else
        {
          delete gpoReceiver;
          gpoReceiver = nullptr;
        }
      }
    }
  }

  //EXTIO_STATUS_CHANGE( gpfnExtIOCallbackPtr, extHw_Changed_ATT );

  LOG_PRO( LOG_DEBUG, "OpenHW() returns gbInitHW && bOpenOK = %s && %s", (gbInitHW ? "true":"false"), (bOpenOK ? "true":"false") );

  // in the above statement, F->handle is the window handle of the panel displayed
  // by the DLL, if such a panel exists
  return gbInitHW && bOpenOK;
}

//---------------------------------------------------------------------------
int EXTIO_CALL StartHW( long LOfreq )
{
  int64_t ret = StartHW64( (int64_t)LOfreq );
  return (int)ret;
}

//---------------------------------------------------------------------------
int64_t EXTIO_CALL StartHW64( int64_t LOfreq )
{
  LOG_PRO( LOG_DEBUG, "StartHW64() called" );

  if ( !gbInitHW || !gpoReceiver )
    return 0;

  bool bStartOK = gpoReceiver->startHW(LOfreq);

  LOG_PRO( LOG_DEBUG, "StartHW64(): %s", (bStartOK ? "successful":"failed") );

  // number of complex elements returned each
  // invocation of the callback routine
  return RFspaceNetReceiver::EXT_BLOCKLEN;
}

//---------------------------------------------------------------------------
void EXTIO_CALL StopHW( void )
{
  LOG_PRO( LOG_DEBUG, "StopHW() called" );
  if ( !gbInitHW || !gpoReceiver )
    return;

  gpoReceiver->rcv.stopDataStream();
}

void EXTIO_CALL CloseHW( void )
{
  LOG_PRO( LOG_DEBUG, "CloseHW() called" );
  // ..... here you can shutdown your graphical interface, if any............
  if ( gbInitHW )
  {
    StopHW();
    stopThread();
    if ( gpoReceiver )
      delete gpoReceiver;
    gpoReceiver = nullptr;
  }
}

//---------------------------------------------------------------------------
int EXTIO_CALL SetHWLO( long LOfreq )
{
  int64_t ret = SetHWLO64( (int64_t)LOfreq );
  return ( ret & 0xFFFFFFFFULL );
}

int64_t EXTIO_CALL SetHWLO64( int64_t LOfreq )
{
  LOG_PRO( LOG_DEBUG, "************************************** SetHWLO(%d) *********************************", LOfreq);
  LOG_PRO( LOG_DEBUG, "SetHWLO64(%ld Hz) called", long(LOfreq) );
  if ( !gpoReceiver )
  {
    LOG_PRO( LOG_DEBUG, "SetHWLO64(): ERROR: No Receiver", LOfreq);
    return 1; // Error
  }
  gpoReceiver->setHWLO(LOfreq);
  return 0;
}

//---------------------------------------------------------------------------
int EXTIO_CALL GetStatus( void )
{
  return 0; // status not supported by this specific HW,
}

//---------------------------------------------------------------------------
void EXTIO_CALL SetCallback( pfnExtIOCallback funcptr )
{
  gpfnExtIOCallbackPtr = funcptr;
  if ( !gpoReceiver )
    return;
  gpoReceiver->setExtIoCallback(funcptr);
  LOG_PRO( LOG_DEBUG, "SetCallback(%p) called .. after having set callback function", funcptr);
}

//---------------------------------------------------------------------------
int64_t EXTIO_CALL GetHWLO64( void )
{
  if ( !gpoReceiver )
    return 0;

  uint64_t f = gpoReceiver->getHWLO();

  return f;

}

long EXTIO_CALL GetHWLO( void )
{
  int64_t f = GetHWLO64();
  if ( f > LONG_MAX )
    f = LONG_MAX;
  return (long)( f & 0xFFFFFFFFULL );
}
//---------------------------------------------------------------------------

long EXTIO_CALL GetHWSR( void )
{
  LOG_PRO( LOG_DEBUG, "GetHWSR() called");
  if ( !gpoReceiver )
    return 0;
  if (!gpoSettings)
    gpoSettings = new RFspaceNetReceiver::Settings();
  long srate = long(gpoSettings->uiSamplerate);
  LOG_PRO( LOG_DEBUG, "GetHWSR(): %ld Hz", srate );
  return long(srate);
}

int EXTIO_CALL ExtIoGetFreqRanges(int idx, int64_t * freq_low, int64_t * freq_high )
{
  LOG_PRO( LOG_DEBUG, "************************************** GET FREQRANGES( )*********************************");
  if ( !gpoReceiver )
    return 0;

  const int64_t * range = gpoReceiver->getFrequencyRanges( idx );

    if(range)
    {
      * freq_low = range[0];
      * freq_high = range[1];
      return 0;
    }
    else
      return 1; //error

}

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------

// following "Attenuator"s visible on "RF" button

int EXTIO_CALL GetAttenuators( int atten_idx, float* gain )
{
  if (!gpoSettings)
    gpoSettings = new RFspaceNetReceiver::Settings();

  // fill in attenuation (gain)
  // use positive attenuation levels if signal is amplified (LNA)
  // use negative attenuation levels if signal is attenuated
  // sort by attenuation: use idx 0 for highest attenuation / most damping
  // this functions is called with incrementing idx
  //    - until this functions return != 0 for no more attenuator setting
  if(!gpoSettings->bIsVUHFRange)
  {
    if ( atten_idx >= 0 && atten_idx < RFspaceNetReceiver::miNumAttenuations )
    {
      *gain = (RFspaceNetReceiver::mafAttenuationATTs[atten_idx] + RFspaceNetReceiver::mafAttenuationADGainsdB[atten_idx]) ;
      LOG_PRO( LOG_DEBUG, "************************************** GetAttenuators(%d) --> %.2f *********************************", atten_idx, *gain );
      return 0;
    }
    LOG_PRO( LOG_DEBUG, "************************************** GetAttenuators(%d) --> ERR *********************************", atten_idx );
  }
  else
  {
    if(!gpoSettings->bUseVUHFExpert)
    {
      if ( atten_idx >= 0 && atten_idx < RFspaceNetReceiver::miNumCompatibilityGains )
          {
            *gain = (RFspaceNetReceiver::mafVUHFCompatibilityGainValues[atten_idx] ) ;
            LOG_PRO( LOG_DEBUG, "************************************** GetAttenuators(%d) --> %.2f *********************************", atten_idx, *gain );
            return 0;
          }
          LOG_PRO( LOG_DEBUG, "************************************** GetAttenuators(%d) --> ERR *********************************", atten_idx );
    }
    else
    {
      if ( atten_idx >= 0 && atten_idx < RFspaceNetReceiver::miNumMultiGains )
      {
        *gain = (RFspaceNetReceiver::mafVUHFMultiGainValues[atten_idx] ) ;
        LOG_PRO( LOG_DEBUG, "************************************** GetAttenuators(%d) --> %.2f *********************************", atten_idx, *gain );
        return 0;
      }
      LOG_PRO( LOG_DEBUG, "************************************** GetAttenuators(%d) --> ERR *********************************", atten_idx );
    }
  }
  return 1;
}

int EXTIO_CALL GetActualAttIdx( void )
{

  if (!gpoSettings)
    gpoSettings = new RFspaceNetReceiver::Settings();

  int ret = RFspaceNetReceiver::getActualAttIdx(gpoSettings);
  LOG_PRO( LOG_DEBUG, "************************************** GetActualAttIdx() --> %d *********************************", ret );
  return ret;
}

int EXTIO_CALL SetAttenuator( int atten_idx )
{
  LOG_PRO( LOG_DEBUG, "************************************** SetAttenuator(%d)*********************************", atten_idx);
  if(!gpoSettings->bIsVUHFRange)
  {
    if ( atten_idx < 0 || atten_idx >= RFspaceNetReceiver::miNumAttenuations )
      atten_idx = 0;
  }
  else
  {
    if(!gpoSettings->bUseVUHFExpert)
    {
      if ( atten_idx < 0 || atten_idx >= RFspaceNetReceiver::miNumCompatibilityGains )
        atten_idx = 0;
    }
    else
    {
      if ( atten_idx < 0 || atten_idx >= RFspaceNetReceiver::miNumMultiGains )
        atten_idx = 0;
    }
  }


  if ( gpoReceiver )
  {
    gpoReceiver->setGain(atten_idx);
    return 0;
  }
  else
  {
    if (!gpoSettings)
      gpoSettings = new RFspaceNetReceiver::Settings();
    LOG_PRO( LOG_DEBUG, "SetAttenuator (%d) : Fallback ", atten_idx);
    if(!gpoSettings->bIsVUHFRange)
      gpoSettings->iAttenuationIdx = atten_idx;
    else
    {
      if(!gpoSettings->bUseVUHFExpert)
        gpoSettings->iCompatibilityValue = atten_idx;
      else
        gpoSettings->iControlValue = atten_idx;

    }
    return 0;
  }

  return 1; // ERROR
}


int EXTIO_CALL ExtIoGetSrates( int srate_idx, double* samplerate )
{

if( srate_idx >= 0 && srate_idx < RFspaceNetReceiver::miNumSamplerates)
    {
      *samplerate = gpoReceiver->maiSamplerates[srate_idx];
      return 0;
    }
  else
  {
    return 1; // ERROR
  }
}

int EXTIO_CALL ExtIoGetActualSrateIdx( void )
{
  if (!gpoSettings)
    gpoSettings = new RFspaceNetReceiver::Settings();

  int ret = gpoSettings->iSampleRateIdx;
  return ret;
}

int EXTIO_CALL ExtIoSetSrate( int srate_idx )
{
  LOG_PRO( LOG_DEBUG, "************************************** ExtIoSetSrate(%d)*********************************", srate_idx);
  if ( gpoReceiver )
  {
    gpoReceiver->setSamplerate(srate_idx);
    return 0;
  }
  else
  {
    if (!gpoSettings)
      gpoSettings = new RFspaceNetReceiver::Settings();
    LOG_PRO( LOG_DEBUG, "************************************** ExtIoSetSrate(): receiver does not exist. set settings *********************************");

    gpoSettings->iSampleRateIdx = srate_idx;
    return 1;
  }
}

long EXTIO_CALL ExtIoGetBandwidth( int srate_idx )
{
  long bandwidth;
  if( srate_idx >= 0 && srate_idx < RFspaceNetReceiver::miNumSamplerates)
  {
    bandwidth = long(gpoReceiver->maiBandwidths[srate_idx]);
    return bandwidth;
  }
  else
  {
    return -1L; // ERROR
  }
}
//---------------------------------------------------------------------------

enum class Setting {
      ID = 0      // enum values MUST be incremental without gaps!
    , CTRL_IP, CTRL_PORT    // NetSDR IP/Port
    , DATA_IP, DATA_PORT    // myPC Data IP/Port
    , SAMPLERATE_IDX
    , RCV_FRQ
    , RCV_BW
    , RCV_ATTEN_IDX
    , BITDEPTH_CHANGE_SMPRATE
    , BAND1_RANGE_MINFREQ
    , BAND1_RANGE_MAXFREQ
    , IS_VUHF_RANGE
    , COMPATIBILITY_VALUE
    , USE_VUHF_MULTIGAIN
    , GAIN_CONTROL_MODE
    , LNA_VALUE
    , MIXER_VALUE
    , IFOUTPUT_VALUE
    , NUM   // Last One == Amount
};



int EXTIO_CALL ExtIoGetSetting( int idx, char* description, char* value )
{
  if (!gpoSettings)
    gpoSettings = new RFspaceNetReceiver::Settings();

  switch( Setting(idx) )
  {
    case Setting::ID:
      snprintf( description, 1024, "%s", "Identifier" );
      snprintf( value, 1024, "%s", SETTINGS_IDENTIFIER );
      return 0;

    case Setting::CTRL_IP:
      snprintf( description, 1024, "%s", "CONTROL IP: IP-Address of 'RFspace NetSDR' for receiver control" );
      snprintf( value, 1024, "%s", gpoSettings->acCtrlIP );
      return 0;
    case Setting::CTRL_PORT:
      snprintf( description, 1024, "%s", "CONTROL PortNo: port number of 'RFspace NetSDR' for receiver control" );
      snprintf( value, 1024, "%d", (int)gpoSettings->uCtrlPortNo );
      return 0;

    case Setting::DATA_IP:
      snprintf( description, 1024, "%s", "DATA IP: IP-Address of Client PC to receive streaming data" );
      snprintf( value, 1024, "%s", gpoSettings->acDataIP );
      return 0;
    case Setting::DATA_PORT:
      snprintf( description, 1024, "%s", "DATA PortNo: port number of Client PC to receive streaming data (default: as CONTROL PortNo)" );
      snprintf( value, 1024, "%d", gpoSettings->uDataPortNo );
      return 0;

    case Setting::SAMPLERATE_IDX:
      snprintf( description, 1024, "%s", "SampleRateIdx" );
      snprintf( value, 1024, "%d", gpoSettings->iSampleRateIdx );
      return 0;

    case Setting::RCV_FRQ:
      snprintf( description, 1024, "%s", "frequency for Receiver" );
      snprintf( value, 1024, "%ld", gpoSettings->iFrequency );
      return 0;

    case Setting::RCV_BW:
      snprintf( description, 1024, "%s", "bandwidth in Hz for Receiver:  10 / 25 / 50 / 80 / 100 / 200 / 400 / 500 / 800 / 1300 / 1600 kHz" );
      snprintf( value, 1024, "%d", gpoSettings->uiBandwidth );
      return 0;

    case Setting::RCV_ATTEN_IDX:
      snprintf( description, 1024, "%s", "MGC AttenuationIdx (0..N) for Receiver" );
      snprintf( value, 1024, "%u", gpoSettings->iAttenuationIdx);
      return 0;

    case Setting::BITDEPTH_CHANGE_SMPRATE:
      snprintf( description, 1024, "%s", "Samplerate at which bitdepth is being changed from 24 to 16 bit and vice verca." );
      snprintf( value, 1024, "%u", gpoSettings->iBitDepthThresSamplerate);
      return 0;

    case Setting::BAND1_RANGE_MINFREQ:
      snprintf( description, 1024, "%s", "Receiver Frequency Range Min Frequency" );
      snprintf( value, 1024, "%lu", gpoSettings->iBand_minFreq);
      return 0;

    case Setting::BAND1_RANGE_MAXFREQ:
      snprintf( description, 1024, "%s", "Receiver Frequency Range Max Frequency" );
      snprintf( value, 1024, "%lu", gpoSettings->iBand_maxFreq);
      return 0;

    case Setting::IS_VUHF_RANGE:
      snprintf( description, 1024, "%s", "Is this reveiver in VUHG range ?" );
      snprintf( value, 1024, "%u", gpoSettings->bIsVUHFRange);
      return 0;

    case Setting::COMPATIBILITY_VALUE:
      snprintf( description, 1024, "%s", "VUHF Compatibility Gain Level 0 ... 3" );
      snprintf( value, 1024, "%u", gpoSettings->iCompatibilityValue);
      return 0;

    case Setting::USE_VUHF_MULTIGAIN:
       snprintf( description, 1024, "%s", "Usage of multi Downconverting Gains" );
       snprintf( value, 1024, "%u", gpoSettings->bUseVUHFExpert);
       return 0;

    case Setting::GAIN_CONTROL_MODE:
      snprintf( description, 1024, "%s", "Usage of AGC in VUHF Frequency Range" );
      snprintf( value, 1024, "%u", gpoSettings->bUseVUHFAutoMode);
      return 0;

    case Setting::LNA_VALUE:
      snprintf( description, 1024, "%s", "LNA Gain Level Value" );
      snprintf( value, 1024, "%u", gpoSettings->iLNAValue);
      return 0;

    case Setting::MIXER_VALUE:
      snprintf( description, 1024, "%s", "Mixer Gain Level Value" );
      snprintf( value, 1024, "%u", gpoSettings->iMixerValue);
      return 0;

    case Setting::IFOUTPUT_VALUE:
      snprintf( description, 1024, "%s", "IF Output Gain Level Value" );
      snprintf( value, 1024, "%u", gpoSettings->iIFOutputValue);
      return 0;
    case Setting::NUM:
      // no break
    default:
      return -1; // ERROR
  }
  return -1; // ERROR
}


void EXTIO_CALL ExtIoSetSetting( int idx, const char* value )
{
  if (!gpoSettings)
    gpoSettings = new RFspaceNetReceiver::Settings();

  int    tempInt;
  // now we know that there's no need to save our settings into some (.ini) file,
  // what won't be possible without admin rights!!!,
  // if the program (and ExtIO) is installed in C:\Program files\..
  if ( idx != 0 && !SDR_settings_valid )
  {
    if ( idx == 1 )
      LOG_PRO( LOG_ERROR, "ExtIoSetSetting(): ignoring settings cause of version mismatch!" );
    return; // ignore settings for some other ExtIO or different version
  }

  switch( Setting(idx) )
  {
    case Setting::ID:
      SDR_settings_valid = ( value && !strcmp( value, SETTINGS_IDENTIFIER ) );
      // make identifier version specific??? - or not ==> never change order of idx!
      break;
    case Setting::CTRL_IP:
      snprintf( gpoSettings->acCtrlIP, 64, "%s", value );
      break;
    case Setting::CTRL_PORT:
      tempInt = atoi( value );
      gpoSettings->uCtrlPortNo = (uint16_t)( tempInt & 0xffff );
      break;
    case Setting::DATA_IP:
      snprintf( gpoSettings->acDataIP, 64, "%s", value );
      break;
    case Setting::DATA_PORT:
      tempInt = atoi( value );
      gpoSettings->uDataPortNo = (uint16_t)( tempInt & 0xffff );
      break;
    case Setting::SAMPLERATE_IDX:
      gpoSettings->iSampleRateIdx = atoi( value );
      break;
    case Setting::RCV_FRQ:
      sscanf( value, "%ld", &gpoSettings->iFrequency );
      break;
    case Setting::RCV_BW:
      gpoSettings->uiBandwidth = atoi( value );
      break;
    case Setting::RCV_ATTEN_IDX:
      gpoSettings->iAttenuationIdx = atoi( value );
      break;
    case Setting::BITDEPTH_CHANGE_SMPRATE:
      gpoSettings->iBitDepthThresSamplerate = atoi( value );
      break;
    case Setting::BAND1_RANGE_MINFREQ:
       gpoSettings->iBand_minFreq = atoi( value );
       break;
    case Setting::BAND1_RANGE_MAXFREQ:
       gpoSettings->iBand_maxFreq = atoi( value );
       break;
    case Setting::IS_VUHF_RANGE:
       gpoSettings->bIsVUHFRange = atoi( value ) ? true : false;
       break;
    case Setting::COMPATIBILITY_VALUE:
       gpoSettings->iCompatibilityValue = atoi( value );
       break;
    case Setting::USE_VUHF_MULTIGAIN:
       gpoSettings->bUseVUHFExpert = atoi( value ) ? true : false;
       break;
    case Setting::GAIN_CONTROL_MODE:
      snprintf( gpoSettings->acGainControlMode, 63, "%s", value );
      break;
/*       gpoSettings->bUseVUHFAutoMode = bool(atoi( value ));
       break;*/
    case Setting::LNA_VALUE:
      gpoSettings->iLNAValue = atoi( value );
       break;
    case Setting::MIXER_VALUE:
      gpoSettings->iMixerValue = atoi( value );
       break;
    case Setting::IFOUTPUT_VALUE:
      gpoSettings->iIFOutputValue = atoi( value );
       break;
    case Setting::NUM:
      // no break
    default:
      break;
  }
}


void EXTIO_CALL ExtIoSDRInfo( int extSDRInfo, int additionalValue, void * additionalPtr )
{
  (void)additionalValue;
  (void)additionalPtr;

  switch ( (extSDR_InfoT)extSDRInfo )
  {
  case extSDR_NoInfo:
  case extSDR_supports_Settings:
  case extSDR_supports_Atten:
  case extSDR_supports_TX:
  case extSDR_controls_BP:
  case extSDR_supports_AGC:
  case extSDR_supports_MGC:
  case extSDR_supports_PCMU8:
  case extSDR_supports_PCMS8:
  case extSDR_supports_PCM32:
  case extSDR_supports_SampleFormats:
    break;
  case extSDR_supports_Logging:
    gbUseProcitecEnums = false;
    break;
  default: ;
  }
}

//---------------------------------------------------------------------------

