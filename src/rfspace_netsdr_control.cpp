
#include "rfspace_netsdr_control.h"

#include "ExtIO_Logging.h"

#include "SimpleSocket.h"

#include "procitec_replacements.h"

#include <string.h>

#define LENGTH_MASK 0x1FFF     // mask for message length
#define TYPE_MASK 0xE000       // mask for upper byte of header
#define PRINT_RECEIVED_DATA   0

////////////////////////////////////////////////////////////////////////////////////////////////////////////

RFspaceNetSDRControl::Options::Options()
{
  clear();
}

void RFspaceNetSDRControl::Options::clear()
{
  Sound_Enabled = false;
  ReflockBoard_Present = false;
  DownConverterBoard_Present = false;
  UpConverterBoard_Present = false;
  X2Board_Present = false;
  MainBoardVariant = 0;
  ReflockBoardVariant = 0;
  DownConverterBoardVariant = 0;
  UpConverterBoardVariant = 0;
}

//**************************************************************

RFspaceNetSDRControl::RcvState::RcvState()
{
  clear();
}

void RFspaceNetSDRControl::RcvState::clear()
{
  eADCstate = ADCstate::SET_IQ_BASEBAND_DATA;
  eUDPstate = UDPstate::STOP_UDP_DATA;
  eBitDepth = BitDepthState::SET_16BIT_MODE;
  eCaptureMode = CaptureMode::SET_CONTIGUOUS_MODE;
  uFIFOByteCaptNum = 0;
}

//**************************************************************

RFspaceNetSDRControl::RcvFrequencies::RcvFrequencies()
{
  clear();
}


void RFspaceNetSDRControl::RcvFrequencies::clear()
{
  Chn1_Freq = -1;
  Chn2_Freq = -1;

  Chn1_Bnd1_MinFreq = -1;
  Chn1_Bnd1_MaxFreq = -1;
  Chn1_Bnd1_VCO_DwnConvFreq = -1;
  Chn1_Bnd2_MinFreq = -1;
  Chn1_Bnd2_MaxFreq = -1;
  Chn1_Bnd2_VCO_DwnConvFreq = -1;

  Chn2_Bnd1_MinFreq = -1;
  Chn2_Bnd1_MaxFreq = -1;
  Chn2_Bnd1_VCO_DwnConvFreq = -1;
  Chn2_Bnd2_MinFreq = -1;
  Chn2_Bnd2_MaxFreq = -1;
  Chn2_Bnd2_VCO_DwnConvFreq = -1;
}

//**************************************************************

RFspaceNetSDRControl::RFGains::RFGains()
{
  clear();
}

void RFspaceNetSDRControl::RFGains::clear()
{
  eChn1RFGain = RfGain::ATT_0DB;
  eChn2RFGain = RfGain::ATT_0DB;
}

//**************************************************************

RFspaceNetSDRControl::RFFilterSelection::RFFilterSelection()
{
  clear();
}

void RFspaceNetSDRControl::RFFilterSelection::clear()
{
  eChn1FiltSel = RfFilterSel::F_AUTO;
  eChn2FiltSel = RfFilterSel::F_AUTO;
}

//**************************************************************

RFspaceNetSDRControl::ADModes::ADModes()
{
  clear();
}

void RFspaceNetSDRControl::ADModes::clear()
{
  eChn1AD_Dither = ADDither::DITH_OFF;
  eChn2AD_Dither = ADDither::DITH_OFF;
  eChn1AD_Gain = ADGain::ADGain_1;
  eChn2AD_Gain = ADGain::ADGain_1;
}

//**************************************************************

RFspaceNetSDRControl::CWStartup::CWStartup()
{
  clear();
}

void RFspaceNetSDRControl::CWStartup::clear()
{
  cw_wpm = uint8_t(WpmMinMax::WPM_MIN);
  eCwFreq = CWFreq::CW_1000Hz;
  for( uint8_t i = 0; i < 10; i++)
  {
    asciiMessage[i] = 0;
  }

}

//**************************************************************

RFspaceNetSDRControl::VUHFGains::VUHFGains()
{
  clear();
}

void RFspaceNetSDRControl::VUHFGains::clear()
{
  isAGCMode = false;
  LNAGainLevel = 0;
  MixerGainLevel = 0;
  IFOutputGainLevel = 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


RFspaceNetSDRControl::RFspaceNetSDRControl(RFspaceNetSDRControl::CallbackIfc * pCB ):

    mpSocket( new CSimpleSocket( CSimpleSocket::SocketTypeTcp ) ),
    mSocket( *mpSocket ),
    mpCallBack( pCB )
{
  resetReceiverData();

  mSocket.Initialize();
}


RFspaceNetSDRControl::~RFspaceNetSDRControl()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  close();

  delete mpSocket;
}

void RFspaceNetSDRControl::resetReceiverData()
{
  mHasNAKMessage = false;

  mTargetName[0] = 0;
  mHasTargetName = false;

  mSerialNumber[0] = 0;
  mHasSerialNumber = false;

  mReceiverStatus = 0;
  mHasReceiverStatus = false;

  mProductId = 0;
  mHasProductId = false;

  mOptions.clear();
  mHasOptions = false;

  mRcvState.clear();
  mHasRcvState = false;

  mRcvFrequencies.clear();
  mHasRcvFrequency = false;

  mRcvFrequencyRanges[0] = 0;
  mHasRcvFrequencyRanges = false;

  mInterfaceVersion = 0;
  mHasInterfaceVersion = false;

  mHardwareFirmwareVersion = 0;
  mHasHardwareFirmwareVersion = false;

  mRcvChnSetup = RFspaceNetSDRControl::RcvChannelSetup::SCM_CTRLCH1;
  mHasRcvChnSetup = false;

  mRcvADAmplScale = 0;
  mHasRcvADAmplScale = false;

  mRFGains.clear();
  mHasRFGains = false;

  mRFFilterSelection.clear();
  mHasRFFilterSelection = false;

  mADModes.clear();
  mHasADModes = false;

  mIQOutSmpRate = RFspaceNetSDRControl::IQOutSmpRate::SR_500kHz;
  mHasIQOutSmpRate = false;

  mUDPPacketSize = UDPPacketSize::LARGE;
  mHasUDPPacketSize = false;

  mUDPIpAddress[0] = 0;
  mUDPPortNum = 0;
  mHasIPandPortNum = false;

  mCwStartup.clear();
  mHasCWStartup = false;

  for(uint8_t i=0; i<10; i++)
  {
    mHexArray[i] = 0;

    if(i<=4)
      mRcvFrequencyRanges[i] = 0;
  }


}



bool RFspaceNetSDRControl::connect( const char * ip, unsigned portNo )
{

  if ( mSocket.IsSocketPeerOpen() )
  {
    LOG_PRO( LOG_ERROR, "ERROR: Socket already connected !");
    ProAssert(0); // close your socket before connecting!
    return false;
  }

  // initialize reception state
  mRxLen = 0;
  mRxMsgLen = 2;
  mRxType = -1;
  mRxMsgHeader = true;

  resetReceiverData();

  bool bConnected = mSocket.Open( ip, portNo );
  if ( bConnected )
  {

    mSocket.SetNonblocking();

    //request information about General Control at initial connection
    requestOptions();
    requestTargetName();
    requestTargetSerialNum();
    requestInterfaceVersion();
    requestHwFwVersions(RFspaceNetSDRControl::HwFw::BOOT_CODE);
    requestProductId();
    requestStatus();

  }

  return bConnected;
}


bool RFspaceNetSDRControl::close()
{

  bool ret = false;

  resetReceiverData();

  if ( mSocket.IsSocketValid() )
    return mSocket.Shutdown(CSimpleSocket::CShutdownMode::Both);

  return ret;


}

bool RFspaceNetSDRControl::connected() const
{
  bool bConnected = ( mSocket.IsSocketValid() && mSocket.IsSocketPeerOpen() );
  return bConnected;
}


void RFspaceNetSDRControl::requestTargetName()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 4;
  char msgType = char(RFspaceNetSDRControl::MsgType::REQ_CTRL_ITEM);  //zum Test mit "msgType"
  const char acBuf[len] = { 4, msgType, 1, 0 };

  mSocket.Send(acBuf, len);

}

void RFspaceNetSDRControl::requestTargetSerialNum()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 4;
  const char acBuf[len] = { 4, 0x20, 2, 0 };

  mSocket.Send(acBuf, len);
}


void RFspaceNetSDRControl::requestInterfaceVersion()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 4;
  const char acBuf[len] = { 4, 0x20, 3, 0 };

  mSocket.Send(acBuf, len);
}


void RFspaceNetSDRControl::requestHwFwVersions(HwFw id)
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 5;
  char acBuf[len] = { 5, 0x20, 4, 0, 0 };

  WRITE_LITTLE_INT8( uint8_t(id) , &acBuf[4] );

  mSocket.Send(acBuf, len);
}


void RFspaceNetSDRControl::requestStatus()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 4;
  const char acBuf[len] = { 4, 0x20, 5, 0 };

  mSocket.Send(acBuf, len);
}


void RFspaceNetSDRControl::requestProductId()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 4;
  const char acBuf[len] = { 4, 0x20, 9, 0 };

  mSocket.Send(acBuf, len);
}


void RFspaceNetSDRControl::requestOptions()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 4;
  const char acBuf[len] = { 4, 0x20, 0x0A, 0 };

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::requestRcvFrequency()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 5;
  char acBuf[len] = { 5, char(msgType), 0x20, 0, 0};

  WRITE_LITTLE_INT8( channel , &acBuf[4] );

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::requestRcvFrequencyRanges()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 5;
  char acBuf[len] = { 5, char(MsgType::REQ_CTRL_RANGE), 0x20, 0, 0};

  WRITE_LITTLE_INT8( channel , &acBuf[4] );

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::requestRcvADAmplScale()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 5;
  char acBuf[len] = { 5, char(msgType), 0x23, 0, 0};

  WRITE_LITTLE_INT8( channel , &acBuf[4] );

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::requestRFGain()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 5;
  char acBuf[len] = { 5, char(msgType), 0x38, 0, 0};

  WRITE_LITTLE_INT8( channel , &acBuf[4] );

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::requestVUHFGains()
{
  if ( mSocket.IsSocketInvalid() )
      return;

    const unsigned int len = 4;
    const char acBuf[len] = { 4, char(msgType), 0x3A, 0 };

    mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::requestRFFilterSelection()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 5;
  char acBuf[len] = { 5, char(msgType), 0x44, 0, 0};

  WRITE_LITTLE_INT8( channel , &acBuf[4] );

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::requestADMode()
{
  if ( mSocket.IsSocketInvalid() )
    return;


  const unsigned int len = 5;
  uint8_t acBuf[len] = { 5, uint8_t(msgType), 0x8A, 0, 0 };

  WRITE_LITTLE_INT8( channel , &acBuf[4] );

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::requestUDPInterface()
{
  if ( mSocket.IsSocketInvalid() )
    return;


  const unsigned int len = 5;
  uint8_t acBuf[len] = { 5, uint8_t(msgType), 0xC5, 0, 0 };

  WRITE_LITTLE_INT8( channel , &acBuf[4] );

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::requestReceiverState()
{
  {
    if ( mSocket.IsSocketInvalid() )
      return;


    const unsigned int len = 5;
    const uint8_t acBuf[len] = { 5, uint8_t(msgType), 0x18, 0, 0 };

    mSocket.Send(acBuf, len);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//************************************ GET FUNCTIONS ********************************************

const char * RFspaceNetSDRControl::getTargetName(bool *pbOk) const
{

  if(pbOk)
    *pbOk = mHasTargetName;

  return mTargetName;

}

const char * RFspaceNetSDRControl::getSerialNumber(bool *pbOk) const
{

  if(pbOk)
    *pbOk = mHasSerialNumber;

  return mSerialNumber;

}

const RFspaceNetSDRControl::Options & RFspaceNetSDRControl::getOptions(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasOptions;

  return mOptions;
}

const int64_t RFspaceNetSDRControl::getRcvFrequency(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasRcvFrequency;

  return int64_t(mRcvFrequencies.Chn1_Freq);

}

const int64_t * RFspaceNetSDRControl::getRcvFrequencyRanges(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasRcvFrequencyRanges;

  return mRcvFrequencyRanges;
}

const RFspaceNetSDRControl::RfGain RFspaceNetSDRControl::getRFGain(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasRFGains;

  return mRFGains.eChn1RFGain;
}

const RFspaceNetSDRControl::VUHFGains & RFspaceNetSDRControl::getVHFUHFGain(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasVUHFGains;

  return mVUHFGains;
}

const uint8_t RFspaceNetSDRControl::getRFFilterSelection(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasRFFilterSelection;

  return uint8_t(mRFFilterSelection.eChn1FiltSel);
}

const char * RFspaceNetSDRControl::getUDPIpAddress(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasIPandPortNum;

  return mUDPIpAddress;
}

const unsigned RFspaceNetSDRControl::getUDPPortNum(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasIPandPortNum;

  return mUDPPortNum;
}


const RFspaceNetSDRControl::CWStartup & RFspaceNetSDRControl::getCWStartup(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasCWStartup;

  return mCwStartup;
}


const RFspaceNetSDRControl::IQOutSmpRate RFspaceNetSDRControl::getIQOutSmpRate(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasIQOutSmpRate;

  return mIQOutSmpRate;
}

const RFspaceNetSDRControl::UDPPacketSize RFspaceNetSDRControl::getUDPPacketSize(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasUDPPacketSize;

  return mUDPPacketSize;
}

const bool RFspaceNetSDRControl::getDithering(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasADModes;

  return  ( mADModes.eChn1AD_Dither == ADDither::DITH_ON );
}

const float RFspaceNetSDRControl::getADGain(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasADModes;

  return (mADModes.eChn1AD_Gain==ADGain::ADGain_1 ? 1.0F : 1.5F );
}

const uint8_t RFspaceNetSDRControl::getRcvChannelSetup(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasRcvChnSetup;

  return uint8_t(mRcvChnSetup);
}

const float RFspaceNetSDRControl::getRcvADAmplScale(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasRcvADAmplScale;

  return mRcvADAmplScale;

}

const uint32_t & RFspaceNetSDRControl::getRcvStatus(bool *pbOk) const
{

  if(pbOk)
    *pbOk = mHasReceiverStatus;

  return mReceiverStatus;

}

const uint32_t RFspaceNetSDRControl::getHWFWNumber(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasHardwareFirmwareVersion;

  return mHardwareFirmwareVersion;
}

const uint32_t RFspaceNetSDRControl::getProductId(bool *pbOk) const
{

  if(pbOk)
    *pbOk = mHasProductId;

  return mProductId;

}

const bool RFspaceNetSDRControl::isUDPDataStreamRunning(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasRcvState;

  return (mRcvState.eUDPstate == UDPstate::START_UDP_DATA ? true : false);
}

const int RFspaceNetSDRControl::getStreamBitDepth(bool *pbOk) const
{
  if(pbOk)
    *pbOk = mHasRcvState;

  switch(mRcvState.eBitDepth)
  {
    case BitDepthState::SET_16BIT_MODE: return 16;
    case BitDepthState::SET_24BIT_MODE: return 24;
    default : return 0;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////
//************************************ SET FUNCTIONS ********************************************

void RFspaceNetSDRControl::setRcvState(  UDPstate eStart_stop_spec, ADCstate eIq_ad_spec, BitDepthState eBit_mode_spec, CaptureMode eCapt_mode_spec, uint32_t eFifo_captNum  )
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const uint32_t len = 8;
  unsigned char acBuf[len] = { 8, 0, 0x18, 0, 0, 0, 0, 0 };
  unsigned int channelMode = 0;


  channelMode = uint32_t(eStart_stop_spec) | uint32_t(eIq_ad_spec) | uint32_t(eBit_mode_spec) | uint32_t(eCapt_mode_spec) | uint32_t(eFifo_captNum);

  WRITE_BIG_INT32( channelMode , &acBuf[4] ); //write channel mode to last 4 bytes of buffer


  mSocket.Send(acBuf, len);
  requestReceiverState();
}


void RFspaceNetSDRControl::setRcvChnSetup( )
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 5;
  unsigned char acBuf[len] = { 5, 0, 0x19, 0, 0 };

  void * wp = &acBuf[len-1];
  WRITE_LITTLE_INT32( uint8_t(chnMode) , wp ); //write channel mode to last byte of buffer

  mSocket.Send(acBuf, len);
}


void RFspaceNetSDRControl::setRcvFreq( int64_t rcvFreqHz)
{
  if ( mSocket.IsSocketInvalid() )
    return;


  const unsigned int len = 10;
  unsigned char acBuf[len] = { 0x0A, 0, 0x20, 0, 0, 0, 0, 0, 0, 0 };
  void * wp = &acBuf[4];
  WRITE_LITTLE_INT8( channel , wp ); //write channel information to byte #5
  wp = &acBuf[len-5];
  WRITE_LITTLE_INT64( rcvFreqHz , wp ); //write frequency to last 5 bytes of buffer

  mSocket.Send(acBuf, len);
  requestRcvFrequency();
  requestRcvFrequencyRanges();
}


void RFspaceNetSDRControl::setRcvADAmplScale( float rcvScale )
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 7;
  unsigned char acBuf[len] = { 7, 0, 0x23, 0, 0, 0, 0 };
  rcvScale = rcvScale * 65536;                     //convert decimal value to hex --> 0.5 * 2^16 = 32768 --> 0x4000 ( see 4.2.5 Receiver A/D Amplitude Scale )
  void * wp = &acBuf[4];
  WRITE_LITTLE_INT8( channel , wp ); //write channel information to byte #5
  wp = &acBuf[len-2];
  WRITE_LITTLE_INT16( uint16_t(rcvScale) , wp ); //write frequency to last 2 bytes of buffer

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::setRFGain( RfGain gain )
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 6;
  unsigned char acBuf[len] = { 6, 0, 0x38, 0, 0, 0 };

  void * wp = &acBuf[4];
  WRITE_LITTLE_INT8( channel , wp ); //write channel information to byte #5
  wp = &acBuf[len-1];
  WRITE_LITTLE_INT8( uint8_t(gain) , wp ); //write frequency to last byte of buffer

  mSocket.Send(acBuf, len);
  requestRFGain();
}

void RFspaceNetSDRControl::setVUHFGains( bool isAutoMode, int LNAGain, int MixGain, int IFOutGain )
{
  if ( mSocket.IsSocketInvalid() )
     return;

  LNAGain = PROCLIP( LNAGain, 0, 15 );
  MixGain = PROCLIP( MixGain, 0, 15 );
  IFOutGain = PROCLIP( IFOutGain, 0, 15 );
  const int AutoMode = isAutoMode ? 1 : 0;

  const unsigned int len = 9;
  unsigned char acBuf[len] = { 9, 0, 0x3A, 0, 0, 0, 0, 0, 0};

  void * wp = &acBuf[4];
  WRITE_BIG_INT8(AutoMode, wp);
  wp = &acBuf[5];
  WRITE_BIG_INT8(LNAGain, wp);
  wp = &acBuf[6];
  WRITE_BIG_INT8(MixGain, wp);
  wp = &acBuf[7];
  WRITE_BIG_INT8(IFOutGain, wp);
  // byte 9 is not set --> value always zero

  mSocket.Send(acBuf, len);
  //requestVUHFGains();
}

void RFspaceNetSDRControl::setRFFilterSelection( RfFilterSel filtSel )
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 6;
  unsigned char acBuf[len] = { 6, 0, 0x44, 0, 0, 0 };

  void * wp = &acBuf[4];
  WRITE_LITTLE_INT8( channel , wp ); //write channel information to byte #5
  wp = &acBuf[len-1];
  WRITE_LITTLE_INT8( uint8_t(filtSel) , wp ); //write frequency to last byte of buffer

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::setADModes( bool isDithering, ADGain gain )
{
  if ( mSocket.IsSocketInvalid() )
    return;
  uint8_t dither;

  if(isDithering)
    dither = uint8_t(ADDither::DITH_ON);
  else
    dither = uint8_t(ADDither::DITH_OFF);

  const unsigned int len = 6;
  unsigned char acBuf[len] = { 6, 0, 0x8A, 0, 0, 0 };
  void * wp = &acBuf[4];
  uint8_t mode =  dither | uint8_t(gain);
  WRITE_LITTLE_INT8( channel , wp ); //write channel information to byte #5

  wp = &acBuf[len-1];
  WRITE_LITTLE_INT8( mode , wp ); //write frequency to last byte of buffer

  mSocket.Send(acBuf, len);
  requestADMode();
}

void RFspaceNetSDRControl::setIQOutSmpRate( IQOutSmpRate smpRate )
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 9;
  unsigned char acBuf[len] = { 9, 0, 0xB8, 0, 0, 0, 0, 0, 0 };
  void * wp = &acBuf[len-4];

  // channel information can be ignored since all channels have to have the same value
  WRITE_LITTLE_INT32( uint32_t(smpRate) , wp ); //write frequency to last 4 bytes of buffer

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::setUPDPacketSize ( UDPPacketSize packetSize)
{
  if ( mSocket.IsSocketInvalid() )
    return;

  const unsigned int len = 5;
  unsigned char acBuf[len] = { 5, 0, 0xC4, 0, 0 };

  void * wp = &acBuf[len-1];
  WRITE_LITTLE_INT8( uint8_t(packetSize) , wp ); //write frequency to last byte of buffer

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::setUDPInterface ( const char * ip, uint16_t portNum )
{
  //requestUDPInterface();
  // return;

  if ( mSocket.IsSocketInvalid() )
    return;

  const uint32_t ipSend =  CSimpleSocket::GetIPv4AddrInfoStatic(ip);

  if (ipSend)
  {
    const unsigned int len = 10;
    unsigned char acBuf[len] = { 0x0A, 0, 0xC5, 0, 0, 0, 0, 0, 0, 0 };

    void * wp = &acBuf[4];
    WRITE_BIG_INT32(ipSend, wp); //BIG ENDIAN because ip and portNum already are little endian format

    wp = &acBuf[8];
    WRITE_LITTLE_INT16(portNum, wp);

    mSocket.Send(acBuf, len);

    LOG_PRO(LOG_DEBUG, "Commanded NetSDR to transmit it's UDP stream to %u.%u.%u.%u:%u"
      , unsigned((ipSend >> 24) & 0xFF)
      , unsigned((ipSend >> 16) & 0xFF)
      , unsigned((ipSend >> 8) & 0xFF)
      , unsigned(ipSend & 0xFF)
      , unsigned(portNum)
      );
  }
  else
  {
    LOG_PRO(LOG_PROTOCOL, "Skipping NetSDR command to transmit it's UDP stream towards ??? without DATA_IP address");
  }
}

void RFspaceNetSDRControl::setCWStartup ( uint8_t wpm, CWFreq cwFreq, const char * asciiMessage)
{
  if ( mSocket.IsSocketInvalid() )
    return;

  uint8_t wpmMin = uint8_t(WpmMinMax::WPM_MIN);
  uint8_t wpmMax = uint8_t(WpmMinMax::WPM_MAX);

  if( (wpm < wpmMin) || (wpm > wpmMax) )
  {
    wpm = PROCLIP(wpm, wpmMin, wpmMax);
    LOG_PRO( LOG_ERROR,"WPM value out of bounds --> setting to value %u", unsigned(wpm));
  }

  uint8_t length = strlen(asciiMessage);

  PROASSERT(length <= 10);

  const unsigned int len = 16;
  char acBuf[len] = { 0x10, 0, 0x50, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  void * wp = &acBuf[4];
  WRITE_LITTLE_INT8( wpm, wp );
  wp = &acBuf[5];
  WRITE_LITTLE_INT8( uint8_t(cwFreq), wp );

  wp = &acBuf[6];
  strncpy( &acBuf[6] , asciiMessage, 10 );

  mSocket.Send(acBuf, len);
}

void RFspaceNetSDRControl::start24BitDataStream()
{
  setRcvState( UDPstate::START_UDP_DATA, ADCstate::SET_IQ_BASEBAND_DATA, BitDepthState::SET_24BIT_MODE, CaptureMode::SET_CONTIGUOUS_MODE );
}

void RFspaceNetSDRControl::start16BitDataStream()
{
  setRcvState( UDPstate::START_UDP_DATA, ADCstate::SET_IQ_BASEBAND_DATA, BitDepthState::SET_16BIT_MODE, CaptureMode::SET_CONTIGUOUS_MODE);
}

void RFspaceNetSDRControl::stopDataStream()
{
  setRcvState( UDPstate::STOP_UDP_DATA, ADCstate::SET_IQ_BASEBAND_DATA, BitDepthState::SET_16BIT_MODE, CaptureMode::SET_CONTIGUOUS_MODE); // args 2,3,4 are dummy values --> ignored by NetSDR
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//************************************* CTRL POLLING ********************************************

bool RFspaceNetSDRControl::poll(  )
{

  if ( mSocket.IsSocketInvalid() )
    return false;

  while (1)
  {
    const int32_t maxReadLen = mRxMsgLen - mRxLen;

    int32_t peekRx = mSocket.GetNumReceivableBytes();
    int32_t rx = 0;
    if(peekRx)
    {

      rx = mSocket.Receive( maxReadLen, &mRxBuffer[mRxLen] );
    }

    if ( rx > 0 )
    {

      mRxLen += rx;
      if ( mRxLen == mRxMsgLen )
      {
        if ( mRxMsgHeader )
        {
          void * wp = &mRxBuffer[0];
          const uint16_t uHdr = READ_LITTLE_INT16( wp );
          mRxType = ((uHdr & TYPE_MASK) >> 13);    // highest 3 Bits
          mRxMsgLen = uHdr & LENGTH_MASK;  // all except highest 3 Bit

          if ( mRxMsgLen > 2 )
          {
            mRxMsgHeader = false;   // continue with parameter bytes
          }
          else // if ( mRxMsgLen <= 2 )
          {
            // NAK message
            processReceivedControlMessage(false);
            mRxLen = 0;
            mRxMsgLen = 2;
            mRxType = -1;
          }
        }
        else
        {

          processReceivedControlMessage(true);
          mRxLen = 0;
          mRxMsgLen = 2;
          mRxType = -1;
          mRxMsgHeader = true;  // continue with next message
        }
      }
      continue;
    }
    else if ( mSocket.IsSocketInvalid() )
    {
      LOG_PRO( LOG_ERROR, "ERROR: mSocket invalid!");
      return false;
    }

    return true;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************** CTRL PROCESSING *******************************************

bool RFspaceNetSDRControl::processReceivedControlMessage(bool isValidMessage)
{
  uint16_t uControlItemCode;
  bool bProcessed = true; // assume so
  void * wp = nullptr;
  if(isValidMessage)
  {
    wp = &mRxBuffer[2];
    uControlItemCode = READ_LITTLE_INT16( wp );

#if PRINT_RECEIVED_DATA
    {
      LOG_PRO( LOG_DEBUG, "receiving %d bytes:", mRxLen );
      for ( int i = 0; i < mRxLen ; ++i )
        LOG_PRO( LOG_DEBUG, "%d: 0x%x", i, unsigned(mRxBuffer[i]) );

      LOG_PRO( LOG_DEBUG, "received %d bytes of type %s with %s of length %u", mRxMsgLen, getTypeText(), getControlItemText(uControlItemCode), mRxMsgLen );
    }
#endif
  }
  else
  {
    uControlItemCode = 0x0000;
    LOG_PRO( LOG_ERROR,  "received NAK message ! " );
  }

  switch ( uControlItemCode )
  {
    case 0x0000:  // 3.2 NAK Message
      mHasNAKMessage = true;
      mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::NAK);
      break;

    case 0x0001:  // 4.1.1 Target Name
      wp = &mRxBuffer[4];
      strncpy(mTargetName,(const char*)(wp),31 );
      mHasTargetName = true;
      LOG_PRO( LOG_DEBUG, "received %s : '%s'", getControlItemText(uControlItemCode), mTargetName);
      break;

    case 0x0002:  // 4.1.2 Serial Number
      wp = &mRxBuffer[4];
      strncpy(mSerialNumber,(const char*)(wp),31 );
      mHasSerialNumber = true;
      LOG_PRO( LOG_DEBUG, "received %s : '%s'", getControlItemText(uControlItemCode),  mSerialNumber);
      break;

    case 0x0003:  // 4.1.3 Interface Version
      wp = &mRxBuffer[4];
      mInterfaceVersion = READ_LITTLE_INT16( wp );
      mHasInterfaceVersion = true;
      LOG_PRO( LOG_DEBUG, "received %s = %.2f", getControlItemText(uControlItemCode), float(mInterfaceVersion)/100); // with %u.%u and modulo 100 there would be e.g. 1.7 instead of 1.07
      break;

    case 0x0004:  // 4.1.4 Hardware/Firmware Versions
    {
      mHasHardwareFirmwareVersion = true;
      wp = &mRxBuffer[4];
      uint8_t id = READ_LITTLE_INT8( wp );
      if(id != 3)
      {
        wp = &mRxBuffer[5];
        mHardwareFirmwareVersion = READ_LITTLE_INT16( wp );
        LOG_PRO( LOG_DEBUG, "received %s (%s) = %.2f ", getControlItemText(uControlItemCode), getHWFWText(id), float(mHardwareFirmwareVersion)/100);
      }
      else
      {
        wp = &mRxBuffer[5];
        uint8_t config_type = READ_LITTLE_INT8( wp );
        wp = &mRxBuffer[6];
        uint8_t revision_num = READ_LITTLE_INT8( wp );
        LOG_PRO( LOG_DEBUG, "received %s (%s) = %u.%u (< ID > . < REVISION_NUM >) ", getControlItemText(uControlItemCode), getHWFWText(id), config_type , revision_num);
        mHardwareFirmwareVersion = 100*config_type + revision_num;
      }
      break;
    }

    case 0x0005:  // 4.1.5 Status/Error Code
    {
      mHasReceiverStatus = true;
      wp = &mRxBuffer[4];
      uint8_t status = READ_LITTLE_INT8( wp );
      LOG_PRO( LOG_DEBUG, "received %s: '%s' ", getControlItemText(uControlItemCode), getStatusText(status) );
      mReceiverStatus = status;
      break;
    }
    case 0x0009:  // 4.1.6 Product ID
      wp = &mRxBuffer[4];
      mProductId = READ_LITTLE_INT32( wp );
      mHasProductId = true;
      LOG_PRO( LOG_DEBUG, "received %s : '%x'", getControlItemText(uControlItemCode), mProductId);
      break;

    case 0x000A:  // 4.1.7 Options
    {
      // parameter2-bits currently not defined !
      parseOptionBits( );
      mHasOptions = true;
      printText(mOptions);
      mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::OPTIONS);
      break;
    }
    case 0x0018:  // 4.2.1 Receiver State --> Main "Start/Stop" command to start or stop data capture by the NetSDR
    {
      mHasRcvState = true;
      parseRcvStateBytes( );
      bool bOk = false;
      printBitDepthText(this->getStreamBitDepth(&bOk));
      mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::RCV_STATE);
      break;
    }
    case 0x0019:  // 4.2.2 Receiver Channel Setup
      mHasRcvChnSetup = true;
      wp = &mRxBuffer[4];
      mRcvChnSetup = RcvChannelSetup(READ_BIG_INT8( wp ));
      break;
    case 0x0020:  // 4.2.3 Receiver Frequency
    {
      parseRcvFrequencies( );
      if(mpCallBack)
        mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::FREQUENCY);
      break;
    }
    case 0x0023:  // 4.2.5 Receiver A/D Amplitude Scale
    {
      mHasRcvADAmplScale = true;
      wp = &mRxBuffer[4];
      uint8_t channel = READ_LITTLE_INT8( wp );
      wp = &mRxBuffer[5];
      float scale = READ_LITTLE_INT16( wp );
      scale = scale / 65536;
      mRcvADAmplScale = scale;
      LOG_PRO( LOG_DEBUG, "received %s for %s: '%.5f'", getControlItemText(uControlItemCode), getRcvChannelText(channel), mRcvADAmplScale);
      if(mpCallBack)
        mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::ADC_SCALE);
      break;
    }
    case 0x0038:  // 4.2.6 RF Gain
    {
      mHasRFGains = true;
      wp = &mRxBuffer[4];
      const RFspaceNetSDRControl::NCOChannel channel = RFspaceNetSDRControl::NCOChannel( READ_LITTLE_INT8(wp ) );
      wp = &mRxBuffer[5];
      int gain = READ_LITTLE_INT8( wp );
      switch(channel)
      {
        case RFspaceNetSDRControl::NCOChannel::CHN1:
          mRFGains.eChn1RFGain = RfGain(gain);
          break;
        case RFspaceNetSDRControl::NCOChannel::CHN2:
          mRFGains.eChn2RFGain = RfGain(gain);
          break;
        case RFspaceNetSDRControl::NCOChannel::CHN12:
          mRFGains.eChn1RFGain = RfGain(gain);
          mRFGains.eChn2RFGain = RfGain(gain);
          break;
        default:
          LOG_PRO( LOG_DEBUG , "ERROR ! --> in  4.2.6 RF Gain");
      }
      printText(mRFGains.eChn1RFGain);

      if(mpCallBack)
        mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::RF_GAIN);
      break;
    }
    case 0x003A:  // 4.2.7 VHF/UHF Downconverter Gain
    {
      mHasVUHFGains = true;
      parseVUHFGains();

      printText(mVUHFGains);

      if(mpCallBack)
        mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::VUHF_INFO);
      break;
    }

    case 0x0044:  // 4.2.8 RF Filter Selection
    {
      mHasRFFilterSelection = true;
      wp = &mRxBuffer[4];
      const RFspaceNetSDRControl::NCOChannel channel = RFspaceNetSDRControl::NCOChannel (READ_LITTLE_INT8( wp ) );
      wp = &mRxBuffer[5];
      uint8_t filtSel = READ_LITTLE_INT8( wp );
      switch(channel)
      {
        case RFspaceNetSDRControl::NCOChannel::CHN1:
              mRFFilterSelection.eChn1FiltSel = RfFilterSel(filtSel);
        break;
        case RFspaceNetSDRControl::NCOChannel::CHN2:
              mRFFilterSelection.eChn2FiltSel = RfFilterSel(filtSel);
        break;
        case RFspaceNetSDRControl::NCOChannel::CHN12:
              mRFFilterSelection.eChn1FiltSel = RfFilterSel(filtSel);
        mRFFilterSelection.eChn2FiltSel = RfFilterSel(filtSel);
        break;
        default: LOG_PRO( LOG_DEBUG,  "ERROR ! --> in  4.2.7 RF Filter Selection");
      }

      if(mpCallBack)
        mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::RF_FILTER);
      break;
    }
    case 0x008A:  // 4.2.9 A/D Modes
    {
      mHasADModes = true;
      parseADModes();
      bool bOk = false;
      printText(getADGain(&bOk));
      if(mpCallBack)
        mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::ADC_MODE);
      break;
    }
    case 0x00B8:  // 4.2.10 IQ Output Data Samplerate
      mHasIQOutSmpRate = true;
      wp = &mRxBuffer[5];
      mIQOutSmpRate = IQOutSmpRate(READ_LITTLE_INT32( wp ));
      printText(mIQOutSmpRate);
      if(mpCallBack)
        mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::IQ_OUT_SAMPLERATE);
      break;

    case 0x00C4:  // 4.4.2 IQ Output Data Samplerate
      mHasUDPPacketSize = true;
      wp = &mRxBuffer[4];
      mUDPPacketSize = UDPPacketSize(READ_LITTLE_INT32( wp ));
      if(mpCallBack)
        mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::UDP_PACKET_SIZE);
      break;

    case 0x00C5:  // 4.4.3 Data Output UDP IP and Port Address
    {
      mHasIPandPortNum = true;

      wp = &mRxBuffer[4];
      uint32_t ip = READ_LITTLE_INT32( wp );
      sprintf(mUDPIpAddress, "%u.%u.%u.%u",(ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF );
      wp = &mRxBuffer[8];
      mUDPPortNum = READ_LITTLE_INT16( wp );

      if(mpCallBack)
        mpCallBack->receiveRFspaceNetSDRControlInfo(CallbackIfc::Info::UDP_INTERFACE);
      break;
    }
    case 0x0150:  // 4.4.4 CW Startup Message
    {
      mHasCWStartup = true;
      parseCWStartup();

      break;
    }
    default:
      bProcessed = false;

  }

  return bProcessed;
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************** PARSE FUNCTIONS *******************************************

void RFspaceNetSDRControl::parseOptionBits( ) // parameter2-bits currently not defined !
{

  void * wp = &mRxBuffer[4];
  unsigned char par1Bits = 0;
  par1Bits = READ_LITTLE_INT8( wp );
  unsigned int par3Bits = 0;
  wp = &mRxBuffer[6];
  par3Bits = READ_LITTLE_INT32( wp );

  unsigned char tmp;
  tmp = par1Bits & 0x01; // first bit
  mOptions.Sound_Enabled = (tmp != 0);
  tmp = par1Bits & 0x02; // second bit
  mOptions.ReflockBoard_Present = (tmp != 0);
  tmp = par1Bits & 0x04; // third bit
  mOptions.DownConverterBoard_Present = (tmp != 0);
  tmp = par1Bits & 0x08; // 4th bit
  mOptions.UpConverterBoard_Present = (tmp != 0);
  tmp = par1Bits & 0x10; // 5th bit
  mOptions.X2Board_Present = (tmp != 0);

  unsigned int tmp2;
  tmp2 = par3Bits & 0x00000F; // first half-byte of parameter 3
  mOptions.MainBoardVariant = tmp2;
  tmp2 = par3Bits & 0x0000F0; // second half-byte of parameter 3
  mOptions.ReflockBoardVariant = (tmp2 >> 4);
  tmp2 = par3Bits & 0x00FF00; // second byte of parameter 3
  mOptions.DownConverterBoardVariant = (tmp2 >> 8);
  tmp2 = par3Bits & 0xFF0000; // third byte of parameter 3
  mOptions.UpConverterBoardVariant = (tmp2 >> 16);
}


void RFspaceNetSDRControl::parseRcvStateBytes( ) // parameter2-bits currently not defined !
{
  void * wp = &mRxBuffer[4];
  unsigned int parBytes = READ_BIG_INT32( wp ); //Big Endian because information fields are only 1 byte each --> Little Endian for fields bigger than 8bits

  mRcvState.eADCstate =       ADCstate(parBytes & 0x80000000); // last bit of parameter byte 1
  mRcvState.eUDPstate =       UDPstate(parBytes & 0x00030000); // first 2 bits of parameter byte 2
  mRcvState.eBitDepth =  BitDepthState(parBytes & 0x00008000); // last bit of parameter byte 3
  mRcvState.eCaptureMode = CaptureMode(parBytes & 0x00000300); // first 2 bits of parameter byte 3
  mRcvState.uFIFOByteCaptNum =         parBytes & 0x000000FF;  // last byte
}


void RFspaceNetSDRControl::parseRcvFrequencies( ) //data up to 40bits when frequency range was requested
{
  void * wp = &mRxBuffer[1];
  const MsgType type = MsgType( READ_BIG_INT8( wp ) );

  switch(type)
  {
    case RFspaceNetSDRControl::MsgType::SET_CTRL_ITEM:
    {
      mHasRcvFrequency = true;
      wp = &mRxBuffer[4];
      const RFspaceNetSDRControl::NCOChannel channel = RFspaceNetSDRControl::NCOChannel( READ_LITTLE_INT8(wp) );
      wp = &mRxBuffer[5];
      uint64_t frequency = READ_LITTLE_INT64(wp);
      frequency = frequency & 0xFFFFFFFFFF;                      //frequency is within first 5 bytes

      if(channel == RFspaceNetSDRControl::NCOChannel::CHN1)
        mRcvFrequencies.Chn1_Freq = frequency;
      else if(channel == RFspaceNetSDRControl::NCOChannel::CHN2)
        mRcvFrequencies.Chn2_Freq = frequency;
      else if(channel == RFspaceNetSDRControl::NCOChannel::CHN12)
      {
        mRcvFrequencies.Chn1_Freq = frequency;
        mRcvFrequencies.Chn2_Freq = frequency;
      }
      else
        LOG_PRO( LOG_DEBUG, "if-else ERROR in RFspaceNetSDRControl::parseRcvFrequencies");

      break;
    }
    case RFspaceNetSDRControl::MsgType::REQ_CTRL_RANGE:
    {
      void * wp = nullptr;
      mHasRcvFrequencyRanges = true;
      wp = &mRxBuffer[4];
      const RFspaceNetSDRControl::NCOChannel channel = RFspaceNetSDRControl::NCOChannel( READ_LITTLE_INT8(wp) );
      wp = &mRxBuffer[5];
      unsigned char rangeNum = READ_LITTLE_INT8(wp);
      int dataIdx = 6;
      for(unsigned char i = 0 ; i < rangeNum; i++)
      {
        uint64_t minFrequency = 0;
        uint64_t maxFrequency = 0;
        uint64_t VCOFrequency = 0;

        wp = &mRxBuffer[dataIdx];
        minFrequency = READ_LITTLE_INT64(wp);
        wp = &mRxBuffer[dataIdx+5];
        maxFrequency = READ_LITTLE_INT64(wp);
        wp = &mRxBuffer[dataIdx+10];
        VCOFrequency = READ_LITTLE_INT64(wp);

        dataIdx = dataIdx+15;
        if(channel == RFspaceNetSDRControl::NCOChannel::CHN1)
        {
          if(i == 0)
          {
            mRcvFrequencies.Chn1_Bnd1_MinFreq = minFrequency & 0xFFFFFFFFFF;
            mRcvFrequencies.Chn1_Bnd1_MaxFreq = maxFrequency & 0xFFFFFFFFFF;
            mRcvFrequencies.Chn1_Bnd1_VCO_DwnConvFreq = VCOFrequency & 0xFFFFFFFFFF;
          }
          else
          {
            mRcvFrequencies.Chn1_Bnd2_MinFreq = minFrequency & 0xFFFFFFFFFF;
            mRcvFrequencies.Chn1_Bnd2_MaxFreq = maxFrequency & 0xFFFFFFFFFF;
            mRcvFrequencies.Chn1_Bnd2_VCO_DwnConvFreq = VCOFrequency & 0xFFFFFFFFFF;
          }
        }
        else if(channel == RFspaceNetSDRControl::NCOChannel::CHN2)
        {
          if(i == 0)
          {
            mRcvFrequencies.Chn2_Bnd1_MinFreq = minFrequency & 0xFFFFFFFFFF;
            mRcvFrequencies.Chn2_Bnd1_MaxFreq = maxFrequency & 0xFFFFFFFFFF;
            mRcvFrequencies.Chn2_Bnd1_VCO_DwnConvFreq = VCOFrequency & 0xFFFFFFFFFF;
          }
          else
          {
            mRcvFrequencies.Chn2_Bnd2_MinFreq = minFrequency & 0xFFFFFFFFFF;
            mRcvFrequencies.Chn2_Bnd2_MaxFreq = maxFrequency & 0xFFFFFFFFFF;
            mRcvFrequencies.Chn2_Bnd2_VCO_DwnConvFreq = VCOFrequency & 0xFFFFFFFFFF;
          }
        }
        else
          LOG_PRO( LOG_DEBUG, "ERROR ! --> Channel selection not allowed (?) ");

        //Only one channel is supported by current NetSDR setup
        mRcvFrequencyRanges[0] = mRcvFrequencies.Chn1_Bnd1_MinFreq;
        mRcvFrequencyRanges[1] = mRcvFrequencies.Chn1_Bnd1_MaxFreq;
        mRcvFrequencyRanges[2] = mRcvFrequencies.Chn1_Bnd2_MinFreq;
        mRcvFrequencyRanges[3] = mRcvFrequencies.Chn1_Bnd2_MaxFreq;
      }
      break;
    }

    case RFspaceNetSDRControl::MsgType::REQ_CTRL_ITEM:  // no break
    default: LOG_PRO( LOG_DEBUG, "Switch Case ERROR! --> parseRcvFrequencies");
  }
}


void RFspaceNetSDRControl::parseADModes( )
{
  void * wp = &mRxBuffer[4];
  NCOChannel channel = NCOChannel(READ_LITTLE_INT8( wp ));
  wp = &mRxBuffer[5];
  unsigned char mode = READ_LITTLE_INT8( wp );

  switch (channel)
  {
    case NCOChannel::CHN1:
    {
      mADModes.eChn1AD_Dither = ADDither((mode & uint8_t(RFspaceNetSDRControl::ADMask::DITHMASK)));
      mADModes.eChn1AD_Gain = ADGain((mode & uint8_t(RFspaceNetSDRControl::ADMask::GAINMASK)));
      break;
    }
    case NCOChannel::CHN2:
    {
      mADModes.eChn2AD_Dither = ADDither((mode & uint8_t(RFspaceNetSDRControl::ADMask::DITHMASK)));
      mADModes.eChn2AD_Gain = ADGain((mode & uint8_t(RFspaceNetSDRControl::ADMask::GAINMASK)));
      break;
    }
    case NCOChannel::CHN12:
    {
      mADModes.eChn1AD_Dither = ADDither((mode & uint8_t(RFspaceNetSDRControl::ADMask::DITHMASK)));
      mADModes.eChn1AD_Gain = ADGain((mode & uint8_t(RFspaceNetSDRControl::ADMask::GAINMASK)));
      mADModes.eChn2AD_Dither = ADDither((mode & uint8_t(RFspaceNetSDRControl::ADMask::DITHMASK)));
      mADModes.eChn2AD_Gain = ADGain((mode & uint8_t(RFspaceNetSDRControl::ADMask::GAINMASK)));

      break;
    }
  }
}

void RFspaceNetSDRControl::parseCWStartup()
{
  void * wp = &mRxBuffer[4];
  mCwStartup.cw_wpm = READ_LITTLE_INT8(wp);
  wp = &mRxBuffer[5];
  mCwStartup.eCwFreq = CWFreq(READ_LITTLE_INT8(wp));

  for(uint8_t i=0; i<10; i++)
  {
    wp = &mRxBuffer[i+6];
    mCwStartup.asciiMessage[i] = READ_LITTLE_INT8(wp);
  }
}

void RFspaceNetSDRControl::parseVUHFGains( )
{
  void * wp = &mRxBuffer[4];
  uint8_t modeParameterByte = READ_BIG_INT8( wp );
  wp = &mRxBuffer[5];
  uint32_t gainParamBytes = READ_BIG_INT32( wp );

  mVUHFGains.isAGCMode = (modeParameterByte =! 0) ? true : false;

  mVUHFGains.LNAGainLevel = gainParamBytes & 0xFF000000;
  mVUHFGains.MixerGainLevel = gainParamBytes & 0x00FF0000;
  mVUHFGains.IFOutputGainLevel = gainParamBytes & 0x0000FF00;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//********************************** PRINT FUNCTIONS ********************************************

void RFspaceNetSDRControl::printText( const Options & opt )
{
  LOG_PRO( LOG_DEBUG, "Receiver Options Information : ");
  LOG_PRO( LOG_DEBUG, "Sound Enabled :              %s ", opt.Sound_Enabled ? "Yes" : "No" );
  LOG_PRO( LOG_DEBUG, "ReflockBoard Present :       %s ", opt.ReflockBoard_Present ? "Yes" : "No" );
  LOG_PRO( LOG_DEBUG, "DownConverterBoard Present : %s ", opt.DownConverterBoard_Present ? "Yes" : "No" );
  LOG_PRO( LOG_DEBUG, "UpConverterBoard Present :   %s ", opt.UpConverterBoard_Present ? "Yes" : "No" );
  LOG_PRO( LOG_DEBUG, "X2Board Present :            %s ", opt.X2Board_Present ? "Yes" : "No" );
  LOG_PRO( LOG_DEBUG, "MainBoardVariant :           %u ", opt.MainBoardVariant);
  LOG_PRO( LOG_DEBUG, "ReflockBoardVariant :        %u ", opt.ReflockBoardVariant);
  LOG_PRO( LOG_DEBUG, "DownConverterBoardVariant :  %u ", opt.DownConverterBoardVariant);
  LOG_PRO( LOG_DEBUG, "UpConverterBoardVariant :    %u ", opt.UpConverterBoardVariant);
}

void RFspaceNetSDRControl::printUDPStateText( const bool & state )
{
  LOG_PRO( LOG_DEBUG, "UDP State Information : ");
  LOG_PRO( LOG_DEBUG, "UDP State : %s", state ? "RUNNING" : "IDLE" );
}

void RFspaceNetSDRControl::printBitDepthText( const int & bitDepth )
{
  LOG_PRO( LOG_DEBUG, "UDP Bit Depth Information : ");
  LOG_PRO( LOG_DEBUG, "UDP Bit Depth : %uBit ", bitDepth);
}

/*
void RFspaceNetSDRControl::printText( const RcvFrequencies & rcvFreq)
{
  LOG_PRO( LOG_DEBUG, "Receiver Frequency Information : ");
  LOG_PRO( LOG_DEBUG, "Channel 1 NCO-Frequency  :                     %lu Hz", rcvFreq.Chn1_Freq );
  LOG_PRO( LOG_DEBUG, "Channel 2 NCO-Frequency  :                     %lu Hz", rcvFreq.Chn2_Freq );
  LOG_PRO( LOG_DEBUG, "****************************************************************************************** ");
  LOG_PRO( LOG_DEBUG, "Range: Channel 1, Band 1 min-max Frequency  :  %lu - %lu Hz", rcvFreq.Chn1_Bnd1_MinFreq, rcvFreq.Chn1_Bnd1_MaxFreq);
  LOG_PRO( LOG_DEBUG, "Range: Channel 1, Band 1 VCO- Frequency  :     %lu Hz", rcvFreq.Chn1_Bnd1_VCO_DwnConvFreq );
  LOG_PRO( LOG_DEBUG, "Range: Channel 1, Band 2 min-max Frequency  :  %lu - %lu Hz", rcvFreq.Chn1_Bnd2_MinFreq, rcvFreq.Chn1_Bnd2_MaxFreq);
  LOG_PRO( LOG_DEBUG, "Range: Channel 1, Band 2 VCO- Frequency  :     %lu Hz", rcvFreq.Chn1_Bnd2_VCO_DwnConvFreq );
  LOG_PRO( LOG_DEBUG, "****************************************************************************************** ");
  LOG_PRO( LOG_DEBUG, "Range: Channel 2, Band 1 min-max Frequency  :  %lu - %lu Hz", rcvFreq.Chn2_Bnd1_MinFreq, rcvFreq.Chn2_Bnd1_MaxFreq);
  LOG_PRO( LOG_DEBUG, "Range: Channel 2, Band 1 VCO- Frequency  :     %lu Hz", rcvFreq.Chn2_Bnd1_VCO_DwnConvFreq );
  LOG_PRO( LOG_DEBUG, "Range: Channel 2, Band 2 min-max Frequency  :  %lu - %lu Hz", rcvFreq.Chn2_Bnd2_MinFreq, rcvFreq.Chn2_Bnd2_MaxFreq);
  LOG_PRO( LOG_DEBUG, "Range: Channel 2, Band 2 VCO- Frequency  :     %lu Hz", rcvFreq.Chn2_Bnd2_VCO_DwnConvFreq );
}
*/
void RFspaceNetSDRControl::printRcvFreqText( const uint64_t & rcvFreq)
{
  LOG_PRO( LOG_DEBUG, "Receiver Frequency Information :");
  LOG_PRO( LOG_DEBUG, "Frequency:   %luHz", rcvFreq );
}

void RFspaceNetSDRControl::printFilterText( const int & filtSel)
{
  LOG_PRO( LOG_DEBUG, "RF FilterSelect Information : ");
  LOG_PRO( LOG_DEBUG, "Filter Select:   %s", getFilterText(filtSel) );
}

void RFspaceNetSDRControl::printText( const bool & dithering)
{
  LOG_PRO( LOG_DEBUG, "AD Dither Information : ");
  LOG_PRO( LOG_DEBUG, "Dithering:   %s", dithering ? "ON" : "OFF" );
}

void RFspaceNetSDRControl::printText( const float & gain)
{
  LOG_PRO( LOG_DEBUG, "AD Gain Information : ");
  if(gain != -1)
    LOG_PRO( LOG_DEBUG, "Gain:   %.1f", gain );
  else
    LOG_PRO( LOG_ERROR, "ERROR! Passed invalid channel argument. Please choose < 1 > or < 2 > in the get function");
}

void RFspaceNetSDRControl::printText( const RfGain & gain )
{
  {
    LOG_PRO( LOG_DEBUG, "RF Gain Information :");
    LOG_PRO( LOG_DEBUG, "RFGain:   %s", getText(gain) );
  }
}

void RFspaceNetSDRControl::printText( const VUHFGains & vuhfGains )
{
  {
    LOG_PRO( LOG_DEBUG, "VUHF Gain Information :");
    LOG_PRO( LOG_DEBUG, "VUHF AutoMode:    %s", vuhfGains.isAGCMode ? "ON" : "OFF" );
    LOG_PRO( LOG_DEBUG, "VUHF LNA Gain:    %u", vuhfGains.LNAGainLevel );
    LOG_PRO( LOG_DEBUG, "VUHF Mix Gain:    %u", vuhfGains.MixerGainLevel );
    LOG_PRO( LOG_DEBUG, "VUHF IF Gain:     %u", vuhfGains.IFOutputGainLevel );
  }
}

void RFspaceNetSDRControl::printText( const IQOutSmpRate & smpRate )
{
  LOG_PRO( LOG_DEBUG, "IQ Output Sample Rate Information :");
  LOG_PRO( LOG_DEBUG, "IQ Output Sample Rate:   %s", getText(smpRate) );
}

void RFspaceNetSDRControl::printText( const UDPPacketSize & packetSize )
{
  LOG_PRO( LOG_DEBUG, "IQ UDP Packet Size Information : ");
  LOG_PRO( LOG_DEBUG, "UDP Packet Size:   %s", getText(packetSize) );
}

void RFspaceNetSDRControl::printUDPIPText( const char * ip )
{
  LOG_PRO( LOG_DEBUG, "UDP IP Information : ");
  LOG_PRO( LOG_DEBUG, "UDP IP : %s", ip);
}

void RFspaceNetSDRControl::printUDPPortNumText( const unsigned portNum )
{
  LOG_PRO( LOG_DEBUG, "UDP Port Number Information : ");
  LOG_PRO( LOG_DEBUG, "UDP Port Number : %u", portNum);
}

void RFspaceNetSDRControl::printText( const CWStartup & cwStartup)
{

  uint8_t len = sizeof(cwStartup.asciiMessage);
  std::string stringMessage;
  for(uint8_t i=0; i<len; i++)
  {
    stringMessage[i] = (uint8_t)cwStartup.asciiMessage[i];
  }

  LOG_PRO( LOG_DEBUG, "CW Startup Information :");
  LOG_PRO( LOG_DEBUG, "CW wpm:         %u", cwStartup.cw_wpm);
  LOG_PRO( LOG_DEBUG, "CW Frequency:   %s", getText(cwStartup.eCwFreq));
  LOG_PRO( LOG_DEBUG, "CW Message:     '%s'", stringMessage.c_str());

}

void RFspaceNetSDRControl::printText( const uint8_t & chnMode)
{
  LOG_PRO( LOG_DEBUG, "Receiver Channel Setup Information : ");
  LOG_PRO( LOG_DEBUG, "Receiver Channel Setup:  %s", getText(chnMode));
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************** TEXT FUNCTIONS ********************************************

const char * RFspaceNetSDRControl::getHWFWText( uint8_t id ) const
{
  switch ( id )
  {
    case uint8_t(RFspaceNetSDRControl::HwFw::BOOT_CODE):    return "Boot Code Version";
    case uint8_t(RFspaceNetSDRControl::HwFw::APP_FW):       return "Application Firmware Version";
    case uint8_t(RFspaceNetSDRControl::HwFw::HW_VER):       return "Hardware Version";
    case uint8_t(RFspaceNetSDRControl::HwFw::FPGA_CONF):    return "FPGA Configuration Version";

    default:  return "ERROR !";
  }
}


const char * RFspaceNetSDRControl::getStatusText( uint8_t code ) const
{
  switch ( code )
  {
    case uint8_t(RFspaceNetSDRControl::RcvStatusCode::RCV_IDLE):            return "Receiver In Idle";
    case uint8_t(RFspaceNetSDRControl::RcvStatusCode::RCV_BUSY):            return "Receiver Busy (data caputre)";
    case uint8_t(RFspaceNetSDRControl::RcvStatusCode::RCV_BOOT_IDLE):       return "Receiver In Boot Mode Idle";
    case uint8_t(RFspaceNetSDRControl::RcvStatusCode::RCV_BOOT_BUSY_PROG):  return "Receiver In Boot Mode Busy Programming";
    case uint8_t(RFspaceNetSDRControl::RcvStatusCode::RCV_AD_OVERLOAD):     return "Receiver A/D Overload Occured !";
    case uint8_t(RFspaceNetSDRControl::RcvStatusCode::RCV_PROG_ERROR):      return "Receiver Boot Mode Programming Error";

    default: return "ERROR !";
  }
}


const char * RFspaceNetSDRControl::getTypeText() const
{
  switch ( mRxType )
  {
    case 0:   return "Response(000)";
    case 1:   return "UnsolicitedControlItem(001)";
    case 2:   return "ResponseRange(010)";
    case 3:   return "DataItemAck(011)";
    case 4:   return "TargetDataItem0(100)";
    case 5:   return "TargetDataItem1(101)";
    case 6:   return "TargetDataItem2(110)";
    case 7:   return "TargetDataItem3(111)";

    default:  return "ERROR !";
  }
}

const char * RFspaceNetSDRControl::getControlItemText(uint16_t uControlItemCode) const
{
  switch ( uControlItemCode )
  {
    case 0x0001:  return "Target Name";
    case 0x0002:  return "Serial Number";
    case 0x0003:  return "Interface Version";
    case 0x0004:  return "Hardware/Firmware Version";
    case 0x0005:  return "Status/Error Code";
    case 0x0009:  return "Product ID";
    case 0x000A:  return "Options";
    case 0x0018:  return "Receiver State";
    case 0x0019:  return "Receiver Channel Setup";
    case 0x0020:  return "Receiver Frequency";
    case 0x0023:  return "Receiver A/D Amplitude Scale";
    case 0x0038:  return "RF Gain";
    case 0x0044:  return "RF Filter Selection";
    case 0x008A:  return "A/D Modes";
    case 0x00B8:  return "IQ Output Data Samplerate";
    case 0x00C4:  return "Data Output Packet Size";
    case 0x00C5:  return "Data Output UDP IP and Port Address";
    case 0x0150:  return "CW Startup Message";

    default:  return "ERROR !";
  }

}

const char * RFspaceNetSDRControl::getRcvChannelSetupText( uint8_t setup ) const
{
  switch ( setup )
  {
    case 0:  return "Setup 0: Single Channel Mode, Control Channel 1";
    case 1:  return "Setup 1: Single Channel Mode, Control Channel 2";
    case 2:  return "Setup 2: Single Channel Mode, Control Channel 1 or 2, I/Q = Chn1 + Chn2";
    case 3:  return "Setup 3: Single Channel Mode, Control Channel 1 or 2, I/Q = Chn1 - Chn2";
    case 4:  return "Setup 4: Dual Channel Mode, single A/D RF Path, Main A/D";
    case 5:  return "Setup 5: Dual Channel Mode, single A/D RF Path, X2 A/D";
    case 6:  return "Setup 6: Dual Channel Mode, dual A/D RF Path";

    default:  return "ERROR !";
  }

}

const char * RFspaceNetSDRControl::getRcvChannelText( uint8_t chn ) const
{
  switch ( chn )
  {
    case uint8_t(RFspaceNetSDRControl::NCOChannel::CHN1):  return "Channel 1";
    case uint8_t(RFspaceNetSDRControl::NCOChannel::CHN2):  return "Channel 2";
    case uint8_t(RFspaceNetSDRControl::NCOChannel::CHN12): return "Both Channels";
    default:  return "ERROR ! --> getRcvChannelText ";
  }

}

const char * getText( RFspaceNetSDRControl::RfGain e )
{
  //RFspaceNetSDRControl::RfGain b = RFspaceNetSDRControl::RfGain::ATT_0DB;
  switch(e)
  {
    case RFspaceNetSDRControl::RfGain::ATT_0DB:       return "0dB";
    case RFspaceNetSDRControl::RfGain::ATT_10DB:      return "-10dB";
    case RFspaceNetSDRControl::RfGain::ATT_20DB:      return "-20dB";
    case RFspaceNetSDRControl::RfGain::ATT_30DB:      return "-30dB";
    default:  return "ERROR";
  }
}

const char * getFilterText( int e )
{
  switch(RFspaceNetSDRControl::RfFilterSel(e))
  {
    case RFspaceNetSDRControl::RfFilterSel::F_AUTO:               return "Auto Selection";
    case RFspaceNetSDRControl::RfFilterSel::F_0_TO_1800kHz:       return "Filter 0 - 1.8MHz";
    case RFspaceNetSDRControl::RfFilterSel::F_1800_TO_2800kHz:    return "Filter 1.8 to 2.8 MHz";
    case RFspaceNetSDRControl::RfFilterSel::F_2800_TO_4000kHz:    return "Filter 2.8 to 4.0 MHz";
    case RFspaceNetSDRControl::RfFilterSel::F_4000_TO_5500kHz:    return "Filter 4.0 to 5.5 MHz";
    case RFspaceNetSDRControl::RfFilterSel::F_5500_TO_7000kHz:    return "Filter 5.5 to 7.0 MHz";
    case RFspaceNetSDRControl::RfFilterSel::F_7000_TO_10000kHz:   return "Filter 7 to 10 MHz";
    case RFspaceNetSDRControl::RfFilterSel::F_10000_TO_14000kHz:  return "Filter 10 to 14 MHz";
    case RFspaceNetSDRControl::RfFilterSel::F_14000_TO_20000kHz:  return "Filter 14 to 20 MHz";
    case RFspaceNetSDRControl::RfFilterSel::F_20000_TO_28000kHz:  return "Filter 20 to 28 MHz";
    case RFspaceNetSDRControl::RfFilterSel::F_28000_TO_35000kHz:  return "Filter 28 to 35 MHz";
    case RFspaceNetSDRControl::RfFilterSel::F_BYPASS:             return "Bypass Filters";
    case RFspaceNetSDRControl::RfFilterSel::F_NOPASS:             return "Nopass Filter";
    case RFspaceNetSDRControl::RfFilterSel::F_MANPATH:            return "Manual DownConverter Path";
    default:  return "ERROR";
  }
}

const char * getText( RFspaceNetSDRControl::ADGain e)
{
  switch (e)
  {
    case RFspaceNetSDRControl::ADGain::ADGain_1:    return "GAIN 1";
    case RFspaceNetSDRControl::ADGain::ADGain_1_5:   return "GAIN 1.5";
    default : return "ERROR";
  }
}

const char * getText( RFspaceNetSDRControl::IQOutSmpRate e)
{
  switch (e)
  {
    case RFspaceNetSDRControl::IQOutSmpRate::SR_12_5kHz:  return "12.5 kHz";
    case RFspaceNetSDRControl::IQOutSmpRate::SR_32kHz:    return "32 kHz";
    case RFspaceNetSDRControl::IQOutSmpRate::SR_62_5kHz:  return "62.5 kHz";
    case RFspaceNetSDRControl::IQOutSmpRate::SR_100kHz:   return "100 kHz";
    case RFspaceNetSDRControl::IQOutSmpRate::SR_125kHz:   return "125 kHz";
    case RFspaceNetSDRControl::IQOutSmpRate::SR_250kHz:   return "250 kHz";
    case RFspaceNetSDRControl::IQOutSmpRate::SR_500kHz:   return "500 kHz";
    case RFspaceNetSDRControl::IQOutSmpRate::SR_625kHz:   return "625 kHz";
    case RFspaceNetSDRControl::IQOutSmpRate::SR_1000kHz:  return "1 MHz";
    case RFspaceNetSDRControl::IQOutSmpRate::SR_1666kHz:  return "1.66 MHz";
    case RFspaceNetSDRControl::IQOutSmpRate::SR_2000kHz:  return "2 MHz";
    default : return "ERROR";
  }
}

const char * getText( RFspaceNetSDRControl::UDPPacketSize e)
{
  switch (e)
  {
    case RFspaceNetSDRControl::UDPPacketSize::SMALL:    return "Small UDP packets ( 388 bytes(24bit data) or 516 bytes(16bit data) )";
    case RFspaceNetSDRControl::UDPPacketSize::LARGE:    return "Large UDP packets (1444 bytes(24bit data) or 1028 bytes(16bit data) )";
    default : return "ERROR";
  }
}

const char * getText( RFspaceNetSDRControl::CWFreq e)
{
  switch (e)
  {
    case RFspaceNetSDRControl::CWFreq::CW_500Hz:    return "500 Hz";
    case RFspaceNetSDRControl::CWFreq::CW_1000Hz:   return "1000 Hz";
    case RFspaceNetSDRControl::CWFreq::CW_1500Hz:   return "1500 Hz";
    default : return "ERROR";
  }
}

const char * getText( uint8_t e )
{
  switch (e)
  {
    case 0:  return "Setup 0: Single Channel Mode, Control Channel 1";
    case 1:  return "Setup 1: Single Channel Mode, Control Channel 2";
    case 2:  return "Setup 2: Single Channel Mode, Control Channel 1 or 2, I/Q = Chn1 + Chn2";
    case 3:  return "Setup 3: Single Channel Mode, Control Channel 1 or 2, I/Q = Chn1 - Chn2";
    case 4:  return "Setup 4: Dual Channel Mode, single A/D RF Path, Main A/D";
    case 5:  return "Setup 5: Dual Channel Mode, single A/D RF Path, X2 A/D";
    case 6:  return "Setup 6: Dual Channel Mode, dual A/D RF Path";

    default:  return "ERROR !";
  }
}

