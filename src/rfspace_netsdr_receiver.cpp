
#include "rfspace_netsdr_receiver.h"
#include "ExtIO_Logging.h"
#include "procitec_replacements.h"

#include <string.h>
#include <cstring>
#include <assert.h>


// from NetSDR specification:
//   This parameter limited to frequencies that are integer divisions by 4 of the 80MHz A/D sample rate
//   The maximum sample rate supported is 2,000,000 Hz in the 16 bits/sample mode.(80MHz/40)
//   The maximum sample rate supported is 1,333,333 Hz in the 24 bits/sample mode.(80MHz/60)
//   The minimum sample rate supported is 32,000 Hz in the 16 or 24 bits/sample mode.( 80MHz/250)
//   
// these tables have to be in sync with RFspaceNetSDRControl::IQOutSmpRate !!!

#define KHZ *1000

const RFspaceNetReceiver::srate_bw RFspaceNetReceiver::srate_bws[] =
{
  { 6400, uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_12kHz  ), 12500,      10 KHZ, "12.5 kHz"  }  // 0
, { 5000, uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_16kHz  ),   16 KHZ,   12 KHZ, "16 kHz"    }
, { 2500, uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_32kHz  ),   32 KHZ,   25 KHZ, "32 kHz"    }
, { 1280, uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_62kHz  ), 62500,      50 KHZ, "62.5 kHz"  }
, { 1000, uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_80kHz  ),   80 KHZ,   64 KHZ, "80 kHz"    }
, { 800,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_100kHz ),  100 KHZ,   80 KHZ, "100 kHz"   }  // 5
, { 640,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_125kHz ),  125 KHZ,  100 KHZ, "125 kHz"   }
, { 500,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_160kHz ),  160 KHZ,  128 KHZ, "160 kHz"   }
, { 400,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_200kHz ),  200 KHZ,  160 KHZ, "200 kHz"   }
, { 320,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_250kHz ),  250 KHZ,  200 KHZ, "250 kHz"   }
, { 256,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_312kHz ), 312500,    250 KHZ, "312.5 kHz" }  // 10
, { 200,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_400kHz ),  400 KHZ,  320 KHZ, "400 kHz"   }
, { 160,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_500kHz ),  500 KHZ,  400 KHZ, "500 kHz"   }
, { 128,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_625kHz ),  625 KHZ,  500 KHZ, "625 kHz"   }
, { 100,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_800kHz ),  800 KHZ,  640 KHZ, "800 kHz"   }
, {  80,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_1000kHz), 1000 KHZ,  800 KHZ, "1 MHz"     }  // 15
, {  64,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_1250kHz), 1250 KHZ, 1000 KHZ, "1.25 MHz"  }
, {  52,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_1538kHz), 1538461,  1200 KHZ, "1.538 MHz" }
, {  48,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_1666kHz), 1666666,  1300 KHZ, "1.666 MHz" }
, {  44,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_1818kHz), 1818181,  1400 KHZ, "1.818 MHz" }
, {  40,  uint32_t(RFspaceNetSDRControl::IQOutSmpRate::SR_2000kHz), 2000 KHZ, 1600 KHZ, "2 MHz"     }  // 20
};

#undef KHZ

//                                                               0       1        2      3       4       5     6     7
const float RFspaceNetReceiver::mafAttenuationATTs[]       = { -30.0F, -30.0F, -20.0F, -20.0F, -10.0F, -10.0F, 0.0F, 0.0F };
const float RFspaceNetReceiver::mafActualAttenuationATTs[] = { -30.0F, -26.5F, -20.0F, -16.5F, -10.0F,  -6.5F, 0.0F, 3.5F };
const float RFspaceNetReceiver::mafAttenuationADGains[]    = {   1.0F,   1.5F,   1.0F,   1.5F,   1.0F,   1.5F, 1.0F, 1.5F };
const float RFspaceNetReceiver::mafAttenuationADGainsdB[]  = {   0.0F,   3.0F,   0.0F,   3.0F,   0.0F,   3.0F, 0.0F, 3.0F };

const float RFspaceNetReceiver::mafVUHFCompatibilityGainValues[] = {0, 1, 2, 3 };
const float RFspaceNetReceiver::mafVUHFMultiGainValues[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
const RFspaceNetSDRControl::RfGain RFspaceNetReceiver::mVUHF_RFGainAttenuations[] =
{
    RFspaceNetSDRControl::RfGain::ATT_30DB
  , RFspaceNetSDRControl::RfGain::ATT_20DB
  , RFspaceNetSDRControl::RfGain::ATT_10DB
  , RFspaceNetSDRControl::RfGain::ATT_0DB
};


const int RFspaceNetReceiver::miNumAttenuations = (int)( NUMEL(mafAttenuationATTs) );
const int RFspaceNetReceiver::miNumCompatibilityGains = (int)( sizeof(mafVUHFCompatibilityGainValues) / sizeof(mafVUHFCompatibilityGainValues[0]) );
const int RFspaceNetReceiver::miNumMultiGains = (int)( sizeof(mafVUHFMultiGainValues) / sizeof(mafVUHFMultiGainValues[0]) );
const int RFspaceNetReceiver::miNumSamplerates = (int)(sizeof(srate_bws) / sizeof(srate_bws[0]));

#define NORMALIZE_DATA 0


int RFspaceNetReceiver::getDefaultHWType()
{
#if NORMALIZE_DATA
  return exthwUSBfloat32;
#else
  return exthwUSBdata16;
#endif
}


int RFspaceNetReceiver::getActualAttIdx(RFspaceNetReceiver::Settings * poSettings)
{
  if (poSettings)
  {
/*    if(!poSettings->bIsVUHFRange)
      return poSettings->iAttenuationIdx;
    else
    {
      if(!poSettings->bUseVUHFExpert)
        return poSettings->iCompatibilityValue;
      else
        return poSettings->iControlValue;
    }*/
    return poSettings->iControlValue;
  }
  else
    return -1;
}

int RFspaceNetReceiver::getIdxForAttValue( float fValue )
{
  int ret = -1;

  for (int idx = 1; idx < miNumAttenuations; ++idx )
  {
    if ( fValue < 0.5F * (mafAttenuationATTs[idx-1] +mafAttenuationADGainsdB[idx-1] + mafAttenuationATTs[idx] + mafAttenuationADGainsdB[idx]) )
    {
      ret = idx -1;
      break;
    }
  }

  if ( ret < 0 )
    ret = miNumAttenuations -1;

  return ret;
}


RFspaceNetReceiver::RFspaceNetReceiver()
: rcv(this)
, udp(this)
{
  mExtIOCallbackPtr = nullptr;
  mpoSettings = nullptr;

  mRcvFreqCallackCount = 0;
  mSampleBufferLenInFrames = 0;
  mIsDithering = false;

  mADGain = 1;
  mHasADGain = false;

  mRFGaindB = 0;
  mHasRFGain = false;

  mRcvFrequencyRanges = nullptr;

  mHasRcvFrequencyRanges = false;
  mHasDifferentLOFrequency = false;

  mTime = 0;
  mTimeWithoutDataInMilliseconds = 0;
  mControlPingTime = 0;
  mIsUDPRunning = false;
  mHasLostTCPConntection = false;
  mHasVUHFFreqRange = true;
  mHasOptions = false;

  mStartUDPTimer = 0;
  mStartData = false;

  mLastReportedFmt = 0;

  mGainControlMode = GainControlMode::AUTO;

  mNetSdrBitDepth = 16;
#if NORMALIZE_DATA
  mLastReportedSampleFormat = mExtHwSampleFormat = extHw_SampleFormat_FLT32;
  mExtHwBitDepth = 32;
#else
  mLastReportedSampleFormat = mExtHwSampleFormat = (mNetSdrBitDepth == 24) ? extHw_SampleFormat_PCM24 : extHw_SampleFormat_PCM16;
  mExtHwBitDepth = (mNetSdrBitDepth == 24) ? 24 : 16;
#endif
}

RFspaceNetReceiver::~RFspaceNetReceiver()
{

}


void RFspaceNetReceiver::setExtIoCallback( pfnExtIOCallback ExtIOCallbackPtr )
{
  mExtIOCallbackPtr = ExtIOCallbackPtr;
}


void RFspaceNetReceiver::receiveRfspaceNetSDRUdpData(const unsigned fmt, const void * voidpvRawSampleData, const int numIQFrames, const bool bSequenceNumError)
{
  mTimeWithoutDataInMilliseconds = 0;
  if (!mExtIOCallbackPtr)
  {
    LOG_PRO(LOG_ERROR, "RFspaceNetReceiver::receiveRfspaceNetSDRUdpData(): mExtIOCallbackPtr == NULL");
    return;
  }

  if (mLastReportedFmt != fmt || bSequenceNumError)
  {
    mSampleBufferLenInFrames = 0;
    if (!fmt || fmt > 4)
      LOG_PRO(LOG_ERROR, "Received format %u of sample data from UDP is unknown!", mLastReportedFmt);

    mNetSdrBitDepth = (fmt <= 2) ? 16 : 24;
#if NORMALIZE_DATA
    mExtHwSampleFormat = extHw_SampleFormat_FLT32;
    mExtHwBitDepth = 32;
#else
    mExtHwSampleFormat = (mNetSdrBitDepth == 24) ? extHw_SampleFormat_PCM24 : extHw_SampleFormat_PCM16;
    mExtHwBitDepth = (mNetSdrBitDepth == 24) ? 24 : 16;
#endif
    if (mLastReportedSampleFormat != mExtHwSampleFormat)
    {
      LOG_PRO(LOG_ERROR, "SEND STATUS CHANGE TO SDR: NEW BIT DEPTH %d Bit. while running => ERROR!.", mExtHwBitDepth);
      EXTIO_STATUS_CHANGE(mExtIOCallbackPtr, mExtHwSampleFormat);
      mLastReportedSampleFormat = mExtHwSampleFormat;
    }

    mLastReportedFmt = fmt;
  }

#if NORMALIZE_DATA
  if ( fmt <= 2 )
  {
    const int16_t * puInputSamples = (const int16_t*)(voidpvRawSampleData);
    const float factor = mpoSettings->mfGainCompensationFactor;
    int srcFrameOffset = 0;
    while ( srcFrameOffset < numIQFrames )
    {
      int numFramesToProcFromInput = (numIQFrames - srcFrameOffset);
      if ( numFramesToProcFromInput > (EXT_BLOCKLEN - mSampleBufferLenInFrames) )  // buffer limit?
        numFramesToProcFromInput = (EXT_BLOCKLEN - mSampleBufferLenInFrames);

      for ( int k = 0; k < numFramesToProcFromInput*2; ++k )
        mSampleBufferFlt[mSampleBufferLenInFrames*2 + k] = PRO_INTN_TO_FLOAT( puInputSamples[srcFrameOffset*2 + k], 16) * factor;
      mSampleBufferLenInFrames += numFramesToProcFromInput;

      if ( mSampleBufferLenInFrames == EXT_BLOCKLEN )
      {
        mExtIOCallbackPtr( EXT_BLOCKLEN, 0, 0.0F, &mSampleBufferFlt[0] );
        mSampleBufferLenInFrames = 0;
      }

      srcFrameOffset += numFramesToProcFromInput;
    }
  }
  else if ( fmt <= 4 ) // 24 bit
  {
    const LITTLE_INT24_T * puInputSamples = reinterpret_cast< const LITTLE_INT24_T*>(voidpvRawSampleData);
    const float factor = mpoSettings->mfGainCompensationFactor;

    int srcFrameOffset = 0;
    while ( srcFrameOffset < numIQFrames )
    {
      int numFramesToProcFromInput = (numIQFrames - srcFrameOffset);
      if ( numFramesToProcFromInput > (EXT_BLOCKLEN - mSampleBufferLenInFrames) )  // buffer limit?
        numFramesToProcFromInput = (EXT_BLOCKLEN - mSampleBufferLenInFrames);

      for ( int k = 0; k < numFramesToProcFromInput*2; ++k )
        mSampleBufferFlt[mSampleBufferLenInFrames*2 + k] = PRO_INTN_TO_FLOAT( int32_t( puInputSamples[srcFrameOffset*2 + k] ), 24 ) * factor;
      mSampleBufferLenInFrames += numFramesToProcFromInput;

      if ( mSampleBufferLenInFrames == EXT_BLOCKLEN )
      {
        mExtIOCallbackPtr( EXT_BLOCKLEN, 0, 0.0F, &mSampleBufferFlt[0] );
        mSampleBufferLenInFrames = 0;
      }

      srcFrameOffset += numFramesToProcFromInput;
    }
  }

#else

  if ( fmt <= 2 )
  {
    const int16_t * puInputSamples = (const int16_t*)(voidpvRawSampleData);
    int srcFrameOffset = 0;

    while ( srcFrameOffset < numIQFrames )
    {
      int numFramesToProcFromInput = (numIQFrames - srcFrameOffset);
      if ( numFramesToProcFromInput > (EXT_BLOCKLEN - mSampleBufferLenInFrames) )  // buffer limit?
        numFramesToProcFromInput = (EXT_BLOCKLEN - mSampleBufferLenInFrames);

      memcpy( &mSampleBuffer16[mSampleBufferLenInFrames*2], &puInputSamples[srcFrameOffset*2], numFramesToProcFromInput*2*sizeof(int16_t));
      mSampleBufferLenInFrames += numFramesToProcFromInput;

      if ( mSampleBufferLenInFrames >= EXT_BLOCKLEN )
      {
        //LOG_PRO(LOG_DEBUG, "Callback %d smp of %d buffered", EXT_BLOCKLEN, mSampleBufferLenInFrames);
        mExtIOCallbackPtr( EXT_BLOCKLEN, 0, 0.0F, &mSampleBuffer16[0] );
        mSampleBufferLenInFrames = 0;
      }

      srcFrameOffset += numFramesToProcFromInput;
    }
  }
  else if ( fmt <= 4 ) // 24 bit
  {
    const int8_t * puInputSamples = (const int8_t*)(voidpvRawSampleData);
    int srcFrameOffset = 0;

    while ( srcFrameOffset < numIQFrames )
    {
      int numFramesToProcFromInput = (numIQFrames - srcFrameOffset);
      if ( numFramesToProcFromInput > (EXT_BLOCKLEN - mSampleBufferLenInFrames) )  // buffer limit?
        numFramesToProcFromInput = (EXT_BLOCKLEN - mSampleBufferLenInFrames);

      memcpy( &mSampleBuffer24[mSampleBufferLenInFrames*2*3], &puInputSamples[srcFrameOffset*2*3], numFramesToProcFromInput*2*3);
      mSampleBufferLenInFrames += numFramesToProcFromInput;

      if ( mSampleBufferLenInFrames >= EXT_BLOCKLEN )
      {
        //LOG_PRO(LOG_DEBUG, "Callback %d smp of %d buffered", EXT_BLOCKLEN, mSampleBufferLenInFrames);
        mExtIOCallbackPtr( EXT_BLOCKLEN, 0, 0.0F, &mSampleBuffer24[0] );
        mSampleBufferLenInFrames = 0;
      }

      srcFrameOffset += numFramesToProcFromInput;
    }

  }
#endif

}

void RFspaceNetReceiver::receiveRFspaceNetSDRControlInfo(Info info)
{
  //enum class Info { FREQUENCY, ADC_SCALE, RF_GAIN, RF_FILTER, ADC_MODE, IQ_OUT_SAMPLERATE, UDP_PACKET_SIZE, UDP_INTERFACE, NAK };
  switch(info)
  {
    case Info::FREQUENCY:
    {
      bool pbOk = false;
      int64_t actualFrequency = rcv.getRcvFrequency(&pbOk);
      if(pbOk)
      {
        EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_LO );
        if(actualFrequency != mpoSettings->iFrequency || mHasDifferentLOFrequency)
        {
          mpoSettings->iFrequency = actualFrequency;
          LOG_PRO( LOG_ERROR, " RECEIVED CALLBACK WITH UNEXPECTED FREQUENCY : %ld Hz ", long(actualFrequency) );
          mHasDifferentLOFrequency = false;
        }
        else
          LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH EXPECTED FREQUENCY : %ld Hz ", long(actualFrequency) );
      }
      else
        LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH ERRONEOUS RETURN OF FREQUENCY : %ld Hz --> ERROR !", long(actualFrequency) );


      pbOk = false;
      const int64_t * actualFrequencyRange = rcv.getRcvFrequencyRanges(&pbOk);

      mRcvFrequencyRanges = actualFrequencyRange;
      mHasRcvFrequencyRanges = true;

      break;
    }
    case Info::ADC_SCALE:
      break;
    case Info::RF_GAIN:
    {
      bool bOk = false;
      int actualRFGain = int(rcv.getRFGain(&bOk));
      if(bOk)
      {
        switch(mReceiverMode)
        {
          case ReceiverMode::HF:
          {
            if(actualRFGain != mRFGaindB)
            {
              mRFGaindB = actualRFGain;
              mpoSettings->iControlValue = getAttIdx();

              mpoSettings->mfGainCompensationFactor = 1.0F / ( pow(10.0F,(mafActualAttenuationATTs[mpoSettings->iControlValue]/20.0F)) ); // log20 to linear --> lin = 10 ^ (db/20)  (voltage)

              LOG_PRO( LOG_ERROR, " RECEIVED CALLBACK WITH UNEXPECTED RF_GAIN : %d dB. New RFGainIdx : %d",  mRFGaindB,  mpoSettings->iControlValue);
              EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_ATT );
            }
            else
              LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH EXPECTED RF_GAIN : %d dB. New RFGainIdx : %d",  mRFGaindB,  mpoSettings->iControlValue);

            break;
          }
          case ReceiverMode::VUHF:
          {
            if(actualRFGain != mRFGaindB) // compare RFGain Values --> compatibility mode
            {
              mRFGaindB = actualRFGain;
              mpoSettings->iControlValue = getAttIdx();

              LOG_PRO( LOG_ERROR, " RECEIVED CALLBACK WITH UNEXPECTED COMPATIBILITY_GAIN_VALUE : %d",  mpoSettings->iControlValue);
              EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_ATT );
            }

            break;
          }
          case ReceiverMode::VUHF_EXP:
            break; // no implementation of VUHF_EXP in this Callback
          default:
            LOG_PRO( LOG_ERROR, "*** SWITCH CASE ERROR:  UNKNOWN RECEIVER MODE !");
        }
      }
      else
        if(mReceiverMode == ReceiverMode::HF)
          LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH ERRONEOUS RETURN OF RF_GAIN : %d --> ERROR !", actualRFGain );
        else
          LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH ERRONEOUS RETURN OF COMPATIBILITY_GAIN_VALUE --> ERROR !" );


      break;
    }

    case Info::VUHF_INFO:
    {
      /*bool bOk = false;
        RFspaceNetSDRControl::VUHFGains actualVUHFGainInfo = rcv.getVHFUHFGain(&bOk);

        if(bOk)
        {
          bool isAutoGainMode = actualVUHFGainInfo.isAGCMode;
          uint8_t LNAValue = actualVUHFGainInfo.LNAValue;
          uint8_t MixValue = actualVUHFGainInfo.MixerValue;
          uint8_t IFOutputValue = actualVUHFGainInfo.IFOutputValue;

          if(isAutoGainMode != mpoSettings->bUseVUHFAutoMode || LNAGainValue != mpoSettings->iLNAValue ||
              MixValue != mpoSettings->iMixerValue || IFOutputValue != mpoSettings->iIFOutputValue )
          {
            mpoSettings->iIFOutputValue = IFOutputValue;

            LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH UNEXPECTED MULTIGAIN_VALUE ! ");
            EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_ATT );
          }
        }*/
      break;
    }
    case Info::RF_FILTER:
      break;
    case Info::ADC_MODE:
    {
      bool bOk = false;
      float actualADGain = rcv.getADGain(&bOk);
      if(bOk)
      {
        if(actualADGain != mADGain) //Debug ==
        {
          mADGain = actualADGain;
          mpoSettings->iControlValue = getAttIdx();

          mpoSettings->mfGainCompensationFactor = 1.0F / ( pow(10.0F,(mafActualAttenuationATTs[mpoSettings->iControlValue]/20.0F)) ); // log20 to linear --> lin = 10 ^ (db/20)  (voltage)

          LOG_PRO( LOG_ERROR, " RECEIVED CALLBACK WITH UNEXPECTED AD_GAIN : %.1f . New GainIdx : %d ",  mADGain, mpoSettings->iControlValue);
          EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_ATT ); // Different Gains (RFGain and MGC) are summed up in Attenuation "ATT"
        }
        else
          LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH EXPECTED AD_GAIN : %.1f . New GainIdx : %d ",  mADGain, mpoSettings->iControlValue);

      }
      else
        LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH ERRONEOUS RETURN OF AD_GAIN : %.1f --> ERROR ! ", actualADGain);

      break;
    }
    case Info::IQ_OUT_SAMPLERATE:
    {
      bool bOk = false;
      uint32_t actualIQSmpRate = uint32_t(rcv.getIQOutSmpRate(&bOk));
      if(bOk)
      {
        if(actualIQSmpRate != mpoSettings->uiLastReportedSamplerate) //Debug ==
        {
          mpoSettings->uiSamplerate = actualIQSmpRate;
          mpoSettings->uiLastReportedSamplerate = mpoSettings->uiSamplerate;
          mpoSettings->iSampleRateIdx = getSmpRateIdx(actualIQSmpRate);
          mpoSettings->uiBandwidth = srate_bws[mpoSettings->iSampleRateIdx].bw;
          LOG_PRO( LOG_ERROR, " RECEIVED CALLBACK WITH UNEXPECTED IQ_OUT_SAMPLERATE : %u Hz", actualIQSmpRate);
          EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_SampleRate );
        }
        else
          LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH EXPECTED IQ_OUT_SAMPLERATE : %u Hz", actualIQSmpRate);
      }
      else
        LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH ERRONEOUS RETURN OF IQ_OUT_SAMPLERATE :  %d Hz -->  ERROR !", actualIQSmpRate );

      break;
    }
    case Info::UDP_PACKET_SIZE:
      break;
    case Info::UDP_INTERFACE:
      break;
    case Info::RCV_STATE:
    {
      mControlPingTime = 0;
      bool bOk = false;
      bool isUDPRunning = rcv.isUDPDataStreamRunning(&bOk);
      if(bOk)
      {
        mIsUDPRunning = isUDPRunning;
      }
      else
        LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH ERRONEOUS RETURN OF RCV_STATE -->  ERROR !" );

      break;
    }
    case Info::NAK:
      break;
    case Info::OPTIONS:
    {
      if(!mHasOptions)
        {
          bool pbOk = false;
          mReceiverOptions = rcv.getOptions(&pbOk);
          if(pbOk)
          {
            mHasVUHFFreqRange = mReceiverOptions.DownConverterBoard_Present;
            mHasOptions = true;
            LOG_PRO( LOG_DEBUG, " RECEIVED CALLBACK WITH OPTIONS !" );
          }
        }
      break;
    }

    default:
      break;
  }
}

bool RFspaceNetReceiver::openHW(Settings * poSettings)
{

  mpoSettings = poSettings;

  bool bTCPConnOK = rcv.connect( mpoSettings->acCtrlIP, mpoSettings->uCtrlPortNo, mpoSettings->nConnectTimeoutMillis );
  if ( !bTCPConnOK )
  {
    LOG_PRO( LOG_ERROR, "Error connecting to receiver %s:%u for control", mpoSettings->acCtrlIP, mpoSettings->uCtrlPortNo );
    return false;
  }

  poSettings->bIsTCPConnected = true;

  int samplerateIdx = poSettings->iSampleRateIdx;
  samplerateIdx = PROCLIP(samplerateIdx, 0, int(NUMEL(srate_bws)));

  poSettings->uiSamplerate = srate_bws[samplerateIdx].srate;
  LOG_PRO( LOG_DEBUG, "********************************** RFspaceNetReceiver::openHW() : SENDING SAMPLERATE FREQUENCY TO NETSDR: %dHz (idx: %d)", poSettings->uiSamplerate,  poSettings->iSampleRateIdx);

  poSettings->iSampleRateIdx = samplerateIdx;

  if(!poSettings->bIsVUHFRange)
  {
    poSettings->iControlValue = poSettings->iAttenuationIdx;
    mReceiverMode = ReceiverMode::HF;
  }
  else
  {
    if(!poSettings->bUseVUHFExpert)
    {
      poSettings->iControlValue = poSettings->iCompatibilityValue;
      mReceiverMode = ReceiverMode::VUHF;
    }
    else
    {
      const char * acGainControlMode = poSettings->acGainControlMode;


      if( strcmp(acGainControlMode, "Auto Gain LNA/MIXER" ) == 0 )
      {
        mGainControlMode = GainControlMode::AUTO;
        poSettings->iControlValue = poSettings->iIFOutputValue;
      }
      else if( strcmp(acGainControlMode, "Control LNA Value" ) == 0 )
      {
        mGainControlMode = GainControlMode::LNA_CTRL;
        poSettings->iControlValue = poSettings->iLNAValue;
      }
      else if( strcmp(acGainControlMode, "Control Mixer Value" ) == 0 )
      {
        mGainControlMode = GainControlMode::MIX_CTRL;
        poSettings->iControlValue = poSettings->iMixerValue;
      }
      else if( strcmp(acGainControlMode, "Control IF Output Value" ) == 0 )
      {
        mGainControlMode = GainControlMode::IF_CTRL;
        poSettings->iControlValue = poSettings->iIFOutputValue;
      }

      if(mGainControlMode == GainControlMode::AUTO)
        poSettings->bUseVUHFAutoMode = true;
      else
        poSettings->bUseVUHFAutoMode = false;

      mReceiverMode = ReceiverMode::VUHF_EXP;
    }
  }

  setGain(poSettings->iControlValue);
  rcv.setIQOutSmpRate( RFspaceNetSDRControl::IQOutSmpRate(poSettings->uiSamplerate) );
  rcv.setRFFilterSelection(RFspaceNetSDRControl::RfFilterSel::F_AUTO); // arg1 -> Filter Selection
  rcv.setUPDPacketSize( RFspaceNetSDRControl::UDPPacketSize::LARGE );
  //rcv.setRcvADAmplScale( 0.5 ); // arg1 -> Skalierung

  return true;
}

bool RFspaceNetReceiver::startHW(int64_t LOfreq)
{
  if(!mpoSettings->bIsSocketBound)
  {
    //initial binding and start streaming
    LOG_PRO(LOG_DEBUG, "Binding %s:%u for UDP data reception", mpoSettings->acDataIP, mpoSettings->uDataPortNo);
    bool bBindOK = udp.bindIfc( mpoSettings->acDataIP, mpoSettings->uDataPortNo );
    if ( !bBindOK )
    {
      LOG_PRO( LOG_ERROR, "Error binding to socket %s:%u for UDP data reception", mpoSettings->acDataIP, mpoSettings->uDataPortNo );
      return false;
    }
    rcv.setUDPInterface( mpoSettings->acDataIP, mpoSettings->uDataPortNo );

    mpoSettings->bIsSocketBound = true;
  }

  mSampleBufferLenInFrames = 0;

  rcv.setRcvFreq(LOfreq);
  mpoSettings->iFrequency = LOfreq;

  setSamplerate(mpoSettings->iSampleRateIdx);

  if (!mStartData)
  {
    //restart udp data (24bit/16bit) after "mStartUDPTimer" ms in "TimerProc" --> NetSDR sometimes streams wrong bitrate (although claiming it would stream the right bitrate).
    //--> give samplerate change (and sometimes therefore bitdepth change) more time.
    mStartUDPTimer = 200; //in ms
    mStartData = true;
  }

  return true;
}


void RFspaceNetReceiver::setHWLO( int64_t LOFreq )
{

  if( LOFreq >= VUHF_FREQUENCY && !mHasVUHFFreqRange && mHasOptions)
  {
    LOG_PRO( LOG_ERROR, "**** RECEIVER SEEMS TO HAVE NO DOWNCONVERTER HARDWARE FOR VUHF FREQUENCY RANGE --> SETTING FREQUENCY TO 10MHZ !");
    int64_t defaultFreq = 10*1000*1000;
    rcv.setRcvFreq( defaultFreq );
    mpoSettings->iFrequency = defaultFreq;
    return;
  }
  else
  {
    LOG_PRO( LOG_DEBUG, "RFspaceNetReceiver::setHWLO() : RECEIVED LOFreq : %ld Hz", long(LOFreq));

    int64_t actualSetLOFreq = LOFreq;

    //Set frequency according to available frequency ranges (information received from NetSDR)
    if(LOFreq < mpoSettings->iBand_minFreq || LOFreq > mpoSettings->iBand_maxFreq)
    {
      actualSetLOFreq =  PROCLIP( LOFreq, mpoSettings->iBand_minFreq, mpoSettings->iBand_maxFreq);
      mHasDifferentLOFrequency = true;
      LOG_PRO( LOG_ERROR, "RFspaceNetReceiver::setHWLO(%ld) : OUT OF AVAILABLE FREQUENCY RANGE! SETTING mpoSettings->iFrequency = %ld Hz", long(LOFreq), long(actualSetLOFreq));
    }
    else
      LOG_PRO( LOG_DEBUG, "RFspaceNetReceiver::setHWLO(%ld) : SETTING mpoSettings->iFrequency = %ld Hz", long(LOFreq), long(actualSetLOFreq));


    if(mpoSettings->iFrequency != actualSetLOFreq)
    {
      mpoSettings->iFrequency = actualSetLOFreq;
      rcv.setRcvFreq( mpoSettings->iFrequency );
    }
  }
}

void RFspaceNetReceiver::setGain( int idx )
{

  switch(mReceiverMode)
  {
    case ReceiverMode::HF:
    {
      idx = PROCLIP(idx, 0, int(NUMEL(mafAttenuationATTs)) );

      mpoSettings->iControlValue = idx;
      mpoSettings->mfGainCompensationFactor = 1.0F / ( pow(10.0F,(mafActualAttenuationATTs[idx]/20.0F)) ); // log20 to linear --> lin = 10 ^ (db/20)  (voltage)

      mADGain = mafAttenuationADGains[idx];
      mRFGaindB = int(mafAttenuationATTs[idx]);

      if(fabs(mADGain - 1.0) < 0.1)
        rcv.setADModes(mIsDithering, RFspaceNetSDRControl::ADGain::ADGain_1);
      else if(fabs(mADGain - 1.5) < 0.1)
        rcv.setADModes(mIsDithering, RFspaceNetSDRControl::ADGain::ADGain_1_5);

      rcv.setRFGain(RFspaceNetSDRControl::RfGain(mRFGaindB));
      LOG_PRO( LOG_DEBUG, "setting ADGain = %f, RFGain %d for Att Idx = %i", mADGain, mRFGaindB, mpoSettings->iControlValue);

      mpoSettings->iControlValue = idx;
      break;
    }
    case ReceiverMode::VUHF:
    {

      idx = PROCLIP( idx, 0, int(NUMEL(mVUHF_RFGainAttenuations)) );

      mpoSettings->iControlValue = idx; // idx: 0 -> Auto Gain ; 1 -> Low Gain ; 2 -> Medium Gain ; 3 -> High Gain
      mRFGaindB = int(mVUHF_RFGainAttenuations[idx]); // has ot be set here
      rcv.setRFGain(mVUHF_RFGainAttenuations[idx]); // in VUHF Compatibility Mode: Auto Gain -> -30dB ; Low Gain -> -20dB; Medium Gain -> -10dB; High Gain -> 0dB
      // --> So we can use madAttenuationATTs[] for VUHF Compatibility Settings
      LOG_PRO( LOG_DEBUG, "setting Compatibility Value Idx : %d", idx);

      mpoSettings->iControlValue = idx;
      break;
    }
    case ReceiverMode::VUHF_EXP:
    {

      int maxValue = int(NUMEL(mafVUHFMultiGainValues));
      idx = PROCLIP( idx, 0, maxValue );

      bool isAutoMode = mpoSettings->bUseVUHFAutoMode;
      int LNAValue = mpoSettings->iLNAValue;
      int MixValue = mpoSettings->iMixerValue;
      int IFOutputValue = mpoSettings->iIFOutputValue;

      switch(mGainControlMode)
      {
        case GainControlMode::AUTO:
          IFOutputValue = idx;
          break;
        case GainControlMode::LNA_CTRL:
          LNAValue = idx;
          break;
        case GainControlMode::MIX_CTRL:
          MixValue = idx;
          break;
        case GainControlMode::IF_CTRL:
          IFOutputValue = idx;
          break;
      }

      LNAValue = PROCLIP( LNAValue, 0, maxValue );
      MixValue = PROCLIP( MixValue, 0, maxValue );
      IFOutputValue = PROCLIP( IFOutputValue, 0, maxValue );

      rcv.setVUHFGains(isAutoMode, LNAValue , MixValue, IFOutputValue);

      LOG_PRO( LOG_DEBUG, "setting AutoMode:             %s ", isAutoMode  ? "ON" : "OFF" );
      if(!isAutoMode)
      {
        LOG_PRO( LOG_DEBUG, "setting LNA Value:            %d ", LNAValue );
        LOG_PRO( LOG_DEBUG, "setting Mixer Value:          %d ", MixValue );
      }
      LOG_PRO( LOG_DEBUG, "setting IFOutput Value:       %d ", IFOutputValue );

      mpoSettings->iControlValue = idx;
      break;
    }
  }
}

void RFspaceNetReceiver::setSamplerate( int idx )
{
  int numSamplerates = int(NUMEL(srate_bws));

  if ( idx < 0 || idx > numSamplerates )
  {
    int newIdx = PROCLIP(idx, 0, numSamplerates);
    LOG_PRO( LOG_ERROR, "RFspaceNetReceiver::setSamplerate(idx %d) : idx out of bounds --> Setting to %d", idx, newIdx);
    idx = newIdx;
  }

  mpoSettings->iSampleRateIdx = idx;
  mpoSettings->uiSamplerate = srate_bws[idx].srate;
  mpoSettings->uiBandwidth = srate_bws[idx].bw;

  bool bOk = false;

  bool bIsStreaming = rcv.isUDPDataStreamRunning(&bOk);

  if (bIsStreaming && bOk)
  {
    LOG_PRO( LOG_ERROR, "************************************** RFspaceNetReceiver::setSamplerate(): send ChangedSamplerate-Callback while running! *********************************");
    EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_SampleRate );
    mpoSettings->uiLastReportedSamplerate = mpoSettings->uiSamplerate;
    return;
  }

  // currently not streaming:
  LOG_PRO(LOG_DEBUG, "************************************** RFspaceNetReceiver::setSamplerate(%u): set Samplerate ************************", mpoSettings->uiSamplerate);
  rcv.setIQOutSmpRate(RFspaceNetSDRControl::IQOutSmpRate(mpoSettings->uiSamplerate));

  {
    int changeBitRangeSmpRateIdx = getMaxSmpRateIdx(mpoSettings->iBitDepthThresSamplerate);
    mNetSdrBitDepth = (mpoSettings->iSampleRateIdx <= changeBitRangeSmpRateIdx) ? 24 : 16;
#if NORMALIZE_DATA
    mExtHwSampleFormat = extHw_SampleFormat_FLT32;
    mExtHwBitDepth = 32;
#else
    mExtHwSampleFormat = (mNetSdrBitDepth == 24) ? extHw_SampleFormat_PCM24 : extHw_SampleFormat_PCM16;
    mExtHwBitDepth = (mNetSdrBitDepth == 24) ? 24 : 16;
#endif
    if (mLastReportedSampleFormat != mExtHwSampleFormat)
    {
      LOG_PRO(LOG_PROTOCOL, "SEND STATUS CHANGE TO SDR: NEW BIT DEPTH %d Bit.", mExtHwBitDepth);
      EXTIO_STATUS_CHANGE(mExtIOCallbackPtr, mExtHwSampleFormat);
      mLastReportedSampleFormat = mExtHwSampleFormat;
    }
  }

}

int64_t RFspaceNetReceiver::getHWLO( void )
{
  return mpoSettings->iFrequency;
}

int RFspaceNetReceiver::getAttIdx()
{
  int ret = -1;
  for(int idx = 0; idx <= miNumAttenuations-1; ++idx)
  {
    if( ( mafAttenuationATTs[idx] == mRFGaindB) && ( fabsf(mafAttenuationADGains[idx] - mADGain) < 0.1 ) )
      LOG_PRO( LOG_DEBUG, "******** RFspaceNetReceiver::getAttIdx() : ADGain: %.1f, RFGain: %.1f --> idx: %d", mafAttenuationATTs[idx], mafAttenuationADGains[idx] , idx);
    ret = idx;
    break;
  }
  return ret;
}

int RFspaceNetReceiver::getSmpRateIdx( uint32_t smpRate )
{
  int ret = -1;
  for(int idx = 0; idx < miNumSamplerates; ++idx)
  {
    int32_t delta = smpRate - srate_bws[idx].srate;
    if( -10 <= delta && delta <= 10 )   // +/- 10 Hz tolerance
    {
      LOG_PRO( LOG_DEBUG, "********* RFspaceNetReceiver::getSmpRateIdx(%u) --> idx: %d", smpRate, idx);
      ret = idx;
      break;
    }
  }

  if(ret == -1)
    LOG_PRO(LOG_ERROR, "********* RFspaceNetReceiver::getSmpRateIdx(%u) --> idx: ERROR !", smpRate);

  return ret;
}


int RFspaceNetReceiver::getMaxSmpRateIdx(uint32_t smpRate)
{
  int ret = -1;
  for (int idx = 0; idx < miNumSamplerates; ++idx)
  {
    if (smpRate >= srate_bws[idx].srate)
      ret = idx;
  }
  if (ret == -1)
    LOG_PRO(LOG_ERROR, "********* RFspaceNetReceiver::getMaxSmpRateIdx(%u) --> idx: ERROR !", smpRate);
  else
    LOG_PRO(LOG_DEBUG, "********* RFspaceNetReceiver::getMaxSmpRateIdx(%u) --> idx: %d", smpRate, ret);

  return ret;
}


const int64_t * RFspaceNetReceiver::getFrequencyRanges(int idx)
{
  if(mHasRcvFrequencyRanges)
  {
    switch(idx)
    {
      case 0:
        return &mRcvFrequencyRanges[0];
      case 1:
        return &mRcvFrequencyRanges[2];
      default :
        return nullptr;
    }
  }
  else
    return nullptr;
}

void RFspaceNetReceiver::TimerProc(uint16_t waitMs)
{
  if(mStartData)
  {
    mStartUDPTimer -= waitMs;

    if(mStartUDPTimer <= 0)
    {
      mStartData = false;
      setGain(mpoSettings->iControlValue);

      if (mNetSdrBitDepth==24)
      {
        LOG_PRO(LOG_DEBUG, "**************** RFspaceNetReceiver::setSamplerate(): START 24 BIT DATA STREAM *********************************");
        rcv.start24BitDataStream();
      }
      else if (mNetSdrBitDepth==16)
      {
        LOG_PRO(LOG_DEBUG, "**************** RFspaceNetReceiver::setSamplerate(): START 16 BIT DATA STREAM *********************************");
        rcv.start16BitDataStream();
      }
    }
  }

  mTime += waitMs;
  mTimeWithoutDataInMilliseconds += waitMs;
  mControlPingTime += waitMs;

  rcv.poll();
  udp.poll();

  if(mTime >= mWaitTimeInMilliseconds)
  {
    rcv.requestReceiverState();
    mTime = 0;

    if( (mControlPingTime >= mWaitTimeInMilliseconds ) )
    {
      LOG_PRO( LOG_DEBUG, "********* RFspaceNetReceiver::ThreadProc(%u ms/ThreadCycle): HAVEN'T RECEIVED CONTROL HEARTBEAT FOR %d MILLISECONDS ! ", waitMs, mControlPingTime);
    }
    else
      LOG_PRO( LOG_DEBUG, "********* RCV_STATE : CONTROL HEARTBEAT IS UP AND RUNNING" );

    if( mTimeWithoutDataInMilliseconds >= mUDPWaitTimeInMilliseconds )
    {
      LOG_PRO( LOG_DEBUG, "********* RFspaceNetReceiver::ThreadProc(%u ms/ThreadCycle): HAVEN'T RECEIVED UDP HEARTBEAT FOR %d MILLISECONDS ! ", waitMs, mTimeWithoutDataInMilliseconds);
    }
    else
      LOG_PRO( LOG_DEBUG, "********* RFspaceNetReceiver::ThreadProc(%u ms/ThreadCycle): UDP DATA IS UP AND RUNNING, TIME WITHOUT DATA: %u ms", waitMs, mTimeWithoutDataInMilliseconds);
  }

}


