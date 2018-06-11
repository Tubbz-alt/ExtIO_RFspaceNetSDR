
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

// NetSDR A/D Converter frequency = 80 MHz
static const uint32_t NETSDR_ADC_FREQ = 80 * 1000 * 1000;
const int RFspaceNetReceiver::netsdr_default_srateIdx = 2;  // == 32 kHz

const RFspaceNetReceiver::srate_bw RFspaceNetReceiver::netsdr_srate_bws[] =
{
    { 12500, 10 KHZ, 24, "12.5 kHz" }  // 0
  , { 16 KHZ, 12 KHZ, 24, "16 kHz" }
  , { 32 KHZ, 25 KHZ, 24, "32 kHz" }    // 2
  , { 62500, 50 KHZ, 24, "62.5 kHz" }
  , { 80 KHZ, 64 KHZ, 24, "80 kHz" }
  , { 100 KHZ, 80 KHZ, 24, "100 kHz" }  // 5
  , { 125 KHZ, 100 KHZ, 24, "125 kHz" }
  , { 160 KHZ, 128 KHZ, 24, "160 kHz" }
  , { 200 KHZ, 160 KHZ, 24, "200 kHz" }
  , { 250 KHZ, 200 KHZ, 24, "250 kHz" }
  , { 312500, 250 KHZ, 24, "312.5 kHz" }  // 10
  , { 400 KHZ, 320 KHZ, 24, "400 kHz" }
  , { 500 KHZ, 400 KHZ, 24, "500 kHz" }
  , { 625 KHZ, 500 KHZ, 24, "625 kHz" }
  , { 800 KHZ, 640 KHZ, 24, "800 kHz" }
  , { 1000 KHZ, 800 KHZ, 24, "1 MHz" }  // 15
  , { 1250 KHZ, 1000 KHZ, 24, "1.25 MHz" }
  , { 1538461, 1200 KHZ, 16, "1.538 MHz" }
  , { 1666666, 1300 KHZ, 16, "1.666 MHz" }
  , { 1818181, 1400 KHZ, 16, "1.818 MHz" }
  , { 2000 KHZ, 1600 KHZ, 16, "2 MHz" }  // 20
};

// CloudIQ A/D Converter frequency = 122.88 MHz
static const uint32_t CLOUDIQ_ADC_FREQ = 122880 * 1000;
const int RFspaceNetReceiver::cloudiq_default_srateIdx = 0; // == 48 kHz

const RFspaceNetReceiver::srate_bw RFspaceNetReceiver::cloudiq_srate_bws[] =
{
  // , { 12 KHZ, 9 KHZ, 24, "12 kHz" }  // 0
  // , { 16 KHZ, 12 KHZ, 24, "16 kHz" }
  // , { 24 KHZ, 19 KHZ, 24, "24 kHz" }
  // , { 32 KHZ, 25 KHZ, 24, "32 kHz" }
    { 48 KHZ, 38 KHZ, 24, "48 kHz" }    // 0
  , { 64 KHZ, 51 KHZ, 24, "64 kHz" }
  , { 96 KHZ, 76 KHZ, 24, "96 kHz" }
  , { 128 KHZ, 102 KHZ, 24, "128 kHz" }
  , { 160 KHZ, 128 KHZ, 24, "160 kHz" }
  , { 192 KHZ, 153 KHZ, 24, "192 kHz" } // 5
  , { 256 KHZ, 204 KHZ, 24, "256 kHz" }
  , { 384 KHZ, 307 KHZ, 24, "384 kHz" }
  , { 512 KHZ, 409 KHZ, 24, "512 kHz" }
  , { 640 KHZ, 512 KHZ, 24, "640 kHz" }
  , { 768 KHZ, 614 KHZ, 24, "768 kHz" } // 10
  , { 960 KHZ, 768 KHZ, 24, "960 kHz" }
  , { 1024 KHZ, 819 KHZ, 24, "1024 kHz" }
  , { 1280 KHZ, 1024 KHZ, 24, "1280 kHz" }
  , { 1536 KHZ, 1228 KHZ, 16, "1536 kHz" }
  , { 1616842, 1290 KHZ, 16, "1616.8 kHz" }   // 15: divisor=76
  , { 1706666, 1365 KHZ, 16, "1706.6 kHz" }
  , { 1807058, 1445 KHZ, 16, "1807.1 kHz" }   // divisor=68
  //, { 1920 KHZ, 1536 KHZ, 16, "1920 kHz" }    // divisor=64 -> stuttering
  //, { 2048 KHZ, 1638 KHZ, 16, "2048 kHz" }    // divisor=60 -> stuttering
};

#undef KHZ

const RFspaceNetReceiver::srate_bw * RFspaceNetReceiver::srate_bws = netsdr_srate_bws;
int RFspaceNetReceiver::miNumSamplerates = (int)(NUMEL(RFspaceNetReceiver::netsdr_srate_bws));
int RFspaceNetReceiver::miDefaultSrateIdx = RFspaceNetReceiver::netsdr_default_srateIdx;

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

#define NORMALIZE_DATA 0


bool RFspaceNetReceiver::applyHwModel(const RFspaceNetReceiver::Settings & roSet)
{
  if (!strcmp(roSet.acModel, "NetSDR"))
  {
    if (srate_bws != netsdr_srate_bws)
    {
      srate_bws = netsdr_srate_bws;
      miNumSamplerates = (int)(NUMEL(netsdr_srate_bws));
      miDefaultSrateIdx = netsdr_default_srateIdx;
      return true;
    }
  }
  else if (!strcmp(roSet.acModel, "CloudIQ"))
  {
    if (srate_bws != cloudiq_srate_bws)
    {
      srate_bws = cloudiq_srate_bws;
      miNumSamplerates = (int)(NUMEL(cloudiq_srate_bws));
      miDefaultSrateIdx = cloudiq_default_srateIdx;
      return true;
    }
  }
  return false;
}


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

  mReportChangedSamplerates = false;

  mStartUDPTimer = 0;
  mStartData = false;

  mLastReportedFmt = 0;

  mGainControlMode = GainControlMode::AUTO;

  mDeviceBitDepth = 16;
#if NORMALIZE_DATA
  mLastReportedSampleFormat = mExtHwSampleFormat = extHw_SampleFormat_FLT32;
  mExtHwBitDepth = 32;
#else
  mLastReportedSampleFormat = mExtHwSampleFormat = (mDeviceBitDepth == 24) ? extHw_SampleFormat_PCM24 : extHw_SampleFormat_PCM16;
  mExtHwBitDepth = (mDeviceBitDepth == 24) ? 24 : 16;
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
    LOG_PRO(LOG_ERROR, "<-- RFspaceNetReceiver::receiveRfspaceNetSDRUdpData(): mExtIOCallbackPtr == NULL");
    return;
  }
  if (!rcv.dataStreamShouldRun())
    return;

  if (mLastReportedFmt != fmt || bSequenceNumError)
  {
    mSampleBufferLenInFrames = 0;
    if (!fmt || fmt > 4)
      LOG_PRO(LOG_ERROR, "<-- Received format %u of sample data from UDP is unknown!", mLastReportedFmt);

    mDeviceBitDepth = (fmt <= 2) ? 16 : 24;
#if NORMALIZE_DATA
    mExtHwSampleFormat = extHw_SampleFormat_FLT32;
    mExtHwBitDepth = 32;
#else
    mExtHwSampleFormat = (mDeviceBitDepth == 24) ? extHw_SampleFormat_PCM24 : extHw_SampleFormat_PCM16;
    mExtHwBitDepth = (mDeviceBitDepth == 24) ? 24 : 16;
#endif
    if (mLastReportedSampleFormat != mExtHwSampleFormat)
    {
      LOG_PRO(LOG_ERROR, "<-- SEND STATUS CHANGE TO SDR: NEW BIT DEPTH %d Bit. while running => ERROR!.", mExtHwBitDepth);
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
        mSampleBufferFlt[mSampleBufferLenInFrames*2 + k] = PRO_INTN_TO_FLOAT( puInputSamples[srcFrameOffset*2 + k], 24 ) * factor;
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
        if(actualFrequency != mpoSettings->iFrequency || mHasDifferentLOFrequency)
        {
          mpoSettings->iFrequency = actualFrequency;
          LOG_PRO(LOG_ERROR, "<-- RECEIVED CALLBACK WITH UNEXPECTED LO FREQUENCY : %ld Hz ", long(actualFrequency) );
          mHasDifferentLOFrequency = false;
          EXTIO_STATUS_CHANGE(mExtIOCallbackPtr, extHw_Changed_LO);
        }
        else
          LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH EXPECTED LO FREQUENCY : %ld Hz ", long(actualFrequency) );
      }
      else
        LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH ERRONEOUS RETURN OF LO FREQUENCY : %ld Hz --> ERROR !", long(actualFrequency) );

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

              LOG_PRO(LOG_ERROR, "<-- RECEIVED CALLBACK WITH UNEXPECTED RF_GAIN : %d dB. New RFGainIdx : %d",  mRFGaindB,  mpoSettings->iControlValue);
              EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_ATT );
            }
            else
              LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH EXPECTED RF_GAIN : %d dB. New RFGainIdx : %d",  mRFGaindB,  mpoSettings->iControlValue);

            break;
          }
          case ReceiverMode::VUHF:
          {
            if(actualRFGain != mRFGaindB) // compare RFGain Values --> compatibility mode
            {
              mRFGaindB = actualRFGain;
              mpoSettings->iControlValue = getAttIdx();

              LOG_PRO(LOG_ERROR, "<-- RECEIVED CALLBACK WITH UNEXPECTED COMPATIBILITY_GAIN_VALUE : %d",  mpoSettings->iControlValue);
              EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_ATT );
            }

            break;
          }
          case ReceiverMode::VUHF_EXP:
            break; // no implementation of VUHF_EXP in this Callback
          default:
            LOG_PRO(LOG_ERROR, "<-- UNKNOWN RECEIVER MODE !");
        }
      }
      else
        if(mReceiverMode == ReceiverMode::HF)
          LOG_PRO(LOG_DEBUG, " <-- RECEIVED CALLBACK WITH ERRONEOUS RETURN OF RF_GAIN : %d --> ERROR !", actualRFGain );
        else
          LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH ERRONEOUS RETURN OF COMPATIBILITY_GAIN_VALUE --> ERROR !" );

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

          LOG_PRO(LOG_ERROR, "<-- RECEIVED CALLBACK WITH UNEXPECTED AD_GAIN : %.1f . New GainIdx : %d ",  mADGain, mpoSettings->iControlValue);
          EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_ATT ); // Different Gains (RFGain and MGC) are summed up in Attenuation "ATT"
        }
        else
          LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH EXPECTED AD_GAIN : %.1f . New GainIdx : %d ",  mADGain, mpoSettings->iControlValue);

      }
      else
        LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH ERRONEOUS RETURN OF AD_GAIN : %.1f --> ERROR ! ", actualADGain);

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
          LOG_PRO(LOG_ERROR, "<-- RECEIVED CALLBACK WITH UNEXPECTED IQ_OUT_SAMPLERATE %u Hz. Expected was %u Hz", actualIQSmpRate, mpoSettings->uiLastReportedSamplerate);
          mpoSettings->uiSamplerate = actualIQSmpRate;
          mpoSettings->uiLastReportedSamplerate = mpoSettings->uiSamplerate;
          mpoSettings->iSampleRateIdx = getSmpRateIdx(actualIQSmpRate);
          mpoSettings->uiBandwidth = srate_bws[mpoSettings->iSampleRateIdx].bw;
          EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_SampleRate );
        }
        else
          LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH EXPECTED IQ_OUT_SAMPLERATE : %u Hz", actualIQSmpRate);
      }
      else
        LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH ERRONEOUS IQ_OUT_SAMPLERATE :  %d Hz -->  ERROR !", actualIQSmpRate );

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
      const bool isUDPRunning = rcv.isUDPDataStreamRunning(&bOk);
      const bool bChanged = (mIsUDPRunning != isUDPRunning);
      if (bOk && bChanged)
      {
        LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH RCV_STATE --> now UDP state changed to '%s'"
          , (isUDPRunning ? "running" : "stopped"));
        mIsUDPRunning = isUDPRunning;
      }
      else if (!bOk)
        LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH ERRONEOUS RETURN OF RCV_STATE -->  ERROR !" );


      const int bitDepth = rcv.getStreamBitDepth(&bOk);
      const bool isRunning = rcv.isUDPDataStreamRunning(&bOk);
      if (bOk && rcv.dataStreamShouldRun())
      {
        if (mDeviceBitDepth != bitDepth)
          LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH RCV_STATE --> RCV_STATE bitDepth %d differs from expected %d Bit @ %u", bitDepth, mDeviceBitDepth, mpoSettings->uiSamplerate);
      }

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
            LOG_PRO(LOG_DEBUG, "<-- RECEIVED CALLBACK WITH OPTIONS !" );
          }
        }
      break;
    }

    case Info::SET_TARGET:
    {
      bool bOk;
      const char * pTarget = rcv.getTargetName(&bOk);
      if (bOk)
      {
        strcpy(mpoSettings->acModel, pTarget);
        bool bAppliedTargetSrates = applyHwModel(*mpoSettings);
        if (bAppliedTargetSrates)
        {
          LOG_PRO(LOG_PROTOCOL, "<-- RECEIVED CALLBACK WITH TARGET '%s' - modified Samplerates", pTarget);
          mReportChangedSamplerates = true;
        }
        else
          LOG_PRO(LOG_PROTOCOL, "<-- RECEIVED CALLBACK WITH TARGET '%s' - unknown or same target", pTarget);
      }
      else
        LOG_PRO(LOG_ERROR, "<-- RECEIVED CALLBACK WITH unknown/erroneous TARGET");
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
    LOG_PRO( LOG_ERROR, "*** *** RFspaceNetReceiver::openHW(): Error connecting to receiver %s:%u for control", mpoSettings->acCtrlIP, mpoSettings->uCtrlPortNo );
    return false;
  }

  poSettings->bIsTCPConnected = true;

  int samplerateIdx = poSettings->iSampleRateIdx;
  samplerateIdx = PROCLIP(samplerateIdx, 0, miNumSamplerates - 1);

  poSettings->iSampleRateIdx = samplerateIdx;
  poSettings->uiSamplerate = srate_bws[samplerateIdx].srate;
  poSettings->uiBandwidth = srate_bws[samplerateIdx].bw;
  LOG_PRO(LOG_DEBUG, "*** *** RFspaceNetReceiver::openHW()");

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

  LOG_PRO(LOG_DEBUG, "*** *** * RFspaceNetReceiver::openHW() -> calling setGain()");
  setGain(poSettings->iControlValue);
  LOG_PRO(LOG_DEBUG, "*** *** * RFspaceNetReceiver::openHW() -> calling setRFFilterSelection()");
  rcv.setRFFilterSelection(RFspaceNetSDRControl::RfFilterSel::F_AUTO); // arg1 -> Filter Selection
  LOG_PRO(LOG_DEBUG, "*** *** * RFspaceNetReceiver::openHW() -> calling setUPDPacketSize()");
  rcv.setUPDPacketSize(RFspaceNetSDRControl::UDPPacketSize::LARGE);
  //rcv.setRcvADAmplScale( 0.5 ); // arg1 -> Skalierung
  LOG_PRO(LOG_DEBUG, "*** *** * RFspaceNetReceiver::openHW() -> calling setIQOutSmpRate(%u Hz)", poSettings->uiSamplerate);
  rcv.setIQOutSmpRate(poSettings->uiSamplerate);
  return true;
}

bool RFspaceNetReceiver::startHW(int64_t LOfreq)
{
  LOG_PRO(LOG_DEBUG, "*** *** RFspaceNetReceiver::startHW() called");
  if (!mpoSettings->bIsSocketBound)
  {
    //initial binding and start streaming
    const uint32_t ipSend = (mpoSettings->acDataIP[0]) ? CSimpleSocket::GetIPv4AddrInfoStatic(mpoSettings->acDataIP) : 0;
    const uint16_t uDataPort = (ipSend) ? mpoSettings->uDataPortNo : mpoSettings->uCtrlPortNo;

    LOG_PRO(LOG_DEBUG, "*** *** * Binding %s:%u for UDP data reception", mpoSettings->acDataIP, uDataPort);
    bool bBindOK = udp.bindIfc(mpoSettings->acDataIP, uDataPort);
    if ( !bBindOK )
    {
      LOG_PRO(LOG_ERROR, "*** *** * Error binding to socket %s:%u for UDP data reception", mpoSettings->acDataIP, uDataPort);
      return false;
    }
    if (ipSend)
      rcv.setUDPInterface(mpoSettings->acDataIP, uDataPort);

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
    mStartUDPTimer = 50;  //in ms
    LOG_PRO(LOG_DEBUG, "*** *** * RFspaceNetReceiver::startHW(): will command to start streaming in %d ms", mStartUDPTimer);
    mStartData = true;
  }

  return true;
}


void RFspaceNetReceiver::setHWLO( int64_t LOFreq )
{
  //if( LOFreq >= VUHF_FREQUENCY && !mHasVUHFFreqRange && mHasOptions)
  //{
  //  LOG_PRO( LOG_ERROR, "*** *** RFspaceNetReceiver::setHWLO(): RECEIVER SEEMS TO HAVE NO DOWNCONVERTER HARDWARE FOR VUHF FREQUENCY RANGE --> SETTING FREQUENCY TO 10MHZ !");
  //  int64_t defaultFreq = 10*1000*1000;
  //  rcv.setRcvFreq( defaultFreq );
  //  mpoSettings->iFrequency = defaultFreq;
  //  return;
  //}
  //else
  {
    LOG_PRO( LOG_DEBUG, "*** *** RFspaceNetReceiver::setHWLO(%ld Hz) called", long(LOFreq));
    int64_t actualSetLOFreq = LOFreq;

    //Set frequency according to available frequency ranges (information received from NetSDR)
    //if(LOFreq < mpoSettings->iBand_minFreq || LOFreq > mpoSettings->iBand_maxFreq)
    //{
    //  actualSetLOFreq =  PROCLIP( LOFreq, mpoSettings->iBand_minFreq, mpoSettings->iBand_maxFreq);
    //  mHasDifferentLOFrequency = true;
    //  LOG_PRO( LOG_ERROR, "*** *** * RFspaceNetReceiver::setHWLO(%ld) : OUT OF AVAILABLE FREQUENCY RANGE! SETTING mpoSettings->iFrequency = %ld Hz", long(LOFreq), long(actualSetLOFreq));
    //}
    //else
    //  LOG_PRO( LOG_DEBUG, "*** *** * RFspaceNetReceiver::setHWLO(%ld) : SETTING mpoSettings->iFrequency = %ld Hz", long(LOFreq), long(actualSetLOFreq));

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
      LOG_PRO( LOG_DEBUG, "*** *** setting ADGain = %f, RFGain %d for Att Idx = %i", mADGain, mRFGaindB, mpoSettings->iControlValue);

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
      LOG_PRO( LOG_DEBUG, "*** *** setting Compatibility Value Idx : %d", idx);

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

      LOG_PRO( LOG_DEBUG, "*** *** setting AutoMode:             %s ", isAutoMode  ? "ON" : "OFF" );
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
  const int numSamplerates = RFspaceNetReceiver::miNumSamplerates;

  if ( idx < 0 || idx > numSamplerates )
  {
    int newIdx = PROCLIP(idx, 0, numSamplerates);
    LOG_PRO( LOG_ERROR, "*** *** RFspaceNetReceiver::setSamplerate(idx %d) : idx out of bounds --> Setting to %d", idx, newIdx);
    idx = newIdx;
  }

  bool bOk = false;
  const bool bIsStreaming = rcv.isUDPDataStreamRunning(&bOk);

  if (bIsStreaming && bOk)
  {
    LOG_PRO( LOG_ERROR, "*** *** RFspaceNetReceiver::setSamplerate(): send ChangedSamplerate-Callback while running!");
    EXTIO_STATUS_CHANGE(mExtIOCallbackPtr , extHw_Changed_SampleRate);
    mpoSettings->uiLastReportedSamplerate = mpoSettings->uiSamplerate;
    return;
  }

  mpoSettings->iSampleRateIdx = idx;
  mpoSettings->uiSamplerate = srate_bws[idx].srate;
  mpoSettings->uiBandwidth = srate_bws[idx].bw;

  // currently not streaming:
  LOG_PRO(LOG_DEBUG, "*** *** * RFspaceNetReceiver::setSamplerate(%u): calling setIQOutSmpRate()", mpoSettings->uiSamplerate);
  rcv.setIQOutSmpRate(mpoSettings->uiSamplerate);

  {
    mDeviceBitDepth = (mpoSettings->bUse16BitForAll) ? 16 : srate_bws[mpoSettings->iSampleRateIdx].maxBitDepth;
    LOG_PRO(LOG_DEBUG, "*** *** * RFspaceNetReceiver::setSamplerate(%u): deviceBitDepth %d Bit", mpoSettings->uiSamplerate, mDeviceBitDepth);
#if NORMALIZE_DATA
    mExtHwSampleFormat = extHw_SampleFormat_FLT32;
    mExtHwBitDepth = 32;
#else
    mExtHwSampleFormat = (mDeviceBitDepth == 24) ? extHw_SampleFormat_PCM24 : extHw_SampleFormat_PCM16;
    mExtHwBitDepth = (mDeviceBitDepth == 24) ? 24 : 16;
#endif
    if (mLastReportedSampleFormat != mExtHwSampleFormat)
    {
      LOG_PRO(LOG_PROTOCOL, "*** *** * SEND STATUS CHANGE TO SDR: NEW BIT DEPTH %d Bit.", mExtHwBitDepth);
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
    if( fabsf( mafAttenuationATTs[idx] - mRFGaindB) < 0.1 && ( fabsf(mafAttenuationADGains[idx] - mADGain) < 0.1 ) )
      LOG_PRO( LOG_DEBUG, "*** *** RFspaceNetReceiver::getAttIdx() : ADGain: %.1f, RFGain: %.1f --> idx: %d", mafAttenuationATTs[idx], mafAttenuationADGains[idx] , idx);
    ret = idx;
    break;
  }
  return ret;
}

int RFspaceNetReceiver::getSmpRateIdx( uint32_t smpRate )
{
  int ret = -1;
  for (int idx = 0; idx < miNumSamplerates; ++idx)
  {
    if (smpRate >= srate_bws[idx].srate)
      ret = idx;
  }

  if (ret == -1)
  {
    ret = RFspaceNetReceiver::miDefaultSrateIdx;
    LOG_PRO(LOG_ERROR, "*** *** RFspaceNetReceiver::getSmpRateIdx(%u) --> idx: ERROR. Returning default idx %d = %u Hz!", smpRate, ret, srate_bws[ret].srate);
  }
  else if (smpRate == srate_bws[ret].srate)
    LOG_PRO(LOG_ERROR, "*** *** RFspaceNetReceiver::getSmpRateIdx(%u) --> idx: %d fits exactly", smpRate, ret);
  else
    LOG_PRO(LOG_ERROR, "*** *** RFspaceNetReceiver::getSmpRateIdx(%u) --> idx: %d with %u Hz is nearest", smpRate, ret, srate_bws[ret].srate);

  return ret;
}


const int64_t * RFspaceNetReceiver::getFrequencyRanges(int idx)
{
  if(mHasRcvFrequencyRanges)
  {
    switch(idx)
    {
    case 0:   return &mRcvFrequencyRanges[0];
    case 1:   return &mRcvFrequencyRanges[2];
    default:  return nullptr;
    }
  }
  else
    return nullptr;
}

void RFspaceNetReceiver::TimerProc(int waitMs)
{
  if(mStartData)
  {
    mStartUDPTimer -= waitMs;

    if(mStartUDPTimer <= 0)
    {
      mStartData = false;
      setGain(mpoSettings->iControlValue);

      if (mDeviceBitDepth == 24)
      {
        LOG_PRO(LOG_DEBUG, "*** *** RFspaceNetReceiver::TimerProc(): START DATA STREAM with 24 BIT");
        rcv.start24BitDataStream();
      }
      else if (mDeviceBitDepth == 16)
      {
        LOG_PRO(LOG_DEBUG, "*** *** RFspaceNetReceiver::TimerProc(): START DATA STREAM with 16 BIT");
        rcv.start16BitDataStream();
      }
    }
  }

  mTime += waitMs;
  mTimeWithoutDataInMilliseconds += waitMs;
  mControlPingTime += waitMs;

  rcv.poll();
  udp.poll();

  //if (mReportChangedSamplerates)  // -> checkReportChangedSamplerates()
  //  EXTIO_STATUS_CHANGE(mExtIOCallbackPtr, extHw_Stop);

  if(mTime >= mWaitTimeInMilliseconds)
  {
    rcv.requestReceiverState();
    mTime = 0;

    if( (mControlPingTime >= mWaitTimeInMilliseconds ) )
      LOG_PRO(LOG_DEBUG, "*** *** RFspaceNetReceiver::TimerProc(every %d ms): HAVEN'T RECEIVED CONTROL HEARTBEAT FOR %d MILLISECONDS!", waitMs, mControlPingTime);
    else
      LOG_PRO(LOG_DEBUG, "*** *** RFspaceNetReceiver::TimerProc(every %d ms): RCV_STATE : CONTROL HEARTBEAT IS UP AND RUNNING", waitMs );

    if (mStartData)
    {
      if (mTimeWithoutDataInMilliseconds >= mUDPWaitTimeInMilliseconds)
        LOG_PRO(LOG_DEBUG, "*** *** RFspaceNetReceiver::TimerProc(every %d ms): HAVEN'T RECEIVED UDP HEARTBEAT FOR %d MILLISECONDS!", waitMs, mTimeWithoutDataInMilliseconds);
      else
        LOG_PRO(LOG_DEBUG, "*** *** RFspaceNetReceiver::TimerProc(every %d ms): UDP DATA IS UP AND RUNNING", waitMs);
    }
  }

}


void RFspaceNetReceiver::checkReportChangedSamplerates()
{
  if (mReportChangedSamplerates)
  {
    EXTIO_STATUS_CHANGE(mExtIOCallbackPtr, extHw_Changed_SRATES);
    EXTIO_STATUS_CHANGE(mExtIOCallbackPtr, extHw_Changed_SampleRate);
    mReportChangedSamplerates = false;
  }
}

