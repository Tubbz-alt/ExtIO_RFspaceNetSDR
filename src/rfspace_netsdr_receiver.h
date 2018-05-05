#pragma once

#include "rfspace_netsdr_control.h"
#include "rfspace_netsdr_udpdata.h"

#include "common/functions/receiverlib/LC_ExtIO_Types.h"

#include "common/base/baselib/base/ByteOrder.h"
#include "common/base/baselib/base/ExtendedTypes.h"

#include <string.h>


class RFspaceNetReceiver
    : public RFspaceNetSDRUdpData::CallbackIfc
    , public RFspaceNetSDRControl::CallbackIfc
{
public:

  enum class ReceiverMode { HF, VUHF, VUHF_EXP };
  enum class GainControlMode { AUTO, LNA_CTRL, MIX_CTRL, IF_CTRL };

  static const int EXT_BLOCKLEN = 4096;   /* only multiples of 512 */

  static int getDefaultHWType();

  // zu speichernde Einstellungen
  class Settings
  {
  public:
    Settings()
    {
      strcpy(acCtrlIP, "10.10.11.2");
      strcpy(acDataIP, "");
      uCtrlPortNo = uDataPortNo = 50000;
      iSampleRateIdx = 0;
      iAttenuationIdx = 0;

      iBitDepthThresSamplerate = 0;


      iBand_minFreq = 0;
      iBand_maxFreq = 0;

      uiBandwidth = 0;
      uiSamplerate = 0;
      uiLastReportedSamplerate = 0;
      iFrequency = 0;
      iLNAValue = 0;
      iMixerValue = 0;
      iIFOutputValue = 0;
      iCompatibilityValue = 0;
      iControlValue = 0;

      bIsSocketBound = false;
      bIsVUHFRange = false;
      bUseVUHFExpert = false;
      bUseVUHFAutoMode = true;
      mfGainCompensationFactor = 1.0F;


    }

    char acCtrlIP[64];
    uint16_t uCtrlPortNo;

    char acDataIP[64];
    uint16_t uDataPortNo;

    char acGainControlMode[64];

    int iSampleRateIdx;
    int iAttenuationIdx;

    int32_t iBitDepthThresSamplerate;

    int64_t iBand_minFreq;
    int64_t iBand_maxFreq;

    uint32_t uiBandwidth;
    uint32_t uiSamplerate;
    uint32_t uiLastReportedSamplerate;
    int64_t iFrequency;
    int iLNAValue;
    int iMixerValue;
    int iIFOutputValue;
    int iCompatibilityValue;
    int iControlValue;


    bool bIsSocketBound;
    bool bIsTCPConnected;
    bool bIsVUHFRange;
    bool bUseVUHFExpert;
    bool bUseVUHFAutoMode;

    float mfGainCompensationFactor;

  };

  static const uint32_t maiSamplerates[];
  static const uint32_t maiBandwidths[];
  static const int miNumSamplerates;

  static const float mafAttenuationATTs[];
  static const float mafActualAttenuationATTs[];
  static const float mafVUHFCompatibilityGainValues[];
  static const float mafVUHFMultiGainValues[];
  static const float mafAttenuationADGains[];
  static const float mafAttenuationADGainsdB[];
  static const int miNumAttenuations;
  static const int miNumCompatibilityGains;
  static const int miNumMultiGains;
  static const RFspaceNetSDRControl::RfGain mVUHF_RFGainAttenuations[];


  static int getActualAttIdx(Settings * poSettings);
  static int getActualSmpRateIdx(Settings * poSettings);
  static int getIdxForAttValue( float fValue );

  RFspaceNetReceiver();
  ~RFspaceNetReceiver();

  void setExtIoCallback( pfnExtIOCallback ExtIOCallbackPtr );

  void receiveRfspaceNetSDRUdpData( const int fmt, const void * pvRawSampleData, const int numIQFrames, const int bitsPerSample ) override;
  void receiveRFspaceNetSDRControlInfo(Info info) override;

  void normalizeData(float *);

  bool openHW(Settings * poSettings);
  bool startHW(int64_t LOfreq);

  void setHWLO( int64_t LOfreq);
  void setGain( int idx);
  void setSamplerate( int idx);

  int64_t getHWLO( void );
  int getAttIdx( void );
  int getSmpRateIdx( uint32_t );
  const int64_t * getFrequencyRanges( int idx );

  void TimerProc(uint16_t waitMs);

  RFspaceNetSDRControl rcv;
  RFspaceNetSDRUdpData udp;

private:

  pfnExtIOCallback mExtIOCallbackPtr;

  Settings * mpoSettings;

  const int64_t VUHF_FREQUENCY = 40*1000*1000;

  int16_t  mSampleBuffer16[ EXT_BLOCKLEN * 2 ];
  uint8_t  mSampleBuffer24[ EXT_BLOCKLEN * 2 * 3 ];  // without int24_t ..
  float    mSampleBufferFlt[ EXT_BLOCKLEN * 2 ];  // when normalizing data ..

  int mSampleBufferLenInFrames;

  uint8_t mRcvFreqCallackCount;
  bool mIsDithering;

  float mADGain;
  bool mHasADGain;

  int mRFGaindB;
  bool mHasRFGain;

  uint8_t mCompatibilityValue;
  bool mHasCompatibilityValue;

  uint8_t mIFOutputValue;
  bool mHasIFOutputValue;

  const int64_t * mRcvFrequencyRanges;
  bool  mHasRcvFrequencyRanges;

  bool mHasDifferentLOFrequency;

  uint16_t mTime;
  const uint16_t mWaitTimeInMilliseconds = 2000;
  const uint16_t mUDPWaitTimeInMilliseconds = 2000;
  const uint16_t mTimeBufferInMilliseconds = 10;
  uint16_t mTimeWithoutDataInMilliseconds;
  uint16_t mControlPingTime;
  bool mIsUDPRunning;
  uint16_t mNumCallbacks = 0;
  bool mHasLostTCPConntection;
  uint8_t mOutBitSize;
  bool mHasVUHFFreqRange;
  bool mHasOptions;

  int mLastReportedBitDepth;

  volatile int mStartUDPTimer;
  volatile bool mStartData;

  RFspaceNetSDRControl::Options mReceiverOptions;

  uint8_t mChangeBitRangeSmpRateIdx;

  ReceiverMode mReceiverMode;
  GainControlMode mGainControlMode;

};

