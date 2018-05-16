#pragma once

#include "rfspace_netsdr_control.h"
#include "rfspace_netsdr_udpdata.h"

#include "LC_ExtIO_Types.h"

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
      strcpy(acCtrlIP, "192.168.8.101");  // "10.10.11.2"
      strcpy(acDataIP, "");               // ""   main PC: 192.168.8.100  PC2: 192.168.8.102
      uCtrlPortNo = uDataPortNo = 50000;  // 50000  / 50002
      iSampleRateIdx = 2;                 // 2: 32 kHz samplerate
      iAttenuationIdx = 6;                // 6: 0 dB  - no ATT  & no ADC Gain

      iBitDepthThresSamplerate = 1333333; // 1333.333 kHz == 80 MHz / 60

      iBand_minFreq = 0;
      iBand_maxFreq = 34 * 1000L * 1000L; // 0 - 34 MHz

      uiBandwidth = 10000;
      uiSamplerate = 12500;
      uiLastReportedSamplerate = 0;
      iFrequency = 7 * 1000 * 1000;
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

      nConnectTimeoutMillis = 1000;
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

    int nConnectTimeoutMillis;
  };

  static const struct srate_bw {
    uint32_t  decim;
    uint32_t  srate;
    uint32_t  srateval;
    uint32_t  bw;
    const char * srateTxt;
  } srate_bws[];

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

  void receiveRfspaceNetSDRUdpData(const unsigned fmt, const void * pvRawSampleData, const int numIQFrames, const bool bSequenceNumError) override;
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
  int getMaxSmpRateIdx(uint32_t);
  const int64_t * getFrequencyRanges( int idx );

  int getExtHwSampleFormat() const  { return mExtHwSampleFormat;  }
  int getExtHwBitDepth() const      { return mExtHwBitDepth;      }

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
  bool mHasVUHFFreqRange;
  bool mHasOptions;

  int mLastReportedSampleFormat;
  unsigned mLastReportedFmt;

  volatile int mStartUDPTimer;
  volatile bool mStartData;

  RFspaceNetSDRControl::Options mReceiverOptions;

  ReceiverMode mReceiverMode;
  GainControlMode mGainControlMode;

  int mNetSdrBitDepth;
  int mExtHwSampleFormat; // HDSDR enum != bit depth
  int mExtHwBitDepth;
};

