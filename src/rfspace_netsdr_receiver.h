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

  // zu speichernde Einstellungen
  class Settings
  {
  public:
    Settings(const char * defaultModel)
    {
      strcpy(acModel, defaultModel);      // NetSDR / CloudIQ
      strcpy(acCtrlIP, "192.168.8.100");  // "10.10.11.2"
      strcpy(acDataIP, "");               // ""   main PC: 192.168.8.100  PC2: 192.168.8.102
      uCtrlPortNo = uDataPortNo = 50000;  // 50000  / 50002
      iSampleRateIdx = 2;                 // 2: 32 kHz samplerate
      bUse16BitForAll = false;            // => use 24 Bit on low samplerates
      iAttenuationIdx = 6;                // 6: 0 dB  - no ATT  & no ADC Gain

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

      if (!strcmp(defaultModel, "NetSDR"))
      {
        iSampleRateIdx = 2; // => srate 32 kHz
        uiSamplerate = RFspaceNetReceiver::netsdr_srate_bws[iSampleRateIdx].srate;
        uiBandwidth = RFspaceNetReceiver::netsdr_srate_bws[iSampleRateIdx].bw;
        bUse16BitForAll = false;            // => use 24 Bit on low samplerates
      }
      else if (!strcmp(defaultModel, "CloudIQ"))
      {
        iSampleRateIdx = 4; // => srate 48 kHz
        uiSamplerate = RFspaceNetReceiver::cloudiq_srate_bws[iSampleRateIdx].srate;
        uiBandwidth = RFspaceNetReceiver::cloudiq_srate_bws[iSampleRateIdx].bw;
        bUse16BitForAll = true;             // => use 24 Bit on low samplerates
      }
    }

    char acModel[32];
    char acCtrlIP[64];
    uint16_t uCtrlPortNo;

    char acDataIP[64];
    uint16_t uDataPortNo;

    char acGainControlMode[64];

    int iSampleRateIdx;
    bool bUse16BitForAll;
    int iAttenuationIdx;

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


  static bool applyHwModel(const RFspaceNetReceiver::Settings &);
  static int getDefaultHWType();

  struct srate_bw
  {
    uint32_t  srate;
    uint32_t  bw;
    int maxBitDepth;
    const char * srateTxt;
  };

  static const int netsdr_default_srateIdx;
  static const RFspaceNetReceiver::srate_bw netsdr_srate_bws[];
  static const int cloudiq_default_srateIdx;
  static const RFspaceNetReceiver::srate_bw cloudiq_srate_bws[];
  static const struct srate_bw * srate_bws;

  static int miNumSamplerates;
  static int miDefaultSrateIdx;

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

  const int64_t * getFrequencyRanges( int idx );

  int getExtHwSampleFormat() const  { return mExtHwSampleFormat;  }
  int getExtHwBitDepth() const      { return mExtHwBitDepth;      }

  void TimerProc(int waitMs);

  void checkReportChangedSamplerates();

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

  int mTime;
  const int mWaitTimeInMilliseconds = 2000;
  const int mUDPWaitTimeInMilliseconds = 2000;
  const int mTimeBufferInMilliseconds = 10;
  int mTimeWithoutDataInMilliseconds;
  int mControlPingTime;
  bool mIsUDPRunning;
  uint16_t mNumCallbacks = 0;
  bool mHasLostTCPConntection;
  bool mHasVUHFFreqRange;
  bool mHasOptions;

  volatile bool mReportChangedSamplerates;

  int mLastReportedSampleFormat;
  unsigned mLastReportedFmt;

  volatile int mStartUDPTimer;
  volatile bool mStartData;

  RFspaceNetSDRControl::Options mReceiverOptions;

  ReceiverMode mReceiverMode;
  GainControlMode mGainControlMode;

  int mDeviceBitDepth;    // 16 / 24
  int mExtHwSampleFormat; // HDSDR enum != bit depth
  int mExtHwBitDepth;     // 16 / 24 / 32
};

