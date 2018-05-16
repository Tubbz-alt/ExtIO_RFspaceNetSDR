#pragma once

#include <string>
#include <cstdint>
#include <vector>


// forward declaration
class CSimpleSocket;

//*********************TODOs*************************************************************************************
//***************************************************************************************************************

class RFspaceNetSDRControl
{
public:

  class CallbackIfc
   {
   public:

    enum class Info { FREQUENCY, ADC_SCALE, RF_GAIN, VUHF_INFO, RF_FILTER, ADC_MODE, IQ_OUT_SAMPLERATE, UDP_PACKET_SIZE, UDP_INTERFACE, RCV_STATE , OPTIONS, NAK };

    virtual ~CallbackIfc() { }
    virtual void receiveRFspaceNetSDRControlInfo(Info info) = 0;
   };

  // 4.1.4 Hardware/Firmware Versions ***************
  enum class HwFw : uint8_t {BOOT_CODE = 0, APP_FW = 1, HW_VER = 2, FPGA_CONF = 3};

  // 4.1.5 Status/Error Code ***************
  enum class RcvStatusCode : unsigned {
    RCV_IDLE = 0xB
        , RCV_BUSY = 0xC
        , RCV_BOOT_IDLE = 0xE
        , RCV_BOOT_BUSY_PROG = 0xF
        , RCV_AD_OVERLOAD = 0x20
        , RCV_PROG_ERROR = 0x80
  };

  // 4.2.6 RF Gain ***************
  enum class RfGain : int8_t { ATT_0DB = 0x00, ATT_10DB = -10 /*0xF6*/, ATT_20DB = -20 /*0xEC*/, ATT_30DB = -30 /*0xE2*/};

  // 4.2.7 RF Filter Selection ***************
  enum class RfFilterSel : uint8_t {
    F_AUTO = 0
        , F_0_TO_1800kHz = 1
        , F_1800_TO_2800kHz = 2
        , F_2800_TO_4000kHz = 3
        , F_4000_TO_5500kHz = 4
        , F_5500_TO_7000kHz = 5
        , F_7000_TO_10000kHz = 6
        , F_10000_TO_14000kHz = 7
        , F_14000_TO_20000kHz = 8
        , F_20000_TO_28000kHz = 9
        , F_28000_TO_35000kHz = 10
        , F_BYPASS = 11
        , F_NOPASS = 12
        , F_MANPATH = 13};

  // 4.2.8 A/D Modes ***************
  enum class ADGain : uint8_t { ADGain_1_5 = 0x02, ADGain_1 = 0x00 };

  // A/D Converter frequency = 80 MHz
  static const uint32_t ADC_FREQ = 80 * 1000 * 1000;

  // 4.2.9I/Q Output Data Sample Rate ***************
  // this enum has to be in sync with RFspaceNetReceiver::srate_bws[]
  enum class IQOutSmpRate : uint32_t {
          SR_12kHz    = ADC_FREQ / 6400
        , SR_16kHz    = ADC_FREQ / 5000
        , SR_32kHz    = ADC_FREQ / 2500
        , SR_62kHz    = ADC_FREQ / 1280
        , SR_80kHz    = ADC_FREQ / 1000
        , SR_100kHz   = ADC_FREQ / 800
        , SR_125kHz   = ADC_FREQ / 640
        , SR_160kHz   = ADC_FREQ / 500
        , SR_200kHz   = ADC_FREQ / 400
        , SR_250kHz   = ADC_FREQ / 320
        , SR_312kHz   = ADC_FREQ / 256
        , SR_400kHz   = ADC_FREQ / 200
        , SR_500kHz   = ADC_FREQ / 160
        , SR_625kHz   = ADC_FREQ / 128
        , SR_800kHz   = ADC_FREQ / 100
        , SR_1000kHz  = ADC_FREQ / 80
        , SR_1250kHz  = ADC_FREQ / 64
        , SR_1538kHz  = ADC_FREQ / 52
        , SR_1666kHz  = ADC_FREQ / 48
        , SR_1818kHz  = ADC_FREQ / 44
        , SR_2000kHz  = ADC_FREQ / 40
  };

  // 4.4.2 Data Output Packet Size ***************
  enum class UDPPacketSize : uint8_t { SMALL = 1, LARGE = 0 };

  // 4.4.4 CW Startup Message ***************
  enum class WpmMinMax : uint8_t { WPM_MAX = 30, WPM_MIN = 10 };
  enum class CWFreq : uint8_t { CW_500Hz = 5, CW_1000Hz = 10, CW_1500Hz = 15};

  struct Options;
  struct RFFilterSelection;
  //struct UDP_IPandPortNum;
  struct CWStartup;
  struct VUHFGains;


  RFspaceNetSDRControl(RFspaceNetSDRControl::CallbackIfc * pCB = nullptr);
  RFspaceNetSDRControl(CSimpleSocket * pSocket, RFspaceNetSDRControl::CallbackIfc * pCB = nullptr);
  ~RFspaceNetSDRControl();

  bool connect(const char * ip, unsigned portNo, int nConnectTimeoutMillis = -1 );
  bool close();
  bool connected() const;


  // GET-FUNCTIONS
  const char * getTargetName(bool *pbOk) const;
  const char * getSerialNumber(bool *pbOk) const;

  const Options & getOptions(bool *pbOk) const;

  const RfGain getRFGain(bool *pbOk) const;
  const VUHFGains & getVHFUHFGain(bool *pbOk) const;
  const uint8_t  getRFFilterSelection(bool *pbOk) const;
  const char * getUDPIpAddress(bool *pbOk) const;
  const unsigned getUDPPortNum(bool *pbOk) const;
  const CWStartup & getCWStartup(bool *pbOk) const;

  const IQOutSmpRate getIQOutSmpRate(bool *pbOk) const;
  const UDPPacketSize getUDPPacketSize(bool *pbOk) const;

  const int64_t getRcvFrequency(bool *pbOk) const;
  const int64_t * getRcvFrequencyRanges(bool *pbOk) const;
  const bool getDithering(bool *pbOk) const;
  const float getADGain(bool *pbOk) const;
  const uint8_t getRcvChannelSetup(bool *pbOk) const;
  const float getRcvADAmplScale(bool *pbOk) const;
  const uint32_t & getRcvStatus(bool *pbOk) const;
  const uint32_t getHWFWNumber(bool *pbOk) const;
  const uint32_t getProductId(bool *pbOk) const;
  const bool isUDPDataStreamRunning(bool *pbOk) const;
  const int getStreamBitDepth(bool *pbOk) const;


  // SET-FUNCTIONS
  void setRcvChnSetup( );
  void setRcvFreq( int64_t rcvFreqHz );
  void setRcvADAmplScale( float rcvScale );
  void setRFGain( RfGain );
  void setVUHFGains( bool isAutoMode, int LNAGain, int MixGain, int IFOutGain );
  void setRFFilterSelection( RfFilterSel );
  void setADModes( bool isDithering, ADGain );
  void setIQOutSmpRate ( IQOutSmpRate );
  void setUPDPacketSize ( UDPPacketSize );
  void setUDPInterface( const char * ip, uint16_t portNum );
  void setCWStartup ( uint8_t wpm, CWFreq , const char * asciiMessage);

  void start24BitDataStream();
  void start16BitDataStream();
  void stopDataStream();

  void requestReceiverState();
  void requestOptions();
  // POLL
  bool poll( );

  // GET-TEXT FUNCTIONS
  const char * getHWFWText( uint8_t id ) const;
  const char * getStatusText( uint8_t code ) const;
  const char * getTypeText() const;
  const char * getControlItemText(uint16_t uControlItemCode) const;
  const char * getRcvChannelSetupText( uint8_t setup ) const;
  const char * getRcvChannelText( uint8_t chn ) const;

private:

  // PRINT-TEXT FUNCTIONS
  static void printText(const Options &, const char * pacPreText = "received from device:");
  static void printUDPStateText(const bool &, const char * pacPreText = "received from device:");
  static void printBitDepthText(const int &, const char * pacPreText = "received from device:");
  static void printRcvFreqText(const uint64_t &, const char * pacPreText = "received from device:");
  static void printFilterText(const int &, const char * pacPreText = "received from device:");
  static void printText(const bool &, const char * pacPreText = "received from device:");
  static void printText(const float &, const char * pacPreText = "received from device:");
  static void printText(const IQOutSmpRate &, const char * pacPreText = "received from device:");
  static void printText(const UDPPacketSize &, const char * pacPreText = "received from device:");
  static void printText(const RfGain &, const char * pacPreText = "received from device:");
  static void printUDPIPText(const char * ip, const char * pacPreText = "received from device:");
  static void printUDPPortNumText(const unsigned, const char * pacPreText = "received from device:");
  static void printText(const CWStartup &, const char * pacPreText = "received from device:");
  static void printText(const uint8_t &, const char * pacPreText = "received from device:");
  static void printText(const VUHFGains &, const char * pacPreText = "received from device:");

public:

  // STRUCT-DECLARATIONS
  //**************************************************************

  struct Options
  {
    Options();
    void clear();

    bool Sound_Enabled;
    bool ReflockBoard_Present;
    bool DownConverterBoard_Present;
    bool UpConverterBoard_Present;
    bool X2Board_Present;
    unsigned MainBoardVariant;
    unsigned ReflockBoardVariant;
    unsigned DownConverterBoardVariant;
    unsigned UpConverterBoardVariant;
  };



  //**************************************************************

  struct RFFilterSelection
  {
    RFFilterSelection();
    void clear();

    RfFilterSel eChn1FiltSel;
    RfFilterSel eChn2FiltSel;

  };


  //**************************************************************


  struct CWStartup
  {
    CWStartup();
    void clear();


    uint8_t cw_wpm;
    CWFreq eCwFreq;
    uint8_t asciiMessage[10];

  };

  //**************************************************************

  struct VUHFGains
  {
    VUHFGains();
    void clear();

    bool isAGCMode;
    uint8_t LNAGainLevel;
    uint8_t MixerGainLevel;
    uint8_t IFOutputGainLevel;
  };



private:

  enum class MsgType : uint8_t { SET_CTRL_ITEM = 0x00, REQ_CTRL_ITEM = 0x20,  REQ_CTRL_RANGE = 0x40};

  // 4.2.1 Receiver State ***************
  enum class UDPstate : unsigned { STOP_UDP_DATA = 0x00010000, START_UDP_DATA = 0x00020000 };
  enum class ADCstate : unsigned { SET_RCV_REAL_AD_DATA = 0x00000000, SET_IQ_BASEBAND_DATA = 0x80000000 };
  enum class BitDepthState : unsigned { SET_24BIT_MODE = 0x00008000, SET_16BIT_MODE = 0x00000000 };
  enum class CaptureMode : unsigned { SET_CONTIGUOUS_MODE = 0x00000000, SET_FIFO_MODE = 0x00000100, SET_HWTRIGGER_MODE = 0x00000300 };

  // 4.2.2 Receiver Channel Setup ***************
  enum class RcvChannelSetup : unsigned {
          SCM_CTRLCH1 = 0 // Single Channel Mode. Use Channel 1 for control. Real FIFO mode is supported. (default mode on power up)
        , SCM_CTRLCH2 = 1 // Single Channel Mode. Use Channel 2 for control(requires X2 Hardware Option). Real FIFO mode is supported.
        // Uses X2 board A/D if present.
        , SCM_IQSUM = 2   //Single Channel Mode. Use Channel 1 or 2 for control(requires X2 Hardware Option). I/Q Data output is the sum
        //of channel 1 and Channel 2 data.
        , SCM_IQDIFF = 3  //Single Channel Mode. Use Channel 1 or 2 for control(requires X2 Hardware Option). I/Q Data output is the
        //difference of channel 1 minus Channel 2 data.
        , DCM_MAINAD = 4  //Dual Channel with single A/D RF Path using main A/D. Channel 1 and 2 control separate NCO frequency only.
        //Uses common clock and output sample rate(Bandwidth). Data packets interleave channel 1 and channel 2 I/Q Data.
        //Real mode is not supported. Main RF Filters are shared with both channels. Both channels start and stop together.
        , DCM_X2AD = 5    //Dual Channel with single A/D RF Path using X2 A/D(requires X2 Hardware Option). Channel 1 and 2 control
        //separate NCO frequency only. Uses common clock and output sample rate(Bandwidth). Data packets interleave channel
        //1 and channel 2 I/Q Data. Real mode is not supported. X2 board RF Filters are shared with both channels. Both
        //channels start and stop together.
        , DCM_DUALAD = 6  //Dual Channel with dual A/D RF Path(requires X2 Hardware Option). Channel 1 and 2 control separate NCO
        //frequency, RF Filters, and attenuators. Uses common clock and output sample rate(Bandwidth). Data packets interleave
        //channel 1 and channel 2 I/Q Data. Real mode is not supported. Both channels start and stop together.
  };

  // 4.2.3 Receiver Frequency ***************
  enum class NCOChannel : uint8_t { CHN1 = 0x00, CHN2 = 0x02, CHN12 = 0xFF};

  // 4.2.8 A/D Modes ***************
  enum class ADDither : uint8_t { DITH_ON = 0x01, DITH_OFF = 0x00 };
  enum class ADMask : uint8_t { DITHMASK = 0x01, GAINMASK = 0x02 };

  const uint8_t channel = uint8_t( NCOChannel::CHN1 );
  const RcvChannelSetup chnMode = RcvChannelSetup::SCM_CTRLCH1;
  const MsgType msgType = MsgType::REQ_CTRL_ITEM;

  struct RcvFrequencies;
  struct ADModes;
  struct RcvState;
  struct RFGains;

  void requestTargetName();
  void requestTargetSerialNum();
  void requestInterfaceVersion();
  void requestHwFwVersions(HwFw);
  void requestStatus();
  void requestProductId();
  void requestRcvFrequency();
  void requestRcvFrequencyRanges();
  void requestRcvADAmplScale();
  void requestRFGain();
  void requestVUHFGains();
  void requestRFFilterSelection();
  void requestADMode();
  void requestUDPInterface();

  // SET FUNCTIONS
  void setRcvState( UDPstate , ADCstate , BitDepthState, CaptureMode , uint32_t uFifoNumCaptures = 0 );

  // PARSE-DATA FUNCTIONS
  void parseRcvStateBytes( );
  void parseOptionBits( );
  void parseRcvFrequencies( );
  void parseADModes( );
  void parseCWStartup();
  void resetReceiverData( );
  void parseVUHFGains( );

  bool processReceivedControlMessage(bool isValid);

  uint8_t   mRxBuffer[64 * 1024 + 4];
  int32_t   mRxLen = 0;           // number of valid received bytes in mRxBuffer[]
  int32_t   mRxMsgLen = 2;        // length of current message in bytes
  int32_t   mRxType = -1;
  bool    mRxMsgHeader = true;

  //**************************************************************

    struct RcvFrequencies
    {
      RcvFrequencies();
      void clear();

      int64_t Chn1_Freq;
      int64_t Chn2_Freq;

      int64_t Chn1_Bnd1_MinFreq;
      int64_t Chn1_Bnd1_MaxFreq;
      int64_t Chn1_Bnd1_VCO_DwnConvFreq;
      int64_t Chn2_Bnd1_MinFreq;
      int64_t Chn2_Bnd1_MaxFreq;
      int64_t Chn2_Bnd1_VCO_DwnConvFreq;

      int64_t Chn1_Bnd2_MinFreq;
      int64_t Chn1_Bnd2_MaxFreq;
      int64_t Chn1_Bnd2_VCO_DwnConvFreq;
      int64_t Chn2_Bnd2_MinFreq;
      int64_t Chn2_Bnd2_MaxFreq;
      int64_t Chn2_Bnd2_VCO_DwnConvFreq;
    };

  //**************************************************************

  struct ADModes
  {
    ADModes();
    void clear();

    ADDither eChn1AD_Dither;
    ADDither eChn2AD_Dither;
    ADGain eChn1AD_Gain;
    ADGain eChn2AD_Gain;

  };

  //**************************************************************

  struct RcvState
  {
    RcvState();
    void clear();

    ADCstate eADCstate;
    UDPstate eUDPstate;
    BitDepthState eBitDepth;
    CaptureMode eCaptureMode;
    uint32_t uFIFOByteCaptNum;
  };

  //**************************************************************

  struct RFGains
  {
    RFGains();
    void clear();

    RfGain eChn1RFGain;
    RfGain eChn2RFGain;

  };
  ////////////////////////////////////////////////////////

  // MEMBER VARIABLES

  bool mHasNAKMessage = false;
  char mTargetName[32];
  bool mHasTargetName = false;

  char mSerialNumber[32];
  bool mHasSerialNumber = false;

  uint32_t mProductId;
  bool    mHasProductId = false;

  Options mOptions;
  bool mHasOptions = false;

  RcvState mRcvState;
  bool mHasRcvState = false;

  RcvFrequencies mRcvFrequencies;
  bool mHasRcvFrequency = false;

  int64_t mRcvFrequencyRanges[4];
  bool mHasRcvFrequencyRanges = false;

  RFGains mRFGains;
  bool mHasRFGains = false;

  VUHFGains mVUHFGains;
  bool mHasVUHFGains = false;

  RFFilterSelection mRFFilterSelection;
  bool mHasRFFilterSelection = false;

  ADModes mADModes;
  bool mHasADModes = false;

  unsigned mUDPPortNum;
  char mUDPIpAddress[32];
  bool mHasIPandPortNum = false;


  CWStartup mCwStartup;
  bool mHasCWStartup = false;

  unsigned mReceiverStatus;
  bool    mHasReceiverStatus = false;

  unsigned mInterfaceVersion;
  bool    mHasInterfaceVersion = false;

  unsigned mHardwareFirmwareVersion;
  bool    mHasHardwareFirmwareVersion = false;

  RcvChannelSetup mRcvChnSetup;
  bool    mHasRcvChnSetup = false;

  float mRcvADAmplScale;
  bool mHasRcvADAmplScale = false;

  IQOutSmpRate mIQOutSmpRate;
  bool mHasIQOutSmpRate = false;

  UDPPacketSize mUDPPacketSize;
  bool mHasUDPPacketSize = false;


  uint8_t mHexArray[10];

public:
  CSimpleSocket * mpSocket;
  CSimpleSocket & mSocket;

private:
  CallbackIfc * mpCallBack = nullptr;
};


const char * getText( RFspaceNetSDRControl::RfGain);
const char * getFilterText( int filterSel );
const char * getText( RFspaceNetSDRControl::ADGain );
const char * getText( RFspaceNetSDRControl::IQOutSmpRate );
const char * getText( RFspaceNetSDRControl::UDPPacketSize );
const char * getText( RFspaceNetSDRControl::CWFreq );
const char * getText( uint8_t chnSetup );
const char * getText( RFspaceNetSDRControl::CallbackIfc::Info);

