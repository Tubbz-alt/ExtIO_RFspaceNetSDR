#pragma once

#include <string>
#include <cstdint>
#include <math.h>

#include "SimpleSocket.h"
#include "PassiveSocket.h"

// forward declaration
class CPassiveSocket;


class RFspaceNetSDRUdpData
{
public:

  class CallbackIfc
  {
  public:
    virtual ~CallbackIfc() { }
    virtual void receiveRfspaceNetSDRUdpData(const unsigned fmt, const void * pvRawSampleData, const int numIQFrames, const bool bSequenceNumError) = 0;
  };

  RFspaceNetSDRUdpData( RFspaceNetSDRUdpData::CallbackIfc * pCB = nullptr );
  ~RFspaceNetSDRUdpData();

  bool bindIfc(const char * ifc, uint16_t portNo);
  bool poll();
  void close();

private:

  void processReceivedDataMessage(int rxLen);
  void resetReceiverData();

  static const int MAX_UDP_LEN = 64 * 1024;  // 64 kB max
  uint8_t   mRxBuffer[MAX_UDP_LEN];

  CPassiveSocket * mpSocket;
  CPassiveSocket & mSocket;
  CallbackIfc * mpCallBack;
  uint16_t muLastControlItemCode;
  uint16_t muExpectedSequenceNum;
};

