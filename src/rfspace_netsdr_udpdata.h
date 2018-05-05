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
    virtual void receiveRfspaceNetSDRUdpData( const int fmt, const void * pvRawSampleData, const int numIQFrames, const int bitsPerSample ) = 0;
  };

  RFspaceNetSDRUdpData( RFspaceNetSDRUdpData::CallbackIfc * pCB = nullptr );
  ~RFspaceNetSDRUdpData();


  void close();

  bool poll( );
  bool processReceivedDataMessage(unsigned rxLen);
  bool bindIfc( const char * ifc, uint16_t portNo );

private:

  void resetReceiverData();

  static const int MAX_UDP_LEN = 64 * 1024;  // 64 kB max
/*
  static const int UDP_MEMORY_SIZE = 10;
  static const int SEQUENCENUM = pow(2,16)-1; //16bit -> 2^16-1
*/

  uint8_t   mRxBuffer[MAX_UDP_LEN];


public:
  CPassiveSocket * mpSocket;
  CPassiveSocket & mSocket;
private:
  CallbackIfc * mpCallBack = nullptr;
  uint16_t uExpectedSequenceNum = 0;

/*  uint8_t mUDPMemoryMatrix[MAX_UDP_LEN][UDP_MEMORY_SIZE];
  uint8_t * mpaSecNum[SEQUENCENUM];
  uint8_t mMemoryMatrixRow;*/

};

