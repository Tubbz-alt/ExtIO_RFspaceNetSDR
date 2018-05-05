
#include "rfspace_netsdr_udpdata.h"

#include "common/base/baselib/base/ByteOrder.h"
#include "common/base/baselib/base/ProStd.h"
#include "common/base/baselib/base/ProLogging.h"


#define LENGTH_MASK 0x1FFF     // mask for message length
#define TYPE_MASK 0xE0         // mask for upper byte of header
#define PRINT_RECEIVED_DATA   0
#define PRINT_RECEIVED_TYPE   0


RFspaceNetSDRUdpData::RFspaceNetSDRUdpData( RFspaceNetSDRUdpData::CallbackIfc * pCB )
: mpSocket( new CPassiveSocket( CPassiveSocket::SocketTypeUdp ) )
, mSocket( *mpSocket )
, mpCallBack( pCB )
{
  resetReceiverData();
  mSocket.Initialize();

  //mMemoryMatrixRow = 0;
}


RFspaceNetSDRUdpData::~RFspaceNetSDRUdpData()
{
  if ( mSocket.IsSocketInvalid() )
    return;

  close();

  delete mpSocket;

}

void RFspaceNetSDRUdpData::resetReceiverData()
{
  uExpectedSequenceNum = 0;
}


void RFspaceNetSDRUdpData::close()
{
  resetReceiverData();

  if ( mSocket.IsSocketInvalid() )
    return;

  mSocket.Close();
}


bool RFspaceNetSDRUdpData::bindIfc( const char * ifc, uint16_t portNo )
{
  if ( mSocket.IsSocketPeerOpen() )
  {
    LOG_PRO( LOG_ERROR, "Socket already connected ! --> ProAssert(0)");
    ProAssert(0); // close your socket before connecting!
    return false;
  }

  resetReceiverData();

  bool bConnected = mSocket.Bind(ifc, portNo);
  if ( bConnected )
  {
    mSocket.SetNonblocking();
    uint32_t paramWinSize = 2 * 1024 * 1024;  // 2 MB
    uint32_t resWinSize = mSocket.SetReceiveWindowSize( paramWinSize );
    if ( resWinSize < paramWinSize )
    {
      LOG_PRO( LOG_DEBUG, "Error setting UDP windows size to %u - result is %u", paramWinSize, resWinSize);
    }

  }

  return bConnected;
  return true;
}


bool RFspaceNetSDRUdpData::poll( )
{

  if ( mSocket.IsSocketInvalid() )
    return false;

  while (1)
  {
     int rx = mSocket.Receive( MAX_UDP_LEN, &mRxBuffer[0] );

    if ( rx > 0 )
    {
      processReceivedDataMessage(rx);
      continue;
    }
    else if ( mSocket.IsSocketInvalid() )
    {
      LOG_PRO( LOG_ERROR, "mSocket invalid" );
      return false;
    }

    return true;

  }
}


bool RFspaceNetSDRUdpData::processReceivedDataMessage(unsigned rxLen)
{
  if ( rxLen < 2 + 2 + 384 )
    return false;

  void * wp = &mRxBuffer[0];
  const uint16_t uControlItemCode = READ_LITTLE_INT16( wp );
  wp = &mRxBuffer[2];
  const uint16_t uSequenceNum = READ_LITTLE_INT16( wp );
  bool bProcessed = true; // assume so

  if ( uSequenceNum != uExpectedSequenceNum )
  {
    LOG_PRO( LOG_DEBUG, "received 0x%x as control Item Code [16Bit --> 0x8404 || 24Bit --> 0x85A4]", uControlItemCode);
    LOG_PRO( LOG_DEBUG, "received %d bytes, sequenceNum %u - expected %u", rxLen, unsigned(uSequenceNum), unsigned(uExpectedSequenceNum) );
  }
  uExpectedSequenceNum = uSequenceNum + 1;
  if ( !uExpectedSequenceNum )
    ++uExpectedSequenceNum;

/*  mpaSecNum[uSequenceNum] = &mUDPMemoryMatrix[0][mMemoryMatrixRow]; // Pointer[uSequenceNum] zeigt auf entsprechenden UDP-Burst
  memcpy( &mUDPMemoryMatrix[0][mMemoryMatrixRow++], &mRxBuffer[0], rxLen); // Daten werden in Speicher-Array geschrieben

  if(mMemoryMatrixRow >= UDP_MEMORY_SIZE)
    mMemoryMatrixRow = 0;*/


#if PRINT_RECEIVED_DATA
  {
    LOG_PRO( LOG_DEBUG, "received %d bytes, sequenceNum %u", rxLen, unsigned(uSequenceNum) );
    //for ( unsigned i = 0; i < 4 ; ++i )
    for ( unsigned i = 4; i < rxLen ; ++i )
      if ( mRxBuffer[i] )
        LOG_PRO( LOG_DEBUG, "%d: 0x%x", i, unsigned(mRxBuffer[i]) );
  }
#endif

  switch ( uControlItemCode )
  {
    case 0x8404:  // 4.5.1.1 Real 16 Bit FIFO Data
    //case 0x8404:  // 4.5.1.2 Complex 16 Bit Data
      if ( rxLen == 4 + 1024 )
      {
#if PRINT_RECEIVED_TYPE
        LOG_PRO( LOG_DEBUG, "4.5.1.2 Complex 16 Bit Data" );
#endif
        const int16_t * p = reinterpret_cast< const int16_t * >( &mRxBuffer[4] );
        const int numIQpairs = 1024 / 4;  // I/Q - Paare
        if ( mpCallBack )
          mpCallBack->receiveRfspaceNetSDRUdpData( 1, p, numIQpairs, 16 );
      }
      else
        bProcessed = false;
      break;
    case 0x8204:  // 4.5.1.1 Real 16 Bit FIFO Data Small MTU
    //case 0x8204:  // 4.5.1.2 Complex 16 Bit Data Small MTU
      if ( rxLen == 4 + 512 )
      {
#if PRINT_RECEIVED_TYPE
        LOG_PRO( LOG_DEBUG, "4.5.1.2 Complex 16 Bit Data Small MTU" );
#endif
        const int16_t * p = reinterpret_cast< const int16_t * >( &mRxBuffer[4] );
        const int numIQpairs = 512 / 4;  // I/Q - Paare

        if ( mpCallBack )
          mpCallBack->receiveRfspaceNetSDRUdpData( 2, p, numIQpairs, 16 );
      }
      else
        bProcessed = false;
      break;

    case 0x85A4:  // 4.5.1.3 Complex 24 Bit Data
      if ( rxLen == 4 + 1440 )
      {
#if PRINT_RECEIVED_TYPE
        LOG_PRO( LOG_DEBUG, "4.5.1.3 Complex 24 Bit Data" );
#endif
        const void * p = reinterpret_cast< const void * >( &mRxBuffer[4] );
        const int numIQpairs = 1440 / 6;  // I/Q - Paare
        if ( mpCallBack )
          mpCallBack->receiveRfspaceNetSDRUdpData( 3, p, numIQpairs, 24 );
      }
      else
        bProcessed = false;
      break;
    case 0x8184:  // 4.5.1.3 Complex 24 Bit Data Small MTU
      if ( rxLen == 4 + 384 )
      {
#if PRINT_RECEIVED_TYPE
        LOG_PRO( LOG_DEBUG, "4.5.1.3 Complex 24 Bit Data Small MTU" );
#endif
        const void * p = reinterpret_cast< const void * >( &mRxBuffer[4] );
        const int numIQpairs = 384 / 6; // I/Q - Paare
        if ( mpCallBack )
          mpCallBack->receiveRfspaceNetSDRUdpData( 3, p, numIQpairs, 24 );
      }
      else
        bProcessed = false;
      break;

    default:
      bProcessed = false;
  }

  return bProcessed;
}

