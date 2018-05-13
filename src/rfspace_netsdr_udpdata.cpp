
#include "rfspace_netsdr_udpdata.h"

#include "ExtIO_Logging.h"
#include "procitec_replacements.h"


RFspaceNetSDRUdpData::RFspaceNetSDRUdpData( RFspaceNetSDRUdpData::CallbackIfc * pCB )
: mpSocket( new CPassiveSocket( CPassiveSocket::SocketTypeUdp ) )
, mSocket( *mpSocket )
, mpCallBack( pCB )
{
  resetReceiverData();
  mSocket.Initialize();
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
  muLastControlItemCode = 0xFFFFU;
  muExpectedSequenceNum = 0;
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

  const uint32_t ipSend = (ifc && ifc[0]) ? CSimpleSocket::GetIPv4AddrInfoStatic(ifc) : 0;
  if (!ipSend)
  {
    LOG_PRO(LOG_PROTOCOL, "Binding to all local interfaces without DATA_IP address '%s'", ifc);
    ifc = "";
  }
  else
  {
    LOG_PRO(LOG_DEBUG, "Binding will be on %u.%u.%u.%u for '%s'"
      , unsigned((ipSend >> 24) & 0xFF)
      , unsigned((ipSend >> 16) & 0xFF)
      , unsigned((ipSend >> 8) & 0xFF)
      , unsigned(ipSend & 0xFF)
      , ifc
      );
  }

  LOG_PRO(LOG_DEBUG, "Binding UDP to interface %s:%u ..", ifc, unsigned(portNo));
  bool bConnected = mSocket.Bind(ifc, portNo);
  if ( bConnected )
  {
    mSocket.SetNonblocking();
    uint32_t paramWinSize = 2 * 1024 * 1024;  // 2 MB
    uint32_t resWinSize = mSocket.SetReceiveWindowSize( paramWinSize );
    if ( resWinSize < paramWinSize )
    {
      LOG_PRO(LOG_ERROR, "Error setting UDP windows size to %u - result is %u", paramWinSize, resWinSize);
    }
  }
  else
  {
    LOG_PRO(LOG_ERROR, "Error binding UDP to interface %s:%u", ifc, unsigned(portNo));
  }

  return bConnected;
}


bool RFspaceNetSDRUdpData::poll( )
{

  if ( mSocket.IsSocketInvalid() )
    return false;

  while (1)
  {
    int rxLen = mSocket.Receive(MAX_UDP_LEN, &mRxBuffer[0]);

    if (rxLen > 0)
    {
      processReceivedDataMessage(rxLen);
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


void RFspaceNetSDRUdpData::processReceivedDataMessage(int rxLen)
{
  const void * wp = &mRxBuffer[0];
  const uint16_t uControlItemCode = READ_LITTLE_INT16( wp );
  wp = &mRxBuffer[2];
  const uint16_t uSequenceNum = READ_LITTLE_INT16( wp );

  const bool bSequenceNumError = (uSequenceNum != muExpectedSequenceNum);
  if (bSequenceNumError)
    LOG_PRO(LOG_PROTOCOL, "received %d bytes, sequenceNum %u - expected %u with control item code 0x%x"
      , rxLen, unsigned(uSequenceNum), unsigned(muExpectedSequenceNum), uControlItemCode);
  muExpectedSequenceNum = uSequenceNum + 1;
  if ( !muExpectedSequenceNum )
    ++muExpectedSequenceNum;

#define RXLEN_VALID(N)  ( (rxLen == 4 + N) ? "Valid" : "Invalid" )
  const bool bChangedControlItem = (muLastControlItemCode != uControlItemCode);
  muLastControlItemCode = uControlItemCode;
  if (bChangedControlItem)
  {
    switch (uControlItemCode)
    {
    case 0x8404:  LOG_PRO(LOG_DEBUG, "received UDP: 16 Bit I/Q. %s Large MTU size %d bytes", RXLEN_VALID(1024), rxLen); break;
    case 0x8204:  LOG_PRO(LOG_DEBUG, "received UDP: 16 Bit I/Q. %s Small MTU size %d bytes", RXLEN_VALID(512), rxLen);  break;
    case 0x85A4:  LOG_PRO(LOG_DEBUG, "received UDP: 24 Bit I/Q. %s Large MTU size %d bytes", RXLEN_VALID(1440), rxLen); break;
    case 0x8184:  LOG_PRO(LOG_DEBUG, "received UDP: 24 Bit I/Q. %s Small MTU size %d bytes", RXLEN_VALID(384), rxLen);  break;
    default:  LOG_PRO(LOG_PROTOCOL, "received UDP packet with unknown control item 0x%x size %d bytes", unsigned(uControlItemCode), rxLen);
    }
  }

  const void * p = reinterpret_cast<const void *>(&mRxBuffer[4]);
  switch ( uControlItemCode )
  {
  case 0x8404:  mpCallBack->receiveRfspaceNetSDRUdpData(1, p, 1024 /4, bSequenceNumError);  break;
  case 0x8204:  mpCallBack->receiveRfspaceNetSDRUdpData(2, p, 512 /4, bSequenceNumError);   break;
  case 0x85A4:  mpCallBack->receiveRfspaceNetSDRUdpData(3, p, 1440 /6, bSequenceNumError);  break;
  case 0x8184:  mpCallBack->receiveRfspaceNetSDRUdpData(4, p, 384 /6, bSequenceNumError);   break;
  default: ;
  }

}

