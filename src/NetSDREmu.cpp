
#include "PassiveSocket.h"       // Include header for active socket object definition
#include "SimpleSocket.h"       // Include header for active socket object definition

#include "LC_ExtIO_Types.h"
#include "ExtIO_Logging.h"
#include "rfspace_netsdr_control.h"

#include <string.h>
#include <unistd.h>
#include <stdint.h>


#define MAX_PACKET 65536

#pragma pack(push, 1) // exact fit - no padding
struct LITTLE_INT24_T
{
  uint8_t lo;
  uint8_t mi;
  int8_t  hi;
};
#pragma pack(pop) //back to whatever the previous packing mode was


volatile bool gRunUDP = false;
volatile bool gPrepData = true;
volatile int gBitDepth = 16;
volatile unsigned gSampleRate = 32000;
const char * gszTarget = "NetSDR";


int NonExtIOCallback(int cnt, int status, float IQoffs, void *IQdata)
{
    (void)cnt;
    (void)IQoffs;
    switch (status)
    {
    case extHw_MSG_ERRDLG:
    case extHw_MSG_ERROR:
    case extHw_MSG_WARNING:
    case extHw_MSG_LOG:
    case extHw_MSG_DEBUG:
        fputs( (const char*)IQdata, stderr );
        return 0;
    default: ;
    }
    return 0;
}


class prepSignalData
{
public:
    uint8_t prepData[2048];
    size_t prepSize = 0;
    unsigned prepIQframes = 0;
    uint16_t seqNo = 0;
    int16_t *p16 = nullptr;
    int prepBitDepth = 24;

    void prepare( int bitDepth = 24, int mtuSize = 0 )
    {
        void * vp = &prepData[0];
        p16 = (int16_t*)vp;
        const int16_t amp = 16384;
        prepBitDepth = bitDepth;

        seqNo = 0;
        p16[1] = seqNo;

        if ( bitDepth == 16 )
        {
            if ( mtuSize == 0 )  // LARGE
            {
                p16[0] = 0x8404;
                prepSize = 4 + 1024;
                prepIQframes = 256; // = 1024 / 2 (16 bit)  / 2 (I/Q)
            }
            else                // SMALL
            {
                p16[0] = 0x8204;
                prepSize = 4 + 512;
                prepIQframes = 128; // = 512 / 2 (16 bit)  / 2 (I/Q)
            }
            for ( int k=0; k < 512; k += 4*2 )
            {
                p16[2+k+0] = amp;   p16[2+k+1] = 0;         // 0 deg
                p16[2+k+2] = 0;     p16[2+k+3] = amp;       // 90 deg
                p16[2+k+4] = -amp;  p16[2+k+5] = 0;         // 180 deg
                p16[2+k+6] = 0;     p16[2+k+7] = -amp;      // -90 deg
            }
        }
        else if ( bitDepth == 24 )
        {
            if ( mtuSize == 0 )  // LARGE
            {
                p16[0] = 0x85A4;
                prepSize = 4 + 1440;
                prepIQframes = 240; // = 1440 / 3 (24 bit)  / 2 (I/Q)
            }
            else                // SMALL
            {
                p16[0] = 0x8184;
                prepSize = 4 + 384;
                prepIQframes = 64;  // = 384 / 3 (24 bit)  / 2 (I/Q)
            }
            vp = &prepData[4];
            LITTLE_INT24_T * p24 = (LITTLE_INT24_T*)vp;
            const int32_t ampPlus = amp;
            const int32_t ampMinus = - int32_t(amp);
            LITTLE_INT24_T z = { 0, 0, 0 };
            LITTLE_INT24_T p = { uint8_t(ampPlus & 0xFFU ), uint8_t((ampPlus >> 8) & 0xFFU ), int8_t((ampPlus >> 16) & 0xFF ) };
            LITTLE_INT24_T m = { uint8_t(ampMinus & 0xFFU), uint8_t((ampMinus >> 8) & 0xFFU), int8_t((ampMinus >> 16) & 0xFF) };

            for ( int k=0; k < 480; k += 4*2 )
            {
                p24[k+0] = p;   p24[k+1] = z;   // 0 deg
                p24[k+2] = z;   p24[k+3] = p;   // 90 deg
                p24[k+4] = m;   p24[k+5] = z;   // 180 deg
                p24[k+6] = z;   p24[k+7] = m;   // -90 deg
            }
        }
    }

    inline void nextSeqNo()
    {
        ++seqNo;
        if (!seqNo)     // no zero!
            ++seqNo;

        p16[1] = seqNo;
    }

};


class RcvEmu
        : public RFspaceNetSDRControl::CallbackIfc
{
public:

    RcvEmu( CSimpleSocket * pClient )
        : mCtrl( pClient, this )
    {
    }

    virtual ~RcvEmu()
    {

    }

    virtual void receiveRFspaceNetSDRControlInfo(Info info) override
    {
        fprintf(stderr, "received %s\n", getText(info) );
        unsigned uReceiverStatus;
        bool bOk;
        switch (info)
        {
        case Info::FREQUENCY:
        case Info::ADC_SCALE:
        case Info::RF_GAIN:
        case Info::VUHF_INFO:
        case Info::RF_FILTER:
        case Info::ADC_MODE:
            break;
        case Info::IQ_OUT_SAMPLERATE:
            {
                unsigned srate = unsigned(mCtrl.getIQOutSmpRate(&bOk));
                fprintf(stderr, "command to use samplerate %u Hz\n", srate);
                gSampleRate = srate;
            }
            break;
        case Info::UDP_PACKET_SIZE:
            if ( mCtrl.getUDPPacketSize(&bOk) == RFspaceNetSDRControl::UDPPacketSize::LARGE )
                fprintf(stderr, "command to use LARGE MTU\n");
            else
                fprintf(stderr, "command to use SMALL MTU\n");
            break;
        case Info::UDP_INTERFACE:
            break;
        case Info::RCV_STATE:
            if ( mCtrl.isUDPDataStreamRunning(&bOk) )
            {
                int bitDepth = mCtrl.getStreamBitDepth(&bOk);
                fprintf(stderr, "command to START %d Bit stream..\n", bitDepth);
                gBitDepth = bitDepth;
                gPrepData = !gRunUDP;
                gRunUDP = true;
            }
            else
            {
                fprintf(stderr, "command to STOP stream\n");
                gRunUDP = false;
                gPrepData = true;
            }
            break;
        case Info::OPTIONS:
        case Info::NAK:
            break;
        case Info::REQ_TARGET:
            mCtrl.setTargetName(gszTarget);
            fprintf(stderr, "\n\nreport Target '%s'\n\n", gszTarget);
        default: ;
        }

    }

    RFspaceNetSDRControl mCtrl;
};


int main(int argc, char **argv)
{
    CPassiveSocket tcpSocket;
    CSimpleSocket *pClient = NULL;
    int samplerate = 32000;
    int bitDepth = 24;  // 16 / 24
    int mtuSize = 0;    // 0 == LARGE, 1 == SMALL
    unsigned ctrlPort = 50000;
    unsigned dataPort = 0;      // 0 == used same port as peer
    bool printUsage = false;

    gpfnExtIOCallbackPtr = NonExtIOCallback;
    SDRsupportsLogging = true;

    while (true)
    {
        int i = 1;
        if ( i < argc )
        {
            if ( !strcmp("-h", argv[i]) || !strcmp("--help", argv[i]) || !strcmp("/h", argv[i]) || !strcmp("/H", argv[i]) || !strcmp("/help", argv[i]) )
            {
                printUsage = true;
                break;
            }
            samplerate = atoi( argv[i] ) * 1000;
            fprintf(stderr, "parsed samplerate %d Hz from %s [kHz]\n", bitDepth, argv[i] );
        }
        if ( ++i < argc )
        {
            bitDepth = atoi( argv[i] );
            fprintf(stderr, "parsed bitDepth %d from %s\n", bitDepth, argv[i] );
        }

        if ( ++i < argc )
        {
            mtuSize = atoi( argv[i] );
            fprintf(stderr, "parsed mtuSize = %d = %s from %s\n", mtuSize, (mtuSize ? "SMALL":"LARGE"), argv[i] );
        }
        if ( ++i < argc )
        {
            ctrlPort = atoi( argv[i] );
            fprintf(stderr, "parsed ctrlPort %u from %s\n", ctrlPort, argv[i] );
        }
        if ( ++i < argc )
        {
            dataPort = atoi( argv[i] );
            fprintf(stderr, "parsed dataPort %u from %s\n", dataPort, argv[i] );
        }
        if ( ++i < argc )
        {
            gszTarget = argv[i];
            fprintf(stderr, "using target %s\n", argv[i] );
        }
        break;
    }

    if ( samplerate < 8000 )
        printUsage = true;
    if ( bitDepth != 16 && bitDepth != 24 )
        printUsage = true;
    if ( bitDepth != 16 && bitDepth != 24 )
        printUsage = true;
    if ( mtuSize != 0 && mtuSize != 1 )
        printUsage = true;
    if ( ctrlPort <= 0 || ctrlPort > 65535 )
        printUsage = true;
    if ( dataPort < 0 || dataPort > 65535 )
        printUsage = true;

    if ( printUsage )
    {
        fprintf(stderr, "usage: NetSDRemu <srate /kHz> <bitDepth 16/24> <mtuSize 0=LARGE/1=SMALL> <ctrlPort =50000> <dataPort = 0> <target ='NetSDR'/'CloudIQ'>\n" );
        return 10;
    }

    // LARGE MTU:
    // 0x04 0x84: COMPLEX 16 bit Data:   1024 bytes = 256 16 bit I/Q data samples
    // 0xa4 0x85: COMPLEX 24 bit Data:   1440 bytes = 240 24 bit I/Q data samples

    // SMALL MTU:
    // 0x04 0x82: COMPLEX 16 bit Data:   512 bytes = 128 16 bit I/Q data samples
    // 0x84 0x81: COMPLEX 24 bit Data:   384 bytes =  64 24 bit I/Q data samples

    tcpSocket.Initialize();
    tcpSocket.Listen( 0, ctrlPort ); // NULL (not "127.0.0.1") to allow testing with remotes

    fprintf(stderr, "\n%s. Local: %s:%u   Peer: %s:%u\n"
          , ( tcpSocket.IsServerSide() ? "Local Listener is Server" : "Local Listener is Client" )
          , tcpSocket.GetLocalAddr(), (unsigned)tcpSocket.GetLocalPort()
          , tcpSocket.GetPeerAddr(), (unsigned)tcpSocket.GetPeerPort()
          );

    while (true)
    {
        fprintf(stderr, "\n\nwaiting for incoming connection ..\n");
        if ((pClient = tcpSocket.Accept()) != NULL)
        {
            CSimpleSocket &ctrlConn = *pClient;
            ctrlConn.WaitUntilReadable(200);
            ctrlConn.WaitUntilWritable(200);
            ctrlConn.SetNonblocking();

            RcvEmu emu( pClient );

            CSimpleSocket dataSocket( CSimpleSocket::SocketTypeUdp );
            dataSocket.Initialize();

            fprintf(stderr, "\n%s. Local: %s:%u   "
                  , ( ctrlConn.IsServerSide() ? "Local Accepted is Server" : "Local Accepted is Client" )
                  , ctrlConn.GetLocalAddr(), (unsigned)ctrlConn.GetLocalPort()  // GetLocalAddr() / GetPeerAddr() return same static buffers!
                  );
            fprintf(stderr, "Peer: %s:%u\n"
                  , ctrlConn.GetPeerAddr(), (unsigned)ctrlConn.GetPeerPort()    // GetLocalAddr() / GetPeerAddr() return same static buffers!
                  );

            const char * udpTargetHost = ctrlConn.GetPeerAddr();
            uint16 udpTargetPort = dataPort ? dataPort : ctrlConn.GetLocalPort();

            bool bBindOK = dataSocket.Bind(0, udpTargetPort);
            fprintf(stderr, "bound UDP 'connection' to port %u %s\n", unsigned(udpTargetPort), (bBindOK ? "successful":"failed") );

            fprintf(stderr, "preparing UDP 'connection' to %s:%u\n", udpTargetHost, unsigned(udpTargetPort) );
            bool bConnOK = dataSocket.Open( udpTargetHost, udpTargetPort );
            if ( !bConnOK )
            {
                fprintf(stderr, "error 'connecting' UDP to peer!\n");
                return 10;
            }

            prepSignalData prep;
            prep.prepare( bitDepth, mtuSize );
            fprintf(stderr, "prepared data packet: size %u bytes, #I/Q frames %u\n", unsigned(prep.prepSize), prep.prepIQframes);

            int numMillisToSleep = samplerate > 500000 ? 10 : 100;
            int framesPerSleep = (samplerate * numMillisToSleep) / 1000;
            fprintf(stderr, "will transmit %d frames every %d msecs\n", framesPerSleep, numMillisToSleep);

            while (true)
            {
                bool bRetCtrlPoll = emu.mCtrl.poll();
                if (!bRetCtrlPoll)
                    break;

                usleep( numMillisToSleep * 1000 );
                if ( gRunUDP )
                {
                    if ( gPrepData || gBitDepth != prep.prepBitDepth )
                    {
                        prep.prepare( gBitDepth, mtuSize );
                        numMillisToSleep = gSampleRate > 500000 ? 10 : 100;
                        framesPerSleep = (gSampleRate * numMillisToSleep) / 1000;
                        fprintf(stderr, "will transmit %d frames every %d msecs\n", framesPerSleep, numMillisToSleep);
                    }
                    for ( int k = 0; k < framesPerSleep; k += prep.prepIQframes )
                    {
                        dataSocket.Send( prep.prepData, prep.prepSize );
                        prep.nextSeqNo();
                    }
                    fprintf(stderr, ".");
                }
            }

            ctrlConn.Close();
            dataSocket.Close();

            //delete pClient;
        }
    }

    tcpSocket.Close();

    return 1;
}


