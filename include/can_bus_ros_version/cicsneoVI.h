/*
Copyright (c) 2014 Intrepid Control Systems, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
//#include "can_bus_ros_version/ftdilib.h"
#include <ftdi.h>
#include <string.h>
#include <wchar.h>
#include <wctype.h>
#include "can_bus_ros_version/icsnVC40.h"
#include <stdlib.h>
#include "can_bus_ros_version/OThread.h"
#include "can_bus_ros_version/OEvent.h"
#include "can_bus_ros_version/OCriticalSection.h"
#include "can_bus_ros_version/coremini.h"

#define NEOVI_READBUFFSIZE 131072
#define MAX_INCOMING_MSG_BUFFER_SIZE 19999
#define MAX_NETWORK_COLLECTION_SIZE (100)

class cicsneoVI {
private:
    bool m_bDeviceOpen;
    struct ftdi_context m_ftdic;

    class CNeoVIReadThread : public OThread {
    public:
        CNeoVIReadThread(void) {}
        ~CNeoVIReadThread(void) {}

        unsigned long Run();
    };

    friend class CNeoVIReadThread;

public:
    cicsneoVI();
    ~cicsneoVI();

    unsigned char* bReadData;

    OEvent* m_pRxMsgAvailableEvent;
    pthread_t threadid;
    OCriticalSection* m_pcsReceivedData;

    bool m_bneoShutdown;

    unsigned long m_lReadDataIndexIn;
    unsigned long m_lReadDataIndexOut;
    unsigned char m_bReadDataOverflow;
    unsigned long m_lReadDataCount;

    unsigned char* bTempPacketBufferHost;
    bool m_bReceivedGoodPacketSinceOpen;

    unsigned char m_bNetworkIDs[MAX_NETWORK_COLLECTION_SIZE];

    unsigned long m_iRxThreadAffinity;
    int m_iRxThreadPriority;

    unsigned long m_InComingMsgCount; //needs a better more informative name

    icsSpyMessage* m_pMsgBuffer;
    icsSpyMessage mMsg;

    CNeoVIReadThread* m_pRxThread;

    bool OpenDevice(NeoDevice* pDevice);
    bool CloseDevice(void);
    int Write(unsigned char* buf, int size);
    int Read(unsigned char* buf, int size);
    bool SetLatencyTimer(unsigned char latency);
    bool PurgeBuffers(void);
    void Reset(void);
    bool TransmitMessages(icsSpyMessage* msg, unsigned long lNetworkID, unsigned long lNumMessages);
    int WaitForRxMessagesWithTimeOut(unsigned int iTimeOut);
    int ReadOutMessages(icsSpyMessage* msg, int& lNumberOfMessages);
    bool ProcessRxPacket(unsigned long lCurrentTime, unsigned char* bPacket, unsigned long lNumberOfBytes);
    bool AddMsgToRXQueue(icsSpyMessage& stMsg, bool bSetEvent = true);
    bool ParseInputStream(unsigned long lCurrentTime, unsigned char* bDataOutput, unsigned long lNumberOfBytes);
    bool Process3rdGenerationCANRx(long unsigned int, unsigned char*, long unsigned int);
    bool ProcessRxPacketNeoRed(long unsigned int, unsigned char*, long unsigned int);
    CoreMiniMsg GenCoreMiniMsg(const short unsigned int*);
    unsigned int RetrieveAppNetID(unsigned int);
    unsigned int RetrieveProtocol(unsigned int, unsigned int);

    enum {
        eReadStateCallAPI,
        eReadStateWaitForOk,
    } eReadState;
};
