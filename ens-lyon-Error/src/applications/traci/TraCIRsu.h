//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef TraCIRsu_H
#define TraCIRsu_H

#include <omnetpp.h>
#include "UDPSocket.h"
#include "ILifecycle.h"
#include "LifecycleOperation.h"
#include "mobility/single/TraCIMobility.h"
#include "VideoStreamWithTrace.h"
#include "VideoStreamMessage_m.h"
#include <list>

// action mode macros
#define SERVER_MODE     1
#define CLIENT_MODE     2

using namespace std;

/**
 * Small IVC Demo
 */
class TraCIRsu: public cSimpleModule, protected cListener, public ILifecycle {
public:
    virtual bool handleOperationStage(LifecycleOperation *operation, int stage,
            IDoneCallback *doneCallback) {
        Enter_Method_Silent
        ();
        throw cRuntimeError("Unsupported lifecycle operation '%s'",
                operation->getClassName());
        return true;
    }

    virtual void receiveSignal(cComponent *source, simsignal_t signalID,
            cObject *obj);

protected:
    virtual int numInitStages() const {
        return 4;
    }

    virtual void initialize(int stage);
    virtual void handleMessage(cMessage* msg);

protected:
    TraCIMobility* traci;
    bool sentMessage;
    UDPSocket socket;
    static simsignal_t mobilityStateChangedSignal;

protected:
    void setupLowerLayer();
    virtual void handleSelfMsg(cMessage* apMsg);
    virtual void handleLowerMsg(cMessage* apMsg);
    virtual void handlePositionUpdate();

    void LogToFile(VideoStreamMessage* msg, bool flag);
    void recvStream(VideoStreamMessage *pkt);
    long frameEncodingNumber(long frameNumber, int numBFrames,
            FrameType frameType);
    struct ReceivedMessage {
        long number;
        simtime_t time;
        long size;
        int type;
        bool endFlag;

        ReceivedMessage(long m_number, simtime_t m_time, int m_type,
                long m_size, bool m_endFlag) {
            number = m_number;
            time = m_time;
            size = m_size;
            type = m_type;
            endFlag = m_endFlag;
        }
    };

    list<ReceivedMessage> m_recvMessageList;

    // module parameters
    double startupDelay;
    long numTraceFrames; ///< number of frames in the trace file (needed to handle wrap around of frames by the server)
    int gopSize;    ///< GOP pattern: I-to-I frame distance (N)
    int numBFrames; ///< GOP pattern: I-to-P frame distance (M)

    // statistics
    bool warmupFinished;    ///< if true, start statistics gathering
    long numPacketsReceived;    ///< number of packets received
    long numPacketsLost;    ///< number of packets lost
    long numFramesReceived; ///< number of frames received and correctly decoded
    long numFramesDiscarded; ///< number of frames discarded due to packet loss or failed dependency

    long numSentPacket;
    long numSentFrame;
    long ReceivedPacketCount;

    // variables for packet and frame loss handling
    uint16_t prevSequenceNumber; ///< (16-bit RTP) sequence number of the most recently received packet
    long prevIFrameNumber; ///< frame number of the most recently received I frame
    long prevPFrameNumber; ///< frame number of the most recently received P frame
    long currentFrameNumber; ///< frame number (display order) of the frame under processing
    long currentEncodingNumber; ///< encoding number (transmission order) of the frame under processing
    FrameType currentFrameType; /// type (I, IDR, P, or B) of the frame under processing
    bool currentFrameDiscard; ///< if true, all the remaining packets of the current frame are to be discarded
    bool currentFrameFinished; ///< if true, the frame has been successfully received and decoded

    int frameCount;
    int packetCount;
};

#endif

