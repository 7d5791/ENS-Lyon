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

#include "applications/traci/TraCIDemo.h"

#include "ModuleAccess.h"
#include "NodeStatus.h"
#include "UDPSocket.h"

Define_Module(TraCIDemo);

simsignal_t TraCIDemo::mobilityStateChangedSignal = registerSignal(
        "mobilityStateChanged");

/*
 * Tokenize1 function for read frame data
 * from the trace file
 */
void Tokenize1(const std::string& str, std::vector<std::string>& tokens,
        const std::string& delimiters = " ") {
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);

    // Find first "non-delimiter".
    std::string::size_type pos = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos) {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));

        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);

        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

inline std::ostream& operator<<(std::ostream& out,
        const TraCIDemo::VideoStreamData& d) {
    out << "seq. number=" << d.currentSequenceNumber << "  trace format="
            << (d.traceFormat == ASU_TERSE ? "ASU_TERSE" : "ASU_VERBOSE")
            << "  number of frames=" << d.numFrames << "  frame period="
            << d.framePeriod << "  current frame=" << d.currentFrame
            << "  frame number=" << d.frameNumber << "  frame time="
            << d.frameTime << "  frame type=" << d.frameType << "  pkts sent="
            << d.numPktSent << "  bytes left=" << d.bytesLeft
            << "  pkt interval= " << d.pktInterval << endl;
    return out;
}

/*
 * This is the initialize function for simulation
 */
void TraCIDemo::initialize(int stage) {
    cSimpleModule::initialize(stage);

    // get the parameters
    maxTime = par("maxTime");
    minCount = par("minCount");
    minDistance = par("minDistance");
    mode = par("mode").stdstringValue();
    startFlag = true;
    frameCount = -1;
    frameLost = 0;
    m_logIndex = 0;

    m_frameIndex = 0;
    m_lastFrameCount = 0;

    numSentFrame = 0;
    numSentPacket = 0;
    prevFrameNumber = -1;

    // get the network name
    memset(network, 0, 127);
    strcpy(network, getParentModule()->getParentModule()->getFullName());

    if (stage == 0) {
        sentMessage = false;
    } else if (stage == 3) {
        bool isOperational;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(
                this)->getSubmodule("status"));
        isOperational = (!nodeStatus)
                || nodeStatus->getState() == NodeStatus::UP;
        if (!isOperational)
            throw cRuntimeError(
                    "This module doesn't support starting in node DOWN state");

        traci = TraCIMobilityAccess().get();
        traci->subscribe(mobilityStateChangedSignal, this);

        setupLowerLayer();
        curTime = simTime();

        numStreams = 0;
        framePeriod = 1.0 / par("fps").longValue();
        numFrames = 0;
        appOverhead = 0;
        maxPayloadSize = 1460 * 8;
        accidentStart = par("accidentStart").doubleValue();
        accidentDuration = par("accidentDuration").doubleValue();

        if (mode == "server") {
            actionMode = SERVER_MODE; // this is the SERVER
            scheduleAt(simTime() + accidentStart + 1,
                    new cMessage("VideoStreamReq"));
            startFlag = true;
            scheduleAt(simTime() + accidentStart + accidentDuration,
                    new cMessage("VideoStreamStop"));
        }

        /*
         *  read frame data from the trace file
         *  into corresponding vectors (e.g., frameSizeVector)
         */
        const char *fileName = par("traceFile").stringValue();

        std::ifstream fin(fileName);
        if (fin.is_open()) {
            int currentPosition;
            std::string line;

            // skip comments
            do {
                currentPosition = fin.tellg();  // save the current position
                std::getline(fin, line);
            } while (line[0] == '#');

            // file format (i.e., 'terse' or 'verbose') detection
            StringVector tokens;
            Tokenize1(line, tokens, " \t");
            traceFormat = (tokens.size() > 2) ? ASU_VERBOSE : ASU_TERSE;

            fin.seekg(currentPosition); // go back to the first non-comment line
            long frameNumber = 0;
            double frameTime = 0.0;
            std::string frameTypeString("");
            FrameType frameType = I;
            long frameSize;
            double psnr_y = 0.0;
            double psnr_u = 0.0;
            double psnr_v = 0.0;

            if (traceFormat == ASU_TERSE) {
                // file format is ASU_TERSE
                while (fin >> frameSize >> psnr_y) ///< never use "!fin.eof()" to check the EOF!
                {
                    frameNumberVector.push_back(frameNumber);
                    frameTimeVector.push_back(frameTime);
                    frameTypeVector.push_back(frameType);
                    frameSizeVector.push_back(frameSize); ///< in byte

                    // manually update the following fields
                    frameNumber++;
                    frameTime += framePeriod;

                    numFrames++;
                }
            } else {
                // file format is ASU_VERBOSE
                while (fin >> frameNumber >> frameTime >> frameTypeString
                        >> frameSize >> psnr_y >> psnr_u >> psnr_v) {
                    frameNumberVector.push_back(frameNumber);
                    frameTimeVector.push_back(frameTime);
                    if (frameTypeString.compare("I") == 0)
                        frameType = I;
                    else if (frameTypeString.compare("IDR") == 0)
                        frameType = IDR;
                    else if (frameTypeString.compare("P") == 0)
                        frameType = P;
                    else
                        frameType = B;
                    frameTypeVector.push_back(frameType);
                    frameSizeVector.push_back(frameSize); ///< in byte
                    numFrames++;
                }
            }

            fin.close();
        }   // end of if(fin.is_open())
        else {
            error("%s: Unable to open the video trace file `%s'",
                    getFullPath().c_str(), fileName);
        }
    }
}




/*
 *  check the packet queue
 */
bool TraCIDemo::CheckPacketSet(int m_id) {
    list<int>::iterator it;
    for (it = m_pktNameSet.begin(); it != m_pktNameSet.end(); it++) {
        if (*it == m_id)
            return true;
    }

    return false;
}

/*
 * Output the results to log files
 */
void TraCIDemo::LogToFile(VideoStreamMessage* msg, bool flag) {
    ofstream of;
    string fileName;
    fileName.empty();
    if (flag) {

        if (numPacketsReceived == 0)
            return;

        m_logIndex++;

        int m_dist = GetDistance(traci->getCurrentPosition(), sourcePos);

        if (m_logIndex == m_frameIndex + 1) {
            frameLost = m_lastFrameCount - frameCount;
        } else if (m_logIndex > m_frameIndex + 1) {
            return;
        } else {
            frameLost = par("fps").longValue() - frameCount;
        }

        char buf[125];
        memset(buf, 0, 125);
        sprintf(buf, "%d", m_dist);

        fileName += string("./results/")
                + getParentModule()->getParentModule()->getFullName()
                + string("-") + getParentModule()->getFullName()
                + string("-NumRecvPkt-LostFrame") + string(".txt");
        of.open(fileName, ios_base::app);

        of << "Time(Sec): " << simTime() << " Distance(m): " << buf
                << " Received Packet(%): "
                << numPacketsReceived * 100
                        / (numPacketsLost + numPacketsReceived)
                << " LostFrames: " << frameLost << endl;

        of.close();
        return;
    }

    bool updateFlag = false;
    ReceivedMessage recvMsg(msg->getFrameNumber(), simTime(),
            msg->getFrameType(), msg->getBitLength(), msg->getFragmentEnd());

    list<ReceivedMessage>::iterator it;
    for (it = m_recvMessageList.begin(); it != m_recvMessageList.end(); it++) {
        ReceivedMessage tmpMsg = *it;
        if ((tmpMsg.number == msg->getFrameNumber())
                && (tmpMsg.type == msg->getFrameType()) && !tmpMsg.endFlag) {

            ReceivedMessage newMsg(0, 0, 0, 0, 0);
            newMsg.number = msg->getFrameNumber();
            newMsg.time = simTime();
            newMsg.endFlag = msg->getFragmentEnd();
            newMsg.size = tmpMsg.size + msg->getBitLength();
            newMsg.type = msg->getFrameType();

            m_recvMessageList.push_back(newMsg);
            m_recvMessageList.erase(it);
            updateFlag = true;
            break;
        } else if ((tmpMsg.number == msg->getFrameNumber())
                && (tmpMsg.type == msg->getFrameType()) && tmpMsg.endFlag) {
            return;
        }
    }

    if (!updateFlag)
        m_recvMessageList.push_back(recvMsg);

    if (msg->getFragmentEnd() == false)
        return;

    ReceivedMessage resultMsg(0, 0, 0, 0, 0);

    for (it = m_recvMessageList.begin(); it != m_recvMessageList.end(); it++) {
        resultMsg = *it;
        if (resultMsg.number == msg->getFrameNumber()) {
            break;
        }
    }

    of.open(
            string("./results/")
                    + getParentModule()->getParentModule()->getFullName()
                    + string("-") + getParentModule()->getFullName()
                    + string(".txt"), ios_base::app);
    if (of.is_open()) {
        switch (resultMsg.type) {
        case I:
            of << resultMsg.number << " " << resultMsg.time << " " << "I "
                    << resultMsg.size << "bits " << resultMsg.endFlag << " "
                    << msg->getName() << endl;
            break;
        case IDR:
            of << resultMsg.number << " " << resultMsg.time << " " << "IDR "
                    << resultMsg.size << "bits " << resultMsg.endFlag << " "
                    << msg->getName() << endl;
            break;
        case B:
            of << resultMsg.number << " " << resultMsg.time << " " << "B "
                    << resultMsg.size << "bits " << resultMsg.endFlag << " "
                    << msg->getName() << endl;
            break;
        case P:
            of << resultMsg.number << " " << resultMsg.time << " " << "P "
                    << resultMsg.size << "bits " << resultMsg.endFlag << " "
                    << msg->getName() << endl;
            break;
        default:
            break;
        }
    }

    frameCount++;
    of.close();
}

void TraCIDemo::setupLowerLayer() {
    socket.setOutputGate(gate("udp$o"));
    socket.joinLocalMulticastGroups();
    socket.bind(4000);
    socket.setBroadcast(true);
}

/*

/*
 * receive the packet handler
 */
void TraCIDemo::handleMessage(cMessage* msg) {
    VideoStreamMessage tmpMsg;
    if (actionMode == SERVER_MODE) { // this is the SERVER mode
        if (startFlag && msg->isSelfMessage()) {
            if (msg->getKind() == FRAME_START) {
                readFrameData(msg);
            } else if (!strcmp(msg->getName(), "VideoStreamReq")) {
                processStreamRequest(msg);
            } else if (!strcmp(msg->getName(), "VideoStreamStop")) {
                startFlag = false;
                return;
            } else {
                sendStreamData(msg);
            }
        }
    }
}

void TraCIDemo::handleSelfMsg(cMessage* msg) {
}

void TraCIDemo::handleLowerMsg(cMessage* msg) {
}

void TraCIDemo::receiveSignal(cComponent *source, simsignal_t signalID,
        cObject *obj) {
    Enter_Method_Silent
    ();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate();
    }
}

/*
 * this is the function for send the video stream message
 */
void TraCIDemo::sendMessage(VideoStreamMessage* sendMsg) {
    sendMsg->setSenderPos(traci->getCurrentPosition());
    sentMessage = true;
    socket.sendTo(sendMsg, IPv4Address::ALL_HOSTS_MCAST, 4000);
}

/*
 * update the position of host handler
 */
void TraCIDemo::handlePositionUpdate() {
}

/*
 * get the distance function between source and destination
 */
double TraCIDemo::GetDistance(Coord source, Coord dest) {
    double distance = 0.0;
    distance = traci->commandDistanceRequest(source, dest, false);
    return distance;
}

/*
 * this is the function for process the stream request
 */
void TraCIDemo::processStreamRequest(cMessage *msg) {
// register video stream...
    VideoStreamData *d = new VideoStreamData;
    d->currentSequenceNumber = 0; ///< made random according to RFC 3550
    d->numPktSent = 0;
    d->numFrames = numFrames;
    d->numFramesSent = 0;
    d->framePeriod = framePeriod;
    d->currentFrame = -1; // this is the start frame
    streamVector.push_back(d);

    // initialize self messages
    d->frameStartMsg = new cMessage("Start of Frame", FRAME_START);
    d->frameStartMsg->setContextPointer(d);
    d->packetTxMsg = new cMessage("Packet Transmission", PACKET_TX);
    d->packetTxMsg->setContextPointer(d);

    // read frame data from the vector and trigger packet transmission
    readFrameData(d->frameStartMsg);

    numStreams++;
    delete msg;
}

/*
 * this is the function for read the frame data
 */
void TraCIDemo::readFrameData(cMessage *frameTimer) {
    VideoStreamData *d = (VideoStreamData *) frameTimer->getContextPointer();

    if (d->numFramesSent == d->numFrames) { // sent the whole frames in the trace file; reset the frame counter
        return;
    } else {
        d->numFramesSent++;
        numSentFrame++;
        ofstream of("./results/video-stream-source-frame.txt", ios_base::app);
        if (of.is_open()) {
            of << simTime() << " : " << numSentFrame << endl;
            of.close();
        }
    }

    d->currentFrame = (d->currentFrame + 1) % d->numFrames; ///> wrap around to the first frame if it reaches the last one
    d->frameNumber = frameNumberVector[d->currentFrame];
    d->frameTime = frameTimeVector[d->currentFrame];
    d->frameType = frameTypeVector[d->currentFrame];
    d->frameSize = frameSizeVector[d->currentFrame];
    d->bytesLeft = d->frameSize;
    if (frameSpreading) {
        d->pktInterval = d->framePeriod
                / ceil(d->bytesLeft / double(maxPayloadSize)); ///> spread out packet transmissions over a frame period
    } else {
        d->pktInterval = 0.0;
    }

    // schedule next frame start
    scheduleAt(simTime() + framePeriod, frameTimer);

    // start packet transmission right away
    sendStreamData(d->packetTxMsg);
}

/*
 * this is the function for send the stream data
 */
void TraCIDemo::sendStreamData(cMessage *pktTimer) {
    VideoStreamData *d = (VideoStreamData *) pktTimer->getContextPointer();

// generate and send a packet
    VideoStreamMessage *pkt = new VideoStreamMessage();
    long payloadSize =
            (d->bytesLeft >= maxPayloadSize) ? maxPayloadSize : d->bytesLeft;
    pkt->setName("VideoStreamPacket");
    pkt->setBitLength(payloadSize + appOverhead);
    pkt->setMarker(d->bytesLeft == payloadSize ? true : false); ///< indicator for the last packet of a frame
    pkt->setSequenceNumber(d->currentSequenceNumber);         ///< in RTP header
    pkt->setFragmentStart(d->bytesLeft == d->frameSize ? true : false); ///< in FU header in RTP payload
    pkt->setFragmentEnd(pkt->getMarker());      ///< in FU header in RTP payload
    pkt->setFrameNumber(d->frameNumber);                      ///< non-RTP field
    pkt->setFrameTime(d->frameTime);                          ///< non-RTP field
    pkt->setFrameType(d->frameType);                          ///> non-RTP field

    sendMessage(pkt);

// update the session VideoStreamData and global statistics
    d->bytesLeft -= payloadSize;
    d->numPktSent++;
    d->currentSequenceNumber = (d->currentSequenceNumber + 1) % 65536; ///> wrap around to zero if it reaches the maximum value (65535)

// reschedule timer if there are bytes left to send
    if (d->bytesLeft > 0) {
        scheduleAt(simTime() + d->pktInterval, pktTimer);
    }

    numSentPacket++;
    ofstream of("./results/video-stream-source-packet.txt", ios_base::app);
    if (of.is_open()) {
        of << simTime() << " : " << numSentPacket << endl;
        of.close();
    }
}


/*
 * this is the function for receive the stream message
 */
void TraCIDemo::recvStream(VideoStreamMessage *pkt) {

// get packet fields
    uint16_t seqNumber = pkt->getSequenceNumber();

    bool isFragmentStart = pkt->getFragmentStart();
    bool isFragmentEnd = pkt->getFragmentEnd();
    long frameNumber = pkt->getFrameNumber();
    FrameType frameType = FrameType(pkt->getFrameType());
    long encodingNumber = frameEncodingNumber(frameNumber, numBFrames,
            frameType);

// in the following, the frame statistics will be updated only when
// - the end fragment has been received or
// - the previously handled frame hasn't been properly processed
    int currentNumPacketsLost = 0;
    int currentNumFramesLost = 0;
    if (warmupFinished == false) {
        // handle warm-up flag based on both simulation time and GoP beginning
        if (simTime() >= simulation.getWarmupPeriod()) {
            if (frameType == I || frameType == IDR) {
                if (isFragmentStart == true) {
                    // initialize variables for handling sequence number and frame number
                    prevSequenceNumber = seqNumber;
                    prevPFrameNumber =
                            (frameNumber > 0) ?
                                    frameNumber - numBFrames - 1 : -1;
                    ///< to avoid loss of B frames after this frame
                    currentFrameNumber = frameNumber;
                    currentEncodingNumber = encodingNumber;
                    currentFrameType = frameType;

                    if (isFragmentEnd == true) {
                        // this frame consists of this packet only
                        currentFrameFinished = true;
                        prevIFrameNumber = frameNumber;
                        numFramesReceived++; ///< count the current frame as well
                    } else {
                        // more fragments to come!
                        currentFrameDiscard = false;
                        currentFrameFinished = false;
                        prevIFrameNumber = -1;
                    }

                    numPacketsReceived++;   ///< update packet statistics
                    warmupFinished = true;  ///< set the flag
                }   // end of fragmentStart check
            }   // end of frameType check
        }   // end of warm-up period check
    }   // end of warm-up flag check and related processing
    else {
        if (seqNumber != (prevSequenceNumber + 1) % 65536) {
            currentNumPacketsLost = (
                    seqNumber > prevSequenceNumber ?
                            seqNumber : prevSequenceNumber + 1)
                    - prevSequenceNumber;

            // detect loss of frame(s) between the one previously handled and the current one.
            // note that the previously handled frame could be the current one as well.
            if (encodingNumber > (currentEncodingNumber + 1) % numTraceFrames) {
                currentNumFramesLost = (
                        encodingNumber > currentEncodingNumber ?
                                encodingNumber : currentEncodingNumber)
                        - currentEncodingNumber - 1;
            }

            // check whether the previously handled frame should be discarded or not
            // as a result of current packet loss
            // TODO: implement the case for decoding threshold (DT) < 1
            if (currentFrameFinished == false) {
                currentNumFramesLost++;
            }

            // set frame discard flag for non-first packet of frame
            // TODO: implement the case for decoding threshold (DT) < 1
            if (isFragmentStart == false) {
                currentFrameDiscard = true;
            }
        }   // end of packet and frame loss detection and related-processing
        prevSequenceNumber = seqNumber; ///< update the sequence number

        switch (frameType) {
        case IDR:
        case I:
            if (isFragmentStart == true) {
                if (isFragmentEnd == true) {
                    // this frame consists of this packet only
                    currentFrameFinished = true;
                    prevIFrameNumber = frameNumber;
                    numFramesReceived++;    ///< count the current frame as well
                } else {
                    // more fragments to come!
                    currentFrameDiscard = false;
                    currentFrameFinished = false;
                }

                // update frame-related flags and variables
                currentFrameNumber = frameNumber;
                currentEncodingNumber = encodingNumber;
                currentFrameType = frameType;
            }   // end of processing of the first packet of I/IDR frame
            else {
                if (isFragmentEnd == true) {
                    if (currentFrameDiscard == false) {
                        // the frame has been received and decoded successfully
                        prevIFrameNumber = currentFrameNumber;
                        numFramesReceived++;
                    } else {
                        currentNumFramesLost++;
                    }
                    currentFrameFinished = true;
                }
            }   // end of processing of the non-first packet of I/IDR frame
            break;

        case P:
            if (isFragmentStart == true) {
                if (prevIFrameNumber == frameNumber - numBFrames - 1
                        || prevPFrameNumber == frameNumber - numBFrames - 1) {
                    // I or P frame that the current frame depends on was successfully decoded
                    currentFrameDiscard = false;

                    if (isFragmentEnd == true) {
                        currentFrameFinished = true; /// no more packet in this frame
                        prevPFrameNumber = frameNumber;
                        numFramesReceived++; ///< count the current frame as well
                    } else {
                        currentFrameFinished = false; ///< more fragments to come
                    }
                } else {
                    // the dependency check failed, so the current frame will be discarded
                    currentFrameDiscard = true;

                    if (isFragmentEnd == true) {
                        currentFrameFinished = true; /// no more packet in this frame
                        currentNumFramesLost++; ///< count the current frame as well
                    } else {
                        currentFrameFinished = false; ///< more fragments to come
                    }
                }

                // update frame-related flags and variables
                currentFrameNumber = frameNumber;
                currentEncodingNumber = encodingNumber;
                currentFrameType = frameType;
            }   // end of processing of the first packet of P frame
            else {
                if (isFragmentEnd == true) {
                    if (currentFrameDiscard == false) {
                        // the frame has been received and decoded successfully
                        prevPFrameNumber = currentFrameNumber;
                        numFramesReceived++;
                    } else {
                        currentNumFramesLost++;
                    }
                    currentFrameFinished = true;
                }
            }   // end of processing of the non-first packet of P frame
            break;

        case B:
            if (isFragmentStart == true) {
                // check frame dependency
                long lastDependonFrameNumber = (frameNumber / (numBFrames + 1))
                        * (numBFrames + 1);
                ///< frame number of the last I or P frame it depends on
                long nextDependonFrameNumber = lastDependonFrameNumber
                        + numBFrames + 1;
                ///< frame number of the next I or P frame it depends on
                bool passedDependency = false;
                if (nextDependonFrameNumber % gopSize == 0) {
                    // next dependent frame is I frame, so we need to check
                    // both next (I) and last frames.
                    if (prevPFrameNumber == lastDependonFrameNumber
                            && prevIFrameNumber == nextDependonFrameNumber) {
                        passedDependency = true;
                    }
                } else {
                    // next dependent frame is P frame, so we need to check
                    // only next (P) frame.
                    if (prevPFrameNumber == nextDependonFrameNumber) {
                        passedDependency = true;
                    }
                }

                if (passedDependency == true) {
                    if (isFragmentEnd == true) {
                        // this frame consists of this packet only
                        currentFrameFinished = true;
                        numFramesReceived++; ///< count the current frame as well
                    } else {
                        // more fragments to come!
                        currentFrameDiscard = false;
                        currentFrameFinished = false;
                    }
                } else {
                    // the dependency check failed, so the current frame will be discarded
                    currentFrameDiscard = true;

                    if (isFragmentEnd == true) {
                        // this frame consists of this packet only
                        currentFrameFinished = true;
                        currentNumFramesLost++; ///< count the current frame as well
                    } else {
                        // more fragments to come!
                        currentFrameFinished = false;
                    }
                }

                // update frame-related flags and variables
                currentFrameNumber = frameNumber;
                currentEncodingNumber = encodingNumber;
                currentFrameType = frameType;
            }   // end of processing of the first packet of B frame
            else {
                if (isFragmentEnd == true) {
                    if (currentFrameDiscard == false) {
                        // the frame has been received and decoded successfully
                        numFramesReceived++;
                    } else {
                        currentNumFramesLost++;
                    }
                    currentFrameFinished = true;
                }
            }   // end of processing of the non-first packet of B frame
            break;

        default:
            error("%s: Unexpected frame type: %d", getFullPath().c_str(),
                    frameType);
        }   // end of switch ()

        // update packet statistics
        numPacketsReceived++;
        numPacketsLost += currentNumPacketsLost;

        // update frame statistics
        numFramesDiscarded += currentNumFramesLost;

    }   // end of 'if (warmupFinshed == false)'
}

// get the frame encoding number
long TraCIDemo::frameEncodingNumber(long frameNumber, int numBFrames,
        FrameType frameType) {
    long encodingNumber = 0;

    switch (frameType) {
    case IDR:
    case I:
        encodingNumber = frameNumber == 0 ? 0 : frameNumber - numBFrames;
        break;
    case P:
        encodingNumber = frameNumber - numBFrames;
        break;
    case B:
        encodingNumber = frameNumber + 1;
        break;
    default:
        error("%s: Unexpected frame type: %d", getFullPath().c_str(),
                frameType);
    } // end of switch ()

    return encodingNumber;
}
