/*
 * VideoStreamWithTrace.h
 *
 *  Created on: Feb 12, 2016
 *      Author: Administrator
 */

#ifndef VIDEOSTREAMWITHTRACE_H_
#define VIDEOSTREAMWITHTRACE_H_



#include <vector>


enum TraceFormat {ASU_TERSE, ASU_VERBOSE}; ///< trace file formats

enum FrameType {I, IDR, P, B};  /// H.264 AVC frame types

//enum MessageKind {FRAME_START = 100, PACKET_TX = 200}; ///< kind values for self messages

typedef std::vector<char> CharVector;
typedef std::vector<double> DoubleVector;
typedef std::vector<FrameType> FrameTypeVector;
typedef std::vector<long> LongVector;
typedef std::vector<std::string> StringVector;

#endif /* VIDEOSTREAMWITHTRACE_H_ */
