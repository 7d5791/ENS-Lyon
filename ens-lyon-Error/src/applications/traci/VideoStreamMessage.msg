//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

//
// TODO generated message class
//

cplusplus {{
//#include "INETDefs.h"
#include "VideoStreamWithTrace.h"
#include "base/Coord.h"
#include "WaveShortMessage_m.h"
}}

enum FrameType;

class noncobject Coord;
class WaveShortMessage;


packet VideoStreamMessage extends WaveShortMessage {
    // RTP header fields
    bool Marker;                ///< 1-bit marker (e.g., indicating the last packet of a frame)
    uint16_t sequenceNumber;	///< 16-bit packet sequence number
    uint32_t timestamp;			///< 32-bit timestamp

    // part of fragmentation unit (FU) header (1 octet) in RTP payload
    bool fragmentStart; ///< indicating the start of fragemented NAL units
    bool fragmentEnd;   ///< indicating the end of fragemented NAL units

    // fields not defined in RFC 3894
    long frameNumber;   ///< display order of the current frame
    double frameTime;	///< cumulative display time of the current frame in millisecond
    int frameType enum(FrameType);	///< one of I, P, B and IDR (in case of H.264)
}
