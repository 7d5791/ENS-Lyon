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

cplusplus {{
//#include "INETDefs.h"
#include "base/Coord.h"
#include "IPvXAddress.h"
}}

enum FrameType;

class noncobject Coord;
class noncobject IPvXAddress;

packet WaveShortMessage {
    
    //Version 
	int wsmVersion = 0;
	
	//Determine which security mechanism was used
	int securityType = 0;
	
	//Channel Number on which this packet was sent
	string psc = "Service with some Data";
	
	//Length of Wave Short Message
	int wsmLength;
	
	//Data of Wave Short Message
	string wsmData = "Some Data";

	//Address from Senser
	int senderAddress = 0;
	
	//Address destination
	int recipientAddress = -1;
	
	//Last Frame received
	int lastFrameReceived = 0;
	
	//id video request
	int idVideo;
	
	//Position of sender
	Coord senderPos;
	
	double speed;
	
	string nodeType;
}
