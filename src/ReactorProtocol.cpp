/*
  ReactorProtocol.cpp - Library for using ReactorProtocol for serial communication for RBE2001 @ WPI.
  Created by Benzun Wisely Babu, August 29, 2012
  Updated by Craig Putnam, September 3, 2013
*/

#include "Arduino.h"
#include "ReactorProtocol.h"


/*
* @brief This is the constructor for the class ReactorProtocol which is used to create the 
*        packets to be trasnmitted over bluetooth
* @arg src The address of the source device, in which the packets are going to be transmitted
*/
ReactorProtocol::ReactorProtocol(byte src) {
    _source_addr = src;
    _dest_addr = 0x00;
}	// end ReactorProtocol


/*
* @brief This function is used to set a particular destination address for the packets made. 
*        By default the packets are sent with adress 0x00 corresponding to broadcast
* @arg dst The destination address of the packet. 
*/
void ReactorProtocol::setDst(byte dst) {
   _dest_addr = dst;
}	// end setDst


/*
* @brief This function is used to set the checksum for the packets. 
*        The check sum is calculated as 0xff-sum. The sum is the Lower byte of the at bytes added.
* @arg pkt : The pointer to the array that contains the bytes for the packet.
* @arg no  : The size of the packet
* @return  The checksum value as a byte
*/
byte ReactorProtocol::calcChecksum(byte *pkt, uint8_t no) {
    byte sum = 0x00;
    for (int i = 1; i <= no; i++) {
        sum += pkt[i];
    }
	sum = 0xFF - sum;
    return (sum);
}	// end calcChecksum


/*
* @brief This function is used to fill the array accoding to the packet defn give the data and the type of the packet
* @arg type The Packet type that needs to be made
* @arg data The data for the packet. Is implemented as a pointer as the data can be more than one byte
* @arg pkt  An array of bytes that will be the storage for the packets that are formed/
* @return   The total size of the packet (in bytes)
*/
int ReactorProtocol::createPkt(byte type, byte* data, byte pkt[10]) {
    byte size = 0x05;		// default length if no data bytes are present
    pkt[0] = 0x5F;			// set the standard packet start byte
    pkt[2] = type;			// ...and the type id for this packet
    pkt[3] = _source_addr;	// ...and the source address (this device)
    pkt[4] = _dest_addr;	// ...and the destination address

    // add data bytes depending on the type of packet
    switch (type) {
    case 1:	// storage tube availability
    case 2:	// supply tube availability
    case 3:	// radiation alert
		size += 0x01;		// adjust length for one additional data byte
	    pkt[1] = size;		// ...and set the packet length
        pkt[5] = data[0];	// add the single data byte
        pkt[6] = (byte) calcChecksum(pkt, 5);
        break;
    case 4:	// stop  movement
    case 5:	// resume movement
    case 7:	// heartbeat
    	// no data bytes for any of these messages
        pkt[1] = size;		// set the packet length
        pkt[5] = (byte) calcChecksum(pkt, 4);
        break;
    case 6:	// robot status
		size += 0x03;		// adjust length for three additional data bytes
	    pkt[1] = size;		// ...and set the the packet length
        pkt[5] = data[0];	// add the three data bytes
        pkt[6] = data[1];
        pkt[7] = data[2];
        pkt[8] = (byte) calcChecksum(pkt, 7);
        break;
//    default:
//        pkt[1] = size;		// now that the size has been determined, finish the packet
//    	break;
    }
	//For Debug purposes
//	Serial.println("createPkt: made the following...");
//	for(int i = 0; i < size + 1; i++)
//	{
//		Serial.println(pkt[i], HEX);
//	}
    return (size + 1);
}	// end createPkt


/*
* @brief    This function is used to extract the data from the packet.
* @arg pkt  The Input byte stream that is the array in the packet
* @arg data The pointer to the Output data that was retrived from the packet
* @arg type The Output that is the the type of the packet,
* @return   TRUE if the packet was processed successfully
*/
bool ReactorProtocol::getData(byte* pkt, byte *data, byte &type) {
  byte size_pkt = pkt[1];	// extract the length byte
  // For Debug only
//  Serial.print("getData: Size: ");
//  Serial.println(size_pkt);
//  for (int i = 0; i < size_pkt + 1; i++) {
//	Serial.println(pkt[i], HEX);
//  }

  //first check the checksum
  byte cs = calcChecksum(pkt, size_pkt - 1);
  if (cs == pkt[size_pkt]) { // is it a valid packet?
        //checksum is OK, extract and return the packet type id
	type = pkt[2];
	//now extract the appropriate amount of data depending on the packet type
	switch (type) {
	case 1:	// storage tube availability
	case 2:	// supply tube availability
	case 3:	// radiation alert
	  data[0] = pkt[5];
	  break;
	case 6:	// robot status
	  data[0] = pkt[5];
	  data[1] = pkt[6];
	  data[2] = pkt[7];
	  break;
	default:  // stop movement (4), resume movement (5), heartbeat (7) come through here
	  // these messages have no data
	  break;
	}
  } else return false;
  return true;
}	// end getData
