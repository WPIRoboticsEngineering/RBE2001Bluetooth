/*
  ReactorProtocol.h - Library for using ReactorProtocol for serial communication for RBE2001 @ WPI.
  Created by Benzun Wisely Babu, August 29, 2012.
*/
 /*
 	Maintanence Messages
 		Keep Alive
 		Check status
 		Disconnect

 	Standard Messages
 		Storage Tube availability
 		Supply Tube availability
 		Radiation Alert
 		Stop Movement
 		Resume Movement
 		Robot Status

 	Packet Structure
 		Start					0x0F/0xAA
 		Length					<1 byte>
 		_packet_type			        <1 byte>
 		_source_addr			        <1 byte>
 		_dest_addr				<1 byte>
 		_data					<variable>
 		checksum				<1byte>

 */
#ifndef ReactorProtocol_h
#define ReactorProtocol_h

#include "Arduino.h"


class ReactorProtocol
{
  public:
    ReactorProtocol(byte src);
    void setDst(byte dst);
    int createPkt(byte type,byte *data,byte *pkt);
    bool getData(byte *pkt,byte *data,byte &type);    
  
  private:
  	void checksum();
    byte calcChecksum(byte* data, uint8_t no);

    byte        _source_addr;
    byte        _dest_addr;
};


#endif