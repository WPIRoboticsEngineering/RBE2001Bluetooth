/*BluetoothMaster.h  
*	Library for using Bluetooth Master for serial communication at the 
*   arena end for RBE2001 @ WPI.
*	This library assumes that the Serial1 will be used on the master end  
*   \TODO make it suitable for serial2 or serial3 as Mega has 4 serial ports
*   Created by Benzun Wisely Babu, September 3, 2012.
*/


#ifndef BluetoothMaster_h
#define BluetoothMaster_h

#include "Arduino.h"

class BluetoothMaster
{
    public:
		BluetoothMaster();
	    void enterCMDMode(char msg[30]);
		void exitCMDMode(char msg[30]);
		void connectRDevice(char *deviceid,char msg[30]);
		void switchMode(char mode,char msg[30]);
		void sendPkt(byte pkt[10],int sz);
		bool readPacket(byte *pkt);
    	void transperentMode();
    	unsigned int  testConStatus();
  
	private:
};

#endif
