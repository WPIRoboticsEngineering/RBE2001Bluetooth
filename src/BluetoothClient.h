/*BluetoothClient.h  
*	Library for using Bluetooth Slave for serial communication at the 
*   robot end for RBE2001 @ WPI.
*	This library assumes that the Serial1 will be used on the client end  
*   Created by Benzun Wisely Babu, August 29, 2012.
*/


#ifndef BluetoothClient_h
#define BluetoothClient_h

#include "Arduino.h"

class BluetoothClient
{
	
public:
	BluetoothClient(); 			//constructor
    void send(byte data);		//used to send a byte
    bool receive(byte &data);	//used to receive a byte

};

#endif