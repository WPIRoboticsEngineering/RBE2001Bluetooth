/*
  BluetoothClient.cpp - Library for using Bluetooth for serial communication for RBE2001 @ WPI.
  Created by Benzun Wisely Babu, September 3, 2012.
*/

#include "Arduino.h"
#include "BluetoothClient.h"

/**
* @brief This is the default contructor which sets uo the serial1 for communication
*        Also sets up the member variable _status that give information about the connection.
*/
BluetoothClient::BluetoothClient() {
	Serial1.begin(115200);
}
/**
* @brief This function is used to send a byte over the bluetooth communication channel
*	     Uses Serial1
*		 its non blocking.
* @return true/false indicating if was sent
*/
void BluetoothClient::send(byte data) {
    Serial1.write(data);
}

/**
* @brief This function is used to receive a byte over the bluetooth
*		 Uses Serial1
* @return The data which was recied over the communication channel
*/
bool BluetoothClient::receive(byte &data) {
	if(Serial1.available() > 0) {
		data = Serial1.read();
		return (true);
	}
	else return (false);
}

