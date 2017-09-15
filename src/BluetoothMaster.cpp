/**
  BluetoothMaster.cpp - Library for using Bluetooth for serial communication for RBE2001 @ WPI.
  Created by Benzun Wisely Babu, September 9, 2012.
*/

#include "Arduino.h"
#include "BluetoothMaster.h"

#define DEBUG_MODE false


/*
* @brief This is the default contructor which sets up the Serial for communication at the masters end
*         The serial communication is implemented using 115200 Baud rate
*/
BluetoothMaster::BluetoothMaster() {
	Serial1.begin(115200);
}


/*
* @brief This function is used to make the bluetooth master as a loop back device and echo what it receives
*/
void BluetoothMaster::transperentMode() {
	if(Serial.available() > 0) 	{
		Serial.write(Serial.read());
	}
	if (Serial.available() > 0) {
		Serial.write(Serial.read());
	}
}


/*
* @brief This function is used to make the Bluetooth enter the CMD mode by sending the approporiate keywords as defined in the datasheet
* @arg buf The output text that was recieved from the bluetooth stored as a char array of 30
*/
void BluetoothMaster::enterCMDMode(char buf[30]) {
  Serial.flush();
  if (DEBUG_MODE) {
	Serial.println("Setting up Bluetooth Comm");
	Serial.println("Entering CMD mode");
  }
  Serial1.print('$');
  Serial1.print('$');
  Serial1.print('$');   //send $$$ to serial1, read CMD
  delay(500);
  int i = 0;
  while(Serial1.available() > 0) {
      buf[i] = Serial1.read();      // read the incoming byte:
      if ((buf[i] != '\c') && (buf[i] != '\n'))
        i++;
  }
  buf[i-1] = '\0';
  if (DEBUG_MODE) {
	Serial.print("Received: ");
	Serial.println(buf);
  }
}


/*
* This function is used to exit the CMD mode of the bluetooth. The specification are in the datasheet
* @arg buf the output message recieveed from the bluetooth stored as a char array of 30
*/
void BluetoothMaster::exitCMDMode(char buf[30]) {
  if (DEBUG_MODE) Serial.println("Exiting command mode");

  Serial1.print('-');
  Serial1.print('-');                          
  Serial1.print('-');                         
  Serial1.print('\r');                        //send ---<cr>
  delay(500);
  int i = 0;
  buf[0] = '\0';
  while (Serial1.available() > 0) {
      buf[i] = Serial1.read();                   // read the incoming byte:
      if ((buf[i] != '\c') && (buf[i] != '\n'))
        i++;
  }
  if (i > 0) buf[i-1] = '\0';
  if (DEBUG_MODE) {
	Serial.print("Received: ");
	Serial.println(buf);
  }
}


/*
* @brief This function is used to make the Bluetooth switch its mode of operation. The specification are in the datasheet.
* @arg mode The mode you want the bluetooth to be in
* @arg The output string recieved from the bluetooth stored as an array of char
*/
void BluetoothMaster::switchMode(char mode, char buf[30]) {
  if (DEBUG_MODE) Serial.println("Switching to Master mode");

  Serial1.print('S');
  Serial1.print('M');
  Serial1.print(',');
  Serial1.print(mode);
  Serial1.print('\r');                        //send SM,1<cr>
  delay(500);
  int i = 0;
  while (Serial1.available() > 0) {
      buf[i] = Serial1.read();                   // read the incoming byte:
      if((buf[i] != '\c') && (buf[i] != '\n'))
        i++;
  }
  buf[i-1] = '\0';
  if (DEBUG_MODE) {
	Serial.print("Received: ");
	Serial.println(buf);
  }
}


/*
* @brief This function is used to connect to the Remote slave device give the slave bluetooth ID
* @arg slaveID The bluetooth address of the device you wish to connect to.
* @arg buf the return message form the the bluetooth 
*/
void BluetoothMaster::connectRDevice(char *slaveID,char buf[30]) {
  if (DEBUG_MODE) {
	Serial.print("Connecting to slave id: ");
	Serial.println(slaveID);
  }
  
  Serial1.print('C');
  Serial1.print(','); 
  
  int i;  
  for (i=0; i < 12; i++) {
    Serial1.write(slaveID[i]);
  }  
  
  Serial1.print('\n');                        //send C,address<cr>
  delay(500);
  i = 0;
  while (Serial1.available() > 0) {
      buf[i] = Serial1.read();                   // read the incoming byte:
      i++;
  }
  if (i != 0) buf[i-2] = '\0';
  if (DEBUG_MODE) {
	Serial.print("Received_1: ");
	Serial.println(buf);
  }
}


/*
* @brief This function is used to read a packet from the data that the bluetooth recieves.
* @arg pkt the pointer to the output array that contains the bytes in th packet
* @return true if a new packet was read
*/
bool BluetoothMaster::readPacket(byte *pkt) {
  int size_pkt;
  byte garbage;
  unsigned char tmoCntr;
  while (Serial1.available()) {
	garbage = Serial1.read();
	if (DEBUG_MODE) Serial.println("<=");
	if (DEBUG_MODE) Serial.println(garbage, HEX);
	if (garbage == 0x5F) {
	  pkt[0] = garbage;
	  tmoCntr = 0x1FF;
	  while (Serial1.available() == 0) {			// spin here looking for the length byte
		  if (--tmoCntr == 0) return false;			// bail if nothing in 255ms
		  delay(1);
	  }
	  size_pkt = Serial1.read();
	  if (DEBUG_MODE) Serial.println(size_pkt, HEX);
	  pkt[1] = size_pkt;
	  tmoCntr = 0xFF;
	  while (Serial1.available() < size_pkt - 1) {	// spin here looking for the rest of the packet
		  if (--tmoCntr == 0) return false;			// bail if not all there in 255ms
		  delay(1);
	  }
	  for (int i = 0; i < size_pkt - 1; i++) {			// shouldn't this be size_pkt - 1????
	    pkt[2+i] = Serial1.read();
	    if (DEBUG_MODE) Serial.println(pkt[2+i], HEX);
	  }
	  return true;
	}
  }
  return false;
}


/* 
* @brief This function is used to send a packet through the bluetooth
* @arg pkt the array of bytes that corresponds to the packet that needs to be sent
* @arg sz the size of the packet.
*/
void BluetoothMaster::sendPkt(byte pkt[10], int sz) {
  Serial1.flush();
  Serial1.write(pkt, sz);
//  if (DEBUG_MODE) {
//	  Serial.println("=>");
//	  for (int i = 0; i < sz; i++) {
//		  Serial.println(pkt[i], HEX);
//	  }
//  }
}


/*
* @brief This function is used to test the status of the bluetooth master unit
* @return 0 if no connection, 1 if the unit is connected to a slave
*/
unsigned int BluetoothMaster::testConStatus() {
	unsigned int i;
	char val, buf[30];

	if (DEBUG_MODE) Serial.println("Testing connection status...");

	Serial1.print('G');
	Serial1.print('K');
	Serial1.print('\n');                        //send GK<cr>
	delay(500);
	val = Serial1.read();                   // read the incoming byte:
	Serial1.flush();
	sprintf(buf,"Received_1: %d", val);
	Serial.println(buf);
	return (unsigned int) val;
}
