// Allows an arduino to behave like the 2001 field computer. The Bluetooth master was originally 
//created for use on an arduino mega and then modified for an uno. There for not all functions in the
// .h file will work. This code sends a storage packet followed by a supply package.


#include <ReactorProtocol.h>
#include <BluetoothMaster.h>

ReactorProtocol pcol(byte(0x00)); //setting pcol byte for source address 
BluetoothMaster btmaster; //init btmaster object

unsigned char mskStorage = 0x01; // storage tube pac
unsigned char mskSupply = 0x01;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200); // init Serial for bluetooth master
}

void loop() {
  // put your main code here, to run repeatedly: 
     
     pcol.setDst(0x00);			//send a broadcast message so that its address is 0x00
     byte pkt[10];				//allocate the memory for the bytes in the packey
     int sz;
     byte data1[1],data2[1];
     data1[0]=mskStorage;					// to send the imformation for the storage tube
     sz=pcol.createPkt(0x01,data1,pkt);		//create the packet that needs to be sent for the storage tube status
     btmaster.sendPkt(pkt,sz);				//send using the bluetooth device(sends out over Serial)
     delay(20);
     data2[0]=mskSupply;					//the data for the supply tube 
     sz=pcol.createPkt(0x02,data2,pkt);		//create the packet for the supply tube data
     btmaster.sendPkt(pkt,sz);				//send using the bluetooth device(sends out over Serial)
    delay(1000);
   }
   
   
