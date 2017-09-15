/*
 This code can be used to see the packets being recieved at the 
 client.
 */
int inByte;
void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
 Serial1.begin(115200);
}
void loop() {
  // read from port 0, send to port 0:
  if (Serial1.available()) {
    int inByte = Serial1.read();
   Serial.println(inByte,HEX); 
  }
  
}

