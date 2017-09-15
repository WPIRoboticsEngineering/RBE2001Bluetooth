void setup() {
    Serial.begin(9600); // Serial output begin. Only needed for debug
    Serial3.begin(9600); // Serial output begin. Only needed for debug
    
}



void loop() {
  byte retry=0;
  Serial3.begin(9600);
  delay(1000);
  if(Serial.available()>1){
    String number = Serial.readString();
    while(retry<2){
      Serial.write("\r\nProvisioning...");
      Serial.print(number);
      Serial3.write("AT");
      delay(1000);
      Serial3.write("AT+VERSION");
      delay(1000);
      Serial3.write("AT+NAMERBE_BT_");
      Serial3.print(number);
      
      delay(1000);
      Serial3.write("AT+BAUD8");
      delay(1000);
      byte written=0;
      while(Serial3.available()>0){
          byte val=Serial3.read();
          if(val>8 && val< 127){
            written++;
            Serial.write(val);
          }
      }
      if(written>1){
        Serial.write("\r\nDone!");
        retry=100;
      }else{
        
        if(retry==0){
          Serial.write("\r\nMaybe this module is 115200 already, retrying");
          Serial3.begin(115200);
          delay(1000);
        }else{
          Serial.write("\r\nFailed!! module dead..");
        }
        retry++;
      }
      
    }
  }
  
}
