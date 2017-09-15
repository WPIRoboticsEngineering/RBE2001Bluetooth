/*
 * RBE2001_FIELD_CONTROL
 *
 * This sketch is the field control program for the RBE 2001 'C15 playing field
 *
 * It uses Bluetooth comms to communicate with the slave device (the robot).
 * It also uses the LiquidCrystal library to drive the LCD display.
 *
 * Originally written by Benzun on 15th September 2012
 * Extensively modified by C. Putnam, A-term 2013
 *
 * Modified by:
 *   C. Putnam, 2/8/14: Added requirement for external 10K pullups on the RX1 & TX1 lines
 *                    : General cleanup of code
 *
 *   C. Putnam, 3/4/14: Added additional slave module addresses (now a total of 30)
 *
 *   C. Putnam, 9/4/14: Updated slave module addresses for A-term, 2014
 *
 *   C. Putnam, 10/6/14: Updated to reflect modified getData method (it no longer filters packets by address)
 *
 *   C. Putnam, 2/5/15: Updated slave module addresses for C-term, 2015
 *
 * LCD-related circuitry:
 * - LCD RS pin to digital pin 27
 * - LCD Enable pin to digital pin 26
 * - LCD D4 pin to digital pin 25
 * - LCD D5 pin to digital pin 24
 * - LCD D6 pin to digital pin 23
 * - LCD D7 pin to digital pin 22
 * - LCD R/W pin to ground
 *
 * Bluetooth-related circuitry:
 * - BT RX pin to digital pin 18 (TX1)
 * - BT TX pin to digital pin 19 (RX1)
 *
 * Spent fuel rod circuitry:
 * - Storage tube 1 switch to digital pin 30
 * - Storage tube 1 LED to digital pin 31
 * - Storage tube 2 switch to digital pin 32
 * - Storage tube 2 LED to digital pin 33
 * - Storage tube 3 switch to digital pin 34
 * - Storage tube 3 LED to digital pin 35
 * - Storage tube 4 switch to digital pin 36
 * - Storage tube 4 LED to digital pin 37
 *
 * Supply fuel rod circuitry:
 * - Supply tube 1 switch to digital pin 40
 * - Supply tube 1 LED to digital pin 41
 * - Supply tube 2 switch to digital pin 42
 * - Supply tube 2 LED to digital pin 43
 * - Supply tube 3 switch to digital pin 44
 * - Supply tube 3 LED to digital pin 45
 * - Supply tube 4 switch to digital pin 46
 * - Supply tube 4 LED to digital pin 47
 * 
 * Team selection switches (five bits form the team number):
 * - digital pin 50 => 0x01 ( 1)
 * - digital pin 51 => 0x02 ( 2)
 * - digital pin 52 => 0x04 ( 4)
 * - digital pin 53 => 0x08 ( 8)
 * - digital pin 7  => 0x10 (16)
 *
 * Stop/Resume movement pushbuttons:
 * - Stop current robot pushbutton to digital pin 5
 * - Resume current robot pushbutton to digital pin 4
 * - Stop fake robot pushbutton to digital pin 3
 * - Resume fake robot pushbutton to digital pin 2
 *
 * Other I/O pins
 * - Enable debug messages on digital pin 10
 * - Ignore lack of heartbeat messages on digital pin 9
 * - Heartbeat LED on digital pin 28
 * - Low level radiation alert LED on digital pin 11
 * - High level radiation alert LED on digital pin 12
 */
 
// various includes
#include <BluetoothMaster.h>
#include <Bounce2.h>
#include <LiquidCrystal.h>
#include <ReactorProtocol.h>
#include <stdio.h>
#include <TimerOne.h>

// function prototypes
void centerLCD(char* msg, unsigned int line);
void timer1ISR(void);
void updateStorage(void);
void updateSupply(void);

// set up module-wide defines
#define MAX_TEAM_NUMBER  24        // <== MAXIMUM NUMBER OF TEAMS (UPDATE AS NECESSARY)
                                   // NOTE: There are 5 team address selection switches, therefore the absolute maximum
                                   // number of teams is 31 (address 0 is the field control computer running this code)

#define COURSE_TERM      "C'15"    // <== COURSE TERM (UPDATE AS NECESSARY)

#define BT_TIMEOUT 3               // how long to wait (in seconds) before timing out the slave for lack of heartbeat

#define onboardLED     13
#define debugMsgEnable 10          // I/O pin to pull low to enable diagnostic message to console
#define debugModePin    9          // I/O pin to pull low to cause lack of heartbeat messages to be ignored
#define HeartBeatLED   28
#define lowlvlradLED   11
#define highlvlradLED  12

#define btnStopCur      6          // I/O pin is pulled low to indicate current robot should stop movement
#define btnResmCur      5          // I/O pin is pulled low to indicate current robot should resume movement
#define btnStopFak      4          // I/O pin is pulled low to indicate fake robot should stop movement
#define btnResmFak      3          // I/O pin is pulled low to indicate fake robot should resume movement
#define fakRobotID     99          // address for the fake robot (used in stop/resume messages)

#define dbncrStblInt   50          // use a 50ms 'stable interval' with the debouncer objects

#define starting_sw_pin 30

#define swStorageTube1  30 
#define ledStorageTube1 31

#define swStorageTube2  32
#define ledStorageTube2 33

#define swStorageTube3  34
#define ledStorageTube3 35

#define swStorageTube4  36
#define ledStorageTube4 37

#define swSupplyTube1   40
#define ledSupplyTube1  41

#define swSupplyTube2   42
#define ledSupplyTube2  43

#define swSupplyTube3   44
#define ledSupplyTube3  45

#define swSupplyTube4   46
#define ledSupplyTube4  47

//#define serialTX1       18
//#define serialRX1       19

#define swHex_16 7
#define swHex_8 53
#define swHex_4 52
#define swHex_2 51
#define swHex_1 50

// enumerated types
enum ConnectionStates {
  bt_not_connected,
  bt_connection_in_progress,
  bt_connected
};

// set up module-wide variables
unsigned char mskStorage = 0;
unsigned char mskSupply = 0;
unsigned char rodWarningLvl;
unsigned char teamNumber;
boolean genDbgMsgs;
boolean robotRunning, fakeRunning;

ConnectionStates mstrConState;

// these variables are accessed inside as well as outside of ISRs, so they need to be marked as volatile
volatile unsigned char onboardLEDstate = 0;
volatile unsigned char updateLCD = 0;
volatile unsigned char updateBluetooth = 0;
volatile unsigned long elapsedTime = 0;
volatile unsigned long msgExpiry = 0;
volatile unsigned int  tickCount = 0;
volatile unsigned int  tickCount2 = 0;
volatile unsigned char ledOnTime = 0;
volatile boolean alive = false;
volatile boolean reconnect = false;

// allocate memory for various I/O buffers
char buf[40];
char buf2[20];
char buf3[20];
char slaveID[14];

// allocate memory for the various packets
byte pkt[10];
byte data[3];
byte type;

byte pkt1[10];
int sz1;
byte data1[1];

byte pkt2[10];
int sz2;
byte data2[1];

// initialize the lookup table of Bluetooth slave module addresses (UPDATE AS NECESSARY)
char btADDR[MAX_TEAM_NUMBER][13] = {
  "201311013172",    // team  1
  "001208240526",    // team  2
  "201312051450",    // team  3
  "201311123844",    // team  4
  "001208240295",    // team  5
  "201311120846",    // team  6
  "201312069226",    // team  7
  "201311123085",    // team  8
  "201306190437",    // team  9
  "201306140425",    // team 10
  "201306144166",    // team 11
  "201306190878",    // team 12
  "201306144500",    // team 13
  "001208240534",    // team 14
  "201311140483",    // team 15
  "201307180887",    // team 16
  "201306190896",    // team 17
  "201311120844",    // team 18
  "201306142966",    // team 19
  "201311013140",    // team 20
  "201311123717",    // team 21          (USED FOR THE DEMO/TEMPLATE ROBOT IN C'15)
  "201306144331",    // team 22          (spare)
  "001208240310",    // team 23          (spare)
  "201306191212"};   // team 24          (spare)

// initialize the LCD library with the numbers of the interface pins
LiquidCrystal lcd(27, 26, 25, 24, 23, 22);

//Reactor Protocol packaging tool
ReactorProtocol pcol(byte(0x00));
BluetoothMaster btmaster;

// Instantiate the Bounce objects - one per switch
Bounce debouncer_1 = Bounce();        // debouncer for the stop current pushbutton
Bounce debouncer_2 = Bounce();        // debouncer for the resume current pushbutton 
Bounce debouncer_3 = Bounce();        // debouncer for the stop fake pushbutton
Bounce debouncer_4 = Bounce();        // debouncer for the resume fake pushbutton


/*
 * this method is executed once to initialize the program
 */
void setup(void) {
  char splash1[14];
  
  // configure the console comm port to 115200 baud
  Serial.begin(115200);
  
  // the master bluetooth module is also configured to 115200 baud
  Serial1.begin(115200);

  // set up the onboard LED and init it to the off state
  pinMode(onboardLED, OUTPUT);
  digitalWrite(onboardLED, LOW);
  
  // set up the Heartbeat LED and init it to the off state
  pinMode(HeartBeatLED, OUTPUT);
  digitalWrite(HeartBeatLED, LOW);

  // set up the low level radiation alert LED and init it to the off state
  pinMode(lowlvlradLED, OUTPUT);
  digitalWrite(lowlvlradLED, LOW);

  // set up the high level radiation alert LED and init it to the off state
  pinMode(highlvlradLED, OUTPUT);
  digitalWrite(highlvlradLED, LOW);

  // set up the stop/resume pushbuttons for the current and fake robots; enable pullups on all
  pinMode(btnStopCur, INPUT_PULLUP);
  pinMode(btnResmCur, INPUT_PULLUP);
  pinMode(btnStopFak, INPUT_PULLUP);
  pinMode(btnResmFak, INPUT_PULLUP);
  
  // after setting up the stop/resume pushbuttons, set up the corresponding Bounce objects (for debouncing the pushbuttons)
  // use a 50ms 'stable interval'
  debouncer_1.attach(btnStopCur);
  debouncer_1.interval(dbncrStblInt);
  debouncer_2.attach(btnResmCur);
  debouncer_2.interval(dbncrStblInt);
  debouncer_3.attach(btnStopFak);
  debouncer_3.interval(dbncrStblInt);
  debouncer_4.attach(btnResmFak);
  debouncer_4.interval(dbncrStblInt);
  
  // set the initial state of the current and fake robots to 'running'
  robotRunning = true;
  fakeRunning = true;

  // configure up the input switches and output LEDs for the storage and supply tubes.
  // configure the input pins to use the internal pullups.
  pinMode(swStorageTube1, INPUT);
  digitalWrite(swStorageTube1, HIGH);
  pinMode(ledStorageTube1, OUTPUT);
  
  pinMode(swStorageTube2, INPUT);
  digitalWrite(swStorageTube2, HIGH);
  pinMode(ledStorageTube2, OUTPUT);
  
  pinMode(swStorageTube3, INPUT);
  digitalWrite(swStorageTube3, HIGH);
  pinMode(ledStorageTube3, OUTPUT);
  
  pinMode(swStorageTube4, INPUT);
  digitalWrite(swStorageTube4, HIGH);
  pinMode(ledStorageTube4, OUTPUT);
    
  pinMode(swSupplyTube1, INPUT);
  digitalWrite(swSupplyTube1, HIGH);
  pinMode(ledSupplyTube1, OUTPUT);
  
  pinMode(swSupplyTube2, INPUT);
  digitalWrite(swSupplyTube2, HIGH);
  pinMode(ledSupplyTube2, OUTPUT);
  
  pinMode(swSupplyTube3, INPUT);
  digitalWrite(swSupplyTube3, HIGH);
  pinMode(ledSupplyTube3, OUTPUT);
  
  pinMode(swSupplyTube4, INPUT);
  digitalWrite(swSupplyTube4, HIGH);
  pinMode(ledSupplyTube4, OUTPUT);
  
  pinMode(debugModePin, INPUT);
  digitalWrite(debugModePin, HIGH);
  
  pinMode(debugMsgEnable, INPUT);
  digitalWrite(debugMsgEnable, HIGH);
  
  // configure the team selection switch
  // configure the input pins to use the internal pullups.
  pinMode(swHex_1, INPUT);
  digitalWrite(swHex_1, HIGH);
  pinMode(swHex_2, INPUT);
  digitalWrite(swHex_2, HIGH);
  pinMode(swHex_4, INPUT);
  digitalWrite(swHex_4, HIGH);
  pinMode(swHex_8, INPUT);
  digitalWrite(swHex_8, HIGH);
  pinMode(swHex_16, INPUT);
  digitalWrite(swHex_16, HIGH);
 
  // set up Timer1
  Timer1.initialize(100000);	                // set up a 100 millisecond period
  Timer1.attachInterrupt(timer1ISR);            // ...and specify the timer ISR
  
  // set up the number of columns (16) and rows (2) on the LCD
  lcd.begin(16, 2);
  
  // print a splash message to the LCD and the console
  strcpy(buf, "RBE 2001 ");
  strcat(buf, COURSE_TERM);
  strcpy(splash1, buf);
  strcat(buf, " Field Control System");
//  strcpy(splash1, "RBE 2001 ");
//  strcat(splash1, COURSE_TERM);
  char splash2[] = "Field Control";
  char team1[] = "Loading...";
 
  Serial.println(buf);  
  centerLCD(splash1, 0);                        // output upper row message
  centerLCD(splash2, 1);                        // ...and then the lower row
  delay(2000);                                  // leave the message up for a while
  lcd.clear();                                  // clears display; leaves the current column and row at (0, 0)

  // load the team number only during initialization
  teamNumber = 16*!digitalRead(swHex_16) + 8*!digitalRead(swHex_8) + 4*!digitalRead(swHex_4) + 2*!digitalRead(swHex_2) + !digitalRead(swHex_1);
  sprintf(buf, "Team: %02d", teamNumber);
  centerLCD(team1, 0);                          // output upper row message
  centerLCD(buf, 1);                            // ...and then the lower row
  Serial.print(team1);
  Serial.println(buf);
  delay(2000);
  
  // check to be sure we have an acceptable team number
  if ((teamNumber == 0) || (teamNumber > MAX_TEAM_NUMBER)) {
    Serial.println("Bad team number - try again...");  
    while (true) {
      centerLCD("Bad team number",0);
      centerLCD("Try again...",1);
      delay(2000);
      lcd.clear();
      delay(1000);
    }
  }

  // init various state variables
  onboardLEDstate = 0;
  updateLCD = 0;
  updateBluetooth = 0;
  elapsedTime = 0;
  msgExpiry = 0;
  tickCount = 0;
  mskStorage = 0;
  mskSupply = 0;
  teamNumber;
  ledOnTime = 0;
  alive = false;
  reconnect = false;
  rodWarningLvl = 0;
  mstrConState = bt_not_connected;

  // now attempt to connect to the slave module 
  connectBt();					// initiate connection with the slave device
  elapsedTime = 0;				// start elapsed time counter by setting it to zero
  msgExpiry = 0;				// start msg expiry conter by setting it to zero
  sprintf(buf2, "Data: ---");
} // end setup


/*
 * ISR for timer1
 */
void timer1ISR(void) {
  tickCount++;                                  // increment the 1st 100ms tick counter
  tickCount2++;                                 // increment the 2nd 100ms tick counter
  if (updateStorageTubes()) blinkLED(3);      // check & update the storage tubes
  if (updateSupplyTubes()) blinkLED(3);       // check & update the supply tubes
  if (ledOnTime) {	                        // if ledOnTime counter is non-zero then
    digitalWrite(onboardLED, HIGH);	        // ...turn on the onboard LED
    ledOnTime--;                              // ...and decrement the counter
  } else digitalWrite(onboardLED, LOW);       // ...otherwise, turn it off

  // the following happens twice per second
  if (tickCount2 == 5) {
    tickCount2 = 0;                             // reset the tick counter
    updateBluetooth = true;
  }
  
  // the following happens once per second
  if (tickCount == 10) {
    tickCount = 0;                              // reset the tick counter
    elapsedTime++;				// increment the elapsed time counter (in seconds)
    updateLCD = true;				// set the LCD update flag
    msgExpiry++; 
  }
  
  // This is used to blink the LEd at .5ms Time period if the device is connected to the slave
  if ((alive) && (tickCount < 5))		// if the slave is alive and its .5s
     digitalWrite(HeartBeatLED, HIGH);		// make LED high for 500ms
  else
     digitalWrite(HeartBeatLED, LOW);		// make LED low for 500ms
  
  //to implement the reconnect mode if the connection is not alive
  if ((mstrConState == bt_connected) && (elapsedTime > BT_TIMEOUT)) {
    elapsedTime = 0;				// reset the counter that tracks the time since connction
    if (!alive)					// if the heartbeat is not received alive is low. so if alive is not true
      reconnect = true;				// ... then reconnect
    if (!digitalRead(debugModePin))      	// this is to overwrite the reconnect if the debug mode is set so students can connect without sending heartbeat
      reconnect = false;			// overwrite reconnect to false
    alive = false;				// ensure the alive state remains false
  } 
} // end timer1ISR


/*
* @brief this function is used to perform all the operations in order to connect to the bluetooth slave and establish connection
*/
void connectBt() {
  char buf[50];
  
  lcd.clear();
  strcpy(buf, "Bluetooth setup");               // load the message for this step
  if (genDbgMsgs) Serial.println(buf);          // output to console only if in debug mode
  centerLCD(buf, 0);		                // display the message
  mstrConState = bt_connection_in_progress;     // trying to establish the bluetooth connection to the slave
  btmaster.enterCMDMode(buf);			// make the master enter command mode
  centerLCD(buf, 1);				// display the output of entering cmdmode CMD
  delay(2000);					// wait 2 sec to ensure you see the reply 
  lcd.clear();
  
  strcpy(buf, "To Master mode");                // load the message for this step
  if (genDbgMsgs) Serial.println(buf);          // output to console only if in debug mode
  centerLCD(buf, 0);		                // display the message
  btmaster.switchMode('1', buf);		// switch the bluetooth module to be in master mode so it can connect to slave
  centerLCD(buf, 1);
  delay(1000);
  lcd.clear();
  
  strcpy(buf, "Connecting Slave");              // load the message for this step
  if (genDbgMsgs) Serial.println(buf);          // output to console only if in debug mode
  centerLCD(buf, 0);		                // display the message
  strcpy(slaveID, btADDR[teamNumber - 1]);      // load the bluetooth slave module address
  btmaster.connectRDevice(slaveID, buf);	// attempt connection with the slave id
  centerLCD(buf, 1);			        // display the result of communcition
  delay(2000);
  lcd.clear();
  
  if (strcmp(buf, "ERR-connected") == 0) {
      Serial.println("Already connected!");
    while (true);
  }
  
  strcpy(buf, "Exit CMD mode");                 // load the message for this step
  if (genDbgMsgs) Serial.println(buf);          // output to console only if in debug mode
  centerLCD(buf, 0);		                // display the message
  delay(2000);
  btmaster.exitCMDMode(buf);		        // exit the command mode so the device is in normal operation
  centerLCD(buf, 1);	
  delay(2000);
  lcd.clear();

  // see if we managed to connect to the slave unit
  if (strcmp(buf, "END") == 0) {  // if we get this, we did not connect
    mstrConState = bt_not_connected;        // bluetooth connection to the slave has not been established
    Serial.println("No connection established. Try again...");
    while (true) {
      centerLCD("No connection",0);
      centerLCD("Try again...",1);
      delay(1000);
      lcd.clear();
      delay(500);
    }
  } else {
    mstrConState = bt_connected;             // bluetooth connection to the slave has been established
    centerLCD("Connection OK", 0);
    centerLCD("Proceeding...", 1);
    Serial.println("Connection successfully established. Proceeding...");
  }
  
  if (!digitalRead(debugModePin)) Serial.println("DEBUG MODE - no heartbeat needed...");
}


/*
 * this method is called repeatedly for as long as the program runs
 */
void loop(void) {
  genDbgMsgs = !digitalRead(debugMsgEnable);    // capture debug message flag state from input switch

  // if the flag is set then attempt to reconnect
  if(reconnect) {
    connectBt();
    reconnect = false;
    elapsedTime = 0;
  }
   
  // the following code has to do with updating the LCD display
  if (updateLCD) {                             // if it's time,  update the display
    updateLCD = false;                         // clear the LCD update flag
    lcd.clear();
    sprintf(buf, "Old: %02X New: %02X", mskStorage, mskSupply);  // format the message
    lcd.setCursor(0, 0);                       // output to upper line
    lcd.print(buf);                            // send msg to LCD
    lcd.setCursor(0, 1);                       // output to lower line
    lcd.print(buf2);                           // send msg to LCD
    digitalWrite(lowlvlradLED, LOW);
    digitalWrite(highlvlradLED, LOW);
   
    // check to see if we are still receiving data from the Bluetooth receiver.
    // if not, then flag that we are disconnected and indicate so on the display
    if (msgExpiry > BT_TIMEOUT) {
      msgExpiry = 0;
      sprintf(buf2, "Data: ***");
    }
    
    switch (rodWarningLvl) {
    case 0x2C:                                 // spent rod exposed
      lcd.setCursor(14, 1);                    // output to lower line
      lcd.print("*");                          // send msg to LCD
      digitalWrite(lowlvlradLED, HIGH);
      break;
    case 0xFF:                                 // new rod exposed
      lcd.setCursor(14, 1);                    // output to lower line
      lcd.print("**");                         // send msg to LCD
      digitalWrite(highlvlradLED, HIGH);
      break;
    default:
      break;
    }  
    rodWarningLvl = 0;                         // reset the level back to zero (no rod exposed)
  }
  
  // the following code is executed only periodically - when the updateBluetooth flag has been set
  if (updateBluetooth) {		        // if it is needed to publish message to bluetooth its updated at the same frequency as the LCD
    // send out the storage and supply tube status messages
    updateBluetooth = false;		        // set flag to false so the frequency is maintained
    pcol.setDst(0x00);			        // send a broadcast message so that its address is 0x00
    data1[0] = mskStorage;			// to send the imformation for the storage tube
    sz1 = pcol.createPkt(0x01, data1, pkt1);	// create the packet that needs to be sent for the storage tube status
    btmaster.sendPkt(pkt1, sz1);		// send using the bluetooth device
    if (genDbgMsgs) Serial.println("Send=>stor");
    delay(20);
    data2[0] = mskSupply;			// the data for the supply tube 
    sz2 = pcol.createPkt(0x02, data2, pkt2);	// create the packet for the supply tube data
    btmaster.sendPkt(pkt2, sz2);		// send using the bluetooth device
    if (genDbgMsgs) Serial.println("Send=>sply");
  
    // update the status of the stop/resume pushbuttons and read their current state  
    boolean stateChanged_1 = debouncer_1.update();
    int state_1 = debouncer_1.read();
    boolean stateChanged_2 = debouncer_2.update();
    int state_2 = debouncer_2.read();
    boolean stateChanged_3 = debouncer_3.update();
    int state_3 = debouncer_3.read();
    boolean stateChanged_4 = debouncer_4.update();
    int state_4 = debouncer_4.read();
   
    // send out the appropriate robot movement message if one of the buttons has been pressed
    // detect the falling edge on the stop and resume buttons for the current and fake robots
    if (stateChanged_1 && state_1 == LOW) {
      if (robotRunning) {
        blinkLED(3);
        pcol.setDst(teamNumber);                    // send this message to the current robot
        sz1 = pcol.createPkt(0x04, data1, pkt1);    // create the packet that needs to be sent for a stop movement message
        btmaster.sendPkt(pkt1, sz1);                // send using the bluetooth device
        if (genDbgMsgs) Serial.println("Send=>CurStop");
        robotRunning = false;                       // the current robot has been ordered to stop movement
      }
    }
    if ( stateChanged_2 && state_2 == LOW ) {
      if (!robotRunning) {
        blinkLED(3);
        pcol.setDst(teamNumber);                    // send this message to the current robot
        sz1 = pcol.createPkt(0x05, data1, pkt1);    // create the packet that needs to be sent for a resume movement message
        btmaster.sendPkt(pkt1, sz1);                // send using the bluetooth device
        if (genDbgMsgs) Serial.println("Send=>CurResume");
        robotRunning = true;                       // the current robot has been ordered to resume movement
      }
    }
 
    if (stateChanged_3 && state_3 == LOW) {
      if (fakeRunning) {
        blinkLED(3);
        pcol.setDst(fakRobotID);                    // send this message to the fake robot
        sz1 = pcol.createPkt(0x04, data1, pkt1);    // create the packet that needs to be sent for a stop movement message
        btmaster.sendPkt(pkt1, sz1);                // send using the bluetooth device
        if (genDbgMsgs) Serial.println("Send=>FakStop");
        fakeRunning = false;                        // the fake robot has been ordered to stop movement
      }
    }
    if ( stateChanged_4 && state_4 == LOW ) {
      if (!fakeRunning) {
        blinkLED(3);
        pcol.setDst(fakRobotID);                    // send this message to the fake robot
        sz1 = pcol.createPkt(0x05, data1, pkt1);    // create the packet that needs to be sent for a resume movement message
        btmaster.sendPkt(pkt1, sz1);                // send using the bluetooth device
        if (genDbgMsgs) Serial.println("Send=>FakResume");
        fakeRunning = true;                         // the fake robot has been ordered to resume movement
      }
    }

    //process received pkts
    if (btmaster.readPacket(pkt)) {		// if you have received a new packet from the bluetooth
      if (genDbgMsgs) Serial.println("See a packet!");
      msgExpiry = 0;                            // note that we've seen incoming data from the robot
      sprintf(buf2, "Data: ---");
//      pcol.setDst(0x00);			// in order to check if its from the correct destination ensure that the message is in broadcast
      if (pcol.getData(pkt, data, type)) {	// extract the data and type from the packet
        if (genDbgMsgs) Serial.print("Checking address: ");
        if (genDbgMsgs) Serial.println(pkt[4]);
        if (pkt[4] == 0x00) {                   // destination address must be 0x00, i.e., broadcast / field control computer; otherwise ignore the packet
          if (genDbgMsgs) Serial.println("Address OK!");
          if (genDbgMsgs) sprintf(buf3, "Packet type: %02X", type);   // indicate what type of packet it is
          if (genDbgMsgs) Serial.println(buf3);
          switch (type) {
          case 0x03:			        // radiation alert message received (one data byte)
            rodWarningLvl = data[0];              // use the data value to set the radiation alert level
            sprintf(buf3, "RA: %02X", data[0]);   // print the radiation alert level
            if (genDbgMsgs) Serial.println("Got Radiation Alert!");
            if (genDbgMsgs) Serial.println(buf3);
            break;
          case 0x06:                              // robot status message received (three data bytes)
            sprintf(buf2, "RS: %02X %02X %02X", data[0], data[1], data[2]);  // print the robot status
            lcd.setCursor(0, 1);                  // output to lower line
            lcd.print(buf2);
            if (genDbgMsgs) Serial.println("Got Robot Status!");
            if (genDbgMsgs) Serial.println(buf2);
            break;
          case 0x07:                              // heartbeat message received (no data)
            alive = true;	                  // note that the robot is still alive
            elapsedTime = 0;		          // ...and reset the elapsed time since heartbeat to zero
            if (genDbgMsgs) Serial.println("Got Heartbeat!");
            break;
          default:
            break;
          }
          //
    //      Serial.println(type, HEX);
    //      Serial.print("with data ");
    //      if(type == 0x06) {			  // if you receive information about the robot
    //        sprintf(buf2, "Data: %0X %0X %0X", data[0], data[1], data[2]);  // print the robot status
    //        Serial.println(buf2);
    //      }
    //      else {
    //        sprintf(buf2, "Data: %0X: %0X", type, data[0]); // for debug purpose display the data and type of packet received
    //        Serial.println(buf2);
    //      }
        }
      }
    }
  }
} // end loop


/*
 * This method is called to output a centered message to the LCD
 * - the first parameter is a pointer to the message buffer
 * - the second parameter is the row that is to get the message
 */
void centerLCD(char* msg, unsigned int line) {
  lcd.setCursor((16 - strlen(msg)) / 2, line); // center message buffer content on the specified line of the display
  lcd.print(msg);                              // ...and then output msg to the LCD
} // end centerLCD


/*
 * This method is called to blink the onboard LED
 * - the first parameter is the time in 100 milliseconds chunks for the LED to be on
 */
void blinkLED(int time) {
  ledOnTime = time;
} // end blinkLED


/*
 * This method is called to query the storage tube switches, turn the corresponding LEDs
 * on or off as appropriate, and then update the storage mask accordingly.
 * - the boolean return value indicates whether there is a change in the state of any of
 *   the storage tubes since the last time the method was invoked
 */
int updateStorageTubes(void) {
  unsigned char oldMask;
  
  oldMask = mskStorage;                        // get the current storage tube mask state
  if (digitalRead(swStorageTube1)) {           // if the rod is in the tube
    mskStorage |= 0x01;                        // ...set the appropriate bit in the storage tube mask
    digitalWrite(ledStorageTube1, HIGH);       // ...and turn on the associated LED
  } else {                                     // ...otherwise
    mskStorage &= 0xFE;                        // ...clear the appropriate bit in the storage tube mask
    digitalWrite(ledStorageTube1, LOW);        // ...and turn off the associated LED
  }
  if (digitalRead(swStorageTube2)) {
    mskStorage |= 0x02;
    digitalWrite(ledStorageTube2, HIGH);
  } else {
    mskStorage &= 0xFD;
    digitalWrite(ledStorageTube2, LOW);
  }
  if (digitalRead(swStorageTube3)) {
    mskStorage |= 0x04;
    digitalWrite(ledStorageTube3, HIGH);
  } else {
    mskStorage &= 0xFB;
    digitalWrite(ledStorageTube3, LOW);
  }
  if (digitalRead(swStorageTube4)) {
    mskStorage |= 0x08;
    digitalWrite(ledStorageTube4, HIGH);
  } else {
    mskStorage &= 0xF7;
    digitalWrite(ledStorageTube4, LOW);
  }
  
  if (oldMask == mskStorage) {                 // if the masks are the same, then nothing has changed
    return false;                              // ...so return false
  } else {                                     // ...otherwise
    return true;                               // ...something has changed so return true
  }
} // end updateStorageTubes


/*
 * This method is called to query the supply tube switches, turn the corresponding LEDs
 * on or off as appropriate, and then update the supply mask accordingly.
 * - the boolean return value indicates whether there is a change in the state of any of
 *   the supply tubes since the last time the method was invoked
 */
int updateSupplyTubes(void) {
  unsigned char oldMask;
  
  oldMask = mskSupply;                         // get the current storage tube mask state
  if (digitalRead(swSupplyTube1)) {            // if the rod is in the tube
    mskSupply |= 0x01;                         // ...set the appropriate bit in the supply tube mask
    digitalWrite(ledSupplyTube1, HIGH);        // ...and turn on the associated LED
  } else {                                     // ...otherwise
    mskSupply &= 0xFE;                         // ...clear the appropriate bit in the storage tube mask
    digitalWrite(ledSupplyTube1, LOW);         // ...and turn off the associated LED
  }
  if (digitalRead(swSupplyTube2)) {
    mskSupply |= 0x02;
    digitalWrite(ledSupplyTube2, HIGH);
  } else {
    mskSupply &= 0xFD;
    digitalWrite(ledSupplyTube2, LOW);
  }
  if (digitalRead(swSupplyTube3)) {
    mskSupply |= 0x04;
    digitalWrite(ledSupplyTube3, HIGH);
  } else {
    mskSupply &= 0xFB;
    digitalWrite(ledSupplyTube3, LOW);
  }
  if (digitalRead(swSupplyTube4)) {
    mskSupply |= 0x08;
    digitalWrite(ledSupplyTube4, HIGH);
  } else {
    mskSupply &= 0xF7;
    digitalWrite(ledSupplyTube4, LOW);
  }
  
  if (oldMask == mskSupply)                    // if the masks are the same, then nothing has changed
    return false;                              // ...so return false
  else                                         // ...otherwise
    return true;                               // ...something has changed so return true
} // end updateSupplyTubes

