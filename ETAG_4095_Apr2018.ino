/*
  Data logging sketch for the ETAG RFID Reader
  Version 1.1 
  Code by:
   Alexander Moreno
   David Mitchell
   Eli Bridge
   Jay Wilhelm
   May-2018

  Licenced in the public domain

  REQUIREMENTS:
  The files "logger.cpp" and "logger.h" must be in the same folder as this sketch file
  Power supply for the board should come from the USB cable or a 5V battery or DV power source.
  A 3.3V CR1025 coin battery should be installed on the back side of the board to maintain the date
      and time when the board is disconnected from a primary power source.

  FLASH MEMORY STRUCTURE:
  The onboard flash memory is divided into pages of 528 bytes each. There are probably several thousand pages.
  Page 1 is reserved for RFID tag codes
  Page 2 is reserved for parameters and counters
  The rest is for backup memory.

  Change log
  5-4-18 - Added interrupt driven RFID reader (jaywilhelm)
*/

//***********INITIALIZE INCLUDE FILES AND I/O PINS*******************
//#include "logger.h"      //include the library for reading and parsing raw RFID input
#include "ManchesterDecoder.h" //Interrupt driven RFID decoder
#include "RTClib.h"
#include <Wire.h>        //include the standard wire library - used for I2C communication with the clock
#include <SD.h>          //include the standard SD card library
#include <SPI.h>

#define serial SerialUSB     //Designate the USB connection as the primary serial comm port
#define DEMOD_OUT_PIN   30   //(PB03) this is the target pin for the raw RFID data
#define SHD_PINA         8   //(PA06) Setting this pin high activates the primary RFID circuit (only one can be active at a time)
#define SHD_PINB         9    //(PA07) Setting this pin high activates the seconday RFID circuit (only one can be active at a time)
#define MOD_PIN          0    //not used - defined as zero
#define READY_CLOCK_PIN  0    //not used - defined as zero
#define SDselect         7    //Chip select for SD card - make this pin low to activate the SD card, also the clock interupt pin
#define csFlash         2   //Chip select for flash memory
#define LED_RFID        31    //Pin to control the LED indicator.  
//#define MOTR            13  //motor reverse
//#define MOTF            11  //motor forward
//#define mSwitch         12  //motor switch

ManchesterDecoder gManDecoder1(DEMOD_OUT_PIN,SHD_PINA,ManchesterDecoder::EM4095);
ManchesterDecoder gManDecoder2(DEMOD_OUT_PIN,SHD_PINB,ManchesterDecoder::EM4095);

//logger L(DEMOD_OUT_PIN, MOD_PIN, READY_CLOCK_PIN); //Designate pins used in the logger library
RTC_RV1805 rtc;

//************************* initialize variables******************************
//byte tagData[5];                   //Variable array of 5 bytes for storing RFID tag codes
//byte tagData2[5];                  //Alternate variable array of 5 bytes for storing RFID tag codes
byte flashArray[528];              //Large array of 528 bytes for writing a full page to onboard flash memory 
byte tagCount;                     //keeps track of number of stored tags
byte byte0;                        //general purpose reusable variable
long long0;                        //general purpose reusable variable
unsigned int pageAddress;          //page address for flash memory
unsigned int byteAddress;          //byte address for flash memory
union flashMem {                   //Union variable for constructing instructions to flash memory
  unsigned long flashAddress;
  byte flashAddrByte[4];
};
//byte tagString;                   // ??
//byte tagString2;                  // ??
byte match;                       // used to determine if tags match.
unsigned long currentMillis;      //Used for exploiting the millisecond counter for timing functions - stores a recent result of the millis() function
unsigned long stopMillis;         //Used for exploiting the millisecond counter for timing functions - stores a less recent result of the millis() function
byte RFcircuit = 1;               //Used to determine which RFID circuit is active. 1 = primary circuit, 2 = secondary circuit.
unsigned int serDelay;            //Used for timing on initiation of serial communication
byte ss, mm, hh, da, mo, yr;          //Byte variables for storing date/time elements
String sss, mms, hhs, das, mos, yrs;  //String variable for storing date/time text elements
String timeString;                    //String for storing the whole date/time line of data
byte incomingByte = 0;                 //Used for incoming serial data
unsigned int timeIn[12];              //Used for incoming serial data during clock setting
byte menu;
byte feedMode = 'O';
unsigned int mDelay = 0;
unsigned int mDelay2 = 0;
volatile bool SLEEP_FLAG;

//********************CONSTANTS (SET UP LOGGING PARAMETERS HERE!!)*******************************
const unsigned int polltime = 3000;       //How long in milliseconds to poll for tags
const unsigned int pausetime = 500;       //How long in milliseconds to wait between polling intervals
const unsigned int readFreq = 200;        //How long to wait after a tag is successfully read.
byte slpH = 22;                            //When to go to sleep at night - hour
byte slpM = 00;                            //When to go to sleep at night - minute
byte wakH = 06;                            //When to wake up in the morning - hour             
byte wakM = 30;                            //When to wake up in the morning - minute   
void getTime();
byte readFlashByte(unsigned long fAddress);
void writeFlashByte(unsigned long fAddress, byte fByte);
unsigned long getFlashAddr();
void writeFlashAddr(unsigned long fAddress);
void setClk();
void dumpMem();
//*******************************SETUP**************************************
void setup() {  // This function sets everything up for logging.
  pinMode(SDselect, OUTPUT);     // Chip select pin for SD card must be an output
  pinMode(csFlash, OUTPUT);      // Chip select pin for Flash memory
  digitalWrite(SDselect, HIGH);  //Make both chip selects high (not selected)
  digitalWrite(csFlash, HIGH);
  gManDecoder1.DisableMonitoring();

  //pinMode(SHD_PINA, OUTPUT);     // Make the primary RFID shutdown pin an output.
  //digitalWrite(SHD_PINA, HIGH);   // turn the primary RFID circuit off (LOW turns on the EM4095)
  //pinMode(SHD_PINB, OUTPUT);
  //digitalWrite(SHD_PINB, HIGH);   // turn the secondary RFID circuit off (LOW turns on the EM4095)
  pinMode(LED_RFID, OUTPUT);
  digitalWrite(LED_RFID, HIGH);  // turn the LED off (LOW turns it on)
//  pinMode(MOTR, OUTPUT);
//  digitalWrite(MOTR, LOW);       // turn motor off
//  pinMode(MOTF, OUTPUT);
//  digitalWrite(MOTF, LOW);       // turn motor off
//  pinMode(mSwitch, INPUT_PULLUP); // motor switch enabled as input with internal pullup resistor 
  
  rtc.begin();  // Is this needed? Seems to be necessary - could be made more efficient though
                   
  //Try to initiate a serial connection
  serial.begin(115200);               //Initiate a serial connection with the given baud rate
  ss = 0;                           //Initialize a variable for counting in the while loop that follows
  while (ss < 5 && !serial) {       //flash LED 5 times while waiting for serial connection to come online
    delay(500);                     //pause for 0.5 seconds
    digitalWrite(LED_RFID, LOW);    //turn the LED on (LOW turns it on)
    delay(500);                     //pause again
    digitalWrite(LED_RFID, HIGH);   //turn the LED off (HIGH turns it off)
    ss = ss + 1;                    //add to counting variable
  }//end while
  digitalWrite(LED_RFID, HIGH);     // make sure LED is off

  getTime();                      //Read from the time registers 
  serial.println();
  serial.print("Clock set to ");  //message confirming clock time
  serial.println(timeString);

  //Set up the SD card
  serial.print("Initializing SD card...\n");              //message to user
  if (!SD.begin(SDselect)) {                            //Initiate the SD card function with the pin that activates the card.
    serial.println("\nSD card failed, or not present");   //SD card error message
    //return;
  }// end check SD
  digitalWrite(SDselect, HIGH); //Make sure SD card is turned off
  
  //Set up communication with the flash memory
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  //transferTags();  //Transfer tag data to Flash memory
  //  flashArray[0] = 0;
  //  readByte();
  // serial.print("Number of tags transferred: ");
  // serial.println(tagCount, DEC);

  serial.print("check Flash Memory initialized: ");
  byte0 = readFlashByte(0x00000404);
  serial.println(byte0, HEX);
  if (byte0 == 0xFF) {
    writeFlashByte(0x00000404, 0xAA);
    byte0 = 0xCC;
    serial.print("Initializing Flash Memory: ");
    writeFlashAddr(0x00000800); //initialize flash memory to page 2, byte address 0
    byte0 = readFlashByte(0x00000404);
    serial.println(byte0, HEX);
  }
  serial.println("Flash Memory IS initialized: ");
  serial.print("Current flash address (binary no leading zeros): ");
  serial.println(getFlashAddr(), BIN);
  //writeFlashByte(0x00000404, 0xFF);   #Uncomment to set flash memory initialization flag to 0xFF, which will cause the memory address to be reset on startup

//  serial.print("Current feed mode: ");
//  serial.println(feedMode);

  menu = 1;
  while (menu == 1) {
    serial.println();
    serial.println("What to do?");             //Ask the user for instruction
    serial.println("    C = set clock");             
    serial.println("    B = Display backup memory");
//    serial.println("    O = set feeder mode to 'OPEN'");
//    serial.println("    A = set feeder mode to 'ALL TAGGED'");
//    serial.println("    T = set feeder mode to 'TARGETED'");
//    serial.println("    M = test and adjust motor");  
    serial.println("    Anything else = start logging");           
    serDelay = 0;                                      //If there's no response then just start logging
    while (serial.available() == 0 && serDelay < 10000) { //wait about 10 seconds for a user response
      delay(1);
      serDelay++;
    }
    if (serial.available()) {          //If there is a response then perform the corresponding operation
      incomingByte = serial.read();
      switch(incomingByte) {
      case 'C': {        //set clock
        setClk();  
      }
      case 'B': {         //display memory
        dumpMem();
      }
      case 'M': {         //test motor
        //motorOpen();
        delay(1000);
        //motorClose();
      }
      break;
      default:
        menu = 0; 
      break;
      }
    } else {                           //if there is no response then set menu to 1 to exit the loop
      menu = 0;
    }
  }
  
//  getTime();
//  serial.println("time is: "); 
//  serial.println(timeString);
//  wakH = bcdToDec(hh);
//  wakM = bcdToDec(mm)+1;
//  setAlarm();
//  delay(10);
//  showClock();

//
//  SLEEP_FLAG = true;
//  pinMode(SDselect, INPUT);     // Chip select pin for SD card must be an output
//
//  serial.print("TESTING SLEEP FUNCTION. SLEEP FLAG IS: ");
//  serial.println(SLEEP_FLAG);
//  if (SLEEP_FLAG == true) {
//      serial.println("I'm going to sleep now.");
//      __WFI();  // wake from interrupt
//     SLEEP_FLAG = false;
//     serial.println("Ok, I'm awake");
//     serial.println();
//  }
// 
//
//  serial.println("AWAKE!"); 
//  
//  

  RFcircuit = 1;  //Indicates that the reader should start with the primary RFID circuit
//  RFcircuit = 2;  //Indicates that the reader should start with the secondary RFID circuit  
  serial.println("Scanning for tags...\n");   //message to user
} // end setup




//******************************MAIN PROGRAM*******************************

void loop() {  //This is the main function. It loops (repeats) forever.
  if (RFcircuit == 1)               //Determin which RFID circuit to activate
    {digitalWrite(SHD_PINA, LOW);} //Turn on primary RFID circuit
    else 
    {digitalWrite(SHD_PINB, LOW);} //Turn on secondary RFID circuit
  
  serial.print("Scanning RFID circuit "); //Tell the user which circuit is active
  serial.println(RFcircuit);

  //scan for a tag - if a tag is sucesfully scanned, return a 'true' and proceed
  currentMillis = millis();                //To determine how long to poll for tags, first get the current value of the built in millisecond clock on the processor
  stopMillis = currentMillis + polltime;   //next add the value of polltime to the current clock time to determine the desired stop time.
  while (stopMillis > millis()) {          //As long as the stoptime is less than the current millisecond counter, then keep looking for a tag
    EM4100Data xd; //special structure for our data
    //alternative is to turn on decoding, wait, then check for tags....
    if(gManDecoder1.DecodeAvailableData(&xd) > 0)
    {   
      //serial.print("RFID Tag Detected: "); //Print a message stating that a tag was found 
      getTime();                           //Call a subroutine function that reads the time from the clock
      displayTag(&xd);                        //Call a subroutine to display the tag data via serial USB
      flashLED();
      logRFID_To_SD(&xd);
      writeRFID_To_FlashLine(&xd);  //function to log to backup memory
      //match = checkTag();
      //serial.print("Match?: ");
      //serial.println(match, DEC);
    } // end ScanForTag
    else
    {
      gManDecoder1.EnableMonitoring();//Let the data collection run in the background
    }
    if(gManDecoder2.DecodeAvailableData(&xd) > 0)
    {   
      //serial.print("RFID 2 Tag Detected: "); //Print a message stating that a tag was found 
      getTime();                           //Call a subroutine function that reads the time from the clock
      displayTag(&xd);                        //Call a subroutine to display the tag data via serial USB
      flashLED();
      logRFID_To_SD(&xd);
      writeRFID_To_FlashLine(&xd);  //function to log to backup memory
      //match = checkTag();
      //serial.print("Match?: ");
      //serial.println(match, DEC);
    } // end ScanForTag
    else
    {
      gManDecoder2.EnableMonitoring();//Let the data collection run in the background
    }
  } //end while

  //The following gets executed when the above while loop times out
  //gManDecoder1.DisableMonitoring();

  
  
  //digitalWrite(SHD_PINA, HIGH);    //Turn off both RFID circuits
  //digitalWrite(SHD_PINB, HIGH);    //Turn off both RFID circuits
  delay(pausetime);               //pause between polling attempts
//      if (RFcircuit == 1)             //switch between active RF circuits.
//        {RFcircuit = 2;}              // comment out the if statement to use just 1 RFID circuit
//        else
//        {RFcircuit = 1;}
  RFcircuit = 1;              //This lines sets the active RF circuit to 1. comment out or delete to use both circuits. Uncomment if you just want to use the primary circuit.
}// end void loop


//*********************SUBROUTINES*************************//

////The Following are all subroutines called by the code above.//////////////

byte bcdToDec(byte val)  {   // Convert binary coded decimals (from the clock) to normal decimal numbers
  return ( (val / 16 * 10) + (val % 16) );
}

byte decToBcd( byte val ) {  // Convert decimal to binary coded decimals for writing to the clock
  return (byte) ((val / 10 * 16) + (val % 10));
}

/*static uint8_t conv2d(const char* p) { // Convert parts of a string to decimal numbers (not used in this version)
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}*/

void initclk() {                //Start the clock running if it is not already
  Wire.begin();                  //Start up the I2C comm funcitons
  Wire.beginTransmission(0x68);  //Send the clock slave address
  Wire.write(0x0C);              //address for clearing the HT (halt time?) bit
  Wire.write(0x3F);              //HT is bit 6 so 00111111 clears it.
  Wire.endTransmission();        //End the I2C transmission
}

//void setClk() {                          //Function to set the clock
//  serial.println("Enter mmddyyhhmmss");  //Ask for user input
//  while (serial.available() == 0) {}    //wait for 12 characters to accumulate
//  for (int n = 0; n < 13; n++) {        //loop to read all the data from the serial buffer once it is ready
//    timeIn[n] = serial.read();         //Read the characters from the buffer into an array of 12 bytes one at a time
//  }
//  while (serial.available())           //Clear the buffer, in case there were extra characters
//  {
//    serial.read();
//  }
//
//  mo = ((timeIn[0] - 48) * 10 + (timeIn[1] - 48)); //Convert two ascii characters into a single decimal number
//  da = ((timeIn[2] - 48) * 10 + (timeIn[3] - 48)); //Convert two ascii characters into a single decimal number
//  yr = ((timeIn[4] - 48) * 10 + (timeIn[5] - 48)); //Convert two ascii characters into a single decimal number
//  hh = ((timeIn[6] - 48) * 10 + (timeIn[7] - 48)); //Convert two ascii characters into a single decimal number
//  mm = ((timeIn[8] - 48) * 10 + (timeIn[9] - 48)); //Convert two ascii characters into a single decimal number
//  ss = ((timeIn[10] - 48) * 10 + (timeIn[11] - 48)); //Convert two ascii characters into a single decimal number
//
//  //Write the new time to the clock usind I2C protocols implemented in the Wire library
//  initclk();                    //Make sure the clock is running
//  Wire.beginTransmission(0x68); //Begin I2C communication using the I2C address for the clock
//  Wire.write(0x00);             //starting register - register 0
//  Wire.write(0x00);             //write to register 0 - psecs (100ths of a second - can only be set to zero
//  Wire.write(decToBcd(ss));     //write to register 1 - seconds
//  Wire.write(decToBcd(mm));     //write to register 2 - minutes
//  Wire.write(decToBcd(hh));     //write to register 3 - hours
//  Wire.write(0x01);             //write to register 4 - day of the week (we don't care about this)
//  Wire.write(decToBcd(da));     //write to register 5 - day of month
//  Wire.write(decToBcd(mo));     //write to register 6 - month
//  Wire.write(decToBcd(yr));     //write to register 7 - year
//  Wire.endTransmission();
//  delay(10);
//  getTime();                      //Read from the time registers you just set
//  serial.print("Clock set to ");  //message confirming clock time
//  serial.println(timeString);
//
//  //When the clock is set (more specifically when serial.read is used) the RFID circuit fails)
//  //I don't know why this happens
//  //Only solution seems to be to restart the device.
//  //So the following messages inform the user to restart the device.
//  serial.print("Restart reader to log data.");
//  while (1) {}                                   //Endless while loop. Program ends here. User must restart.
//}


void setAlarm() {  //Set alarm registers
  serial.println("setting alarm....");
  writeI2C(0x68, 0x0E, 0); //write to register 0x0E (alarm seconds) - Do this first because you cannot leave the counter sitting at this register
  delay(10);
  writeI2C(0x68, 0x0A, B10010001);  //write to register 0x0A (Alarm Month) - Bits 7-4 must be 1001 to enable the alarm, bits 3-0 designate the month value, but this does not matter for an alarm repeated daily
  delay(10);
  writeI2C(0x68, 0x0B, B11000001); //write to register 0x0B (Alarm Day) - Repeat bits 7-6 set to 1, bits 5-0 for day value do not matter (set to 1)
  delay(10);
  writeI2C(0x68, 0x0C, B00111111 & decToBcd(wakH)); //write to register 0x0C (Alarm Hour) - Repeat bits 7-6 set to 0, bits 5-0 for hour value
  delay(10);
  writeI2C(0x68, 0x0D, B01111111 & decToBcd(wakM)); //write to register 0x0D - bit 7 must be 0, bits 6-0 are the minutes wake up value.
  delay(10);



//  serial.println("setting alarm....");
//  Wire.begin();                 //Start I2C functions
//  Wire.beginTransmission(0x68); //Begin I2C communication using the I2C address for the clock
//  Wire.write(0xA0);             //set to starting register - register h0A: Alarm Month
//  Wire.write(0x81);    //B10010001    //write to register 0x0A - Bits 7-4 must be 1001 to enable the alarm, bits 3-0 designate the month value, but this does not matter for an alarm repeated daily
//  Wire.write(0x1);        //write to register 0x0B - Repeat bits 7-6 set to 1, bits 5-0 for day value do not matter (set to 1)
//  //Wire.write(B00111111 & decToBcd(wakH));    //write to register 0x0C - bits 7-6 must be 0, bits 6-0 are the hour wake up value.
//  //Wire.write(B01111111 | decToBcd(wakM));    //write to register 0x0D - bit 7 must be 0, bits 6-0 are the minutes wake up value.
//  Wire.endTransmission();
//  delay(10);
//  Wire.beginTransmission(0x68);  //I2C address for the clock
//  Wire.write(0x0F);              //start to read from register 1 (seconds)
//  Wire.endTransmission();
//  Wire.requestFrom(0x68, 1);     //Request bytes
//  if (Wire.available()) {
//    byte0 = Wire.read(); //read
//  }
//  Wire.endTransmission();
}

void writeI2C(byte dev, byte addr, byte val) {  //Set alarm registers
  Wire.begin();                 //Start I2C functions
  Wire.beginTransmission(dev); //Begin I2C communication using the I2C address for the clock
  Wire.write(addr);             //set register 
  Wire.write(val);              //write to register   
  Wire.endTransmission();
}


void showClock() {  
  Wire.beginTransmission(0x68);  //I2C address for the clock
  Wire.write(0x00);              //start to read from register 0
  Wire.endTransmission();
  Wire.requestFrom(0x68, 20);     //Request bytes
  if (Wire.available()) {
    for (int i=0; i <= 20; i++){
      byte0 = Wire.read(); //read
      serial.print("Clock register ");
      serial.print(i, HEX);
      serial.print(" ");
      serial.print(bcdToDec(byte0), DEC);
      serial.print(" ");
      serial.println(byte0, BIN);
    }
  }
  Wire.endTransmission();
}

void getTime() {  //Read in the time from the clock registers
  DateTime now = rtc.now();
  ss = now.second(); //second
  mm = now.minute(); //minute
  hh = now.hour(); //hour
  da = now.month(); //day of month
  mo = now.month(); //month
  yr = now.year(); //year
  timeString = String(now.month()) + "/" + String(now.day()) + "/" +
               String(now.year()) + " " + String(now.hour()) + ":" + 
               String(now.minute()) + ":" + String(now.second()); 
}
//void getTime() {  //Read in the time from the clock registers
//  Wire.beginTransmission(0x68);  //I2C address for the clock
//  Wire.write(0x01);              //start to read from register 1 (seconds)
//  Wire.endTransmission();
//  Wire.requestFrom(0x68, 7);     //Request seven bytes from seven consecutive registers on the clock.
//  if (Wire.available()) {
//    ss = Wire.read(); //second
//    mm = Wire.read(); //minute
//    hh = Wire.read(); //hour
//    da = Wire.read(); //day of week, which we don't care about. this byte gets overwritten in next line
//    da = Wire.read(); //day of month
//    mo = Wire.read(); //month
//    yr = Wire.read(); //year
//  }
//  Wire.endTransmission();
//  MakeTimeString();
//}

void MakeTimeString() {
  sss = ss < 10 ? "0" + String(bcdToDec(ss), DEC) : String(bcdToDec(ss), DEC); //These lines convert decimals to characters to build a
  mms = mm < 10 ? "0" + String(bcdToDec(mm), DEC) : String(bcdToDec(mm), DEC); //string with the date and time
  hhs = hh < 10 ? "0" + String(bcdToDec(hh), DEC) : String(bcdToDec(hh), DEC); //They use a shorthand if/then/else statement to add
  das = da < 10 ? "0" + String(bcdToDec(da), DEC) : String(bcdToDec(da), DEC); //leading zeros if necessary
  mos = mo < 10 ? "0" + String(bcdToDec(mo), DEC) : String(bcdToDec(mo), DEC);
  yrs = yr < 10 ? "0" + String(bcdToDec(yr), DEC) : String(bcdToDec(yr), DEC);
  timeString = mos + "/" + das + "/" + yrs + " " + hhs + ":" + mms + ":" + sss; //Construct the date and time string
}


void displayTag(EM4100Data *xd) {
  /*serial.print("Tag ");
  for (int n = 0; n < 5; n++) {             //loop to send tag data over serial comm one byte at a time
    if (tagData[n] < 10) serial.print("0"); //send a leading zero if necessary (so "03" does not get shortened to "3")
    serial.print(tagData[n], HEX);          //Send byte
  }*/

  serial.println();
  
  char tbuf[32];
  serial.print("Card Number (hex): ");
  ManchesterDecoder::GetHexString(xd,tbuf,sizeof(tbuf));
  serial.println(tbuf);
  
  uint32_t cardNum = ManchesterDecoder::ConvertEM4100DataToNumber(xd);
  serial.print("Card Number (dec): ");
  serial.println(cardNum);
    
  serial.print(" detected on antenna ");  // add a note about which atenna was used
  serial.print(RFcircuit);
  serial.print(" at ");           // add the time the tag was logged and complete the data line
  serial.println(timeString);
}

void flashLED() {
  for (unsigned int n = 0; n < readFreq ; n = n + 30) {      //loop to Flash LED and delay reading after getting a tag
    digitalWrite(LED_RFID, LOW);                          //The approximate duration of this loop is determined by the readFreq value
    delay(5);                                             //It determines how long to wait after a successful read before polling for tags again
    digitalWrite(LED_RFID, HIGH);
    delay(25);
  }
}

void logRFID_To_SD(EM4100Data *xd) {
  File dataFile = SD.open("datalog.txt", FILE_WRITE);        //Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
  if (dataFile) {
    /*for (int n = 0; n < 5; n++) {               //loop to print out the RFID code to the SD card
      if (tagData[n] < 10) dataFile.print("0"); //add a leading zero if necessary
      dataFile.print(tagData[n], HEX);          //print to the SD card
    }*/
    char tbuf[48]; 
    ManchesterDecoder::GetHexString(xd,tbuf,sizeof(tbuf));
    
    dataFile.print(tbuf);
    dataFile.print(",");                        //comma for data delineation
    dataFile.print(RFcircuit);                  //log which antenna was active
    dataFile.print(",");                        //comma for data delineation
    dataFile.println(timeString);               //log the time
    dataFile.close();                           //close the file
    serial.println("saved to SD card.");        //serial output message to user
  } // check dataFile is present
  else {
    serial.println("error opening datalog.txt");  //error message if the "datafile.txt" is not present or cannot be created
  }// end check for file
}

byte checkTag() {
  byte tagString;
  char tagData[16];
  char tagData2[16];
  File myfile = SD.open("TAGS.TXT");  // attempt to open the file with a list of tag numbers
  if (myfile) {                       // if the file is available, read the file
    while (myfile.available()) {
      for (int n = 0; n < 5; n++) {             //loop to send tag data over serial comm one byte at a time
        tagString = asciiToHex(myfile.read());
        tagData2[n] = (tagString << 4) + asciiToHex(myfile.read());
      }
      myfile.read();          //Read in the line return but don't do anything with it.
      match = 1;
      for (int n = 0; n < 5; n++)
        if (tagData[n] != tagData2[n]) {
          match = 0;
          break;
        }
      if (match == 1) {  //they all matched
        //serial.println("Match found");
        while (myfile.available()) myfile.read(); //read the rest of the buffer to clear it.
        myfile.close();
        return match;
      }
      //serial.print(tagString, HEX);
      //serial.println(tagString2, HEX);
      //serial.write(myfile.read());
    }
    myfile.close();
    return match;
  }
  return 0;
  //else {backup plan in case SD card is down...Read tag files from eeprom? from Flash?}
}

byte asciiToHex(byte x) {
  if (x > 0x39) x -= 7; // adjust for hex letters upper or lower case
  x -= 48;
  return x;
}

//void transferTags() {
//  tagCount = 0;
//  byteAddress = 0;
//  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
//  //erasePage0();
//  delay(20);
//  File myfile = SD.open("TAGS.TXT");  // attempt to open the file with a list of tag numbers
//  //  if(!myfile) {
//  //    return;
//  //  }
//  //  else
//  if (myfile) {                      // if the file is available, read the file
//    serial.println("File found");
//    while (myfile.available()) {
//      for (int n = 0; n < 5; n++) {             //loop to read tag data and send over serial comm one byte at a time
//        tagString = asciiToHex(myfile.read());
//        tagData2[n] = (tagString << 4) + asciiToHex(myfile.read());
//        if (tagData2[n] < 10) serial.print("0"); //add a leading zero if necessary
//        serial.print(tagData2[n], HEX);
//        //serial.print(" ");
//      }
//      serial.println();
//      myfile.read();          //Read in the line return but don't do anything with it.
//      tagCount += 1;           //add 1 to tagCount
//      //pageAddress = 0;
//      //flashAddress = (pageAddress << 10) + byteAddress;
//
//      digitalWrite(csFlash, LOW); //activate flash chip
//      SPI.transfer(0x58); //opcode for read modify write
//      //SPI.transfer((flashAddress >> 16) & 0xFF); // first of three address bytes
//      //SPI.transfer((flashAddress >> 8) & 0xFF);  // second address byte
//      //SPI.transfer(flashAddress & 0xFF);        // third address byte
//      SPI.transfer(0x00);                  //first of three address bytes
//      SPI.transfer(byteAddress >> 8);      // second address byte
//      SPI.transfer(byteAddress & 0xFF);   // third address byte
//      //SPI.transfer(0x03);
//      for (int n = 0; n < 5; n++) {
//        SPI.transfer(tagData2[n]);
//        //        //SPI.transfer(0x77);
//        //        serial.print(tagData2[n], HEX);
//        //        serial.print(" ");
//      }
//      //      serial.println(byteAddress, DEC);
//      byteAddress += 5;
//      digitalWrite(csFlash, HIGH); //deactivate flash chip - allow write to happen
//      delay(20);
//    } //end while
//  } //end else
//  return;
//} // end function transferTags

void erasePage0() {
  //SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csFlash, LOW); //activate flash chip
  SPI.transfer(0x81); //opcode for page erase
  SPI.transfer(0); // first of three address bytes
  SPI.transfer(0);  // second address byte
  SPI.transfer(0);        // third address byte
  digitalWrite(csFlash, HIGH); //deactivate flash chip - allow erase to happen
}

void dumpMem() {
  char tagData[16];
  serial.println("Transmitting data from backup memory.");
  unsigned long fAddressEnd = getFlashAddr();   // get flash address
  serial.print("last flash memory address: ");
  serial.println(fAddressEnd, BIN);
  unsigned long fAddress = 0x00000800;          // first address for stored data
  while (fAddress < fAddressEnd) {
    digitalWrite(csFlash, LOW);                   // activate flash chip
    SPI.transfer(0x03);                           // opcode for low freq read
    SPI.transfer(fAddress >> 16);               // write most significant byte of Flash address
    SPI.transfer((fAddress >> 8) & 0xFF);       // second address byte
    SPI.transfer(fAddress & 0xFF);              // third address byte
    while ((fAddress & 0x000003FF) < 500) {       // repeat while the byte address is less than 500
      serial.print("from flash memory address: ");
      serial.println(fAddress, BIN);
//    tagData[0] = SPI.transfer(0);
//    tagData[1] = SPI.transfer(0);
//    tagData[2] = SPI.transfer(0);
//    tagData[3] = SPI.transfer(0);
//    tagData[4] = SPI.transfer(0);  
    for (int n = 0; n < 5; n++) {             // loop to read in an RFID code from the flash and send it out via serial comm
       tagData[n] = SPI.transfer(0);
       if (tagData[n] < 10) serial.print("0"); // add a leading zero if necessary
       serial.print(tagData[n], HEX);         // send out tag datum
       }
    RFcircuit = SPI.transfer(0);               // read which antenna was active
    mo = SPI.transfer(0);                      // read in date and time
    da = SPI.transfer(0);
    yr = SPI.transfer(0);
    hh = SPI.transfer(0);
    mm = SPI.transfer(0);
    ss = SPI.transfer(0);
//    for (int n = 0; n < 5; n++) {             // loop to read in an RFID code from the flash and send it out via serial comm
//       if (tagData[n] < 10) serial.print("0"); // add a leading zero if necessary
//       serial.print(tagData[n], HEX);         // send out tag datum
//       }
    serial.print(",");                       // comma for delineation
    serial.print(RFcircuit);                  // which circuit
    serial.print(",");                        // comma for delineation
    MakeTimeString();                         // transform bcd time data into a string
    serial.println(timeString);               // send out date/time
    fAddress = fAddress + 12;                 // update flash address
    if (fAddress >= fAddressEnd) break;       // break if we are at the end of the backup data stream
    }
    // When the byte address exceeds 500 the page address needs to be incremented
    fAddress = (fAddress & 0xFFFFC00) + 0x0400;   //set byte address to zero and add 1 to the page address
    digitalWrite(csFlash, HIGH);              // turn off flash
    if (fAddress >= fAddressEnd) break;       // break if we are at the end of the backup data stream
    delay(10);                                 // wait a bit
  }
}


unsigned long getFlashAddr() {    //get the address counter for the flash memory from page 1 address 0
  unsigned long fAddress;
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x03);           //opcode for low freq read
  SPI.transfer(0x00);           //first of three address bytes
  SPI.transfer(0x04);           // 00000100  second address byte - selects page 1
  SPI.transfer(0x00);           // third address byte selects byte address 0
  fAddress = SPI.transfer(0) & 0xFF;               //Shift in the address value
  fAddress = (fAddress << 8) + SPI.transfer(0);   //Shift in the address value
  fAddress = (fAddress << 8) + SPI.transfer(0);   //Shift in the address value
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  return fAddress;
}

void writeFlashAddr(unsigned long fAddress) {           //write the address counter for the flash memory on page 1 byte address 0
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x58);           //opcode for read modify write
  SPI.transfer(0x00);           //first of three address bytes
  SPI.transfer(0x04);           // 000001oo  second address byte - selects page 1
  SPI.transfer(0x00);           // third address byte selects byte address 0
  SPI.transfer(fAddress >> 16);           // write most significant byte of Flash address
  SPI.transfer((fAddress >> 8) & 0xFF);   // second address byte
  SPI.transfer(fAddress & 0xFF);          // third address byte
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  delay(20);
}

void writeRFID_To_FlashLine(EM4100Data *xd) {
  unsigned long fAddress = getFlashAddr(); //Get the current flash memory address
  //serial.print("transferring to address: ");
  //serial.print(fAddress, BIN);
  //serial.print(" ");
  //displayTag();
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x58);           //opcode for read modify write
  SPI.transfer(fAddress >> 16);           // write most significant byte of Flash address
  SPI.transfer((fAddress >> 8) & 0xFF);   // second address byte
  SPI.transfer(fAddress & 0xFF);          // third address byte

  for (int n = 0; n < 10; n+=2) {             //loop to log the RFID code to the flash
    uint8_t data0 = (xd->lines[n].data_nibb << 4) | xd->lines[n+1].data_nibb;
    SPI.transfer(data0);
    //SPI.transfer(tagData[n]);
  }
//  SPI.transfer(tagData[0]);
//  SPI.transfer(tagData[1]);
//  SPI.transfer(tagData[2]);
//  SPI.transfer(tagData[3]);
//  SPI.transfer(tagData[4]);
  SPI.transfer(0x16);
  SPI.transfer(0x18);
  SPI.transfer(0x33);
  SPI.transfer(0xAA);
  SPI.transfer(0xBB);
  
  SPI.transfer(RFcircuit);                  //log which antenna was active
  SPI.transfer(mo);
  SPI.transfer(da);
  SPI.transfer(0xBB);
  SPI.transfer(hh);
  SPI.transfer(mm);
  SPI.transfer(ss);
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  unsigned int bAddress = fAddress & 0x03FF;      //and with 00000011 11111111 to isolate byte address
  bAddress = bAddress + 12;                       //add 12 to accound for new bytes (5 for RFID, 1 for RFcircuit, and 6 for date/time)
  if (bAddress > 500) {                           //stop writing if beyond byte address 500 (this is kind of wasteful)
    fAddress = (fAddress & 0xFFFFC00) + 0x0400;   //set byte address to zero and add 1 to the page address
  } else {
    fAddress = (fAddress & 0xFFFFC00) + bAddress; //just add to the byte address
  }
  //digitalWrite(csFlash, HIGH); //deactivate flash chip
  delay(20);
  writeFlashAddr(fAddress);  //Write the updated address to flash.
  serial.println("saved to flash.");        //serial output message to user
}


byte readFlashByte(unsigned long fAddress) {
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x03);           //opcode for low freq read
  SPI.transfer(fAddress >> 16);           //first of three address bytes
  SPI.transfer((fAddress >> 8) & 0xFF);   // second address byte
  SPI.transfer(fAddress & 0xFF);          // third address byte
  byte fByte = SPI.transfer(0);
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  return fByte;
}

void writeFlashByte(unsigned long fAddress, byte fByte) {
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x58);           //opcode for read modify write
  SPI.transfer(fAddress >> 16);           //first of three address bytes
  SPI.transfer((fAddress >> 8) & 0xFF);   // second address byte
  SPI.transfer(fAddress & 0xFF);          // third address byte
  SPI.transfer(fByte);
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  delay(20);
}


void setClk() {                          //Function to set the clock)
  serial.println("Enter mmddyyhhmmss");  //Ask for user input
   while(serial.available() == 0) {}     //wait for 12 characters to accumulate
   for (int n = 0; n < 13; n++) {        //loop to read all the data from the serial buffer once it is ready
      timeIn[n] = serial.read();         //Read the characters from the buffer into an array of 12 bytes one at a time
   }
   while(serial.available())          //Clear the buffer, in case there were extra characters
   {serial.read();}                   //Read in the date and time data
   //Transform the input into decimal numbers    
   mo = ((timeIn[0]-48)*10 + (timeIn[1]-48));    //Convert two ascii characters into a single decimal number
   da = ((timeIn[2]-48)*10 + (timeIn[3]-48));    //Convert two ascii characters into a single decimal number
   yr = ((timeIn[4]-48)*10 + (timeIn[5]-48));    //Convert two ascii characters into a single decimal number
   hh = ((timeIn[6]-48)*10 + (timeIn[7]-48));    //Convert two ascii characters into a single decimal number       
   mm = ((timeIn[8]-48)*10 + (timeIn[9]-48));    //Convert two ascii characters into a single decimal number
   ss = ((timeIn[10]-48)*10 + (timeIn[11]-48));  //Convert two ascii characters into a single decimal number
  
   //set the clock using the provided numbers
   rtc.adjust(DateTime(yr, mo, da, hh, mm, ss));
   delay(1500);

  //Verify that the clock is set
  serial.print("Clock set to ");  //message confirming clock time
  getTime();                      //Read from the time registers you just set 
  serial.println(timeString);

  //When the clock is set (more specifically when serial.read is used) the RFID circuit fails)
  //I don't know why this happens
  //Only solution seems to be to restart the device. 
  //So the following messages inform the user to restart the device.
  serial.print("Restart reader to log data."); 
  while(1){}   //Endless while loop. Program ends here. User must restart.
}





void clockSleepSetup(){
  attachInterrupt(digitalPinToInterrupt(SDselect), EIC_ISR, LOW);  // Attach interrupt to pin 7 with an ISR and when the pin state CHANGEs
  SYSCTRL->XOSC32K.reg |=  (SYSCTRL_XOSC32K_RUNSTDBY | SYSCTRL_XOSC32K_ONDEMAND); // set external 32k oscillator to run when idle or sleep mode is chosen
  REG_GCLK_CLKCTRL  |= GCLK_CLKCTRL_ID(GCM_EIC) |  // generic clock multiplexer id for the external interrupt controller
                       GCLK_CLKCTRL_GEN_GCLK1 |  // generic clock 1 which is xosc32k
                       GCLK_CLKCTRL_CLKEN;       // enable it
  while (GCLK->STATUS.bit.SYNCBUSY);              // write protected, wait for sync
  EIC->WAKEUP.reg |= EIC_WAKEUP_WAKEUPEN5;        // Set External Interrupt Controller to use channel 5 (pin 7)
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;   // Enable Standby or "deep sleep" mode
}

void EIC_ISR(void) {  //interrupt service routine for wake up (may not be necessary)
  SLEEP_FLAG == false;
}


//void readByte(){                  //for testing only
//    digitalWrite(csFlash, LOW);   //activate flash chip
//    SPI.transfer(0x03);           //opcode for low freq read
//    SPI.transfer(0x00);           //first of three address bytes
//    SPI.transfer(0x00);           // second address byte
//    SPI.transfer(0x12);           // third address byte
//    flashArray[0] = SPI.transfer(0);
//    digitalWrite(csFlash, HIGH); //deactivate flash chip
//}

void writeByte() {                //for testing only
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x58);           //opcode for read-modify-write
  SPI.transfer(0x00);           //first of three address bytes
  SPI.transfer(0x00);           // second address byte
  SPI.transfer(0x12);           // third address byte
  SPI.transfer(0xE4);
  digitalWrite(csFlash, HIGH); //deactivate flash chip
}

//void motorOpen() {
//   serial.println("Open feeder...");
//   if (digitalRead(mSwitch)) {
//      motorClose();
//   }
//   while (!digitalRead(mSwitch)) {
//    digitalWrite(MOTF, LOW);
//    digitalWrite(MOTR, HIGH);
//    delay(10);
//   }
//   delay(50);
//   //delay(mDelayOpen)
//   digitalWrite(MOTR, LOW);
//}
//
//void motorClose() {
//   serial.println("Close feeder...");
//   while (digitalRead(mSwitch)) {
//    digitalWrite(MOTF, HIGH);
//    digitalWrite(MOTR, LOW);
//    delay(10);
//   }
//   //delay(mDelayClose)
//   delay(30);
//   digitalWrite(MOTR, HIGH);   // Apply the brakes
//   delay(100);
//   digitalWrite(MOTR, LOW);
//   digitalWrite(MOTF, LOW);
//
//   
//}
  
