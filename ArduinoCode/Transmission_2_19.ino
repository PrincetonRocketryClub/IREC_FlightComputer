/* 
 *  This code transmits data from the barometer in the following format:
 *  
 *  PACKETNUM*#PRESSURE#ALTITUDE#TIME_STAMP#&
 *  
 *  The data is also written to the built-in SD card in the file data.txt.
 *  It overwrites the file each time the code is uploaded to the Teensy.
 *  
 *  Code for the accelerometer  (and GPS) can be found on Github 
 *  (Transmission_2_18.ino).
 *  
 *  -Michael Hauge
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>


// Altitude and Apogee Setup
#include <MovingAverage.h>
float initialAltitude;
float initialPressure;
MovingAverage average(0.1f);
float prevAltitude;
int currentCount;


// Barometer Setup
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


// SD Card Setup
#include <SD.h>
File myFile;


// Feather9x_TX Setup
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_RST     9    // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     4    // "C"

// Frequency (915.0, 868.0, or 434.0)
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Variable declaration
int counter;
String message;
float currentAltitude;
int n;


void setup() 
{
  counter = 0;
  
  // Barometer
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("No BMP085 detected");
    while(1);
  }


  // SD Card
  SD.begin(BUILTIN_SDCARD);
  SD.remove("data.txt");


  // Altitude and Apogee
  sensors_event_t event;
  bmp.getEvent(&event);
  initialPressure = event.pressure;
  initialAltitude = bmp.pressureToAltitude(initialPressure, event.pressure);
  average.reset( bmp.pressureToAltitude(initialPressure, event.pressure) - initialAltitude);  


  // Radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(115200);
  Serial1.begin(9600);
  //delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}




void loop()
{
  counter++;
  message = (String)counter;

  
  //Barometer
  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    message += "*#" + String(event.pressure) + "#";
  }
  else
  {
    Serial.println("Barometer error");
  }


    // Altitude and Apogee
  currentAltitude = average.update( bmp.pressureToAltitude(initialPressure, event.pressure) - initialAltitude );
  message += (String)currentAltitude + "#";
  //Serial.println( currentAltitude ); 


  // Time stamp
  message += (String)millis() + "#&";


  // SD Card
  myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile) {
    myFile.println(message);
    myFile.close();
  } else {
    Serial.println("SD Card Error");
  }
    
  
  // Radio
  n = message.length();
  
  char radiopacket[n+1];
  strcpy(radiopacket, message.c_str());
  Serial.println(radiopacket);
  
  delay(10);
  rf95.send((uint8_t *)radiopacket, sizeof(radiopacket)); 
  delay(10);
  rf95.waitPacketSent();
  
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  delay(10);
  if (rf95.waitAvailableTimeout(750))
  {
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
  delay(100);
}
