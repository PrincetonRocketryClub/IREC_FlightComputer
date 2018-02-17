/* 
 *  This is a combination of code that has been proven to transmit barometer data 
 *  with code that should also transmit accelerometer data (untested).
 *  
 *  The transmission is of the form:
 *    *#PRESSURE#PITCH#ROLL#HEADING#&
 *    
 *  Note: I am not sure of the proper connections to the accelerometer; this may
 *  require some revision of the code. Also, ignore the commented-out sections
 *  of GPS-related code; I still need to work to debug this.
 *  
 *  -Michael Hauge
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>


//Accelerometer
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/* This driver reads raw data from the BNO055
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

/*
//GPS
#include <TinyGPS.h>

TinyGPS gps;
String gpsData(TinyGPS &gps);
*/


//Barometer
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>

/* Teensy 3.x w/wing  */
#define RFM95_RST     9   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     4    // "C"

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


void setup() 
{
  //Barometer
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }


  //Accelerometer
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

  //Radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(115200);
  Serial1.begin(9600);
  //delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  //delay(10);
  digitalWrite(RFM95_RST, HIGH);
  //delay(10);

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
  String message = "";
  //String gpsStuff = "";

  
  //Barometer
  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    //Serial.println("Sensor Success");
    /* Display atmospheric pressure in hPa */
    message = "*#" + String(event.pressure) + "#";
  }
  else
  {
    Serial.println("Sensor error");
  }

  
  //Accelerometer
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //Pitch, Roll, Heading
  message += (String)euler.x() + "#" + (String)euler.y() + "#" + (String)euler.z() + "#&";
  //delay(BNO055_SAMPLERATE_DELAY_MS);
  

  /*
  //------GPS------
  bool newdata = false;
  unsigned long start = millis();
  //Update every .1 seconds
  while (millis() - start < 100) {
    if(Serial1.available()) {
      char c = Serial1.read();
      if (gps.encode(c)) {
        newdata = true;
      }
    }
  }

  if(newdata)
    Serial.println(gpsData(gps));
  //------GPS------

  message += gpsStuff;
  */


  
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server

  int n = message.length();
  
  char radiopacket[n+1];
  strcpy(radiopacket, message.c_str());
  //itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  //radiopacket[19] = 0;
  
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, n+1);

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
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
  //delay(1000);
}

/*
//------GPS------
String gpsData(TinyGPS &gps)
{
  String toReturn = "";
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;
  unsigned long fix_age;
  gps.f_get_position(&flat, &flon, &fix_age);
  if (fix_age == TinyGPS::GPS_INVALID_AGE) {
    return "0"; //No fix - no valid data
  } else {
    toReturn += "1: Valid Data"; //Fix - valid data
    gps.get_position(&lat, &lon, &age);
    toReturn += " Lat/Long(10^-5 deg): " + (String)lat + ", " + (String)lon;
    toReturn += "\nFix age: " + (String)age + "ms.";

    gps.get_datetime(&date, &time, &age);
    toReturn += " Date(ddmmyy): " + (String)date + " Time(hhmmsscc): " + (String)time;
    toReturn += " Fix age: " + (String)age + "ms.";

    toReturn += " Alt(cm): " + (String)gps.altitude() + " Course(10^-2 deg): " + (String)gps.course();
    toReturn += " Speed(10^-2 knots): " + (String)gps.speed();

    return toReturn;
  }
}
*/
