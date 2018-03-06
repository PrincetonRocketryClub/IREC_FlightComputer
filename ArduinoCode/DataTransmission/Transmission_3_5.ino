/* 
 *  This code builds on Transmission_2_24 with the capability to transmit data
 *  from the accelerometer, barometer, and GPS in the following format:
 *  
 *  *#PACKETNUM#AX#AY#AZ#ROLL#PITCH#HEADING#ALTB#ALTGPS#VELOCITY#LAT#LON#TIME_STAMP#&
 *  
 *  The data is also written to the built-in SD card in the file data.txt.
 *  It overwrites the file each time the code is uploaded to the Teensy.
 *  
 *  Relocated data collection to methods gpsGrab, accGrab, and barGrab.
 *   
 *  - Jacob Walrath
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

//Accelerometer
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/* This driver reads raw data from the BNO055
 * Connections
 * ===========
 * Connect SCL to analog 5
 * Connect SDA to analog 4
 * Connect VDD to 3.3V DC
 * Connect GROUND to common ground
 */
//Set the delay between fresh samples 
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();
String accGrab(Adafruit_BNO055 &bno);

//GPS
#include <TinyGPS.h>
TinyGPS gps;
String gpsGrab(TinyGPS &gps);

// Barometer Setup
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
String barGrab(Adafruit_BMP085_Unified &bmp);


// SD Card Setup
#include <SD.h>
File myFile;
void sdWrite(String &message);


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

  //Accelerometer
  
  if(!bno.begin())
  {
    //There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
  
  
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
  message = "*#" + (String) counter + "#";

  
  //Accelerometer
  message += accGrab(bno);
  
  
  //Barometer
  message += barGrab(bmp);
  

  // GPS
  /*
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
  // Only pull data at the appropriate time
  //if(newdata)
  */
  message += gpsGrab(gps);


  // Time stamp
  message += (String)millis() + "#&";


  // SD Card
  sdWrite(message);
    
  
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

// Returns a formatted String of GPS data
String gpsGrab(TinyGPS &gps)
{
  // These two lines may have to go in the loop
  char c = Serial1.read();
  gps.encode(c);
  
  String toReturn = "";
  long lat, lon;
  float flat, flon;
  unsigned long age;
  unsigned long fix_age;
  gps.f_get_position(&flat, &flon, &fix_age);
  if (fix_age == TinyGPS::GPS_INVALID_AGE) {
    return "0#0#0#0#"; // No fix - no valid data
  } else {
    toReturn += (String)gps.altitude() + "#";
    toReturn += (String)(gps.speed() * 51.44) + "#";
    gps.get_position(&lat, &lon, &age);
    toReturn += (String)lat + "#" + (String)lon + "#";
    return toReturn;
  }
}

// Returns a formatted String of accelerometer data
String accGrab(Adafruit_BNO055 &bno)
{
  String accData;
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  accData = (String)accel.x() + "#" + (String)accel.y() + "#" + (String)accel.z() + "#";
  
  //Roll, Pitch, Heading
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  accData += (String)euler.x() + "#" + (String)euler.y() + "#" + (String)euler.z() + "#";
  //delay(BNO055_SAMPLERATE_DELAY_MS);

  return accData;
}

// Returns a formatted String of barometer data
String barGrab(Adafruit_BMP085_Unified &bmp)
{
  String barData;
  
  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);

    // Altitude and Apogee
  currentAltitude = average.update( bmp.pressureToAltitude(initialPressure, event.pressure) - initialAltitude );
  barData = (String)currentAltitude + "#";

  return barData;
}

void sdWrite(String &message)
{
  myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile) {
    myFile.println(message);
    myFile.close();
  } else {
    Serial.println("SD Card Error");
  }
}

