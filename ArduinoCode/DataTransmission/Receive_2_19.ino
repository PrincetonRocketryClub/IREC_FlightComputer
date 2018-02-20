/*
 *  Receives data packet from Teensy and prints values 
 *  to serial port to be read by a GUI. The packet
 *  is formatted as follows:
 *  
 *  PACKETNUM*#PRESSURE#ALTITUDE#TIME_STAMP#&
 *  
 *  Working and current as of 19 FEB 2018.
 *  
 *  Note: dummy replies to the transmitter are necessary
 *  or else the tranceivers get 'tripped up' and we lose
 *  roughly 2/3 of the data packets sent (Although they
 *  are still saved to the microSD card onboard the 
 *  teensy). As it stands, the feather still drops a couple
 *  transmissions intermittenly, but not a significant
 *  amount.
 *  
 *  - Jacob Walrath
 */


// Libraries needed for radio transmission
#include <SPI.h>
#include <RH_RF95.h>

// Adafruit feather32u4 pins
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// Set the receiver frequency. Usually 915.0, 868.0, or 434.0
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // initialize the serial port so
  // we can write to it
  while (!Serial);
  Serial.begin(9600);
  delay(10);
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize the LoRa radio module
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  // Checks for successful frequency initialization  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  // The default transmitter power is 13dBm, using PA_BOOST.
  // This set it to the max of 23dBm for better transmission range
  rf95.setTxPower(23, false);
}

void loop()
{
  // Checks for a transmission
  if (rf95.available())
  {
    // Now we should have a message, so we store it   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // Check to see if we've received an actual message
    if (rf95.recv(buf, &len))
    {
      // Light blinks to signal that a message has been received
      digitalWrite(LED, HIGH);
      
      // Converts the radiopacket to type char and prints to serial
      Serial.println((char*)buf);

      // Necessary 10ms delay between receiving a message and
      // sending back a reply
      delay(10);
      
      // Send a reply
      uint8_t data[] = "";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      
      // Light blinks off after sending a reply
      digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}
