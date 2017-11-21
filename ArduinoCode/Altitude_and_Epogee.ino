#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <MovingAverage.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* SDA to A4
SCL to A5
GND to GND
3Vo to 3.3V
VIN to nothing
*/


Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

Adafruit_BNO055 bno = Adafruit_BNO055();

float initialAltitude;
float seaLevelPressure;
MovingAverage average(0.1f);


void setup(void) 
{
  Serial.begin(57600);

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    while(1);
  }
  
  seaLevelPressure = 1028.16;
  sensors_event_t event;
  bmp.getEvent(&event);
  initialAltitude = bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure);
  average.reset( bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure) - initialAltitude );                                      
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);

  sensors_event_t event2;
  bno.getEvent(&event2);

  if (event.pressure)
  {
    /*
    Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
     
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    
    
    Serial.print("Altitude:    "); 

    */
    /*Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure) - initialAltitude);*/

    Serial.print( average.update( bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure) - initialAltitude ) );                                    
    /*Serial.println(" m");*/
    Serial.println("");
  }
  else
  {
    Serial.println("Sensor error");
  }
  
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println(accel, DEC);
  delay(1000);
}
