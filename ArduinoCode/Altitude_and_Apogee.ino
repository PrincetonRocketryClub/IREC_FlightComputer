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

float initialAltitude;
float initialPressure;
MovingAverage average(0.1f);
float prevAltitude;
int currentCount;


void setup(void) 
{
  Serial.begin(9600);
  
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    while(1);
  }
  
  sensors_event_t event;
  bmp.getEvent(&event);
  initialPressure = event.pressure;
  initialAltitude = bmp.pressureToAltitude(initialPressure,
                                        event.pressure);
  average.reset( bmp.pressureToAltitude(initialPressure,
                                        event.pressure) - initialAltitude );                                      
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);

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

    float currentAltitude = average.update( bmp.pressureToAltitude(initialPressure,
                                        event.pressure) - initialAltitude );
    Serial.println( currentAltitude ); 
    if ( currentAltitude < prevAltitude) {
        currentCount++;
        if (currentCount >= 5 && currentAltitude > 100) {
          Serial.println("APOGEE");  
        }
    }
    else {
      currentCount = 0;
    }                        
    prevAltitude = average.update( bmp.pressureToAltitude(initialPressure,
                                        event.pressure) - initialAltitude );
    /*Serial.println(" m");*/
    Serial.println("");
  }
  else
  {
    Serial.println("Sensor error");
  }
  
  delay(200);
}
