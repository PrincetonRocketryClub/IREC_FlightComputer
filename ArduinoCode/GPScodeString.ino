#include <TinyGPS.h>

TinyGPS gps;

String gpsData(TinyGPS &gps);
String stringFloat(double f, int digits = 2);
String data;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
}

void loop() {
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
    data = gpsData(gps);
}

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
    
    gps.f_get_position(&flat, &flon, &age);
    toReturn += " Lat/Long(float): " + stringFloat(flat, 5) + ", " + stringFloat(flon, 5);
    toReturn += " Fix age: " + (String)age + "ms.";

    gps.get_datetime(&date, &time, &age);
    toReturn += " Date(ddmmyy): " + (String)date + " Time(hhmmsscc): " + (String)time;
    toReturn += " Fix age: " + (String)age + "ms.";

    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    toReturn += " Date: " + (String)static_cast<int>(month) + "/" + (String)static_cast<int>(day) + "/" + (String)year;
    toReturn += " Time: " + (String)static_cast<int>(hour) + ":" + (String)static_cast<int>(minute) + ":" + (String)static_cast<int>(second)
      + "." + (String)static_cast<int>(hundredths);
    toReturn += " Fix age: " + (String)age + "ms.";

    toReturn += " Alt(cm): " + (String)gps.altitude() + " Course(10^-2 deg): " + (String)gps.course();
    toReturn += " Speed(10^-2 knots): " + (String)gps.speed();

    toReturn += " Alt(float): " + stringFloat(gps.f_altitude()) + " Course(float): " + stringFloat(gps.f_course());
    toReturn += " Speed(knots): "  + stringFloat(gps.f_speed_knots()) + " (mph): " + stringFloat(gps.f_speed_mph())
      + " (mps): " + stringFloat(gps.f_speed_mps()) + " (kmph): " + stringFloat(gps.f_speed_kmph());

    gps.stats(&chars, &sentences, &failed);
    toReturn += " Stats: characters: " + (String)chars + " sentences: " + (String)sentences + " failed checksum: " + (String)failed;

    return toReturn;
  }
}

String stringFloat(double number, int digits)
{
   String toReturn = "";
  
  if (number < 0.0) {
     toReturn = "-";
     number = -number;
  }
  
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;
  
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  toReturn += (String)int_part;
  
  if (digits > 0)
    toReturn += ".";
  
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    toReturn += (String)toPrint;
    remainder -= toPrint;
  }

  return toReturn;
}

