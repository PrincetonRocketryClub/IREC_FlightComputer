#include <TinyGPS.h>

TinyGPS gps;

void gpsPrint(TinyGPS &gps);
void printFloat(double f, int digits = 2);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
}

void loop() {
  bool newdata = false;
  unsigned long start = millis();
  //Update every .1 seconds
  while (millis() - start < 100) {
    if (Serial1.available()) {
      char c = Serial1.read();
      if(gps.encode(c)) {
        newdata = true;
      }
    }
  }

  if (newdata) {
    Serial.println("New Data");
    Serial.println("------------");
    gpsPrint(gps);
    Serial.println("------------");
    Serial.println();
  }
}

void gpsPrint(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;
  unsigned long fix_age;
  gps.f_get_position(&flat, &flon, &fix_age);
  if (fix_age == TinyGPS::GPS_INVALID_AGE) {
    Serial.print(0);
  } else {
    Serial.print(1);
    gps.get_position(&lat, &lon, &age);
    Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
    Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

    gps.f_get_position(&flat, &flon, &age);
    Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
    Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
    gps.get_datetime(&date, &time, &age);
    Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): ");
      Serial.print(time);
    Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
      Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
    Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); 
      Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
      Serial.print("."); Serial.print(static_cast<int>(hundredths));
    Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");
    Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): ");
      Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
    Serial.print("Alt(float): "); printFloat(gps.f_altitude()); Serial.print(" Course(float): ");
      printFloat(gps.f_course()); Serial.println();
    Serial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); Serial.print(" (mph): ");
      printFloat(gps.f_speed_mph());
    Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): ");
      printFloat(gps.f_speed_kmph()); Serial.println();
    gps.stats(&chars, &sentences, &failed);
    Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
      Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
  }
}

void printFloat(double number, int digits)
{
  if (number < 0.0) {
     Serial.print('-');
     number = -number;
  }

  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  if (digits > 0)
    Serial.print(".");
    
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

