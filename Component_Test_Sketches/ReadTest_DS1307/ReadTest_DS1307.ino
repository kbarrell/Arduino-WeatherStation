#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>

union shared_bits
{
  time_t timepoint;
  uint32_t tnow;
};

int DS18B20_Pin = 9;  //DS18B20 Signal pin is 9

void setup() {
  Serial.begin(9600);
  while (!Serial) ; // wait for serial
  delay(200);
  Serial.println("DS1307RTC Read Test + Temp");
  Serial.println("-------------------");

}

void loop() {
  tmElements_t tm;
  union shared_bits myBits;
    
  Serial.print("calling now:\t");  
  if (RTC.read(tm)) {
    myBits.timepoint =  makeTime(tm);      //  COnvert tm elements read from RTC to a Unix Epoch 32 bit value
    Serial.print("Ok, Time = ");
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.print("\t");
    Serial.print(myBits.tnow, BIN);      // Use the parallel interpretation as a unint32 to print it
    Serial.print("\t");
    Serial.print(myBits.tnow, HEX);
    Serial.println();
  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
    delay(9000);
  }
  delay(3000);
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}
