#include "cactus_io_DS18B20.h"

int DS18B20_Pin = 9;  //Signal pin is 9

//  Create DS18B20 object
DS18B20 ds(DS18B20_Pin);

void setup() {
  ds.readSensor();

  Serial.begin(9600);
  Serial.println("cactus.io | Weather Station DS18B20 Sensor Test");
  Serial.println("Temp (C)\tTemp (F)");
}

void loop() {
  ds.readSensor();

  Serial.print(ds.getTemperature_C()); Serial.print(" *C\t");
  Serial.print(ds.getTemperature_F()); Serial.println(" *F");

  delay(2000);
}
