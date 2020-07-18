#include <cactus_io_BME280_I2C.h>
#include "cactus_io_DS18B20.h"


int DS18B20_Pin = 9;  //DS18B20 Signal pin is 9

//  Create DS18B20 object & BME280 object
DS18B20 ds(DS18B20_Pin);
BME280_I2C bme;     // I2C using address 0x77

void setup() {
  ds.readSensor();

  Serial.begin(9600);
  Serial.println("cactus.io | Weather Station DS18B20, BMEA280 Sensor Test");
  Serial.println("DS Temp\t\tBME Temp\tHumidity\t\tPressure");

  if (!bme.begin())  {
    Serial.println("Could not find BME280 sensor -  check wiring");
    while (1);
  }
}

void loop() {
  ds.readSensor();
  bme.readSensor();

  Serial.print(ds.getTemperature_C()); Serial.print(" *C\t");
  Serial.print(bme.getTemperature_C()); Serial.print(" *C\t");
  Serial.print(bme.getHumidity());   Serial.print(" %\t\t");
  Serial.print(bme.getPressure_MB());  Serial.println(" mb");

  //  Add a 2 second delay.
  delay(2000);
}
