#include <cactus_io_BME280_I2C.h>
#include "cactus_io_DS18B20.h"

#include "TimerOne.h"     // Timer Interrupt set to 2 sec for read sensors
#include <math.h>
#include <Wire.h>         // For RTC
#include <DS1307RTC.h>    // For TinyRTC breakout board

#define TX_Pin 8  // used to indicate web data tx

#define WindSensor_Pin (2)       //The pin location of the anemometer sensor
#define WindVane_Pin  (A3)       // The pin connecting to the wind vane sensor
#define VaneOffset  0		   // The anemometer offset from magnetic north


volatile bool isSampleRequired;    // set true every 2.5s.   Get wind speed
volatile unsigned int timerCount;  // used to determine the 2.5s timer count
volatile unsigned long rotations;  // cup rotation counter for wind speed calcs
volatile unsigned long contactBounceTime;  // Timer to avoid contact bounce in wind speed sensor
volatile float windSpeed;        // speed in km per hour

bool txState;			// current LED state for tx rx LED
int vaneValue;          //  raw analog value from wind vane
int vaneDirection;          //  translated 0-360 direction
int calDirection;       //  converted value with offset applied
int lastDirValue;          //  last direction value

int DS18B20_Pin = 9;  //DS18B20 Signal pin is 9

//  Create DS18B20 object & BME280 object
DS18B20 ds(DS18B20_Pin);
BME280_I2C bme;     // I2C using address 0x77

void setup() {
  // setup anemometer values
  lastDirValue = 0;
  rotations = 0;
  isSampleRequired = false;
  
  // setup timer values
  timerCount = 0;
  
  ds.readSensor();

  Serial.begin(9600);
  while (!Serial);      //wait for serial
  delay(200);
  
  Serial.println("cactus.io | Weather Station DS18B20, BMEA280 Sensor Test");
  Serial.println("DS Temp\t\tBME Temp\tHumidity\t\tPressure");

  if (!bme.begin())  {
      Serial.println("Could not find BME280 sensor -  check wiring");
      while (1);
  }
  
  Serial.println("Davis Anemometer Test");
  Serial.println("Speed  (KPH)\tDirection");
  
  pinMode(WindSensor_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING);

  
  //Setup the timer for 0.5s
  Timer1.initialize(500000);     
  Timer1.attachInterrupt(isr_timer);
  
  sei();   // Enable Interrupts
}

void loop() {
  tmElements_t tm;    
  ds.readSensor();
  bme.readSensor();

  Serial.print(ds.getTemperature_C()); Serial.print(" °C\t");
  Serial.print(bme.getTemperature_C()); Serial.print(" °C\t");

if (RTC.read(tm)) {
    Serial.print("\nRecorded: ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.print(' ');
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.write(' ');
  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
  }
  Serial.print(ds.getTemperature_C()); Serial.print(" *C\t");
  Serial.print(bme.getTemperature_C()); Serial.print(" *C\t");
  Serial.print(bme.getHumidity());   Serial.print(" %\t\t");
  Serial.print(bme.getPressure_MB());  Serial.println(" hPa");

  //  Add a 2 second delay.
  delay(2000); 
  
  if(isSampleRequired) {
	
	getWindDirection();
	
	Serial.print(windSpeed);   Serial.print(" kph\t");
	Serial.print(calDirection);   Serial.println("deg.");
	
	isSampleRequired = false;
   }
}

// Interrupt handler routine for timer interrupt
void isr_timer() {
	
	timerCount++;
	
	if(timerCount == 5) {
		// convert to km/h using the formula V=P(2.25/T)*1.609
		// i.e. V = P(2.25/3)*1.609 = P * 1.207	
		windSpeed = rotations * 1.207;
		rotations = 0;   
		txState = !txState;     
		digitalWrite(TX_Pin, txState);
		isSampleRequired = true;
		timerCount = 0;
	}
}

// Interrupt handler routine to increment the rotation count for wind speed
void isr_rotation ()   {

  if ((millis() - contactBounceTime) > 15 ) {  // debounce the switch contact.
    rotations++;
    contactBounceTime = millis();
  }
}

// Get Wind Direction
void getWindDirection() {
	
	vaneValue = analogRead(WindVane_Pin);
	vaneDirection = map(vaneValue, 0, 1023, 0, 359);
	calDirection = vaneDirection + VaneOffset;
	
	if(calDirection > 360)
		calDirection = calDirection - 360;
	
	if(calDirection > 360)
		calDirection = calDirection - 360;
}

// Field format utility for printing
void print2digits(int number)  {
	if (number >= 0 && number <10) {
		Serial.write('0');
	}
	Serial.print(number);
}
