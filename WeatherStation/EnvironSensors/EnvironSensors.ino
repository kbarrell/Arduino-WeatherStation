#include <cactus_io_BME280_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "TimerOne.h"     // Timer Interrupt set to 2 sec for read sensors
#include <math.h>
#include <Wire.h>         // For RTC
#include <DS1307RTC.h>    // For TinyRTC breakout board

#define TX_Pin 8  // used to indicate web data tx
#define ONE_WIRE_BUS_PIN 9     //Data bus pin for DS18B20's

#define WindSensor_Pin (2)       //The pin location of the anemometer sensor
#define WindVane_Pin  (A3)       // The pin connecting to the wind vane sensor
#define VaneOffset  0		   // The anemometer offset from magnetic north

#define Bucket_Size  0.2    // mm bucket capacity to trigger tip count
#define RG11_Pin  3         // Interrupt pin for rain sensor

volatile bool isSampleRequired;    // set true every 2.5s.   Get wind speed
volatile unsigned int timerCount;  // used to determine the 2.5s timer count
volatile unsigned long rotations;  // cup rotation counter for wind speed calcs
volatile unsigned long contactBounceTime;  // Timer to avoid contact bounce in wind speed sensor
volatile float windSpeed;        // speed in km per hour

volatile unsigned long tipCount;   // rain bucket tipcounter used in interrupt routine
volatile unsigned long contactTime; // timer to manage contact bounce in interrupt routine
volatile float totalRainfall;       // total amount of rainfall recorded

bool txState;			// current LED state for tx rx LED
int vaneValue;          //  raw analog value from wind vane
int vaneDirection;          //  translated 0-360 direction
int calDirection;       //  converted value with offset applied
int lastDirValue;          //  last direction value


//  Create BME280 object
BME280_I2C bme;     // I2C using address 0x77

// Setup a oneWire instance to communicate with OneWire devices
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature DSsensors(&oneWire);    // Pass the OneWire reference to Dallas Temperature

// Assign the addresses of the DS18B20 sensors (determined by reading them previously)
DeviceAddress airTempAddr = { 0x28, 0x1A, 0x30, 0x94, 0x3A, 0x19, 0x01, 0x55 };
DeviceAddress caseTempAddr = { 0x28, 0xAA, 0x68, 0x93, 0x41, 0x14, 0x01, 0xD8 };


void setup() {
  
  txState = HIGH;
  
  // setup anemometer values
  lastDirValue = 0;
  rotations = 0;
  isSampleRequired = false;
  
  // setup timer values
  timerCount = 0;
  
  // Initialise the Temperature measurement library & set sensor resolution to 12 (10) bits
  DSsensors.setResolution(airTempAddr, 12);
  DSsensors.setResolution(caseTempAddr, 10);
 

  Serial.begin(9600);
  while (!Serial);      //wait for serial
  delay(200);
  
  Serial.println(" Weather Station DS18B20, BME280, RG11, Davis Sensor Test");
  Serial.println("DS Temp\t\tBME Temp\tHumidity\t\tPressure\tRainfall\tSpeed\tDirection");

  if (!bme.begin())  {
      Serial.println("Could not find BME280 sensor -  check wiring");
      while (1);
  }
   
  // Setup pins & interrupts	
  pinMode(TX_Pin, OUTPUT);
  pinMode(WindSensor_Pin, INPUT);
  pinMode(RG11_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING);
  attachInterrupt(digitalPinToInterrupt(RG11_Pin),isr_rg, FALLING);

  //Setup the timer for 0.5s
  Timer1.initialize(500000);     
  Timer1.attachInterrupt(isr_timer);
  
  sei();   // Enable Interrupts
}

void loop() {
  tmElements_t tm;    
   
  DSsensors.requestTemperatures();    // Read temperatures from all DS18B20 devices
  bme.readSensor();

  if(isSampleRequired) {
	
    getWindDirection();	
  
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

    Serial.print("DS18 Air:   ");  Serial.print(DSsensors.getTempC(airTempAddr));  Serial.print(" °C\t");
    Serial.print("DS18 Case:   ");  Serial.print(DSsensors.getTempC(caseTempAddr));  Serial.print(" °C\t");
    Serial.print(bme.getTemperature_C()); Serial.print(" °C\t");
    Serial.print(bme.getHumidity());   Serial.print(" %\t\t");
    Serial.print(bme.getPressure_MB());  Serial.print(" hPa\t");
	Serial.print(totalRainfall);  Serial.print(" mm\t\t");
	Serial.print(windSpeed);   Serial.print(" kph\t");
	Serial.print(calDirection);   Serial.println("deg.");
	
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

// Interrrupt handler routine that is triggered when the rg-11 detects rain   
void isr_rg ()   { 

   if ((millis() - contactTime) > 15 ) {  // debounce of sensor signal
      tipCount++;
	  totalRainfall = tipCount * Bucket_Size;
      contactTime = millis();
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
