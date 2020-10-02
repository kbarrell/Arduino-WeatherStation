#include <cactus_io_BME280_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "TimerOne.h"     // Timer Interrupt set to 2 sec for read sensors
#include <math.h>
#include <Wire.h>         // For accessing RTC
#include <DS1307RTC.h>    // For TinyRTC breakout board
#include <TimeLib.h>      // For epoch time en/decode

// Set hardware pin assignments & pre-set constants
#define TX_Pin 8 				 // used to indicate web data tx
#define ONE_WIRE_BUS_PIN 9 	    //Data bus pin for DS18B20's

#define WindSensor_Pin (2)       //The pin location of the anemometer sensor
#define WindVane_Pin  (A3)       // The pin connecting to the wind vane sensor
#define VaneOffset  0		   // The anemometer offset from magnetic north
#define Bucket_Size  0.2 	   // mm bucket capacity to trigger tip count
#define RG11_Pin  3        		 // Interrupt pin for rain sensor
#define BounceInterval  15		// Number of ms to allow for debouncing

// Set timer related settings for sensor sampling & calculation
#define Timing_Clock  500000    //  0.5sec in millis
#define Sample_Interval   5		//  = number of Timing_Clock cycles  i.e. 2.5sec interval
#define Report_Interval   6   //  = number of sample intervals contributing to each upload report (each 5 min)
#define Speed_Conversion  1.4481   // convert rotations to km/h.  = 2.25/(Sample_Interval x Timing_Clock)* 1.609 
									// refer Davis anemometer technical spec
									

volatile bool isSampleRequired;    // set true every Sample_Interval.   Get wind speed
volatile unsigned int timerCount;  // used to determine when Sample_Interval is reached
volatile unsigned int sampleCount;	// used to determin when Report_Interval is reached
volatile unsigned long rotations;  // cup rotation counter for wind speed calcs
volatile unsigned long contactBounceTime;  // Timer to avoid contact bounce in wind speed sensor
volatile float windSpeed, windGust;        // speed in km per hour

volatile unsigned long tipCount;   // rain bucket tipcounter used in interrupt routine
volatile unsigned long contactTime; // timer to manage contact bounce in interrupt routine
volatile float sampleRainfall;       // total amount of rainfall recorded in sample period (2.5s)
volatile float obsReportRainfall;    // total amount of rainfall in the reporting period  (5 min)
volatile float dailyRainfall;		//  total amount of rainfall in 24 hrs to 9am

// Define structures for handling reporting via TTN
typedef struct obsSet {
	uint16_t 	windGustX10; // observed windgust speed (km/h) X10  ~range 0 -> 1200
	uint16_t	windGustDir; // observed wind direction of Gust (compass degrees)  0 -> 359
	uint16_t	tempX10;	// observed temp (°C) +100 x 10   ~range -200->600
	uint16_t	humidX10;	// observed relative humidty (%) x 10   range 0->1000
	uint16_t 	pressX10;	// observed barometric pressure at station level (hPa)  x 10  ~range 8700 -> 11000 
	uint16_t	rainflX10;	// observed accumulated rainfall (mm) x10   ~range 0->1200
	uint16_t	windspX10;	// observed windspeed (km/h) x10 ~range 0->1200
	uint16_t	windDir;	// observed wind direction (compass degrees)  range 0->359
	uint16_t	dailyRainX10; //  accumulated rainfall (mm) X10 for period to 9am daily
} OBSSET;
	
union obsPayload
{
	OBSSET	obsReport;
	uint8_t	readAccess[sizeof(obsSet)];
};

int  currentObs, reportObs;   //References which obsPayload [0,1 or 2]is being filled etc. 
bool txState;				// current LED state for tx rx LED
int vaneValue;         	 	//  raw analog value from wind vane
int vaneDirection;          //  translated 0-360 direction
int calDirection, calGustDirn;     	//  converted value with offset applied
int lastDirValue;          //  last direction value


//  Create BME280 object
BME280_I2C bme;     // I2C using address 0x77

// Setup a oneWire instance to communicate with OneWire devices
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature DSsensors(&oneWire);    // Pass the OneWire reference to Dallas Temperature lib

// Assign the addresses of the DS18B20 sensors (determined by reading them previously)
DeviceAddress airTempAddr = { 0x28, 0x1A, 0x30, 0x94, 0x3A, 0x19, 0x01, 0x55 };
DeviceAddress caseTempAddr = { 0x28, 0xAA, 0x68, 0x93, 0x41, 0x14, 0x01, 0xD8 };


void setup() {
  
  txState = HIGH;
  
  // prepare obsPayload indices
  currentObs = 0;
  reportObs = 1;
  
  // setup anemometer values
  lastDirValue = 0;
  rotations = 0;
  isSampleRequired = false;
  windGust = 0;
  calGustDirn = 0;
  
  // setup RG11 rain totals
  sampleRainfall = 0;
  obsReportRainfall = 0;
  dailyRainfall = 0;
  
  // setup timer values
  timerCount = 0;
  sampleCount = 0;
  
  // Initialise the Temperature measurement library & set sensor resolution to 12 (10) bits
  DSsensors.setResolution(airTempAddr, 12);
  DSsensors.setResolution(caseTempAddr, 10);
 

  Serial.begin(9600);
  while (!Serial);      //wait for serial
  delay(200);
  
  Serial.println(" Weather Station DS18B20, BME280, RG11, Davis Sensor Test");
  Serial.println("DS Temp\t\tBME Temp2\tHumidity\t\tPressure\tRainfall\tSpeed\tDirection");
  

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
  Timer1.initialize(Timing_Clock);     
  Timer1.attachInterrupt(isr_timer);
  
  sei();   // Enable Interrupts
}

void loop() {
  tmElements_t tm;
  time_t obsReportTime;
  union obsPayload sensorObs[2];  

  DSsensors.requestTemperatures();    // Read temperatures from all DS18B20 devices
  bme.readSensor();

  if(isSampleRequired) {
	  
	sampleCount++;
	
    getWindDirection();
	if (windSpeed > windGust) {      // Check last sample of windspeed for new Gust record
		windGust = windSpeed;
		calGustDirn = calDirection;
	}
	
	sampleRainfall = tipCount * Bucket_Size;    // update totals with the rainfall for this sample period
	obsReportRainfall += sampleRainfall;
	dailyRainfall += sampleRainfall;
	tipCount = 0;
	
	//  Does this sample complete a reporting cycle?   If so, prepare payload.
	if (sampleCount == Report_Interval) {
		sensorObs[currentObs].obsReport.windGustX10 = windGust * 10.0;
		sensorObs[currentObs].obsReport.windGustDir = (windGust > 0) ? calGustDirn : 0;
		sensorObs[currentObs].obsReport.tempX10 = DSsensors.getTempC(airTempAddr)* 10.0;
		sensorObs[currentObs].obsReport.humidX10 = bme.getHumidity()*10.0;
		sensorObs[currentObs].obsReport.pressX10 = bme.getPressure_MB()*10.0;
		sensorObs[currentObs].obsReport.rainflX10 = obsReportRainfall * 10.0;
		sensorObs[currentObs].obsReport.windspX10 = windSpeed * 10.0;
		sensorObs[currentObs].obsReport.windDir =  (windSpeed > 0) ? calDirection : 0;
		sensorObs[currentObs].obsReport.dailyRainX10 = dailyRainfall * 10.0;
			
		//  Do print  i.e. substitute for a send it 
		Serial.print("DS18 Air:   ");  Serial.print(sensorObs[currentObs].obsReport.tempX10);  Serial.print(" °C\t");
		Serial.print(sensorObs[currentObs].obsReport.humidX10);   Serial.print(" %\t\t");
		Serial.print(sensorObs[currentObs].obsReport.pressX10);  Serial.print(" hPa\t");
		Serial.print(sensorObs[currentObs].obsReport.rainflX10);  Serial.print(" mm\t\t");
		Serial.print(sensorObs[currentObs].obsReport.dailyRainX10);  Serial.print(" mm\t\t");
		Serial.print(sensorObs[currentObs].obsReport.windspX10);   Serial.print(" kph\t");
		Serial.print(sensorObs[currentObs].obsReport.windDir);   Serial.print("deg.\t");
		Serial.print(sensorObs[currentObs].obsReport.windGustX10);   Serial.print(" kph\t");
		Serial.print(sensorObs[currentObs].obsReport.windGustDir);   Serial.print("deg.\t");
		Serial.print(sensorObs[currentObs].obsReport.dailyRainX10);  Serial.println(" mm\t\t");
		printIt(sensorObs[currentObs].readAccess, sizeof(OBSSET));        //  Check dump of 16 Byte obsSet structure
		
		sampleCount = 0;
		currentObs = 1- currentObs;
		reportObs = 1 - currentObs;   // switch reporting to last collected observation
		windGust = 0;
	}
			
  
//    if (RTC.read(tm)) {
//	  obsReportTime = makeTime(tm);
//      Serial.print("\nRecorded: ");
//      Serial.print(tm.Day);
//      Serial.write('/');
//      Serial.print(tm.Month);
//      Serial.write('/');
//      Serial.print(tmYearToCalendar(tm.Year));
//      Serial.print(' ');
//      print2digits(tm.Hour);
//      Serial.write(':');
//      print2digits(tm.Minute);
//      Serial.write(':');
//      print2digits(tm.Second);
//      Serial.write(' ');   
//    } else {
//      if (RTC.chipPresent()) {
//        Serial.println("The DS1307 is stopped.  Please run the SetTime");
//        Serial.println("example to initialize the time and begin running.");
//        Serial.println();
//      } else {
//      Serial.println("DS1307 read error!  Please check the circuitry.");
//      Serial.println();
//      }
//    }
	
	isSampleRequired = false;
  }
}

// Interrupt handler routine for timer interrupt
void isr_timer() {
	
	timerCount++;

	if(timerCount == Sample_Interval) {
		// convert to km/h using the formula V=P(2.25/T)*1.609 where T = sample interval
		// i.e. V = P(2.25/2.5)*1.609 = P * Speed_Conversion factor  (=1.4481  for 2.5s interval)
		windSpeed = rotations * Speed_Conversion; 
		rotations = 0;   
		txState = !txState;     
		digitalWrite(TX_Pin, txState);      // Transmit LED
		isSampleRequired = true;
		timerCount = 0;						// Restart the interval count
	}
}

// Interrupt handler routine to increment the rotation count for wind speed
void isr_rotation ()   {

  if ((millis() - contactBounceTime) > BounceInterval ) {  // debounce the switch contact.
    rotations++;
    contactBounceTime = millis();
  }
}

// Interrrupt handler routine that is triggered when the rg-11 detects rain   
void isr_rg ()   { 

   if ((millis() - contactTime) > BounceInterval ) {  // debounce of sensor signal
      tipCount++;
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

// Print utility for packed structure
void printIt(uint8_t *charArray, int length) {
  int i;
	char charMember;
	Serial.print("buff length:"); Serial.println(length);
	for (i=0; i<length; i++) {
		charMember = charArray[i];
		Serial.println(charMember, BIN);
	}
	Serial.println("===EndOfBuffer========");
}
	
	
