/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 * Copyright (c) 2018 Thomas Laurenson
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 * 
 * Sketch Summary:
 * Target device: Dragino LoRa Shield (US900) with Arduino Uno
 * Target frequency: AU915 sub-band 2 (916.8 to 918.2 uplink)
 * Authentication mode: Activation by Personalisation (ABP)
 *
 * This example requires the following modification before upload:
 * 1) Enter a valid Network Session Key (NWKSKEY)
 *    For example: 0x07f319fc
 * 2) Enter a valid Application Session Key (APPSKEY)
 *    For example: { 0xd9, 0x54, 0xce, 0xbe, 0x9b, 0x5b, 0x76, 0x2d, 0x56, 0x26, 0xc9, 0x4d, 0x82, 0x22, 0xf3, 0xad };
 * 3) Enter a valid Device Address (DEVADDR)
 *    For example: { 0xe4, 0x07, 0xe3, 0x3b, 0xef, 0xf3, 0x80, 0x6c, 0x7c, 0x6e, 0x42, 0x43, 0x56, 0x7c, 0x22, 0x37 };
 * 
 * The NWKSKEY, APPSKEY and DEVADDR values should be obtained from your
 * LoRaWAN server (e.g., TTN or any private LoRa provider).
 *
 *******************************************************************************/

#include <lmic.h>			//  MCCI.   TODO  latest library verions update
#include <hal/hal.h>
#include <SPI.h>
#include <cactus_io_BME280_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//#include "TimerOne.h"     // Timer Interrupt set to 2 sec for read sensors
#include <math.h>
#include <Wire.h>         // For accessing RTC
#include <SD2405RTC.h>    // For Gravity RTC breakout board.   Set RTC to UTC time
#include <TimeLib.h>      // For epoch time en/decode
#include <Timezone.h>	  // For AU Eastern STD/DST so that daily readings are 24hr to 9am (local)

// Sensor-related definitions
// Set hardware pin assignments & pre-set constants
#define TX_Pin 4 				   // used to indicate web data tx
#define ONE_WIRE_BUS_PIN 29 	  //Data bus pin for DS18B20's

#define WindSensor_Pin (18)       //The pin location of the anemometer sensor
#define WindVane_Pin  (A13)       // The pin connecting to the wind vane sensor
#define VaneOffset  0		   // The anemometer offset from magnetic north
#define Bucket_Size  0.2 	   // mm bucket capacity to trigger tip count
#define RG11_Pin  3        		 // Interrupt pin for rain sensor
#define BounceInterval  15		// Number of ms to allow for debouncing
#define SampleInt_Pin   19		// Interrupt pin for RTC-generated sampling clock

// Set timer related settings for sensor sampling & calculation
#define Timing_Clock  1000000    //  0.5sec in millis
#define Sample_Interval   5		//  = number of Timing_Clock cycles  i.e. 2.5sec interval
#define Report_Interval   7    //  = number of sample intervals contributing to each upload report (each 5 min)
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
volatile float dailyRainfall;		//  total amount of rainfall in 24 hrs to 9am (local time)

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
 } obsSet;
		
union obsPayload
{
	obsSet	obsReport;
	uint8_t	readAccess[sizeof(obsSet)];
}sensorObs[2];

// AU Eastern Time Zone (Sydney, Melbourne)   Use next 3 lines for one time setup to be written to EEPROM
//TimeChangeRule auEDST = {"AEDT", First, Sun, Oct, 2, 660};    //Daylight time = UTC + 11:00 hours
//TimeChangeRule auESTD = {"AEST", First, Sun, Apr, 2, 600};     //Standard time = UTC + 10:00 hours
//Timezone auEastern(auEDST, auESTD);

// If TimeChangeRules are already stored in EEPROM, comment out the three
// lines above and uncomment the line below.
Timezone auEastern(100);       //assumes rules stored at EEPROM address 100 & that RTC set to UTC
TimeChangeRule *tcr;		//pointer to the timechange rule
time_t utc, local;

int  currentObs, reportObs;   //References which obsPayload [0,1 or 2]is being filled etc. 
bool txState;				// current LED state for tx rx LED
int vaneValue;         	 	//  raw analog value from wind vane
int vaneDirection;          //  translated 0-360 direction
int calDirection, calGustDirn;     	//  converted value with offset applied
int lastDirValue;          //  last direction value

//  Test data for uploading - an hour's worth (at 2 min sample frequency)
static int observations[7][15] {
	{900, 953, 1012, 1067, 1118, 1164, 1216, 1266, 1347, 1398, 1441, 1492, 1537, 1588, 1631},  //temp
	{8703, 8834, 8964, 9091, 9205, 9337, 9468, 9591, 9720, 9844, 9977, 10104, 10231, 10368, 10491},  //press
	{233, 282, 334, 381, 438, 487, 539, 585, 633, 671, 724, 774, 824, 877, 944},   //humidity
	{0, 0, 12, 22, 33, 48, 55, 101, 151, 233, 0, 0, 121, 188, 0},   //rainfall
	{0, 0, 22, 26, 27, 13, 0, 0, 55, 105, 310, 555, 845, 201, 41},  //windspeed
	{300, 322, 294, 310, 340, 350, 10, 15, 12, 200, 210, 175, 170, 180, 260},   //wind direction
	{0, 0, 12, 22, 33, 48, 55, 101, 151, 233, 233, 233, 452, 0, 188}      // dailyrain
};



// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x1A, 0x71, 0xFD, 0x1C, 0xFC, 0x99, 0x53, 0x84, 0xE2, 0xCD, 0x7B, 0xEE, 0xBB, 0x7F, 0xE3, 0xF9 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x14, 0xEE, 0x5D, 0xE6, 0x45, 0xDE, 0x42, 0xA1, 0xA7, 0xAA, 0xF9, 0xAF, 0x36, 0x94, 0x90, 0x6E };

//  Create BME280 object
BME280_I2C bme;     // I2C using address 0x77

// Setup a oneWire instance to communicate with OneWire devices
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature DSsensors(&oneWire);    // Pass the OneWire reference to Dallas Temperature lib

// Assign the addresses of the DS18B20 sensors (determined by reading them previously)
DeviceAddress airTempAddr = { 0x28, 0x1A, 0x30, 0x94, 0x3A, 0x19, 0x01, 0x55 };
DeviceAddress caseTempAddr = { 0x28, 0xAA, 0x68, 0x93, 0x41, 0x14, 0x01, 0xD8 };


// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed.
static const u4_t DEVADDR = 0x26002FB5; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 300 ;		// 5 min reporting cycle

// Pin mapping
// TL Modifications:
// Specifically for Arduino Uno + Dragino LoRa Shield US900
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)\n"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
//            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
			break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
			
			
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){

//static int sample = 0;
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
//		sample %= 15;     // loop through test data arrays
//		payload.obsReport.tempX10 = observations[0][sample];
//		payload.obsReport.pressX10 = observations[1][sample];
//		payload.obsReport.humidX10 = observations[2][sample];
//		payload.obsReport.rainflX10 = observations[3][sample];
//		payload.obsReport.windspX10 = observations[4][sample];
//		payload.obsReport.windDir = observations[5][sample];
//		payload.obsReport.windGustX10 = observations[4][sample];
//		payload.obsReport.windGustDir = observations[5][sample];
//		payload.obsReport.dailyRainX10 = observations[6][sample];
//cli();
//        LMIC_setTxData2(1, sensorObs[reportObs].readAccess, sizeof(obsSet), 0);
        Serial.println(F("Packet queued"));
		Serial.print(currentObs);Serial.print(F("\t")); Serial.println(sensorObs[reportObs].obsReport.humidX10);
        Serial.print(F("Sending packet on frequency: "));
        Serial.println(LMIC.freq);
//		sei();
//		sample += 1;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
	
	while (!Serial); // wait for Serial to be initialized
	Serial.begin(115200);
	delay(200);     // per sample code on RF_95 test
	Serial.println(F("Starting")); Serial.flush();


	
//	setSyncProvider(RTC.get);
//	setSyncInterval(500);     // resync system time to RTC every 500 sec

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
 
	if (!bme.begin())  {
      Serial.println("Could not find BME280 sensor -  check wiring");
     while (1);
	}

	
	#ifdef VCC_ENABLE
	// For Pinoccio Scout boards
		pinMode(VCC_ENABLE, OUTPUT);
		digitalWrite(VCC_ENABLE, HIGH);
		delay(1000);
	#endif
	
  // Setup pins & interrupts	
	pinMode(TX_Pin, OUTPUT);
	pinMode(WindSensor_Pin, INPUT);
	pinMode(RG11_Pin, INPUT);
	pinMode(SampleInt_Pin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING);
	attachInterrupt(digitalPinToInterrupt(RG11_Pin),isr_rg, FALLING);
	attachInterrupt(digitalPinToInterrupt(SampleInt_Pin), isr_timer, FALLING);
Serial.println(F("attached interrupts")); Serial.flush();
  		stop();  /////*****
   
    // LMIC init
    os_init();
Serial.println(F("returned from os_init"));
	stop();  /////*****
		
    // Reset the MAC state. Session and pending data transfers will be discarded.
//**    Serial.print(F("Max Clock Error\t"));
//**    Serial.println(MAX_CLOCK_ERROR);
//**    LMIC_setClockError(MAX_CLOCK_ERROR * 20/100);   //**
    LMIC_reset();


    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    // Specify to operate on AU915 sub-band 2
    #elif defined(CFG_au921)
    Serial.println(F("Loading AU915/AU921 Configuration..."));
    // Set to AU915 sub-band 2
    LMIC_selectSubBand(1); 
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    //LMIC.dn2Dr = DR_SF9;
	LMIC.dn2Dr = DR_SF7CR;    //** now uses SF7 

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);
  Serial.println(F("ready to do_send"));
    // Start job
    do_send(&sendjob);
	
	  //Setup the timer for 0.5s
//	Timer1.initialize(Timing_Clock);     
//	Timer1.attachInterrupt(isr_timer);
	
	Serial.println(F("enabling interrupts"));  
	sei();   // Enable Interrupts
	
}

void loop() {

	DSsensors.requestTemperatures();    // Read temperatures from all DS18B20 devices
	bme.readSensor();
	
	
	if(isSampleRequired) {
	  
		sampleCount++;
	Serial.print(sampleCount);   Serial.print(F("\t= sample\t"));  Serial.print(currentObs); Serial.print(reportObs);
		getWindDirection();
		Serial.print(F("\t")); Serial.print(windSpeed);  Serial.print(F("\t"));  Serial.println(calDirection);
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
			cli();
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
//		printIt(sensorObs[currentObs].readAccess, sizeof(obsSet));        //  Check dump of 16 Byte obsSet structure
		
			sampleCount = 0;
			currentObs = 1- currentObs;
			reportObs = 1 - currentObs;   // switch reporting to last collected observation
			windGust = 0;
			sei();
		}
			
		isSampleRequired = false;
	}
	
      
    os_runloop_once();
    
}

// Interrupt handler routine for timer interrupt
void isr_timer() {
	
	timerCount++;

	if(timerCount == Sample_Interval) {
		cli();
		// convert to km/h using the formula V=P(2.25/T)*1.609 where T = sample interval
		// i.e. V = P(2.25/2.5)*1.609 = P * Speed_Conversion factor  (=1.4481  for 2.5s interval)
		windSpeed = rotations * Speed_Conversion; 
		rotations = 0;   
		txState = !txState;     
		digitalWrite(TX_Pin, txState);      // Transmit LED
		isSampleRequired = true;
		timerCount = 0;						// Restart the interval count
		sei();
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

// Debugging utility
void stop() {
	while(1);
}
