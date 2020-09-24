#include <math.h>
#include <Wire.h>         // For accessing RTC
#include <SD2405RTC.h>    // For Gravity RTC breakout board
#include <TimeLib.h>      // For epoch time en/decode

// Define structures for handling reporting via TTN
typedef struct obsSet {
	time_t 	obsReportTime;  // unixtime    u32bits
	int			tempX10;	// observed temp (°C) x 10   ~range -200->600
	uint16_t	humidX10;	// observed relative humidty (%) x 10   range 0->1000
	int		 	pressX10;	// observed barometric pressure at station level (hPa) - 1000 x 10  ~range -500->500 
	uint16_t	rainflX10;	// observed accumulated rainfall (mm) x10   ~range 0->1200
	uint16_t	windspX10;	// observed windspeed (km/h) x10 ~range 0->1200
	int			windDir;	// observed wind direction (compass degress)  range 0->359
 }obsSet;
	
	
	
union obsPayload
{
	obsSet	obsReport;
	char	readAccess[sizeof(obsSet)];
};

static uint8_t mydata[5] { 0xB8, 0xA0, 0xA1, 0xA2, 0xD7};

void setup()  {
	
	tmElements_t  tm;
	time_t created_time, system_time;
	int i,j;
	
	while (!Serial);
	Serial.begin(115200);
	delay (100);
	Serial.println("Starting");
	setSyncProvider(RTC.get);
	if(timeStatus()!= timeSet) 
     Serial.println("Unable to sync with the RTC");
  else
     Serial.println("RTC has set the system time");
	setSyncInterval(500);
	
	

	
	RTC.read(tm);
	Serial.print("   Year = ");//year
  Serial.print(tm.Year);
  Serial.print("   Month = ");//month
  Serial.print(tm.Month);
  Serial.print("   Day = ");//day
  Serial.print(tm.Day);
//  Serial.print("   Week = ");//week
//  Serial.print(tm.Week);
  Serial.print("   Hour = ");//hour
  Serial.print(tm.Hour);
  Serial.print("   Minute = ");//minute
  Serial.print(tm.Minute);
  Serial.print("   Second = ");//second
  Serial.println(tm.Second);
  delay(1000);

    system_time = now();
	created_time = makeTime(tm);;
	Serial.println(created_time, HEX);
  Serial.println(created_time, BIN);
  Serial.println(created_time);
  Serial.println(system_time);
  digitalClockDisplay();
  printBuffer();
  for (i=0; i<4; i++) {
	  mydata[i] = (created_time >> i*8) & 0xFF;
	 printBuffer(); 
  }
}

void loop()
{
  
}

void printBuffer() {
	int j;
	for (j=0; j<5; j++) {
		Serial.print(mydata[j], HEX);
    if (j == 3) Serial.print(' ');
	  }
	  Serial.println();
}
	

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
