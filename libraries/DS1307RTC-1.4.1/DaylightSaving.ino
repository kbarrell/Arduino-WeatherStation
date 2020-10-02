#include <EEPROM.h>
#include  <RTClib.h>


RTC_DS3231 rtc;
int DST;


void setup()
{

DST = EEPROM.get(0, DST);

if(DST != 0 && DST != 1)  

{     

DST = 1;  

EEPROM.put(0, DST);

}

// Uncomment the following 2 lines if you need to set the DST (if you miss the actual day etc). Change the +1 to -1 in the fall.
// DateTime t2 = rtc.now();   
// rtc.adjust(DateTime(t2.year(), t2.month(), t2.day(), t2.hour()+1, t2.minute(), t2.second()));

}


void loop()
{

DateTime now = rtc.now();   

if (now.dayOfTheWeek() == 0 && now.month() == 3 && now.day() >= 8 && now.day() <= 16 && now.hour() == 2 && now.minute() == 0 && now.second() == 0 && DST == 0)     
{       

rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour()+1, now.minute(), now.second()));       
DST = 1;       
EEPROM.put(0, DST);     

}     
else if(now.dayOfTheWeek() == 0 && now.month() == 11 && now.day() >= 1 && now.day() <= 8 && now.hour() == 2 && now.minute() == 0 && now.second() == 0 && DST == 1)     
{       

rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour()-1, now.minute(), now.second()));       
DST = 0;       
EEPROM.put(0, DST);     

}

}