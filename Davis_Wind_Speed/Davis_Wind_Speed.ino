
#include "TimerOne.h"     // Timer Interrupt set to 2 sec for read sensors
#include <math.h>

#define TX_Pin 8  // used to indicate web data tx

#define WindSensor_Pin (2)       //The pin location of the anemometer sensor
#define WindVane_Pin  (A2)       // The pin connecting to the wind vane sensor
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

void setup() {

  // setup anemometer values
  lastDirValue = 0;
  rotations = 0;
  isSampleRequired = false;
  
  // setup timer values
  timerCount = 0;
  
  Serial.begin(9600);
  Serial.println("Davis Anemometer Test");
  Serial.println("Speed  (KPH)\tKnots\tDirection\tStrength");
  
  pinMode(WindSensor_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING);

  
  //Setup the timer for 0.5s
  Timer1.initialize(500000);     
  Timer1.attachInterrupt(isr_timer);
  
  sei();   // Enable Interrupts
}

void loop() {
 
   if(isSampleRequired) {
	
	getWindDirection();
	
	Serial.print(windSpeed);   Serial.print(" kph\t");
	Serial.print(calDirection);   Serial.println("*");
	
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
