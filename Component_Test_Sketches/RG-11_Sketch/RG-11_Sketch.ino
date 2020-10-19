/* This is a demo for the Hydreon Rain Sensor. 

For information about hooking up this board to the Arduino then click on this link 
to visit the hookup guide on the cactus.io website.
http://cactus.io/hookups/weather/rain/hydreon/hookup-arduino-to-hydreon-rg-11-rain-sensor
*/

#define RG11_Pin 3

#define Bucket_Size 0.2    //  mm

volatile unsigned long tipCount;     // bucket tip counter used in interrupt routine
volatile unsigned long ContactTime;  // Timer to manage any contact bounce in interrupt routine

long lastCount;
float totalRainfall;

void setup() {

   lastCount = 0;
   tipCount = 0;
   totalRainfall = 0;

   Serial.begin(9600);
   Serial.println("Hydreon RG-11 Rain Sensor | cactus.io");
   Serial.print("Bucket Size: "); Serial.print(Bucket_Size); Serial.println(" mm");
  
   pinMode(RG11_Pin, INPUT);   // set the digital input pin to input for the RG-11 Sensor
   attachInterrupt(digitalPinToInterrupt(RG11_Pin), rgisr, FALLING);     // attach interrupt handler to input pin.
   // we trigger the interrupt on the voltage falling from 5V to GND

   sei();         //Enables interrupts
}

void loop() {
   // we only display the tip count when it has been incremented by the sensor
   cli();         //Disable interrupts
  
   if(tipCount != lastCount) {
      lastCount = tipCount;
      totalRainfall = tipCount * Bucket_Size;
      Serial.print("Tip Count: "); Serial.print(tipCount);
      Serial.print("\tTotal Rainfall: "); Serial.println(totalRainfall); 
   }
  
   sei();         //Enables interrupts
}

// Interrrupt handler routine that is triggered when the rg-11 detects rain   
void rgisr ()   { 

   if ((millis() - ContactTime) > 15 ) {  // debounce of sensor signal
      tipCount++;
      ContactTime = millis();
   } 
} 
// end of rg-11 rain detection interrupt handler 
