#include <SPI.h>           // used to communicated via the spi bus
#include <Ethernet.h>      // used to communicate with the ethernet controller

#include "cactus_io_DS18B20.h"

int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2

// Create DS18B20 object
DS18B20 ds(DS18S20_Pin);  // on digital pin 2

// Here we setup the webserver. We need to supply a mac address. Some ethernet boards have a label attached
// with a mac address you can use.
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 45);      // IP address, may need to change depending on network
EthernetServer server(80);           // create a server at port 80

void setup() {
  
  // disable the SD card by switching pin 4 high
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
  
  Serial.begin(9600);
  Serial.println("cactus.io | Basic Webserver Tutorial");
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
}

void loop() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
       if (client.available()) {
          char c = client.read();
          Serial.write(c);
          // if you've gotten to the end of the line (received a newline
          // character) and the line is blank, the http request has ended,
          // so you can send a reply
          if (c == '\n' && currentLineIsBlank) {
             // send a standard http response header
             client.println("HTTP/1.1 200 OK");
             client.println("Content-Type: text/html");
             client.println("Connection: close");  // the connection will be closed after completion of the response
             client.println("Refresh: 5");  // refresh the page automatically every 5 sec
             client.println();
             client.println("<!DOCTYPE HTML>");
             client.println("<html>");
             // output the value from the DS18B20 temperature sensor
             ds.readSensor();
             client.print("<br>&nbsp;&nbsp;<span style=\"font-size: 26px\";>Temperature is ");    
             client.print(ds.getTemperature_C());
             client.println(" &deg;C</span><br />");
             client.println("</html>");
             break;
           }
           if (c == '\n') {
             // you're starting a new line
             currentLineIsBlank = true;
           } else if (c != '\r') {
             // you've gotten a character on the current line
             currentLineIsBlank = false;
           }
         }
       }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    // Serial.println("client disconnected");

}
