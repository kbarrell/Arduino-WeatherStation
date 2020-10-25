# Arduino-WeatherStation
Arduino-based weather station and web reporting via TTN to TagoIO.     ** Not yet complete **

This project expands on a multi-sensor weather station design originally published at http://cactus.io/projects/weather/arduino-weather-station. Rather than the Ethernet-based backhaul of that original design, this current integration adopts LoRaWAN radio technology to web-connect the weather station via The Things Network (TTN) https://www.thethingsnetwork.org. 

The project objective has been to deploy a full-capability station to collect weather observations and upload them to a web hosting platform for reporting and analysis. Taking advantage of the back-end integrations available via TTN, the TagoIO cloud IoT platform, https://tago.io has been used to provide the primary reporting dashboard. In addition, it further enables onward distribution of data to a public-interest weather observations website providing community access to both Australian Bureau of Meteorology and private citizen-supplied observations: http://bom-wow.metoffice.gov.uk. See the [Project Wiki](../../wiki) for design and integration details.

Caveat:  Multiple libraries and examples for LoRaWAN use with Arduino have been published over recent years.  As this project was the author's first exploration of LoRaWAN technology, the platforms chosen and approaches adopted are based on limited exposure to the field.  They are not necessarily the most robust currently available for anyone starting afresh.
