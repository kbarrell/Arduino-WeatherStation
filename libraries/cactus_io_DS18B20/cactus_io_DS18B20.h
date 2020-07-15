/*
 Copyright (c) 2007, Jim Studt
 
 This library derived from the OneWire library by Jim Studt
 
 http://www.pjrc.com/teensy/td_libs_OneWire.html
 
 This is the same library as the one on the Arduino.cc website
 
 2015-06-10 Code modified by cactus.io to make it fit in with the style
 and naming conventions we use for our libraries. This includes renaming
 the class onewire to DS18B20.
*/
 
// #ifndef cactus_io_DS18B20_h
// #define cactus_io_DS18B20_h

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <inttypes.h>

// you can exclude onewire_search by defining that to 0
#ifndef DS18B20_SEARCH
#define DS18B20_SEARCH 1
#endif

// You can exclude onewire_crc16 by defining that to 0
#ifndef DS18B20_CRC16
#define DS18B20_CRC16 0
#endif

class DS18B20
{
  private:
#if DS18B20_SEARCH
    uint8_t address[8];
    char searchJunction;
    uint8_t searchExhausted;
#endif
    byte addr[8];
    uint8_t pin;
    uint8_t port;
    uint8_t bitmask;
    volatile uint8_t *outputReg;
    volatile uint8_t *inputReg;
    volatile uint8_t *modeReg;
    
    float temperature_C;
    float temperature_F;

  public:
    DS18B20( uint8_t pin);
    void readSensor(void);
    byte * getSerialNumber(void);
    float getTemperature_C(void);
    float getTemperature_F(void);
    
    // Perform a 1-Wire reset cycle. Returns 1 if a device responds
    // with a presence pulse.  Returns 0 if there is no device or the
    // bus is shorted or otherwise held low for more than 250uS
    uint8_t reset();

    // Issue a 1-Wire rom select command, you do the reset first.
    void select( uint8_t rom[8]);

    // Issue a 1-Wire rom skip command, to address all on bus.
    void skip();

    // Write a byte. If 'power' is one then the wire is held high at
    // the end for parasitically powered devices. You are responsible
    // for eventually depowering it by calling depower() or doing
    // another read or write.
    void write( uint8_t v, uint8_t power = 0);

    // Read a byte.
    uint8_t read();

    // Write a bit. The bus is always left powered at the end, see
    // note in write() about that.
    void write_bit( uint8_t v);

    // Read a bit.
    uint8_t read_bit();

    // Stop forcing power onto the bus. You only need to do this if
    // you used the 'power' flag to write() or used a write_bit() call
    // and aren't about to do another read or write. You would rather
    // not leave this powered if you don't have to, just in case
    // someone shorts your bus.
    void depower();

#if DS18B20_SEARCH
    // Clear the search state so that if will start from the beginning again.
    void reset_search();

    // Look for the next device. Returns 1 if a new address has been
    // returned. A zero might mean that the bus is shorted, there are
    // no devices, or you have already retrieved all of them.  It
    // might be a good idea to check the CRC to make sure you didn't
    // get garbage.  The order is deterministic. You will always get
    // the same devices in the same order.
    uint8_t search(uint8_t *newAddr);
#endif

    // Compute a Dallas Semiconductor 8 bit CRC, these are used in the
    // ROM and scratchpad registers.
    static uint8_t crc8( uint8_t *addr, uint8_t len);
#if DS18B20_CRC16
    // Compute a Dallas Semiconductor 16 bit CRC. Maybe. I don't have
    // any devices that use this so this might be wrong. I just copied
    // it from their sample code.
    static unsigned short crc16(unsigned short *data, unsigned short len);
#endif

};
