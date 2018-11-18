/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Adafruit_CircuitPlayground.h>

#include "BluefruitConfig.h"



#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#define numel(x)  (sizeof(x) / sizeof(*x))
#define SIZE_ALL 100 // size of sliding window for mag


float X, Y, Z, mag;
short slidingWindowAll_y[SIZE_ALL]; // sliding window of raw mag data
short slidingWindowAll_x[SIZE_ALL]; // sliding window of raw mag data
short slidingWindowAll_z[SIZE_ALL]; // sliding window of raw mag data
//short slidingWindowAll_mag[SIZE_ALL]; // sliding window of raw mag data
String result = "result";
//unsigned byte a = 0; // loop count when start is received
//unsigned byte z = 0; // loop count when stop is received
//byte dur; // loop count between when start and stop is received
boolean collect = false; // flag to collect data between start and stop
/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
// Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  CircuitPlayground.begin();
//  while (!Serial);  // required for Flora & Micro
  delay(500);
  
  Serial.begin(9600);
  CircuitPlayground.begin();
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/

byte ii = 0; // iteration of loop() during collect period


void loop(void)
{
//  ii = ii + 1;
//  Serial.println("ii " + String(ii));
//  Serial.print(" ble.available() " + String(ble.available())); 
//  Serial.print(" collect " + String(collect)); 

  // Echo received data
  while ( ble.available() )
  {
    byte c = ble.read();

//    Serial.print((char)c);

    // Hex output too, helps w/debugging!
//    Serial.print(" [0x");
    if (c <= 0xF) Serial.print(F("0"));
//    Serial.print(c, HEX);
//    Serial.print("] ");
    if ((char)c == 'a')
    {
      collect = true;
//      a = ii;
      ii = 0;     
    }
    if ((char)c == 'z')
    {
      Serial.println("stop");
      collect = false;
      byte start = (byte)SIZE_ALL - (byte)ii; // start index for the 100-sized array which usually has less than 100 samples
      if (start < 0){ // if more than 100 samples only look at last 100, usually ~95
        start = 0;
      } 
      float xMean = getMean(slidingWindowAll_x, start, SIZE_ALL);
      float yMean = getMean(slidingWindowAll_y, start, SIZE_ALL);
      float zMean = getMean(slidingWindowAll_z, start, SIZE_ALL);
      float xMax = getMax(slidingWindowAll_x, start, SIZE_ALL);
      float yMax = getMax(slidingWindowAll_y, start, SIZE_ALL);
      float zMax = getMax(slidingWindowAll_z, start, SIZE_ALL);
      float xMin = getMin(slidingWindowAll_x, start, SIZE_ALL);
      float yMin = getMin(slidingWindowAll_y, start, SIZE_ALL);
      float zMin = getMin(slidingWindowAll_z, start, SIZE_ALL);

//      Serial.println("magMax: " + String(magMax));
//      Serial.println("magMin: " + String(magMin));
//      Serial.println("magMean: " + String(magMean));
      String features = "";
      features += "xMean;" + String(xMean);
      features += ";xMax;" + String(xMax);
      features += ";xMin;" + String(xMin);
      features += ";yMean;" + String(yMean);
      features += ";yMax;" + String(yMax);
      features += ";yMin;" + String(yMin);
      features += ";zMean;" + String(zMean);
      features += ";zMax;" + String(zMax);
      features += ";zMin;" + String(zMin);
      features += "!";
      Serial.println(features);
      ble.print(features);
    }
  }
   if (collect) // between start and stop
    {
      ii += 1;
      X = CircuitPlayground.motionX();
      Y = CircuitPlayground.motionY();
      Z = CircuitPlayground.motionZ();
      mag = sqrt(X*X + Y*Y + Z*Z);
//      addToWindow(slidingWindowAll_mag,SIZE_ALL, mag*100);
      addToWindow(slidingWindowAll_x,SIZE_ALL, X*100);
      addToWindow(slidingWindowAll_y,SIZE_ALL, Y*100);
      addToWindow(slidingWindowAll_z,SIZE_ALL, Z*100);
//      Serial.println();
//      Serial.println("ii-a " + String(ii-a));
//      Serial.println("ii " + String(ii));
//      Serial.print("mag " + String(slidingWindowAll_mag[SIZE_ALL-1]));
      Serial.print(",x " + String(slidingWindowAll_x[SIZE_ALL-1]));
      Serial.print(",y " + String(slidingWindowAll_y[SIZE_ALL-1]));
      Serial.print(",z " + String(slidingWindowAll_z[SIZE_ALL-1]));
      Serial.println();
      
    }
  delay(13); // delay so that ~100 loops in 1.5 second
}

// add 1 element to a sliding window array
void addToWindow(short *slidingWindow, byte ssize, short x) {
  for (byte i = 1; i < ssize; i++) {
    slidingWindow[i -  1] = slidingWindow[i]; // shift old values 1 to the left, 0th element is deleted by being overwritten
  }
  slidingWindow[ssize - 1] = x; // last element in array set to new value
}

float getMean(short window[], byte start, byte ssize) {
//  Serial.println("start: " + String(start));
//  Serial.println("ssize: " + String(ssize));
  float arr_sum;
  arr_sum = 0;
  for (byte i = start; i < ssize; i++) {
//    Serial.println(arr_sum);
    arr_sum = arr_sum + window[i];
  }
//  Serial.println(arr_sum/(float) ssize);
  return arr_sum / (ssize - start);
}

short getMax(short window[], byte start, byte ssize) {
  short arr_m = window[start];
  for (byte i = start; i < ssize; i++) {
    if (window[i] > arr_m) {
      arr_m = window[i];
    }
  }
  return arr_m;
}

short getMin(short window[], byte start, byte ssize) {
  short arr_m = window[start];
  for (byte i = start; i < ssize; i++) {
    if (window[i] < arr_m) {
      arr_m = window[i];
    }
  }
  return arr_m;
}


