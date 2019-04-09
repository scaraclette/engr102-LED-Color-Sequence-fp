/*
* Name: Gusti Scarlett Halima
* Date: December 9th 2017
* Class: ENGR 102 Intro to Electronics (Fall 2017)
* Software Version: Arduino 1.8.2
* What it does: This is a my final project which is a
* bluetooth controlled RGB LED color sequence circuit.
* This code was based on Gerrikoio's tutorial on how to
* use a Feather M0 Blufruit LE board and connect it to an
* app made for an Android phone. Original code I wrote 
* starts on the loop.
*/

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
/*********************************************************************
* This is modified program is linked to an MIT App Inventor App to control
* up to 6 GP-O's. This program only accepts data from the App. It
* does not reply to the app in any way.
* 
* App developed by Gerrikoio (modified by Scarlett), using an example provided with the nRF51822
* library from Adafruit.
* 
* MIT license applies.
* 
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

  FACTORYRESET_ENABLE     Perform a factory reset when running this sketch
 
                            Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                            running this at least once is a good idea.
 
                            When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0. If you are making changes to your
                            Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why. Factory reset will erase
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
    #define FACTORYRESET_ENABLE         1
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
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);



// -------------- DEMO APP GP-O PIN REFERENCE --------------------------------------------------
// These are the pin reference values (if you do not want a pin set then enter a 0 in the array) 
// MIT App button "Pin1" is 1st item in the array (refers to Feather MO pin-5), button "Pin2" is the 2nd (refers to Feather MO pin-6) etc. 
// ---------------------------------------------------------------------------------------------

const byte pinRef[3] = {5, 6, 9}; //Modified to only use 3 pins for the RGBled
int sequenceArray[10];
int count = 0;
boolean printSequence = false;

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
  delay(500);

  // Configure the pin modes
  for (byte i = 0; i < 3; i++) {
    pinMode(pinRef[i], OUTPUT);
    digitalWrite(pinRef[i], HIGH);
  }

  Serial.begin(115200); //Note this part omg 
  Serial.println(F("MIT App Inventor GPIO Control Example"));
  Serial.println(F("------------------------------------------------"));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

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

  /* Print Bluefruit information */
  ble.info();

  ble.verbose(false);  // debug info is a little annoying after this point!

  Serial.println(F("Waiting for connection"));
  delay(500);
  for (byte i = 0; i < 6; i++) {
    digitalWrite(pinRef[i], LOW);
    delay(500);
  }


  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Connected. Switching to Data mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
  Serial.println(F("******************************"));
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  if (ble.available()) {
    byte c = ble.read();
    Serial.print("c byte value: "); //Some byte value, how does it work though? what are the raw bytes?
    Serial.println(c);
    Serial.print("c byte to char value: ");
    Serial.println((char)c);
    
    if (c == 69) {
      printSequence = true;
      printArray();
      for (int i = 0; i < 10; i++) {
          if (sequenceArray[i] == 0) {
            break;
            }
          lightColor(sequenceArray[i]);
          delay(500);
        }
        resetArray();
        printArray();
        count = 0;
        printSequence = false;
        digitalWrite(pinRef[0], LOW);
        digitalWrite(pinRef[1], LOW);
        digitalWrite(pinRef[2], LOW);
      }

    //Cannot add sequence if count = 10
    if (!printSequence && ((c >= 65 && c <= 67) || (c >= 88 && c <= 90) ) && (count != 10)) {
      lightColor(c);
      sequenceArray[count] = c;
      count++;
      printArray();
      }    
  }
  else {
      digitalWrite(pinRef[0], LOW);
      digitalWrite(pinRef[1], LOW);
      digitalWrite(pinRef[2], LOW);
  }
}

void resetArray() {
  for (int i = 0; i < 10; i++) {
    sequenceArray[i] = 0;
    }
  }
  

void printArray() {
  for (int i = 0; i < 10; i++) {
      Serial.print(sequenceArray[i]);
      Serial.print(", ");
    }
    Serial.println();
  }

void lightColor(byte c) {
      switch(char(c)) {
          case 'A': { //Blue
              digitalWrite(pinRef[0], HIGH);
              digitalWrite(pinRef[1], LOW);
              digitalWrite(pinRef[2], LOW);
            }; break;
          case 'B': {
              digitalWrite(pinRef[0], LOW);
              digitalWrite(pinRef[1], HIGH);
              digitalWrite(pinRef[2], LOW);
            }; break;
          case 'C': {
              digitalWrite(pinRef[0], LOW);
              digitalWrite(pinRef[1], LOW);
              digitalWrite(pinRef[2], HIGH);
            }; break;
           case 'X': {
              digitalWrite(pinRef[0], HIGH); //blue
              digitalWrite(pinRef[1], HIGH); //green
              digitalWrite(pinRef[2], LOW); //red
            }; break;
          case 'Y': {
              digitalWrite(pinRef[0], HIGH);
              digitalWrite(pinRef[1], LOW);
              digitalWrite(pinRef[2], HIGH);
            }; break;
          case 'Z': {
              digitalWrite(pinRef[0], LOW);
              digitalWrite(pinRef[1], HIGH);
              digitalWrite(pinRef[2], HIGH);
            }; break; 
           default: {
              digitalWrite(pinRef[0], LOW); //blue
              digitalWrite(pinRef[0], LOW); //green
              digitalWrite(pinRef[0], LOW); //red
            }; break; 
            
        }
}

