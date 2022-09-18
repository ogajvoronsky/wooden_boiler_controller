# A blank Mongoose OS app

## Overview

This is an empty app, serves as a skeleton for building Mongoose OS
apps from scratch.

# esp12F_relay_X8 
const byte relay1Pin = 16;   // GPIO16 *
const byte relay2Pin = 14;   // GPIO14
const byte relay3Pin = 12;   // GPIO12
const byte relay4Pin = 13;   // GPIO13
const byte relay5Pin = 15;   // GPIO15 *
const byte relay6Pin = 0;    // GPIO0
const byte relay7Pin = 4;    // GPIO4
const byte relay8Pin = 5;    // GPIO5
//const byte io0Pin = 0;     // GPIO00 - flash Jumper - also used for relay6
const byte io2Pin = 2;       // GPIO02 - on header available
//const byte adcPin = A0;    // A0 - on header available 
const uint8_t ledEspPin = 2; // GPIO02 is the (blue) LED on the ESP-12F
// RX Serial = 3             // on header available - can be used for sketch upload
// TX Serial = 1             // on header available