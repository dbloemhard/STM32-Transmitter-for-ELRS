/*
 * This file is part of Simple TX
 *
 * Simple TX is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Simple TX is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 // Hardware setup (Pins and UART selection)

// Port setup
// #define ELRS_PORT Serial1 - this is hard coded
// Comment out the next line to use full duplex (TX and RX wires)
#define ELRS_HALF_DUPLEX
#define ELRS_TX   PB6
#define ELRS_RX   PB7
// pins used for Serial input (Headtracker)
#define HT_PORT   Serial2
#define HT_TX     PA2
#define HT_RX     PA3

// pins that used for the Joystick
const uint32_t analogInPinAileron = PA4;
const uint32_t analogInPinElevator = PA5;
const uint32_t analogInPinThrottle = PA6;
const uint32_t analogInPinRudder = PA7;
const uint32_t VOLTAGE_READ_PIN = PB1;

// pins that used for the switch
const uint32_t DIGITAL_PIN_SWITCH_ARM = PA1;  // Arm switch
const uint32_t DIGITAL_PIN_SWITCH_AUX2 = PC14; //
const uint32_t DIGITAL_PIN_SWITCH_AUX3 = PC15;  //
// If the following line is uncommented, the 3 position switch will send low/mid/high on channel 6
// Alternatively it will send one position as Ch6 high, middle as nothing, 3rd position as ch7 high
//#define USE_3POS_SWITCH_AS_1_CHANNEL 
const uint32_t DIGITAL_PIN_SWITCH_AUX4 = 0;  // Set to 0 to disable this input

const uint32_t DIGITAL_PIN_BUTTON = PA0;
// pins that used for output
const uint32_t DIGITAL_PIN_LED = PC13;  

// pins that used for buzzer
const uint32_t DIGITAL_PIN_BUZZER = PB10;