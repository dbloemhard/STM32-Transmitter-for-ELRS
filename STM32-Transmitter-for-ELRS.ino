/*
// Simple STM32F4 transmitter
// STM32 F411CE (WeAct Blackpill F411)
// ELRS 2.4G RX as a TX module
// Based on SimpleTX by kkbin505
// https://github.com/kkbin505/Arduino-Transmitter-for-ELRS

 * This file is part of STMple TX
 *
 * STMple TX is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * STMple TX is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "hardware.h"
#include "NonBlockingRtttl.h"
#include "EEPROM.h"
#include "config.h"
#include "crsf.h"
#include "elrsLua.h"
#include "led.h"
#include "tone.h"

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!
//#define TIMINGDEBUG  // When this is active Serial port will print out timing details

int Aileron_value = 0; // values read from the pot
int Elevator_value = 0;
int Throttle_value = 0;
int Rudder_value = 0;
int previous_throttle = 191;

struct ADCValues {
    int aileron;
    int elevator;
    int throttle;
    int rudder;
    int voltage;
};
ADCValues rawValues;

int loopCount = 0; // for ELRS seeting

int AUX1_Arm = 0; // switch values read from the digital pin
int AUX2_value = 0;
int AUX3_value = 0;
int AUX4_value = 0;

float batteryVoltage;
unsigned long buttonStartTime = 0;
const unsigned long longPressDuration = 2000;

int currentPktRate = 0;
int currentPower = 0;
int currentDynamic = 0;
int currentSetting = 0;
bool bindStarted = false;
bool wifiStarted = false;
int stickMoved = 0;
int stickInt = 0;
uint32_t stickMovedMillis = 0;
bool calibrationRequested=false;

bool lastArmState = false;        // Tracks previous arm switch position
uint32_t armTimer = 0;            // Records when the switch was turned on
bool gestureTriggerReady = false; // Arming arm-switch trigger sequence flag

uint32_t currentMillis = 0;
uint32_t lastDisplayTime = 0;

int16_t rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;
uint32_t lastModuleRequestTime = 0;
CRSF crsfClass;

// Instantiate application layer, passing driver object to it
ELRSLua elrsLua(crsfClass);

// Headtracker Serial2
//HardwareSerial Serial2(RX2, TX2);
HardwareSerial HT_PORT(HT_RX, HT_TX);

// SBUS Protocol constants
const byte SBUS_START_BYTE = 0x0F;
const byte SBUS_END_BYTE = 0x00;
const int SBUS_PACKET_SIZE = 25;
const uint16_t SBUS_MID_VALUE = 993; // The static value from the Betaflight PR

// SBUS parsing is glitchy on SoftSerial (Arduino only has one Serial port).
// This code attempts to filter out these glitches
// Set a reasonable Max Change per frame (SBUS sends ~100 frames per second)
// A jump of 20 in 10ms is very fast for head tracking; anything larger is likely a glitch.
// Smoothing factor is how fast the code will try to catch up to quick changes
const int MAX_FRAME_DELTA = 20; 
const int SMOOTHING_FACTOR = 3;

uint16_t sbusChannels[16];
uint16_t tempChannels[16];   // Temporary channels for validation
byte packetBuffer[SBUS_PACKET_SIZE];
int bufferIndex = 0;

uint32_t totalPackets = 0;
uint32_t goodPackets = 0;

bool headTrackerEnabled=false; // Must be enabled specifically by holding throttle MAX when powering on (otherwise you cannot use the radio normally)


// -----------------------------------------------------------------------------------------------------
// Calibration

#define CALIB_MARK      0x55
#define CALIB_MARK_ADDR 0x00
#define CALIB_VAL_ADDR  CALIB_MARK_ADDR + 1

#define CALIB_CNT       50       // times of switch on/off
#define CALIB_CENT_TMO  5000    //ms
#define CALIB_TMO       15000   // ms
int     cal_reset = 0 ;

struct CalibValues {
    int aileronMin;
    int aileronMax;
    int aileronCenter;
    int elevatorMin;
    int elevatorMax;
    int elevatorCenter;
    int thrMin;
    int thrMax;
    int rudderMin;
    int rudderMax;
    int rudderCenter;
};

CalibValues calValues;

bool calibrationPresent() {
    return EEPROM.read(CALIB_MARK_ADDR) == CALIB_MARK;
}

void calibrationSave() {
    EEPROM.update(CALIB_MARK_ADDR, CALIB_MARK);
    EEPROM.put(CALIB_VAL_ADDR, calValues);
}

void calibrationLoad() {
    EEPROM.get(CALIB_VAL_ADDR, calValues);
}

void calibrationReset() {
    EEPROM.put(CALIB_MARK_ADDR, (uint8_t)0xFF);
    calValues = {
        .aileronMin     = ANALOG_CUTOFF,
        .aileronMax     = 1023 - ANALOG_CUTOFF,
        .aileronCenter  = 988,
        .elevatorMin    = ANALOG_CUTOFF,
        .elevatorMax    = 1023 - ANALOG_CUTOFF,
        .elevatorCenter = 988,
        .thrMin         = ANALOG_CUTOFF,
        .thrMax         = 1023 - ANALOG_CUTOFF,
        .rudderMin      = ANALOG_CUTOFF,
        .rudderMax      = 1023 - ANALOG_CUTOFF,
        .rudderCenter    = 988,
    };
    EEPROM.put(CALIB_VAL_ADDR, calValues);
}

uint8_t aux2cnt = 0;

unsigned long calibrationTimerStart;

//Flash LED and beep for (times)
void calibrationChirp(uint8_t times) {
    
    digitalWrite(DIGITAL_PIN_LED, HIGH);
    delay(500);
    digitalWrite(DIGITAL_PIN_LED, LOW);
    delay(500);

    for (uint8_t i = 0; i < times; i++) {
#ifdef ACTIVE_BUZZER
        digitalWrite(DIGITAL_PIN_BUZZER, HIGH);
#else
        tone(DIGITAL_PIN_BUZZER,5000,100);
#endif
        digitalWrite(DIGITAL_PIN_LED, HIGH);
        delay(100);
#ifdef ACTIVE_BUZZER
        digitalWrite(DIGITAL_PIN_BUZZER, LOW);
#endif
        digitalWrite(DIGITAL_PIN_LED, LOW);
        delay(100);
    }
    digitalWrite(DIGITAL_PIN_LED, LOW);
}

bool calibrationRun() {
    if (!AUX1_Arm){
        Serial.println("Cannot calibrate while arm switch is active");
        cal_reset = 0;
        calibrationRequested = false;
        delay(1000);
        return false;
    }

    // Reset variables to "centers"
    while(cal_reset<1){
    Serial.println("Starting Calibration");
    calibrationChirp(2);    // start calibration
    calibrationTimerStart = millis();
    const int centerValue = (1023 - ANALOG_CUTOFF - ANALOG_CUTOFF) / 2;
    calValues.aileronMin     = 65535;
    calValues.aileronMax     = 0;
    calValues.aileronCenter  = 0;
    calValues.elevatorMin    = 65535;
    calValues.elevatorMax    = 0;
    calValues.elevatorCenter = 0;
    calValues.thrMin         = 65535;
    calValues.thrMax         = 0;
    calValues.rudderMin      = 65535;
    calValues.rudderMax      = 0;
    calValues.rudderCenter   = 0;
    cal_reset++;
    }

    currentMillis = millis();

    if (currentMillis <= (calibrationTimerStart + CALIB_CENT_TMO)){
        // A Center
        int val = analogRead(analogInPinAileron);
        calValues.aileronCenter = val;
        
        // E Center
        val = analogRead(analogInPinElevator);
        calValues.elevatorCenter = val;

        // R Center
        val = analogRead(analogInPinRudder);
        calValues.rudderCenter = val;
    
        Serial.print("Aileron_Min:");
        Serial.print("Center Stick: AilerCenter:");
        Serial.print(calValues.aileronCenter);
        Serial.print(" ElevatorCenter:");
        Serial.print(calValues.elevatorCenter);
        Serial.print(" RudderCenter:");
        Serial.print(calValues.rudderCenter);
        Serial.println();

        blinkLED(DIGITAL_PIN_LED, 1000);  // Slow flash while centering
        
    }
    // 15 seconds for moving sticks
    else if (currentMillis > (calibrationTimerStart + CALIB_CENT_TMO) && currentMillis <  (calibrationTimerStart + CALIB_TMO)){
        // A Min-Max
        int val = analogRead(analogInPinAileron);
        if (val < calValues.aileronMin) {
            calValues.aileronMin = val;
        } 
        if (val > calValues.aileronMax) {
            calValues.aileronMax = val;
        }
        // E Min-Max
        val = analogRead(analogInPinElevator);
        if (val < calValues.elevatorMin) {
            calValues.elevatorMin = val;
        } 
        if (val > calValues.elevatorMax) {
            calValues.elevatorMax = val;
        }

        // T Min-Max
        val = analogRead(analogInPinThrottle);
        if (val < calValues.thrMin) {
            calValues.thrMin = val;
        } 
        if (val > calValues.thrMax) {
            calValues.thrMax = val;
        }

        // R Min-Max
        val = analogRead(analogInPinRudder);
        if (val < calValues.rudderMin) {
            calValues.rudderMin = val;
        } 
        if (val > calValues.rudderMax) {
            calValues.rudderMax = val;
        }

        Serial.print("Move sticks full range: Aileron_Min:");
        Serial.print(calValues.aileronMin);
        Serial.print(" Max:");
        Serial.print(calValues.aileronMax);
        Serial.print(" ElevatorMin:");
        Serial.print(calValues.elevatorMin);
        Serial.print(" Max:");
        Serial.print(calValues.elevatorMax);
        Serial.print(" RudderMin:");
        Serial.print(calValues.rudderMin);
        Serial.print(" Max:");
        Serial.print(calValues.rudderMax);
        Serial.print(" ThrottleMin:");
        Serial.print(calValues.thrMin);
        Serial.print(" Max:");
        Serial.print(calValues.thrMax);
        Serial.println();

        blinkLED(DIGITAL_PIN_LED, 100);  // Fast flash while moving to limits
    }
    else {
        Serial.println("Calibration Done");  
        calibrationSave();
        cal_reset = 0;
        calibrationRequested = false;
        calibrationChirp(3);    // ok
    }
    return true;

}

// -----------------------------------------------------------------------------------------------------
// Handle analog input
// -----------------------------------------------------------------------------------------------------
void getStickVals() {
    // constrain to avoid overflow
    rawValues.aileron = analogRead(analogInPinAileron);
    if (rawValues.aileron <= calValues.aileronCenter){
        Aileron_value = map(rawValues.aileron, calValues.aileronMin, calValues.aileronCenter, ADC_MIN, ADC_MID);
    }
    else{
        Aileron_value = map(rawValues.aileron, calValues.aileronCenter, calValues.aileronMax, ADC_MID+1, ADC_MAX);
    }
    
    rawValues.elevator = analogRead(analogInPinElevator);
    if (rawValues.elevator <= calValues.elevatorCenter){
        Elevator_value = map(rawValues.elevator, calValues.elevatorMin, calValues.elevatorCenter, ADC_MIN, ADC_MID);
    }
    else{
        Elevator_value = map(rawValues.elevator, calValues.elevatorCenter, calValues.elevatorMax, ADC_MID+1, ADC_MAX);
    }

    rawValues.throttle = analogRead(analogInPinThrottle);
    Throttle_value = map(rawValues.throttle, calValues.thrMax, calValues.thrMin, ADC_MIN, ADC_MAX);

    rawValues.throttle = analogRead(analogInPinRudder);
    if (rawValues.throttle <= calValues.rudderCenter){
        Rudder_value = map(rawValues.throttle, calValues.rudderMin, calValues.rudderCenter, ADC_MIN, ADC_MID);
    }
    else{
        Rudder_value = map(rawValues.throttle, calValues.rudderCenter, calValues.rudderMax, ADC_MID+1, ADC_MAX);
    }
    //Serial.println();

    //Constrain value to avoid overflow
    Aileron_value  = constrain(Aileron_value,  ADC_MIN, ADC_MAX); 
    Elevator_value = constrain(Elevator_value, ADC_MIN, ADC_MAX); 
    Throttle_value = constrain(Throttle_value, ADC_MIN, ADC_MAX); 
    Rudder_value   = constrain(Rudder_value,   ADC_MIN, ADC_MAX); 

    //Handle reverse
    if (Is_Aileron_Reverse == 1){
        Aileron_value  = 1023-Aileron_value;
    }
    if (Is_Elevator_Reverse == 1){
        Elevator_value = 1023-Elevator_value;
    }
    if (Is_Throttle_Reverse == 1){
        Throttle_value = 1023-Throttle_value;
    }
    if (Is_Rudder_Reverse == 1){
        Rudder_value   = 1023-Rudder_value;
    }
}

// -----------------------------------------------------------------------------------------------------
// Head tracking support
// -----------------------------------------------------------------------------------------------------
void getSBUS() {
    while (Serial2.available() > 0) {
        byte b = Serial2.read();
        // State Machine: Look for Start Byte to begin a packet
        if (bufferIndex == 0 && b != SBUS_START_BYTE) {
            continue; // Skip until we find 0x0F 
        }
        packetBuffer[bufferIndex++] = b;
        // Once we have 25 bytes, verify the footer
        if (bufferIndex == SBUS_PACKET_SIZE) {
            totalPackets++;
            if (packetBuffer[SBUS_PACKET_SIZE - 1] == SBUS_END_BYTE) {
                goodPackets++;
                parseSBUS(); // Start and end bytes are valid, update channels
            }
            bufferIndex = 0; // Reset for next packet
        }
    }
}

void parseSBUS() {
  // SBUS uses 11-bit channel encoding across 22 bytes
  sbusChannels[0]  = ((packetBuffer[1]      | packetBuffer[2] << 8)                          & 0x07FF);
  sbusChannels[1]  = ((packetBuffer[2] >> 3 | packetBuffer[3] << 5)                          & 0x07FF);
  sbusChannels[2]  = ((packetBuffer[3] >> 6 | packetBuffer[4] << 2 | packetBuffer[5] << 10)  & 0x07FF);
  sbusChannels[3]  = ((packetBuffer[5] >> 1 | packetBuffer[6] << 7)                          & 0x07FF);
  sbusChannels[4]  = ((packetBuffer[6] >> 4 | packetBuffer[7] << 4)                          & 0x07FF);
  sbusChannels[5]  = ((packetBuffer[7] >> 7 | packetBuffer[8] << 1 | packetBuffer[9] << 9)   & 0x07FF);
  sbusChannels[6]  = ((packetBuffer[9] >> 2 | packetBuffer[10] << 6)                         & 0x07FF);
  sbusChannels[7]  = ((packetBuffer[10] >> 5 | packetBuffer[11] << 3)                        & 0x07FF);
  sbusChannels[8]  = ((packetBuffer[12]      | packetBuffer[13] << 8)                        & 0x07FF);
  sbusChannels[9]  = ((packetBuffer[13] >> 3 | packetBuffer[14] << 5)                        & 0x07FF);
  sbusChannels[10] = ((packetBuffer[14] >> 6 | packetBuffer[15] << 2 | packetBuffer[16] << 10) & 0x07FF);
  sbusChannels[11] = ((packetBuffer[16] >> 1 | packetBuffer[17] << 7)                        & 0x07FF);
  sbusChannels[12] = ((packetBuffer[17] >> 4 | packetBuffer[18] << 4)                        & 0x07FF);
  sbusChannels[13] = ((packetBuffer[18] >> 7 | packetBuffer[19] << 1 | packetBuffer[20] << 9)  & 0x07FF);
  sbusChannels[14] = ((packetBuffer[20] >> 2 | packetBuffer[21] << 6)                        & 0x07FF);
  sbusChannels[15] = ((packetBuffer[21] >> 5 | packetBuffer[22] << 3)                        & 0x07FF);
}

void setFromSBUS(int rcChannel, int sbusChannel) {
    int newValue = sbusChannels[sbusChannel];
    int oldValue = rcChannels[rcChannel];

    // SBUS values should be between 172 and 1811.
    if (newValue < 170 || newValue > 1820) return;

    // If the change is massive, it's likely a glitch.
    // Only update if the change is within a realistic physical range.
    if (abs(newValue - oldValue) < MAX_FRAME_DELTA) {
        rcChannels[rcChannel] = newValue;
    } 
    // We want it to eventually reach the value even if it's a huge jump (also helps for startup)
    else { rcChannels[rcChannel] += (newValue > oldValue) ? SMOOTHING_FACTOR : -SMOOTHING_FACTOR; }
}

// Map Stick/SBUS data to rcChannels array
void mapChannels(){
    if (headTrackerEnabled) {
        // Map channel data
        #ifdef USE_HT_ROLL
        //rcChannels[AILERON] = sbusChannels[0];
        setFromSBUS(AILERON,0);
        #else
        rcChannels[AILERON]   = map(Aileron_value,  ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); 
        #endif

        #ifdef USE_HT_PITCH
        //rcChannels[ELEVATOR] = sbusChannels[1];
        setFromSBUS(ELEVATOR,1);
        #else
        rcChannels[ELEVATOR]  = map(Elevator_value, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
        #endif

        #ifdef USE_HT_YAW
        //rcChannels[RUDDER] = sbusChannels[2];
        setFromSBUS(RUDDER,2);
        #else
        rcChannels[RUDDER]    = map(Rudder_value,   ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
        #endif

        rcChannels[THROTTLE]  = map(Throttle_value, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    }
    else {
        // If HT isnt enabled on startup, just read the stick values as normal
        // This allows you to use the TX as a regular TX when HT isnt enabled or needed
        rcChannels[AILERON]   = map(Aileron_value,  ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); 
        rcChannels[ELEVATOR]  = map(Elevator_value, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
        rcChannels[THROTTLE]  = map(Throttle_value, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
        rcChannels[RUDDER]    = map(Rudder_value,   ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    }

    if(stickInt=0){
        previous_throttle=rcChannels[THROTTLE];
        stickInt=1;
    }
}

void getAUXInputs(){
    /*
     * Handle digital input
     */

    AUX1_Arm = digitalRead(DIGITAL_PIN_SWITCH_ARM);
    if (Is_Arm_Reverse == 1) {
      AUX1_Arm = ~AUX1_Arm;
    }

    AUX2_value = digitalRead(DIGITAL_PIN_SWITCH_AUX2);
    AUX3_value = digitalRead(DIGITAL_PIN_SWITCH_AUX3);
    if (DIGITAL_PIN_SWITCH_AUX4 != 0){
      AUX4_value = digitalRead(DIGITAL_PIN_SWITCH_AUX4);
    }
    
    // Aux Channels
    rcChannels[AUX1] = (AUX1_Arm == 1)   ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
    #ifdef USE_3POS_SWITCH_AS_1_CHANNEL
      rcChannels[AUX2] = (AUX2_value == 1) ? (CRSF_DIGITAL_CHANNEL_MIN+CRSF_DIGITAL_CHANNEL_MAX)/2 : CRSF_DIGITAL_CHANNEL_MIN;
      rcChannels[AUX2] = (AUX3_value == 1) ? rcChannels[AUX2] : CRSF_DIGITAL_CHANNEL_MAX;
    #else
      rcChannels[AUX2] = (AUX2_value == 1) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
      rcChannels[AUX3] = (AUX3_value == 1) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
    #endif
    if (DIGITAL_PIN_SWITCH_AUX4 != 0){
      rcChannels[AUX4] = (AUX4_value == 1) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
    }    
}

// -----------------------------------------------------------------------------------------------------
// Startup stick commands
// -----------------------------------------------------------------------------------------------------
void checkGestureSetting() {
    bool currentArmState = AUX1_Arm; 

    // 1. Detect the moment the switch goes from DISARMED to ARMED (Rising Edge)
    if (currentArmState && !lastArmState) {
        armTimer = millis();
        gestureTriggerReady = true; 
    }

    // 2. Detect the moment the switch goes from ARMED back to DISARMED (Falling Edge)
    if (!currentArmState && lastArmState) {
        // Debounce: Ensure it was held armed for at least 1000ms
        if (gestureTriggerReady && (millis() - armTimer >= 1000)) {
            selectSetting(); 
            gestureTriggerReady = false; 
        }
    }
    lastArmState = currentArmState;
}

void selectSetting() {
    // startup stick commands (rate/power selection / initiate bind / turn on tx module wifi)
    // Right stick:
    // Up Left - Rate/Power setting 1 (250hz / 100mw / Dynamic)
    // Up Right - Rate/Power setting 2 (50hz / 100mw)
    // Down Left - Start TX bind (for 3.4.2 it is now possible to bind to RX easily). Power cycle after binding
    // Down Right - Start TX module wifi (for firmware update etc)
    // Left Stick
    // Throttle MAX - Enable Head Tracking (if defines are set)

    if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron left
        // currentPktRate = SETTING_1_PktRate;
        // currentPower = SETTING_1_Power;
        // currentDynamic = SETTING_1_Dynamic;
        // currentSetting = 1;
        crsfClass.crsfSendCommand(ELRS_PKT_RATE_COMMAND, SETTING_1_PktRate);
        crsfClass.crsfSendCommand(ELRS_POWER_COMMAND, SETTING_1_Power);
        crsfClass.crsfSendCommand(ELRS_DYNAMIC_POWER_COMMAND, SETTING_1_Dynamic);
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron right
        // currentPktRate = SETTING_2_PktRate;
        // currentPower = SETTING_2_Power;
        // currentDynamic = SETTING_2_Dynamic;
        // currentSetting = 2;
        crsfClass.crsfSendCommand(ELRS_PKT_RATE_COMMAND, SETTING_2_PktRate);
        crsfClass.crsfSendCommand(ELRS_POWER_COMMAND, SETTING_2_Power);
        crsfClass.crsfSendCommand(ELRS_DYNAMIC_POWER_COMMAND, SETTING_2_Dynamic);
    } else if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron left
        // currentSetting = 3;  // Bind
        if (!wifiStarted) {  // cant start bind while wifi is active
            if (bindStarted) {
                crsfClass.crsfSendCommand(ELRS_BIND_COMMAND, ELRS_END_COMMAND);
                bindStarted = false;
            }
            else {
                crsfClass.crsfSendCommand(ELRS_BIND_COMMAND, ELRS_START_COMMAND);
                bindStarted = true;
            }
        }
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron right
        // currentSetting = 4;  // TX Wifi
        crsfClass.crsfSendCommand(ELRS_WIFI_COMMAND, ELRS_START_COMMAND);
        if (!bindStarted) {  // cant start bind while wifi is active
            if (wifiStarted) {
                crsfClass.crsfSendCommand(ELRS_WIFI_COMMAND, ELRS_END_COMMAND);
                wifiStarted = false;
            }
            else {
                crsfClass.crsfSendCommand(ELRS_WIFI_COMMAND, ELRS_START_COMMAND);
                wifiStarted = true;
            }
        }
    // } else {
    //     currentSetting = 0;
    }

    // override stick data with SBUS data when throttle is held high during power on
    if (rcChannels[THROTTLE] > RC_MAX_COMMAND) {
        headTrackerEnabled=true;
    }

}

bool checkStickMove(){
    // check if stick moved, warring after 10 minutes
    if(abs(previous_throttle - rcChannels[THROTTLE]) < 30){
        stickMoved = 0;
        //Serial.println(abs(previous_throttle - rcChannels[THROTTLE]));
    }else{
        previous_throttle = rcChannels[THROTTLE];
        stickMovedMillis = millis();
        stickMoved = 1;
    }

    if (millis() - stickMovedMillis > STICK_ALARM_TIME){
       // Serial.println((millis() - stickMovedMillis));
        return true;
    }else{
        return false;
    }
}

void checkLongPress(){
    if (digitalRead(DIGITAL_PIN_BUTTON) == LOW) {
        if (buttonStartTime == 0)
            buttonStartTime = millis(); // Record start time
    } else {
        if (buttonStartTime> 0) {  // pressStartTime is non-zero
            unsigned long pressDuration = millis() - buttonStartTime;
            buttonStartTime = 0;
            if (pressDuration >= longPressDuration) {
                // Set flag to start calibration
                calibrationRequested = true;
            } 
        }
    }
}

// Flash Led and play buzzer depending on current state
void statusDisplay(){
    if (batteryVoltage < WARNING_VOLTAGE && batteryVoltage >= BEEPING_VOLTAGE) {
        blinkLED(DIGITAL_PIN_LED, 500);
    }else if(batteryVoltage < BEEPING_VOLTAGE && batteryVoltage >= ON_USB){
        blinkLED(DIGITAL_PIN_LED, 250);
        playingTones(2);
    }else if(currentSetting == 3){
        blinkLED(DIGITAL_PIN_LED, 100);  // Bind (fast flash)
    }else if(currentSetting == 4){
        blinkLED(DIGITAL_PIN_LED, 1000); // Wifi (slow flash)
    }

    // check sticks
    if (checkStickMove() == true){
        blinkLED(DIGITAL_PIN_LED, 100);
        playingTones(5);
    }
#ifdef PASSIVE_BUZZER
    else { // Stop the stick warning tone if you move the sticks
      if ( rtttl::isPlaying() ) {
        rtttl::stop();
      }
    }
#endif
}

void logData(){
    if (millis() - lastDisplayTime > 250) {
        lastDisplayTime = millis();

        char buf[6]; 
        // Serial.print("Raw AETR:");
        // sprintf(buf, "%5d", rawValues.aileron);
        // Serial.print(buf);
        // sprintf(buf, "%5d", rawValues.elevator);
        // Serial.print(buf);
        // sprintf(buf, "%5d", rawValues.throttle);
        // Serial.print(buf);
        // sprintf(buf, "%5d", rawValues.rudder);
        // Serial.print(buf);

        Serial.print("Bat:");
        sprintf(buf, "%5.2f", batteryVoltage);
        Serial.print(buf);Serial.print(batteryVoltage);
        Serial.print("v  Channels AETR: ");
        sprintf(buf, "%5d", rcChannels[AILERON]);
        Serial.print(buf);
        sprintf(buf, "%5d", rcChannels[ELEVATOR]);
        Serial.print(buf);
        sprintf(buf, "%5d", rcChannels[THROTTLE]);
        Serial.print(buf);
        sprintf(buf, "%5d", rcChannels[RUDDER]);
        Serial.print(buf);
        Serial.print(" AUX: ");
        Serial.print(AUX1_Arm);
        Serial.print(AUX2_value);
        Serial.print(AUX3_value);
        Serial.print(AUX4_value);
        Serial.print("  "); 
        if (crsfClass.moduleConnected){
            // Generate the 4-line bar metric matching EdgeTX's logic
            String barIndicator = "[    ]";
            Serial.print("ELRS ");
            if (crsfClass.telemetryActive) {
                Serial.print("RSSI1: "); Serial.print(crsfClass.linkStats.uplinkRssi1); Serial.print("dBm");
                Serial.print(" | TX Pwr: "); Serial.print(crsfClass.linkStats.txPower); 
                Serial.print(" | LQ: "); Serial.print(crsfClass.linkStats.rfMode); Serial.print(" : "); Serial.print(crsfClass.linkStats.uplinkLQ); Serial.print(" ");

                if (crsfClass.linkStats.uplinkLQ >= 80)      barIndicator = "[====]";
                else if (crsfClass.linkStats.uplinkLQ >= 50) barIndicator = "[=== ]";
                else if (crsfClass.linkStats.uplinkLQ >= 20) barIndicator = "[==  ]";
                else if (crsfClass.linkStats.uplinkLQ > 0)   barIndicator = "[=   ]";
            }
            Serial.print(barIndicator); Serial.print("  "); 
        }
        if (totalPackets > 0){  // Only output SBUS data if we have received at least one SBUS packet
            Serial.print("SBUS Channel data: ");
            Serial.print(sbusChannels[0]);
            Serial.print(",");
            Serial.print(sbusChannels[1]);
            Serial.print(",");
            Serial.print(sbusChannels[2]);
            Serial.print(",");
            Serial.print(sbusChannels[3]);
            Serial.print("  Packets (total/good): ");
            Serial.print(totalPackets);
            Serial.print("/");
            Serial.print(goodPackets);
        }
        Serial.println();

    /*
    if (digitalRead(DIGITAL_PIN_BUTTON) == LOW) {
        Serial.print("Button LOW ");
    } else {
        Serial.print("Button HIGH ");   
    }    
    Serial.print("buttonStartTime: ");
    Serial.print(buttonStartTime);
    Serial.print(" millis(): ");
    Serial.println(millis());
    */

    }
}

// -----------------------------------------------------------------------------------------------------
// Board setup
// -----------------------------------------------------------------------------------------------------
void setup()
{
    // inialize rc data
    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        rcChannels[i] = CRSF_DIGITAL_CHANNEL_MIN;
    }

    // Configure STM32 analog inputs to 12-bit resolution (0 - 4095)
    analogReadResolution(12);    //analogReference(EXTERNAL);
    pinMode(analogInPinAileron, INPUT_ANALOG);
    pinMode(analogInPinElevator, INPUT_ANALOG);
    pinMode(analogInPinThrottle, INPUT_ANALOG);
    pinMode(analogInPinRudder, INPUT_ANALOG);
    pinMode(VOLTAGE_READ_PIN, INPUT_ANALOG);

    pinMode(DIGITAL_PIN_SWITCH_ARM, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX2, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX3, INPUT_PULLUP);
    if (DIGITAL_PIN_SWITCH_AUX4 != 0){
      pinMode(DIGITAL_PIN_SWITCH_AUX4, INPUT_PULLUP);
    }
    pinMode(DIGITAL_PIN_LED, OUTPUT);    // LED
    pinMode(DIGITAL_PIN_BUZZER, OUTPUT); // BUZZER
    pinMode(DIGITAL_PIN_BUTTON, INPUT_PULLUP);
#ifdef PASSIVE_BUZZER
    digitalWrite(DIGITAL_PIN_BUZZER, LOW); // BUZZER OFF
    if(STARTUP_MELODY!=""){
      rtttl::begin(DIGITAL_PIN_BUZZER, STARTUP_MELODY);
    }
#endif
    // inialize voltage:
    batteryVoltage = 0.0;
#ifdef PPMOUTPUT
    pinMode(ppmPin, OUTPUT);
    digitalWrite(ppmPin, !onState); // set the PPM signal pin to the default state (off)
    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;
    OCR1A = 100;             // compare match register, change this
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);   // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();
#endif

#ifdef PASSIVE_BUZZER
    if(STARTUP_MELODY!="") {
      while( !rtttl::done() ) {
        rtttl::play();
      }
    }
    else {
     delay(2000); // Give enough time for uploading firmware (2 seconds)     
    }
#else  
    // passive buzzer, no startup tone
    delay(2000); // Give enough time for uploading firmware (2 seconds)
    // digitalWrite(DIGITAL_PIN_BUZZER, HIGH); //BUZZER OFF
#endif

    // Serial output over USB (Uart6)
    Serial.begin(115200);

// CRSF over Uart1
#ifdef ELRS_HALF_DUPLEX
    crsfClass.begin(ELRS_RX, ELRS_TX, true);
#else
    crsfClass.begin(ELRS_RX, ELRS_TX, false);
#endif
    
    // Start SBUS port
    //Serial2.setRx(RX2);
    //Serial2.setTx(TX2);
    Serial2.begin(100000); // SBUS Input

    digitalWrite(DIGITAL_PIN_LED, LOW); // LED ON
    
    calibrationLoad();
}

#ifdef TIMINGDEBUG
uint32_t loopCounter = 0;
uint32_t maxLoopTime = 0;
uint32_t minLoopTime = 0;
// uint32_t totLoopTime = 0;
uint32_t maxElrsTime = 0;
uint32_t minElrsTime = 0;
// uint32_t totElrsTime = 0;
// uint32_t elrsCounter = 0;
uint32_t lastStatsPrintTime = 0;
uint32_t totalTransmitFrames = 0;
#endif
uint32_t lastSyncPrintTime = 0;

// -----------------------------------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------------------------------
void loop()
{
#ifdef TIMINGDEBUG
    static uint32_t nextLogTimeUs = 0;
    uint32_t nowUs = micros();

    if (nextLogTimeUs = 0) nextLogTimeUs = nowUs + 4000;
    loopCounter++;
    if (nowUs >= nextLogTimeUs) {
    uint32_t startLoopUs = micros();
#endif
    // melody player needs to be first in the loop in order to play correctly.
    rtttl::play();

    // Check for long press of the boot button (and set calibration flag)
    checkLongPress();

    // Calibration (if requested)
    //calibrationRun(AUX1_Arm, AUX2_value);
    if (calibrationRequested)
        calibrationRun();
    else {
        // Read Voltage
        batteryVoltage = analogRead(VOLTAGE_READ_PIN) / VOLTAGE_SCALE; // 98.5

        // Flash led and use beeper
        statusDisplay();

        // Get the gimbal stick values
        getStickVals();

        // read data from SBUS (headtracking or external data source)
        getSBUS();

        // map sticks/SBUS data to rcChannels
        mapChannels();

        // Read AUX channels
        getAUXInputs();

        //if (loopCount == 0) {
        // Check if sticks are held in specific position on startup (bind/wifi/packet rate select)
        //    selectSetting();
        //}
        // Check for quick settings change via arm toggle gesture
        checkGestureSetting(); 

#ifdef DEBUG        
        // Debug data
        logData();
#endif
 
        // ---------------------------------------------------------------------------
        // Read Telemetry data every loop (telemetry processed in crsfCheckTelemetry)
        crsfClass.crsfReadTelemetry();

        // -------------------------------------------------
        // Poll the telemetry data and run lua script logic
        elrsLua.update();       

        // --------------------------------
        // Send RC data to external module
        uint32_t currentMicros = micros();
        if (currentMicros > crsfTime) {
            // Command packet interleaving
            if (crsfClass.commandQueue.hasItems()) {
                crsfClass.crsfSendQueuedCommand();
            } else {
                crsfClass.crsfSendChannels(rcChannels);
            }

#ifdef TIMINGDEBUG
            uint32_t elrsTime = micros() - currentMicros;
            if (elrsTime > maxElrsTime) maxElrsTime = elrsTime;
            if ((elrsTime < minElrsTime) || minLoopTime == 0) minElrsTime = elrsTime;
#endif
            uint32_t adjustment = crsfClass.crsfNextInterval();
            crsfTime = currentMicros + adjustment; //CRSF_TIME_BETWEEN_FRAMES_US;
#ifdef DEBUG
            if (millis() - lastSyncPrintTime > 2000) {
                Serial.println("\n============ [Loop timing] ============");
                Serial.print("Next Interval : "); Serial.print(adjustment); Serial.println(" microseconds");
                Serial.println("=======================================");
                lastSyncPrintTime = millis();
            }
#endif
        }
    } // Not calibrating
#ifdef TIMINGDEBUG
    uint32_t elapsedUs = micros() - startLoopUs;
    if (elapsedUs > maxLoopTime) maxLoopTime = elapsedUs;
    if ((elapsedUs < minLoopTime) || minLoopTime == 0) minLoopTime = elapsedUs;
    totalTransmitFrames++;
    // Push the target grid perfectly forward by 4000us (prevents error drift)
    nextLogTimeUs += 4000; 
    }
    // ---------------------------------------------------------------------------
    if (millis() - lastStatsPrintTime > 5000) {
        lastStatsPrintTime = millis();
        
        Serial.println("\n============ [HANDSET PERFORMANCE METRICS] ============");
        Serial.print("Max Loop Duration : "); Serial.print(maxLoopTime); Serial.println(" microseconds");
        Serial.print("Min Loop Duration : "); Serial.print(minLoopTime); Serial.println(" microseconds");
        Serial.print("Avg Loop Duration : "); Serial.print(5000000/totalTransmitFrames); Serial.println(" microseconds");
        Serial.print("Total Transmit Frames   : "); Serial.println(totalTransmitFrames);
        Serial.print("Max ELRS Duration : "); Serial.print(maxElrsTime); Serial.println(" microseconds");
        Serial.print("Min ELRS Duration : "); Serial.print(minElrsTime); Serial.println(" microseconds");
        Serial.print("Avg ELRS Duration : "); Serial.print(5000000/loopCount); Serial.println(" microseconds");
        Serial.print("Total ELRS Frames   : "); Serial.println(loopCount);
        Serial.println("=======================================================");
        
        // Reset max peak-hold values to check for performance dips in the next window
        maxLoopTime = 0;
        minLoopTime = 0;
        maxElrsTime = 0;
        minElrsTime = 0;
        totalTransmitFrames = 0;
        loopCount = 0;
    }    
#endif
}

#ifdef PPMOUTPUT
ISR(TIMER1_COMPA_vect) { // leave this alone
    static boolean state = true;

    TCNT1 = 0;

    if (state) { // start pulse
        digitalWrite(ppmPin, onState);
        OCR1A = PULSE_LENGTH * 2;
        state = false;
    } else { // end pulse and calculate when to start the next pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        digitalWrite(ppmPin, !onState);
        state = true;

        if (cur_chan_numb >= CHANNEL_NUMBER) {
            cur_chan_numb = 0;
            calc_rest = calc_rest + PULSE_LENGTH; //
            OCR1A = (FRAME_LENGTH - calc_rest) * 2;
            calc_rest = 0;
        } else {
            OCR1A = (map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000) - PULSE_LENGTH) * 2;
            calc_rest = calc_rest + map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000);
            cur_chan_numb++;
        }
    }
}
#endif
