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
#include "crsf.h"

volatile uint8_t  CRSF::telemRingBuffer[TELEM_RING_BUF_SIZE] = {0};
volatile uint16_t CRSF::ringBufferHead = 0;
volatile uint16_t CRSF::ringBufferTail = 0;

// crc implementation from CRSF protocol document rev7
static uint8_t crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++){
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

bool checkCrc(uint8_t *packet, uint8_t length) {
    return (crsf_crc8(&packet[2], packet[1] - 1) == packet[length - 1]);
}

// Serial begin
void CRSF::begin(uint32_t rxPin, uint32_t txPin, bool halfDuplex) {
  ELRS_PORT.setTx(txPin);
  if (halfDuplex) {
    ELRS_PORT.setRx(txPin); 
    ELRS_PORT.setHalfDuplex();
  } else {
    ELRS_PORT.setRx(rxPin); 
  }
  ELRS_PORT.begin(SERIAL_BAUDRATE, SERIAL_8N1); 
  //port.begin(SERIAL_BAUDRATE);
/*
    #ifdef ELRS_HALF_DUPLEX
    // Explicitly Configure the underlying hardware registers for Single-Wire mode
        USART_TypeDef *usartInstance = (USART_TypeDef *)ELRS_PORT.getHandle()->Instance;
    // Enable Single-Wire Selection (HDSEL)
    // This physically binds the internal RX register paths directly onto PB6.
        usartInstance->CR3 |= USART_CR3_HDSEL;
    // Explicitly ensure inversion bits remain disabled (CRSF uses standard logic)
        usartInstance->CR2 &= ~(USART_CR2_TXINV | USART_CR2_RXINV);
    #endif
*/
  // USART_TypeDef *usartInstance = USART1; 
  // Enable Single-Wire Half-Duplex mode
  //usartInstance->CR3 |= USART_CR3_HDSEL; 
}

// Public static method to allow the global IRQ handler to feed the ring buffer safely
void CRSF::addByteToRingBuffer(uint8_t incomingByte) {
  uint16_t nextHead = (ringBufferHead + 1) & (TELEM_RING_BUF_SIZE - 1);
  if (nextHead != ringBufferTail) {
      telemRingBuffer[ringBufferHead] = incomingByte;
      ringBufferHead = nextHead;
  }
}

// prepare data packet
//void CRSF::crsfPrepareDataPacket(uint8_t packet[], int16_t channels[]) {
// Send RC Channels
void CRSF::crsfSendChannels(int16_t channels[]) {
    uint8_t packet[CRSF_PACKET_SIZE];
    // const uint8_t crc = crsf_crc8(&packet[2], CRSF_PACKET_SIZE-3);
    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
    */

    // packet[0] = UART_SYNC; //Header
    packet[0] = MODULE_ADDRESS; // Header
    packet[1] = 24;           // length of type (24) + payload + crc
    packet[2] = ELRS_CHANNELS;
    packet[3] = (uint8_t)(channels[0] & 0x07FF);
    packet[4] = (uint8_t)((channels[0] & 0x07FF) >> 8 | (channels[1] & 0x07FF) << 3);
    packet[5] = (uint8_t)((channels[1] & 0x07FF) >> 5 | (channels[2] & 0x07FF) << 6);
    packet[6] = (uint8_t)((channels[2] & 0x07FF) >> 2);
    packet[7] = (uint8_t)((channels[2] & 0x07FF) >> 10 | (channels[3] & 0x07FF) << 1);
    packet[8] = (uint8_t)((channels[3] & 0x07FF) >> 7 | (channels[4] & 0x07FF) << 4);
    packet[9] = (uint8_t)((channels[4] & 0x07FF) >> 4 | (channels[5] & 0x07FF) << 7);
    packet[10] = (uint8_t)((channels[5] & 0x07FF) >> 1);
    packet[11] = (uint8_t)((channels[5] & 0x07FF) >> 9 | (channels[6] & 0x07FF) << 2);
    packet[12] = (uint8_t)((channels[6] & 0x07FF) >> 6 | (channels[7] & 0x07FF) << 5);
    packet[13] = (uint8_t)((channels[7] & 0x07FF) >> 3);
    packet[14] = (uint8_t)((channels[8] & 0x07FF));
    packet[15] = (uint8_t)((channels[8] & 0x07FF) >> 8 | (channels[9] & 0x07FF) << 3);
    packet[16] = (uint8_t)((channels[9] & 0x07FF) >> 5 | (channels[10] & 0x07FF) << 6);
    packet[17] = (uint8_t)((channels[10] & 0x07FF) >> 2);
    packet[18] = (uint8_t)((channels[10] & 0x07FF) >> 10 | (channels[11] & 0x07FF) << 1);
    packet[19] = (uint8_t)((channels[11] & 0x07FF) >> 7 | (channels[12] & 0x07FF) << 4);
    packet[20] = (uint8_t)((channels[12] & 0x07FF) >> 4 | (channels[13] & 0x07FF) << 7);
    packet[21] = (uint8_t)((channels[13] & 0x07FF) >> 1);
    packet[22] = (uint8_t)((channels[13] & 0x07FF) >> 9 | (channels[14] & 0x07FF) << 2);
    packet[23] = (uint8_t)((channels[14] & 0x07FF) >> 6 | (channels[15] & 0x07FF) << 5);
    packet[24] = (uint8_t)((channels[15] & 0x07FF) >> 3);
    packet[25] = crsf_crc8(&packet[2], packet[1] - 1); // CRC
    
    crsfWritePacket(packet,CRSF_PACKET_SIZE);
}

// prepare elrs setup packet (power, packet rate...)
//void CRSF::crsfPrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value) {
void CRSF::crsfSendCommand(uint8_t command, uint8_t value) {
    uint8_t packetCmd[CRSF_CMD_PACKET_SIZE];

    packetCmd[0] = MODULE_ADDRESS;
    packetCmd[1] = 6; // length of Command (4) + payload + crc
    packetCmd[2] = ELRS_SETTINGS_WRITE;
    packetCmd[3] = MODULE_ADDRESS; // Destination Address (0x00 = Parameter Broadcast Request)
    packetCmd[4] = HANDSET_ADDRESS; // Source Address (0xEA = Radio Handset Transmitter)
    packetCmd[5] = command;
    packetCmd[6] = value;
    packetCmd[7] = crsf_crc8(&packetCmd[2], packetCmd[1] - 1); // CRC

    //crsfWritePacket(packetCmd,CRSF_CMD_PACKET_SIZE);
    crsfQueuePacket(packetCmd,CRSF_CMD_PACKET_SIZE);
}

void CRSF::crsfPingDevices() {
    uint8_t packetCmd[6];

    packetCmd[0] = MODULE_ADDRESS;
    packetCmd[1] = 4; // length of Command (2) + payload + crc
    packetCmd[2] = ELRS_DEVICE_PING; 
    packetCmd[3] = BROADCAST_ADDRESS; // Destination Address (0x00 = Parameter Broadcast Request)
    packetCmd[4] = HANDSET_ADDRESS; // Source Address (0xEA = Radio Handset Transmitter)
    packetCmd[5] = crsf_crc8(&packetCmd[2], packetCmd[1] - 1);  //crc
    //crsfWritePacket(packetCmd, 6);
    crsfQueuePacket(packetCmd, 6);
}

void CRSF::crsfRequestSetting(uint8_t settingIndex, uint8_t chunk) {
    uint8_t packetCmd[8];

    packetCmd[0] = MODULE_ADDRESS;        // 0xEE (Target Device: External TX Module)
    packetCmd[1] = 6;                     // LENGTH: 7 Bytes remaining (Type + 5 Payload Bytes + 1 CRC)
    packetCmd[2] = ELRS_SETTINGS_READ;    // 0x2C (CRSF_FRAMETYPE_PARAMETER_READ)
    packetCmd[3] = MODULE_ADDRESS;        // Destination Address: 0xEE (The Module itself)
    packetCmd[4] = LUA_SCRIPT_ADDRESS;    // HANDSET_ADDRESS;          // Source/Origin Address: 0xEA (Your Transmitter Handset) or 0xEF ELRS_LUA
    packetCmd[5] = settingIndex;          // Parameter Index position to read (1, 2, 3...)
    packetCmd[6] = chunk;                 // Value Chunk Offset parameter
    packetCmd[7] = crsf_crc8(&packetCmd[2], packetCmd[1] - 1);  // crc

    //crsfWritePacket(packetCmd, 8);
    crsfQueuePacket(packetCmd, 8);

    Serial.print("[CRSF] -> Querying Menu Parameter Index: ");
    Serial.print(settingIndex);
    Serial.print(", chunk: ");
    Serial.print(chunk); Serial.println();
    // Serial.print("0x"); Serial.print(packetCmd[0],HEX);
    // Serial.print(" "); Serial.print(packetCmd[1],HEX);
    // Serial.print(" "); Serial.print(packetCmd[2],HEX);
    // Serial.print(" "); Serial.print(packetCmd[3],HEX);
    // Serial.print(" "); Serial.print(packetCmd[4],HEX);
    // Serial.print(" "); Serial.print(packetCmd[5],HEX);
    // Serial.print(" "); Serial.print(packetCmd[6],HEX);
    // Serial.print(" "); Serial.println(packetCmd[7],HEX);
}

// Request Info Message by sending ELRS_SETTINGS_WRITE (0x2D) with param/value 0
void CRSF::crsfRequestElrsStatus() {
    crsfSendCommand(0,0);
}

void CRSF::crsfSendPendingCommand() {
    if (commandQueue.hasItems()) {
        crsfPacket pendingPacket;
        if (commandQueue.pop(pendingPacket)) {
            crsfWritePacket(pendingPacket.data, pendingPacket.length);
            // Serial.write("Pushing pending 0x"); Serial.println(pendingPacket.data[2],HEX);
            // ELRS_PORT.write(pendingPacket.data, pendingPacket.length);
            // if (ELRS_PORT.isHalfDuplex()) {
            //   ELRS_PORT.enableHalfDuplexRx();
            // }
            // Serial.print("Sent: ");
            // for (int i = 0; i < pendingPacket.length; i++) {
            //   Serial.print(pendingPacket.data[i],HEX); Serial.print(" ");
            // } Serial.println();
        }
    }
}

void CRSF::crsfQueuePacket(uint8_t packet[], uint8_t packetLength) {
    // Push it onto the ring buffer queue
    if (!commandQueue.push(packet, packetLength)) {
        // Handle full buffer error if necessary
    }
}

void CRSF::crsfWritePacket(uint8_t packet[], uint8_t packetLength) {
    ELRS_PORT.write(packet, packetLength);
    if (ELRS_PORT.isHalfDuplex()) {
      // CRITICAL for Half-Duplex: If you don't flush, your incoming listening code will get corrupted by your own echo.
      ELRS_PORT.enableHalfDuplexRx();
    }
}

void CRSF::crsfProcessPacket() {
  if (telemetryQueue.hasItems()) {
    crsfPacket telemetryPacket;
    if (telemetryQueue.pop(telemetryPacket)) {
      uint8_t packetType = telemetryPacket.data[2];

      // Refresh watchdog timer upon any valid frame from the module
      moduleConnected = true;
      lastValidFrameTime = millis(); // Refresh our active timestamp clock
      Serial.print("Received Packet type 0x"), Serial.println(packetType,HEX);

      switch (packetType) {
        case ELRS_LINK_STATS:
          crsfParseLinkStatsPacket(telemetryPacket.data);
          break;
        case ELRS_DEVICE_INFO:
          moduleInfoReceived = true;
          crsfParseDeviceInfoPacket(telemetryPacket.data, telemetryPacket.length);     
          break;           
        case ELRS_SETTINGS_RESPONSE:
          crsfParseSettingsPacket(telemetryPacket.data, telemetryPacket.length);
          break;
        case ELRS_STATUS:
          moduleStatusReceived = true;
          crsfParseElrsStatusPacket(telemetryPacket.data, telemetryPacket.length);
          break;
      }
    }
  }
}

void CRSF::crsfParseLinkStatsPacket(uint8_t rxBuffer[]) {
  if (!telemetryActive) {
    telemetryActive = true;
    //Serial.println("\n=================================");
    //Serial.println("[INFO] -> TELEMETRY RECOVERED <-");
    //Serial.println("=================================");
  }
  lastLinkStatsFrameTime = millis(); // Refresh our active timestamp clock

  linkStats.uplinkRssi1   = (int8_t)rxBuffer[3]; 
  linkStats.uplinkRssi2   = (int8_t)rxBuffer[4]; 
  linkStats.uplinkLQ      = rxBuffer[5];
  linkStats.uplinkSNR     = (int8_t)rxBuffer[6]; 
  linkStats.activeAntenna = rxBuffer[7];
  linkStats.rfMode        = rxBuffer[8];
  linkStats.txPower       = rxBuffer[9];
  linkStats.downlinkRssi  = (int8_t)rxBuffer[10];
  linkStats.downlinkLQ    = rxBuffer[11];
  linkStats.downlinkSNR   = (int8_t)rxBuffer[12];
}

void CRSF::crsfParseDeviceInfoPacket(uint8_t rxBuffer[], uint8_t length) {
  // Clear any existing stored text profile settings
  memset(txModule.name, 0, sizeof(txModule.name)); 

  // Step A: Parse out the Device Name characters starting at Index 5
  uint8_t currentIdx = 5;
  uint8_t charCount = 0;
  
  // Extract up to the null-terminator divider boundary
  while (rxBuffer[currentIdx] != 0x00 && currentIdx < (length + 2) && charCount < (CRSF_MAX_STRING_LEN - 1)) {
    txModule.name[charCount++] = (char)rxBuffer[currentIdx++];
  }
  txModule.name[charCount] = '\0'; // Guarantee safe char string sealing

  // Step B: Advance past the explicit 0x00 null terminator byte
  currentIdx++; 

  // Step C: Ensure we have enough remaining bytes left inside the frame payload to read metadata bounds
  if (currentIdx + 14 <= (length + 2)) {
    // Extract 32-bit values (Sent in Big-Endian format over the CRSF bus)
    txModule.serialNumber[0] = rxBuffer[currentIdx++];
    txModule.serialNumber[1] = rxBuffer[currentIdx++];
    txModule.serialNumber[2] = rxBuffer[currentIdx++];
    txModule.serialNumber[3] = rxBuffer[currentIdx++];

    txModule.hwVersion[0] = rxBuffer[currentIdx++];
    txModule.hwVersion[1] = rxBuffer[currentIdx++];
    txModule.hwVersion[2] = rxBuffer[currentIdx++];
    txModule.hwVersion[3] = rxBuffer[currentIdx++];
    
    txModule.swVersion[0] = rxBuffer[currentIdx++];
    txModule.swVersion[1] = rxBuffer[currentIdx++];
    txModule.swVersion[2] = rxBuffer[currentIdx++];
    txModule.swVersion[3] = rxBuffer[currentIdx++];

    // Extract the final configuration structural control tags
    totalSettingsCount = rxBuffer[currentIdx++]; 
    txModule.protocolVersion   = rxBuffer[currentIdx++];

    // reset the currently loaded count
    txModule.paramCount = 0;
    // Debug output
    Serial.println("============ [ELRS DEVICE DETECTED] ============");
    Serial.print("Module Name         : "); Serial.println(txModule.name);
    Serial.print("Serial number       : "); Serial.print((char)txModule.serialNumber[0]); Serial.print((char)txModule.serialNumber[1]); Serial.print((char)txModule.serialNumber[2]); Serial.println((char)txModule.serialNumber[3]); 
    Serial.print("Hardware Version    : "); Serial.print(txModule.hwVersion[0], HEX); Serial.print(txModule.hwVersion[1], HEX); Serial.print(txModule.hwVersion[2], HEX); Serial.println(txModule.hwVersion[3], HEX); 
    Serial.print("Firmware Version    : "); Serial.print(txModule.swVersion[0], HEX); Serial.print(txModule.swVersion[1], HEX); Serial.print(txModule.swVersion[2], HEX); Serial.println(txModule.swVersion[3], HEX); 
    Serial.print("Available Settings  : "); Serial.print(totalSettingsCount); Serial.println(" Menu Items");
    Serial.print("Protocol Version    : "); Serial.println(txModule.protocolVersion);
    Serial.println("================================================\n");
  }
}

void CRSF::clearModule() {
    memset(txModule.name, 0, sizeof(txModule.name));   
    for (int i = 0; i < txModule.paramCount; i++) {
        txModule.params[i].id = 0;
        txModule.params[i].parentFolder = 0;
        txModule.params[i].type = static_cast<crsfValueType>(0);
        txModule.params[i].currentVal = 0;
        memset(txModule.params[i].label, 0, sizeof(txModule.params[i].label)); 
        for (int j = 0; j < txModule.params[i].choicesCount; j++) {
            memset(txModule.params[i].choices[j].text, 0, sizeof(txModule.params[i].choices[j].text)); 
        }
        txModule.params[i].choicesCount = 0;
    }
    txModule.paramCount = 0;
    txModule.paramsLoaded = false;
    moduleInfoReceived = false;
    moduleStatusReceived = false;
    // serialNumber, hwVersion, swVersion, protocolVersion will be reset when a module is next detected
}

void CRSF::parseChoicesString(int paramIndex) {
    uint8_t currentIdx = 0;
    uint8_t activeOptionSlot = 0;
    uint8_t charCount = 0;
    txModule.params[paramIndex].choicesCount = 0;

    // Keep parsing until we hit the end of the packet payload max length
    while (currentIdx < CRSF_MAX_PARAM_DATA_LEN) {
        char c = txModule.params[paramIndex].valueString[currentIdx++];
        
        if (c == 0x00) {
            // Seal the final string choice segment and return.
            txModule.params[paramIndex].choices[activeOptionSlot].text[charCount] = '\0';
            txModule.params[paramIndex].choicesCount = activeOptionSlot + 1;
            return; 
        }
        else if (c == ';') {
            // Semicolon delimiter encountered: seal the current choice string segment
            txModule.params[paramIndex].choices[activeOptionSlot].text[charCount] = '\0';
            
            // Advance to the next safe storage choice slot parameter bounds
            if (activeOptionSlot < (CRSF_MAX_PARAMS - 1)) {
                activeOptionSlot++;
            }
            charCount = 0;
        } 
        else {
            if (charCount < (CRSF_MAX_STRING_LEN - 1)) {
                txModule.params[paramIndex].choices[activeOptionSlot].text[charCount++] = c;
            }
        }
    }
   
}

int CRSF::getParamSlot(uint8_t id) {
    // 1. Search if this parameter ID is already initialized
    for (int i = 0; i < txModule.paramCount; i++) {
        if (txModule.params[i].id == id) return i;
    }
    // 2. If it's a new ID, allocate a fresh index slot if we have space left
    if (txModule.paramCount < CRSF_MAX_PARAMS) {
        int freshSlot = txModule.paramCount;
        txModule.params[freshSlot].id = id;
        // Initialize all parameter data
        txModule.params[freshSlot].parentFolder = 0;
        txModule.params[freshSlot].type = static_cast<crsfValueType>(0);
        txModule.params[freshSlot].currentVal = 0;
        txModule.params[freshSlot].minVal = 0;
        txModule.params[freshSlot].maxVal = 0;
        txModule.params[freshSlot].precision = 0;
        txModule.params[freshSlot].step = 0;
        txModule.params[freshSlot].timeout = 0;
        memset(txModule.params[freshSlot].label, 0, sizeof(txModule.params[freshSlot].label));
        memset(txModule.params[freshSlot].valueString, 0, sizeof(txModule.params[freshSlot].valueString));
        txModule.params[freshSlot].valueStringCharCount = 0;
        memset(txModule.params[freshSlot].units, 0, sizeof(txModule.params[freshSlot].units));
        memset(txModule.params[freshSlot].choices, 0, sizeof(txModule.params[freshSlot].choices));
        txModule.params[freshSlot].choicesCount = 0;
        txModule.paramCount++;
        return freshSlot;
    }
    return -1; // Out of safe tracking memory space
}

void CRSF::crsfParseSettingsPacket(uint8_t rxBuffer[], uint8_t length) {
  uint8_t currentIdx = 5;
  uint8_t fieldIndex = rxBuffer[currentIdx++]; //5

  int slot = getParamSlot(fieldIndex);
  if (slot != -1) {
    chunksRemaining = rxBuffer[currentIdx++]; //6

    if (currentChunk == 0) {
      // Parent folder and type only included on initial chunk packet
      txModule.params[slot].parentFolder = rxBuffer[currentIdx++]; //7
      txModule.params[slot].type = static_cast<crsfValueType>(rxBuffer[currentIdx++] & 0x7F);  //8

      // Parse out the Parameter Label String starting at Index 9
      uint8_t labelCharCount = 0;
      currentIdx = 9;  // (moot, since it will be set to that from the last currentIdx++ command - but just to be sure)
      while (rxBuffer[currentIdx] != 0x00 && currentIdx < (length + 2) && labelCharCount < (CRSF_MAX_STRING_LEN - 1)) {
        txModule.params[slot].label[labelCharCount++] = (char)rxBuffer[currentIdx++];
      }
      txModule.params[slot].label[labelCharCount] = '\0'; 
      currentIdx++; 
    } 

    switch(txModule.params[slot].type) {
        case CRSF_TEXT_SELECTION:
            // Dropdown/options selection - the main type used in ELRS
            // New parameters (currentChunk==0) start the choices after the label string.
            // Continuation chunks enter straight into choices strings at Index 7
            while (rxBuffer[currentIdx] != 0x00 && currentIdx < (length+1) && txModule.params[slot].valueStringCharCount < (CRSF_MAX_PARAM_DATA_LEN - 1)) {
              txModule.params[slot].valueString[txModule.params[slot].valueStringCharCount++] = (char)rxBuffer[currentIdx++];
            }
            txModule.params[slot].valueString[txModule.params[slot].valueStringCharCount] = '\0';
            txModule.params[slot].valueStringCharCount--; // Decrement the valueString char counter because the loop above insists on picking up the crc byte in chunked params


            // --- FINAL CHUNK RECEPTION CLOSURE ---
            if (chunksRemaining == 0) {
              // The selected option is after the choices string null terminator
              currentIdx++;
              if (currentIdx < (length + 2)) {
                txModule.params[slot].currentVal = rxBuffer[currentIdx];
              }
              // Next byte is min selection
              currentIdx++;
              if (currentIdx < (length + 2)) {
                txModule.params[slot].minVal = rxBuffer[currentIdx];
              }     
              // Next byte is max selection (also number of choices)
              currentIdx++;
              if (currentIdx < (length + 2)) {
                txModule.params[slot].maxVal = rxBuffer[currentIdx];
              }    
              // Next byte is 0 in all cases i checked (maybe reserved for step?)
              currentIdx++;
              // Parse out the units String
              currentIdx++;
              uint8_t unitCharCount = 0;
              while (rxBuffer[currentIdx] != 0x00 && currentIdx < (length + 2) && unitCharCount < (CRSF_MAX_STRING_LEN - 1)) {
                txModule.params[slot].units[unitCharCount++] = (char)rxBuffer[currentIdx++];
              }
              txModule.params[slot].units[unitCharCount] = '\0';      

              parseChoicesString(slot);

              Serial.print("[PARAMETER SAVED] ");
              Serial.print(txModule.params[slot].label);
              Serial.print(" | Choices("); Serial.print(txModule.params[slot].minVal); Serial.print("-"); Serial.print(txModule.params[slot].maxVal); Serial.print("): ");
              
              if (txModule.params[slot].choicesCount > 0) {
                for (int i = 0; i < txModule.params[slot].choicesCount; i++) {
                    Serial.print(txModule.params[slot].choices[i].text); 
                    if (i < txModule.params[slot].choicesCount - 1) {
                        Serial.print(", ");
                    }
                } 
              }
              Serial.print(" | Active Selection: "); Serial.print(txModule.params[slot].choices[txModule.params[slot].currentVal].text);
              Serial.println(txModule.params[slot].units);
              // Move on to process the next sequential hardware parameter tree setting
              currentChunk = 0;
              currentSettingsIndex++;
            }
            else {
              // Data payload remains chunked, increment block indicators to fetch remaining parts
              currentChunk++;
              Serial.print("Chunk stored");
              //Serial.print("Chunk stored. Current string: "); Serial.println(txModule.params[slot].valueString);
            }
            break;
        case CRSF_FOLDER:
            // Menu folder containing sub-items
            // Theoretically this could be chunked too, but not done in ELRS (Yet)
            // if (chunksRemaining == 0) {
            Serial.print("[PARAMETER SAVED] ");
            Serial.print(txModule.params[slot].label);
            Serial.print(" | Menu (ID: "); Serial.print(txModule.params[slot].id); Serial.println(")");
            // Move on to process the next sequential hardware parameter tree setting
            currentChunk = 0;
            currentSettingsIndex++;
            break;
        case CRSF_INFO:
            // Display non-editable static string
            // Not sure yet - havent reached one of these...
            // Move on to process the next sequential hardware parameter tree setting
            Serial.print("Info packet. Packet data: ");
            for (int i = 0; i < length; i++){
              Serial.print(rxBuffer[i],HEX);Serial.print(" "); 
            } Serial.println();
            currentChunk = 0;
            currentSettingsIndex++;
            break;
        case CRSF_COMMAND:
            // Execute command / status such as initiate bind or BLE joystick
            // Sample Packet: EA 17 2B EA EE F 0 E D 45 6E 61 62 6C 65 20 57 69 46 69 0 0 C8 0 5D 
            // Sync byte, Len, type (0x2b settings info), dest (0xEA handset), source (0xEE module), id (0x0f=15), chunks remaining (0), parent (0x0e=14), type (0x0d=command)
            // label (Enable WiFi), string terminator 0x0, step/state (0), timeout (0xc8=200=2s), null terminated status string
            // Label has been saved and currentIdx is sitting at step/state
            txModule.params[slot].step = rxBuffer[currentIdx++];         
            txModule.params[slot].timeout = rxBuffer[currentIdx++] * 10;
            // info/status stored in the valueString property
            while (rxBuffer[currentIdx] != 0x00 && currentIdx < (length+1) && txModule.params[slot].valueStringCharCount < (CRSF_MAX_PARAM_DATA_LEN - 1)) {
              txModule.params[slot].valueString[txModule.params[slot].valueStringCharCount++] = (char)rxBuffer[currentIdx++];
            }
            txModule.params[slot].valueString[txModule.params[slot].valueStringCharCount] = '\0';
            txModule.params[slot].valueStringCharCount--; // Decrement the valueString char counter because the loop above insists on picking up the crc byte in chunked params
            Serial.print("[PARAMETER SAVED] ");
            Serial.print(txModule.params[slot].label);
            Serial.print(" | Command (Current state "); 
            switch(txModule.params[slot].step) {
              case 0: // IDLE
                Serial.print("IDLE"); 
                break;
              case 1: // CLICK - user has clicked the command to execute
                Serial.print("CLICK"); 
                break;
              case 2: // EXECUTING - command is executing
                Serial.print("EXECUTING"); 
                break;
              case 3: // ASKCONFIRM - command pending user OK
                Serial.print("ASKCONFIRM"); 
                break;
              case 4: // CONFIRMED - user has clicked confirm
                Serial.print("CONFIRMED"); 
                break;
              case 5: // CANCEL - user has requested cancel
                Serial.print("CANCEL"); 
                break;
              case 6: // QUERY - host is requested updated status
                Serial.print("QUERY"); 
                break;
            }
            Serial.print(" | Timeout ");
            Serial.print(txModule.params[slot].timeout); 
            Serial.print(" | Info/Status : "); Serial.print(txModule.params[slot].valueString); Serial.println(")");
            currentChunk = 0;
            currentSettingsIndex++;
            break;
        default:
            // String, Float or Int types defined in spec but not used in ELRS
            // Move on to process the next sequential hardware parameter tree setting
            Serial.print("Unhanded info packet type. Packet data: ");
            for (int i = 0; i < length; i++){
              Serial.print(rxBuffer[i],HEX);Serial.print(" "); 
            } Serial.println();
            currentChunk = 0;
            currentSettingsIndex++;
        break;
    }
    lastParameterQueryTime = 0;  //Move on to the next parameter immediately
  }
}

void CRSF::crsfParseElrsStatusPacket(uint8_t rxBuffer[], uint8_t length) {
    // rxBuffer[3] = destination (handset, 0xEF/0xEA), rxBuffer[4] = source (TX module, 0xEE).
    // Discard frames not originating from the ELRS TX module address.
    if (rxBuffer[4] != MODULE_ADDRESS) {
        return;
    }
    elrsStatus.packetsBad  = rxBuffer[5];
    elrsStatus.packetsGood = ((uint16_t)rxBuffer[6] << 8) | rxBuffer[7]; 
    uint8_t rawFlags = rxBuffer[8]; 
    elrsStatus.isConnected   = (rawFlags & 0x01) != 0; // Bit 0 active
    elrsStatus.modelMismatch = (rawFlags & 0x04) != 0; // Bit 2 active
    elrsStatus.isArmed       = (rawFlags & 0x08) != 0; // Bit 3 active
    memset(elrsStatus.statusMessage, 0, sizeof(elrsStatus.statusMessage));
    uint8_t stringStartIdx = 9;
    uint8_t charCounter = 0;
    while (rxBuffer[stringStartIdx] != 0x00 && stringStartIdx < (length + 2) && charCounter < 31) {
        elrsStatus.statusMessage[charCounter++] = (char)rxBuffer[stringStartIdx++];
    }
    elrsStatus.statusMessage[charCounter] = '\0';

    // Debug output
    Serial.println("============ [ELRS LINK STATUS 0x2E] ============");
    // Serial.print("Raw Data            : ");
    // for (int i = 0; i < length; i++){
    //   Serial.print(rxBuffer[i],HEX);Serial.print(" "); 
    // }
    // Serial.println();
    Serial.print("Pkt Stream Health : Good: "); Serial.print(elrsStatus.packetsGood);
    Serial.print(" | Bad: "); Serial.println(elrsStatus.packetsBad);
    
    Serial.print("Link State Flags  : ");
    if (elrsStatus.isConnected)   Serial.print("[CONNECTED] ");   else Serial.print("[DISCONNECTED] ");
    if (elrsStatus.isArmed)       Serial.print("[ARMED] ");       else Serial.print("[DISARMED] ");
    if (elrsStatus.modelMismatch) Serial.print("[!!! MODEL MISMATCH !!!] ");
    Serial.println();

    if (strlen(elrsStatus.statusMessage) > 0) {
        Serial.print("Module Alert Msg  : "); Serial.println(elrsStatus.statusMessage);
    }
    Serial.println("=================================================");
}

void CRSF::crsfParseElrsSyncPacket(uint8_t rxBuffer[]) {
    // Ensure the packet matches the OpenTX Sync Subtype
    // rxBuffer[0] = Sync byte
    // rxBuffer[1] = length
    // rxBuffer[2] = Packet Type 0x3A
    // rxBuffer[3] = Source 0xEE
    // rxBuffer[4] = Destination 0xEF
    
    if (rxBuffer[1] >= 11 && rxBuffer[5] == CRSF_SUBTYPE_OPENTX_SYNC) {      
        // Extract 32-bit Big-Endian Rate Interval (bytes 1-4 of payload)
        uint32_t rawInterval = ((uint32_t)rxBuffer[6] << 24) | 
                               ((uint32_t)rxBuffer[7] << 16) | 
                               ((uint32_t)rxBuffer[8] << 8)  | 
                                rxBuffer[9];
        // Extract 32-bit Signed Big-Endian Phase Shift (bytes 5-8 of payload)
        int32_t rawShift = ((int32_t)rxBuffer[10] << 24) | 
                           ((int32_t)rxBuffer[11] << 16) | 
                           ((int32_t)rxBuffer[12] << 8)  | 
                            rxBuffer[13];
        // ExpressLRS scales microsecond telemetry values by 10
        targetIntervalUs   = (uint32_t)((float)rawInterval * 0.1f);
        currentPhaseShift  = (int32_t)((float)rawShift * 0.1f);
        syncPacketReceived = true;
        // if (millis()>lastSyncPacketDisplay+1000) {
        //   Serial.print("Received Sync Packet. Interval: ");Serial.print(targetIntervalUs);Serial.print(" Phase Shift: ");Serial.println(currentPhaseShift);
        //   Serial.print("Raw Data            : ");
        //   for (int i = 0; i < 14; i++){
        //     Serial.print(rxBuffer[i],HEX);Serial.print(" "); 
        //   } Serial.println();
        //   lastSyncPacketDisplay = millis();
        // }
    }
}

uint32_t CRSF::crsfNextInterval() {
        // Static baseline interval from received Sync packet, or hardcoded interval
        uint32_t baseInterval = (targetIntervalUs > 0) ? targetIntervalUs : CRSF_TIME_BETWEEN_FRAMES_US;
        
        // Apply Proportional feedback correction if a sync frame came in
        int32_t correction = 0;
        if (syncPacketReceived) {
            correction = currentPhaseShift;
            syncPacketReceived = false; // Reset the flag
        }
        // Return the dynamic duration for the *next* single loop cycle
        return baseInterval + correction;
}

void CRSF::crsfCheckTelemetry() {
    //crsfReadPacket(); 
    crsfProcessPacket();
    uint32_t now = millis();
    if (telemetryActive && (now - lastLinkStatsFrameTime > 1000)) {
      telemetryActive = false;
      //Serial.println("\n=================================");
      //Serial.println("[ALERT] !!! TELEMETRY LOST !!!");
      //Serial.println("=================================");
    }
    // Standard operational watchdog: If no data for 3000ms, drop back to pinging
    if (moduleConnected && (now - lastValidFrameTime > 3000)) {
        telemetryActive = false;
        moduleConnected = false;
        ready = false;
        parameterDiscoveryActive = false;
        clearModule();
        connectionState = ELRS_PINGING;
        Serial.println("[HANDSHAKE] Telemetry Timeout. Module connection Lost.");
    }
  }


void CRSF::crsfInitModule() {
    uint32_t now = millis();

    switch(connectionState) {
        case ELRS_PINGING:
            // Every 1000ms, send a Device Ping (0x28)
            if (now - lastHandshakeTime > 500) {
                lastHandshakeTime = now;
                crsfPingDevices(); // Sends 0x28
                Serial.println("[HANDSHAKE] Sending Device Ping (0x28)...");
            }
            
            // Transition condition: If the parser caught a 0x29 packet and saved a name
            if (strlen(txModule.name) > 0) {
                moduleConnected = true;
                connectionState = ELRS_STATS;
                lastHandshakeTime = now;
                Serial.println("[HANDSHAKE] Module Detected! Getting Status...");
            }
            break;

        case ELRS_STATS:
            // Line 575 from elrs.lua: Send LinkStat request via 0x2D
            if (now - lastHandshakeTime > 400) {
                lastHandshakeTime = now;
                crsfRequestElrsStatus();
                Serial.println("[HANDSHAKE] Sending LinkStat Request (0x2D, 0, 0)...");
            }
            // Transition condition: If a valid Link Stats packet (0x2E) is received
            if (moduleStatusReceived) {
                connectionState = ELRS_CONNECTED;
                currentSettingsIndex = 0;  
                currentChunk = 0;      
                parameterDiscoveryActive = true;
                lastParameterQueryTime = now;
                txModule.paramsLoaded = false;
                Serial.println("[HANDSHAKE] Telemetry Stream Active! Initiating Parameter Discovery...");
                // Proactively request the first setting frame immediately
                crsfRequestSetting(currentSettingsIndex, currentChunk); 
            }
            break;

        case ELRS_CONNECTED:
            if (now - lastLinkStatRequestTime > 2000) {
                lastLinkStatRequestTime = now;
                crsfRequestElrsStatus();
                Serial.println("[HANDSHAKE] Sending LinkStat Request (0x2D, 0, 0)...");
            }
            if (parameterDiscoveryActive) {
                if (now - lastParameterQueryTime > 500) {
                    lastParameterQueryTime = now;
                    // currentSettingsIndex++;  // only increment after successfully receiving last parameter.
                    // Stop scanning once we hit the max listed in the device info
                    if (currentSettingsIndex > totalSettingsCount) {
                        parameterDiscoveryActive = false;
                        txModule.paramsLoaded = true;
                        ready = true;
                        Serial.println("[DISCOVERY] >>> PARAMETER TREE MATRIX FULLY POPULATED AND CACHED <<<");
                    } else {

                        crsfRequestSetting(currentSettingsIndex, currentChunk);
                    }
                }
            }
            break;
    }
}


// ----------------------------------------------------------------------------
// AUTOMATED CORE SERIAL EVENT HANDLER
// ----------------------------------------------------------------------------
// This function is built directly into the Arduino/STM32 core. It runs in the 
// background at the end of loop() if bytes are waiting in the hardware registers.
// void serialEvent1() {
//   while (ELRS_PORT.available() > 0) {
//     crsfClass.addByteToRingBuffer(ELRS_PORT.read());
//   }
// }void CRSF::crsfReadTelemetry() {
    while (ELRS_PORT.available() > 0) {
        if (ELRS_PORT.peek() != HANDSET_ADDRESS && ELRS_PORT.peek() != LUA_SCRIPT_ADDRESS) {
            ELRS_PORT.read(); // Clear junk/out of sync bytes immediately
            continue;
        }

        if (ELRS_PORT.available() < 2) return;    // We need at least 2 bytes available to extract the length
        uint8_t addrByte = ELRS_PORT.read();      // Read header byte
        uint8_t payloadLength = ELRS_PORT.read(); // Read length byte

        if (payloadLength < 2 || payloadLength > (CRSF_MAX_PACKET_SIZE - 2)) {
            continue; // Bad tracking frame size, continue flushing
        }

        // non-blocking hardware timeout until the rest of the block finishes arriving
        uint32_t startWait = micros();
        while (ELRS_PORT.available() < payloadLength) {
            if (micros() - startWait > 1000) { // 1ms timeout cap limit
                return; // Packet assembly timed out or fragmented, drop frame fragments
            }
        }

        uint8_t localPacketBuffer[CRSF_MAX_PACKET_SIZE];
        localPacketBuffer[0] = addrByte;
        localPacketBuffer[1] = payloadLength;
        for (uint8_t i = 0; i < payloadLength; i++) {
            localPacketBuffer[2 + i] = ELRS_PORT.read();
        }
        uint8_t totalPacketBytesCount = payloadLength + 2;

        if (checkCrc(localPacketBuffer, totalPacketBytesCount)) {
          if (localPacketBuffer[2] == ELRS_RADIO_ID) {
              crsfParseElrsSyncPacket(localPacketBuffer);  // Process sync packet immediately/dont add to queue
          } else {
              telemetryQueue.push(localPacketBuffer, totalPacketBytesCount);
          }      
        }
    }
}