// =============================================================================

// Folgender Code ist für das BMS ANT-BLE24BHUB angefertigt

// =============================================================================


#ifndef ANTBMS_H
#define ANTBMS_H

#include <Arduino.h>
#include <HardwareSerial.h>

// =============================================================================
// AntBMS Protocol Constants
// =============================================================================
namespace AntBMSProtocol {
  // Frame bytes
  const uint8_t START1 = 0x7E;
  const uint8_t START2 = 0xA1; 
  const uint8_t END1 = 0xAA;
  const uint8_t END2 = 0x55;
  
  // Function codes
  const uint8_t STATE = 0x01;
  const uint8_t PARAM_READ = 0x02;
  const uint8_t PARAM_SET = 0x22;
  const uint8_t CONTROL_SET = 0x51;
  
  // Register address STATE
  const uint16_t ADR_STATE = 0x0000;
  
  // Register addresses PARAMETERCONFIG
  const uint16_t ADR_PARAM_VOLT = 0x0000;
  const uint16_t ADR_PARAM_TEMP = 0x0038;
  const uint16_t ADR_PARAM_CURRENT = 0x0068;
  const uint16_t ADR_PARAM_BALANCE = 0x008C;

  
  // Register addresses CONTROL SET
  const uint16_t ADR_CON_CHG_MOS_ON = 0x0006;
  const uint16_t ADR_CON_CHG_MOS_OFF = 0x0004;
  const uint16_t ADR_CON_DIS_MOS_ON = 0x0003;
  const uint16_t ADR_CON_DIS_MOS_OFF = 0x0001;

  // Register address to set the Cell number
  const uint16_t ADR_CELL_NUM = 0x009A;
  
  // Adresse für SaveApply
  const uint16_t ADR_CON_SAVE = 0x0007;

  // Adresse für FactoryReset
  const uint16_t ADR_F_RESET = 0x000C;

  // Data lengths
  const uint8_t LENGTH_STATE = 0xBE;
  const uint8_t LENGTH_PARAM_VOLT = 0x34;
  const uint8_t LENGTH_PARAM_TEMP = 0x2C;
  const uint8_t LENGTH_PARAM_CURRENT = 0x20;
  const uint8_t LENGTH_PARAM_BALANCE = 0x0C;
  const uint8_t LENGTH_CONTROL = 0x00;

  // Data length to set the Cell number
  const uint8_t LENGTH_CELL_NUM = 0x02;
  

}

// =============================================================================
// Data Structures
// =============================================================================

struct BMSStateData {
  // Cell data
  int numCells = 10;          // Default-Wert
  float cellVoltages[24];     // Default-Datenlänge
  
  float unitMax, unitMin, unitDiff, avgVoltage;
  
  // Temperatures
  float temperature_T1, temperature_T2, temperature_T3, temperature_T4;
  float temperature_MOS, temperature_PCB;
  
  // Pack data
  float packVoltage;
  float current;
  uint16_t stateOfCharge;
  
  // Status
  bool balanceStatus, chargeMOS, dischargeMOS;
  
  // Capacity
  float physicalAH, remainingAH;
  float totalDischargeAH, totalChargeAH;
  
  // Runtime
  uint32_t runtimeSeconds;
  uint16_t totalCycles;
  
  bool valid;
  
  BMSStateData() : valid(false), numCells(0) {}
};

struct BMSVoltageParams {
  // Protection parameters
  float cellOVProt, cellOVRec, cellOVProt2, cellOVRec2;
  float packOVProt, packOVRec;
  float cellUVProt, cellUVRec, cellUVProt2, cellUVRec2;
  float packUVProt, packUVRec;
  float cellDiffProt, cellDiffRec;
  
  // Warning parameters
  float cellOVWarn, cellOVWarnRec, packOVWarn, packOVWarnRec;
  float cellUVWarn, cellUVWarnRec, packUVWarn, packUVWarnRec;
  float cellDiffWarn, cellDiffWarnRec;
  
  bool valid;
  
  BMSVoltageParams() : valid(false) {}
};

struct BMSTemperatureParams {
  // Protection parameters
  float chgHTProt, chgHTRec, disCHGHTProt, disCHGHTRec;
  float mosHTProt, mosHTRec;
  float chgLTProt, chgLTRec, disCHGLTProt, disCHGLTRec;
  
  // Warning parameters
  float chgHTWarn, chgHTWarnRec, disCHGHTWarn, disCHGHTWarnRec;
  float mosHTWarn, mosHTWarnRec;
  float chgLTWarn, chgLTWarnRec, disCHGLTWarn, disCHGLTWarnRec;
  
  bool valid;
  
  BMSTemperatureParams() : valid(false) {}
};

struct BMSCurrentParams {
  // Protection parameters
  float chgOCProt, disCHGOCProt, disCHGOCProt2, scProt;
  uint16_t chgOCDelay, disCHGOCDelay, disCHGOCDelay2, scDelay;
  
  // Warning parameters
  float chgOCWarn, chgOCWarnRec, disCHGOCWarn, disCHGOCWarnRec;
  
  // SOC parameters
  uint16_t socLowLV1Warn, socLowLV2Warn;
  
  bool valid;
  
  BMSCurrentParams() : valid(false) {}
};

struct BMSBalanceParams {
  float balLimitV, balStartV;
  float balDiffOn, balDiffOff;
  uint16_t balCur, balChgCur;
  
  bool valid;
  
  BMSBalanceParams() : valid(false) {}
};

// =============================================================================
// Main AntBMS Class
// =============================================================================

class AntBMS {
public:
  // Constructor
  AntBMS(uint8_t rxPin = 20, uint8_t txPin = 21, uint32_t baudRate = 19200);
  
  // Initialization
  bool begin();
  void end();
  
  // Data reading methods
  bool readStateData();
  bool readVoltageParams();
  bool readTemperatureParams();
  bool readCurrentParams();
  bool readBalanceParams();
  bool readAllParams();
  
  // SaveApply
  bool SaveApply();

  // Factory Reset
  bool FactoryReset();

  // Funktionsdeklaration Steuerung
  bool setControl(const char input);
  
  
  // Data access methods
  const BMSStateData& getStateData() const { return stateData_; }
  const BMSVoltageParams& getVoltageParams() const { return voltageParams_; }
  const BMSTemperatureParams& getTemperatureParams() const { return temperatureParams_; }
  const BMSCurrentParams& getCurrentParams() const { return currentParams_; }
  const BMSBalanceParams& getBalanceParams() const { return balanceParams_; }
  
  // Utility methods
  void printStateData() const;
  void printVoltageParams() const;
  void printTemperatureParams() const;
  void printCurrentParams() const;
  void printBalanceParams() const;
  void printAllData() const;

  // Anzahl der Zellen setten

  void configureBMSCells();
  
  // Safety checks
  bool checkVoltageParamsSafety() const;
  bool checkTemperatureParamsSafety() const;
  bool checkCurrentParamsSafety() const;
  bool checkBalanceParamsSafety() const;
  bool checkAllParamsSafety() const;
  
  // Configuration
  void setTimeout(unsigned long timeout) { timeout_ = timeout; }
  void setDebugMode(bool debug) { debugMode_ = debug; }
  
  // Status
  bool isConnected() const { return connected_; }
  unsigned long getLastReadTime() const { return lastReadTime_; }
  
private:
  // Hardware configuration
  uint8_t rxPin_, txPin_;
  uint32_t baudRate_;
  HardwareSerial* serial_;
  bool connected_;
  unsigned long timeout_;
  bool debugMode_;
  unsigned long lastReadTime_;
  
  // Data storage
  BMSStateData stateData_;
  BMSVoltageParams voltageParams_;
  BMSTemperatureParams temperatureParams_;
  BMSCurrentParams currentParams_;
  BMSBalanceParams balanceParams_;
  
  // Communication buffers
  uint8_t rxBuffer_[256];
  int rxIndex_;
  
  // Private helper methods
  uint16_t calculateCRC16_TX(const uint8_t* data, int length) const;
  uint16_t calculateCRC16_RX(const uint8_t* data, int length) const;
  uint16_t readUint16LE(const uint8_t* data, int offset) const;
  uint32_t readUint32LE(const uint8_t* data, int offset) const;
  void printHex(const uint8_t* data, int length) const;
  
  bool sendCommand(const uint8_t* command, int cmdLength, uint8_t* response, int* respLength);
  bool buildAndSendCommand(uint8_t functionCode, uint16_t address, uint8_t dataLength);

  // Anzahl Zellen setten

  bool buildAndSendCommandBMSCell(uint8_t functionCode, uint16_t address, uint8_t dataLength, uint8_t cell_num);
  
  // Parser methods
  bool parseStateResponse(const uint8_t* response, int length);
  bool parseVoltageParams(const uint8_t* response, int length);
  bool parseTemperatureParams(const uint8_t* response, int length);
  bool parseCurrentParams(const uint8_t* response, int length);
  bool parseBalanceParams(const uint8_t* response, int length);

  // Parse Set
  bool parseSetConResponse(const uint8_t* response, int length, const char input);

  // Parse ConfigureBMSCells
  bool parseConfigureBMSCells(const uint8_t* response, int length, uint16_t cell_num);
  
  // Parse SaveApply

  bool parseSave(const uint8_t* response, int length);


  // Validation methods
  bool validateFrame(const uint8_t* response, int length, int expectedMinLength) const;
  bool validateCRC(const uint8_t* response, int length) const;
  bool validateSetControl(const uint8_t* response, int length) const;
};

// =============================================================================
// Implementation
// =============================================================================

AntBMS::AntBMS(uint8_t rxPin, uint8_t txPin, uint32_t baudRate)
  : rxPin_(rxPin), txPin_(txPin), baudRate_(baudRate), 
    serial_(nullptr), connected_(false), timeout_(1000), 
    debugMode_(false), lastReadTime_(0), rxIndex_(0) {
  serial_ = new HardwareSerial(1);
}

bool AntBMS::begin() {
  if (!serial_) return false;
  
  serial_->begin(baudRate_, SERIAL_8N1, rxPin_, txPin_);
  connected_ = true;
  
  if (debugMode_) {
    Serial.println("AntBMS Communication System Initialized");
    Serial.printf("RX Pin: %d, TX Pin: %d, Baud Rate: %d\n", rxPin_, txPin_, baudRate_);
  }
  
  return true;
}

void AntBMS::end() {
  if (serial_) {
    serial_->end();
    connected_ = false;
  }
}

bool AntBMS::readStateData() {
  if (!connected_) return false;
  
  if (debugMode_) Serial.println("Reading State data...");
  
  bool success = buildAndSendCommand(AntBMSProtocol::STATE, 
                                    AntBMSProtocol::ADR_STATE, 
                                    AntBMSProtocol::LENGTH_STATE);
  if (success) {
    lastReadTime_ = millis();
  }
  return success;
}

// SaveApply
bool AntBMS::SaveApply() {
  if (!connected_) return false;
  
  if (debugMode_) Serial.println("Save Settings...");
  
  bool success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_CON_SAVE, 
                                    AntBMSProtocol::LENGTH_CONTROL);
  if (success) {
    lastReadTime_ = millis();
  }
  return success;
}

// Factory Reset
bool AntBMS::FactoryReset() {
  if (!connected_) return false;
  
  if (debugMode_) Serial.println("Factory Reset...");
  
  bool success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_F_RESET, 
                                    AntBMSProtocol::LENGTH_CONTROL);
  if (success) {
    lastReadTime_ = millis();
  }
  return success;
}


bool AntBMS::readVoltageParams() {
  if (!connected_) return false;
  
  if (debugMode_) Serial.println("Reading Voltage parameters...");
  
  bool success = buildAndSendCommand(AntBMSProtocol::PARAM_READ, 
                                    AntBMSProtocol::ADR_PARAM_VOLT, 
                                    AntBMSProtocol::LENGTH_PARAM_VOLT);
  if (success) {
    lastReadTime_ = millis();
  }
  return success;
}

bool AntBMS::readTemperatureParams() {
  if (!connected_) return false;
  
  if (debugMode_) Serial.println("Reading Temperature parameters...");
  
  bool success = buildAndSendCommand(AntBMSProtocol::PARAM_READ, 
                                    AntBMSProtocol::ADR_PARAM_TEMP, 
                                    AntBMSProtocol::LENGTH_PARAM_TEMP);
  if (success) {
    lastReadTime_ = millis();
  }
  return success;
}

bool AntBMS::readCurrentParams() {
  if (!connected_) return false;
  
  if (debugMode_) Serial.println("Reading Current parameters...");
  
  bool success = buildAndSendCommand(AntBMSProtocol::PARAM_READ, 
                                    AntBMSProtocol::ADR_PARAM_CURRENT, 
                                    AntBMSProtocol::LENGTH_PARAM_CURRENT);
  if (success) {
    lastReadTime_ = millis();
  }
  return success;
}

bool AntBMS::readBalanceParams() {
  if (!connected_) return false;
  
  if (debugMode_) Serial.println("Reading Balance parameters...");
  
  bool success = buildAndSendCommand(AntBMSProtocol::PARAM_READ, 
                                    AntBMSProtocol::ADR_PARAM_BALANCE, 
                                    AntBMSProtocol::LENGTH_PARAM_BALANCE);
  if (success) {
    lastReadTime_ = millis();
  }
  return success;
}

bool AntBMS::readAllParams() {
  bool success = true;
  success &= readVoltageParams();
  delay(100);
  success &= readTemperatureParams();
  delay(100);
  success &= readCurrentParams();
  delay(100);
  success &= readBalanceParams();
  return success;
}

bool AntBMS::setControl(const char input) {
  if (!connected_) return false;
  
  if (debugMode_) Serial.println("Set Charge Mode...");
  
  bool success;

  switch (input)
  {
  case 'c':
    success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_CON_CHG_MOS_ON, 
                                    AntBMSProtocol::LENGTH_CONTROL);
    Serial.println("Ladevorgang gestartet.");
    break;
  case 'x':
    success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_CON_CHG_MOS_OFF, 
                                    AntBMSProtocol::LENGTH_CONTROL);
    Serial.println("Ladevorgang gestoppt.");
    break;
  case 'd':
    success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_CON_DIS_MOS_ON, 
                                    AntBMSProtocol::LENGTH_CONTROL);
    Serial.println("Batterie steht für die Nutzung zur Verfügung.");
    break;
  case 'f':
    success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_CON_DIS_MOS_OFF, 
                                    AntBMSProtocol::LENGTH_CONTROL);
    Serial.println("Batterienutzung beendet.");
    break;

  }
  /*bool success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_CON_CHG_MOS_ON, 
                                    AntBMSProtocol::LENGTH_CONTROL);*/
  if (success) {
    lastReadTime_ = millis();
  }
  return success;
}


// Helper Methods Implementation
uint16_t AntBMS::calculateCRC16_TX(const uint8_t* data, int length) const {
  uint16_t crc = 0xFFFF;
  
  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos];
    
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}

uint16_t AntBMS::calculateCRC16_RX(const uint8_t* data, int length) const {
  uint16_t crc = 0xFFFF;
  
  for (int pos = 1; pos < length; pos++) {
    crc ^= (uint16_t)data[pos];
    
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}

uint16_t AntBMS::readUint16LE(const uint8_t* data, int offset) const {
  return data[offset] | (data[offset + 1] << 8);
}

uint32_t AntBMS::readUint32LE(const uint8_t* data, int offset) const {
  return data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16) | (data[offset + 3] << 24);
}

void AntBMS::printHex(const uint8_t* data, int length) const {
  if (!debugMode_) return;
  
  for (int i = 0; i < length; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

bool AntBMS::buildAndSendCommand(uint8_t functionCode, uint16_t address, uint8_t dataLength) {
  uint8_t data[5] = {
    AntBMSProtocol::START2,
    functionCode,
    (uint8_t)(address & 0xFF),
    (uint8_t)(address >> 8),
    dataLength
  };
  
  uint16_t crc = calculateCRC16_TX(data, 5);
  uint8_t crcLow = crc & 0xFF;
  uint8_t crcHigh = (crc >> 8) & 0xFF;
  
  uint8_t command[10] = {
    AntBMSProtocol::START1, AntBMSProtocol::START2,
    functionCode,
    (uint8_t)(address & 0xFF), (uint8_t)(address >> 8),
    dataLength,
    crcLow, crcHigh,
    AntBMSProtocol::END1, AntBMSProtocol::END2
  };
  
  uint8_t response[256];
  int responseLength = 0;
  
  if (sendCommand(command, 10, response, &responseLength)) {
    // Determine which parser to use based on function code and address
    if (functionCode == AntBMSProtocol::STATE) {
      return parseStateResponse(response, responseLength);
    } else if (functionCode == AntBMSProtocol::PARAM_READ) {
      if (address == AntBMSProtocol::ADR_PARAM_VOLT) {
        return parseVoltageParams(response, responseLength);
      } else if (address == AntBMSProtocol::ADR_PARAM_TEMP) {
        return parseTemperatureParams(response, responseLength);
      } else if (address == AntBMSProtocol::ADR_PARAM_CURRENT) {
        return parseCurrentParams(response, responseLength);
      } else if (address == AntBMSProtocol::ADR_PARAM_BALANCE) {
        return parseBalanceParams(response, responseLength);
      } else if (address == AntBMSProtocol::ADR_CON_CHG_MOS_ON){
        return parseSetConResponse(response, responseLength, 'c');
      } else if (address == AntBMSProtocol::ADR_CON_CHG_MOS_OFF){
        return parseSetConResponse(response, responseLength, 'x');
      } else if (address == AntBMSProtocol::ADR_CON_DIS_MOS_ON){
        return parseSetConResponse(response, responseLength, 'd');
      } else if (address == AntBMSProtocol::ADR_CON_DIS_MOS_OFF){
        return parseSetConResponse(response, responseLength, 'f');
      } else if (address == AntBMSProtocol::ADR_CON_SAVE){
        return parseSave(response, responseLength);
      } else if (address == AntBMSProtocol::ADR_F_RESET){
        Serial.println("Factory Reset was successful!");
        return true;
      }
        
    }
  }
  return false;
}

/// Anzahl der Zellen setten
bool AntBMS::buildAndSendCommandBMSCell(uint8_t functionCode, uint16_t address, uint8_t dataLength, uint8_t cell_num) {

  uint8_t response[256];
  int responseLength = 0;

  uint8_t data[7] = {
    AntBMSProtocol::START2,
    functionCode,
    (uint8_t)(address & 0xFF),
    (uint8_t)(address >> 8),
    dataLength, 
    cell_num, 0x00
  };
  
  uint16_t crc = calculateCRC16_TX(data, 7);
  uint8_t crcLow = crc & 0xFF;
  uint8_t crcHigh = (crc >> 8) & 0xFF;
  

  uint8_t commandCell[12] = {
    AntBMSProtocol::START1, AntBMSProtocol::START2,
    functionCode,
    (uint8_t)(address & 0xFF), (uint8_t)(address >> 8),
    dataLength, 
    cell_num, 0x00,
    crcLow, crcHigh,
    AntBMSProtocol::END1, AntBMSProtocol::END2
  };

  Serial.println("Funktioniert es?");

  if (sendCommand(commandCell, 12, response, &responseLength)){
        return parseConfigureBMSCells(response, responseLength, cell_num);
  }
  
}

bool AntBMS::sendCommand(const uint8_t* command, int cmdLength, uint8_t* response, int* respLength) {
  if (!serial_ || !connected_) return false;
  
  // Clear pending data
  while (serial_->available()) {
    serial_->read();
  }
  
  // Send command
  serial_->write(command, cmdLength);
  
  if (debugMode_) {
    Serial.print("Sent: ");
    printHex(command, cmdLength);
  }
  
  // Wait for response
  rxIndex_ = 0;
  memset(rxBuffer_, 0, sizeof(rxBuffer_));
  unsigned long startTime = millis();
  bool responseEnded = false;
  
  while (millis() - startTime < timeout_ && !responseEnded) {
    if (serial_->available() > 0) {
      rxBuffer_[rxIndex_] = serial_->read();
      rxIndex_++;
      startTime = millis();
      
      if (rxIndex_ >= sizeof(rxBuffer_)) {
        if (debugMode_) Serial.println("Warning: Buffer overflow prevented");
        break;
      }
      
      // Check for end of frame
      if (rxIndex_ >= 2 && rxBuffer_[rxIndex_-2] == AntBMSProtocol::END1 && 
          rxBuffer_[rxIndex_-1] == AntBMSProtocol::END2) {
        responseEnded = true;
      }
    }
    else if (rxIndex_ > 0 && millis() - startTime > 100) {
      responseEnded = true;
    }
  }
  
  if (rxIndex_ > 0) {
    memcpy(response, rxBuffer_, rxIndex_);
    *respLength = rxIndex_;
    
    if (debugMode_) {
      Serial.print("Received: ");
      printHex(response, *respLength);
    }
    
    return validateCRC(response, *respLength);
  }
  
  if (debugMode_) Serial.println("No response received");
  return false;
}

bool AntBMS::validateFrame(const uint8_t* response, int length, int expectedMinLength) const {
  if (length < expectedMinLength || 
      response[0] != AntBMSProtocol::START1 || 
      response[1] != AntBMSProtocol::START2) {
    if (debugMode_) {
      Serial.printf("Invalid frame: length=%d, header=0x%02X%02X\n", 
                    length, response[0], response[1]);
    }
    return false;
  }
  
  if (response[length-2] != AntBMSProtocol::END1 || 
      response[length-1] != AntBMSProtocol::END2) {
    if (debugMode_) {
      Serial.printf("Invalid end markers: 0x%02X%02X (expected 0xAA55)\n", 
                    response[length-2], response[length-1]);
    }
    return false;
  }
  
  return true;
}

bool AntBMS::validateCRC(const uint8_t* response, int length) const {
  if (length < 8) return false;
  
  uint8_t dataLength = response[5];
  uint16_t calculatedCRC = calculateCRC16_RX(response, dataLength + 6);
  
  uint8_t receivedCRC_Low = response[dataLength + 6];
  uint8_t receivedCRC_High = response[dataLength + 7];
  uint16_t receivedCRC = (receivedCRC_High << 8) | receivedCRC_Low;
  
  if (debugMode_) {
    Serial.printf("Received CRC-16_RX: 0x%04X\n", receivedCRC);
  }
  
  if (calculatedCRC == receivedCRC) {
    if (debugMode_) Serial.println("CRC-Check erfolgreich - CRC richtig");
    return true;
  } else {
    if (debugMode_) {
      Serial.printf("CRC-Check fehlgeschlagen - CRC falsch (Berechnet: 0x%04X, Empfangen: 0x%04X)\n", 
                    calculatedCRC, receivedCRC);
    }
    return false;
  }
}

// Parser implementations
bool AntBMS::parseStateResponse(const uint8_t* response, int length) {
  if (!validateFrame(response, length, 146)) return false;
  
  stateData_ = BMSStateData(); // Reset
  
  if (debugMode_) {
    Serial.printf("Valid AntBMS state response: Cmd=0x%02X, DataLength=%d\n", 
                  response[2], response[5]);
  }
  
  int numCells =response[9];
  Serial.print("numCell ist: ");
  Serial.println(numCells);
  // Parse cell voltages (Bytes 34-55)
  stateData_.numCells = numCells;
  for (int i = 0; i < numCells ; i++) {
    uint16_t rawVoltage = readUint16LE(response, 34 + i * 2);
    stateData_.cellVoltages[i] = rawVoltage * 0.001;
  }
  
  // Parse temperatures (Bytes 56-67)
  stateData_.temperature_T1 = readUint16LE(response, 34 + numCells*2);
  stateData_.temperature_T2 = readUint16LE(response, 36 + numCells*2);
  stateData_.temperature_T3 = readUint16LE(response, 38 + numCells*2);
  stateData_.temperature_T4 = readUint16LE(response, 40 + numCells*2);
  stateData_.temperature_MOS = readUint16LE(response, 42 + numCells*2);
  stateData_.temperature_PCB = readUint16LE(response, 44 + numCells*2);
  
  // Parse pack data
  stateData_.packVoltage = readUint16LE(response, 46 + numCells*2) * 0.01;
  stateData_.current = (int16_t)readUint16LE(response, 48 + numCells*2);
  stateData_.stateOfCharge = readUint16LE(response, 50 + numCells*2);
  
  // Parse status
  stateData_.balanceStatus = (response[53 + numCells*2] == 0x01);
  stateData_.chargeMOS = (response[54 + numCells*2] == 0x01);
  stateData_.dischargeMOS = (response[55 + numCells*2] == 0x01);
  
  // Parse capacities
  stateData_.physicalAH = readUint32LE(response, 58 + numCells*2) * 0.000001;
  stateData_.remainingAH = readUint32LE(response, 62 + numCells*2) * 0.000001;
  stateData_.runtimeSeconds = readUint32LE(response, 74 + numCells*2);
  
  // Parse statistics
  stateData_.unitMax = readUint16LE(response, 82 + numCells*2) * 0.001;
  stateData_.unitMin = readUint16LE(response, 86 + numCells*2) * 0.001;
  stateData_.totalCycles = readUint16LE(response, 88 + numCells*2);
  stateData_.unitDiff = readUint16LE(response, 90 + numCells*2) * 0.001;
  stateData_.avgVoltage = readUint16LE(response, 92 + numCells*2) * 0.001;
  
  stateData_.totalDischargeAH = readUint32LE(response, 94 + numCells*2) * 0.001;
  stateData_.totalChargeAH = readUint32LE(response, 108 + numCells*2) * 0.001;
  
  stateData_.valid = true;
  return true;
}

bool AntBMS::parseVoltageParams(const uint8_t* response, int length) {
  if (!validateFrame(response, length, 68)) return false;
  
  voltageParams_ = BMSVoltageParams(); // Reset
  
  // Parse protection parameters
  voltageParams_.cellOVProt = readUint16LE(response, 6) * 0.001;      ///
  voltageParams_.cellOVRec = readUint16LE(response, 8) * 0.001;       ////
  voltageParams_.cellOVProt2 = readUint16LE(response, 10) * 0.001;
  voltageParams_.cellOVRec2 = readUint16LE(response, 12) * 0.001;
  voltageParams_.packOVProt = readUint16LE(response, 14) * 0.1;
  voltageParams_.packOVRec = readUint16LE(response, 16) * 0.1;
  
  voltageParams_.cellUVProt = readUint16LE(response, 18) * 0.001;
  voltageParams_.cellUVRec = readUint16LE(response, 20) * 0.001;
  voltageParams_.cellUVProt2 = readUint16LE(response, 22) * 0.001;
  voltageParams_.cellUVRec2 = readUint16LE(response, 24) * 0.001;
  voltageParams_.packUVProt = readUint16LE(response, 26) * 0.1;
  voltageParams_.packUVRec = readUint16LE(response, 28) * 0.1;
  
  voltageParams_.cellDiffProt = readUint16LE(response, 30) * 0.001;
  voltageParams_.cellDiffRec = readUint16LE(response, 32) * 0.001;
  
  // Parse warning parameters
  voltageParams_.cellOVWarn = readUint16LE(response, 38) * 0.001;
  voltageParams_.cellOVWarnRec = readUint16LE(response, 40) * 0.001;
  voltageParams_.packOVWarn = readUint16LE(response, 42) * 0.1;
  voltageParams_.packOVWarnRec = readUint16LE(response, 44) * 0.1;
  voltageParams_.cellUVWarn = readUint16LE(response, 46) * 0.001;
  voltageParams_.cellUVWarnRec = readUint16LE(response, 48) * 0.001;
  voltageParams_.packUVWarn = readUint16LE(response, 50) * 0.1;
  voltageParams_.packUVWarnRec = readUint16LE(response, 52) * 0.1;
  voltageParams_.cellDiffWarn = readUint16LE(response, 54) * 0.001;
  voltageParams_.cellDiffWarnRec = readUint16LE(response, 56) * 0.001;
  
  voltageParams_.valid = true;
  return true;
}

bool AntBMS::parseTemperatureParams(const uint8_t* response, int length) {
  if (!validateFrame(response, length, 60)) return false;
  
  temperatureParams_ = BMSTemperatureParams(); // Reset
  
  // Parse protection parameters
  temperatureParams_.chgHTProt = readUint16LE(response, 6);
  temperatureParams_.chgHTRec = readUint16LE(response, 8);
  temperatureParams_.disCHGHTProt = readUint16LE(response, 10);
  temperatureParams_.disCHGHTRec = readUint16LE(response, 12);
  temperatureParams_.mosHTProt = readUint16LE(response, 14);
  temperatureParams_.mosHTRec = readUint16LE(response, 16);
  
  temperatureParams_.chgLTProt = (int16_t)readUint16LE(response, 18);
  temperatureParams_.chgLTRec = readUint16LE(response, 20);
  temperatureParams_.disCHGLTProt = (int16_t)readUint16LE(response, 22);
  temperatureParams_.disCHGLTRec = (int16_t)readUint16LE(response, 24);
  
  // Parse warning parameters
  temperatureParams_.chgHTWarn = readUint16LE(response, 30);
  temperatureParams_.chgHTWarnRec = readUint16LE(response, 32);
  temperatureParams_.disCHGHTWarn = readUint16LE(response, 34);
  temperatureParams_.disCHGHTWarnRec = readUint16LE(response, 36);
  temperatureParams_.mosHTWarn = readUint16LE(response, 38);
  temperatureParams_.mosHTWarnRec = readUint16LE(response, 40);
  temperatureParams_.chgLTWarn = readUint16LE(response, 42);
  temperatureParams_.chgLTWarnRec = readUint16LE(response, 44);
  temperatureParams_.disCHGLTWarn = (int16_t)readUint16LE(response, 46);
  temperatureParams_.disCHGLTWarnRec = (int16_t)readUint16LE(response, 48);
  
  temperatureParams_.valid = true;
  return true;
}

bool AntBMS::parseCurrentParams(const uint8_t* response, int length) {
  if (!validateFrame(response, length, 48)) return false;
  
  currentParams_ = BMSCurrentParams(); // Reset
  
  // Parse protection parameters
  currentParams_.chgOCProt = readUint16LE(response, 6) * 0.1;
  currentParams_.chgOCDelay = readUint16LE(response, 8);
  currentParams_.disCHGOCProt = readUint16LE(response, 10) * 0.1;
  currentParams_.disCHGOCDelay = readUint16LE(response, 12);
  currentParams_.disCHGOCProt2 = readUint16LE(response, 14) * 0.1;
  currentParams_.disCHGOCDelay2 = readUint16LE(response, 16);
  currentParams_.scProt = readUint16LE(response, 18);
  currentParams_.scDelay = readUint16LE(response, 20);
  
  // Parse warning parameters
  currentParams_.chgOCWarn = readUint16LE(response, 26) * 0.1;
  currentParams_.chgOCWarnRec = readUint16LE(response, 28) * 0.1;
  currentParams_.disCHGOCWarn = readUint16LE(response, 30) * 0.1;
  currentParams_.disCHGOCWarnRec = readUint16LE(response, 32) * 0.1;
  
  // Parse SOC parameters
  currentParams_.socLowLV1Warn = readUint16LE(response, 34);
  currentParams_.socLowLV2Warn = readUint16LE(response, 36);
  
  currentParams_.valid = true;
  return true;
}

bool AntBMS::parseBalanceParams(const uint8_t* response, int length) {
  if (!validateFrame(response, length, 28)) return false;
  
  balanceParams_ = BMSBalanceParams(); // Reset
  
  balanceParams_.balLimitV = readUint16LE(response, 6) * 0.001;
  balanceParams_.balStartV = readUint16LE(response, 8) * 0.001;
  balanceParams_.balDiffOn = readUint16LE(response, 10) * 0.001;
  balanceParams_.balDiffOff = readUint16LE(response, 12) * 0.001;
  balanceParams_.balCur = readUint16LE(response, 14);
  balanceParams_.balChgCur = readUint16LE(response, 16);
  
  balanceParams_.valid = true;
  return true;
}

// Parse SET_CONTROL
bool AntBMS::parseSetConResponse(const uint8_t* response, int length, const char input) {
  if (!validateFrame(response, length, 12)) return false;
  
  if (debugMode_) {
    Serial.printf("Valid AntBMS SetControl response: Cmd=0x%02X, DataLength=%d\n", 
                  response[2], response[5]);
  }
  
  // Check Frame
  if(response[2] == 0x61 && response[5] == 0x02 && response[6] == 0x01){
    Serial.println("Validate Set Control was successful.");
    return true;
  }
  

  return true;
}

bool AntBMS::parseConfigureBMSCells(const uint8_t* response, int length, uint16_t cell_num){
  if (!validateFrame(response, length, 12)) return false;
  
  if (debugMode_) {
    Serial.printf("Configure BMS Cell response: Cmd=0x%02X, DataLength=%d\n", 
                  response[2], response[5]);
  }
  
  // Check Frame
  if(response[2] == 0x42 && response[5] == 0x02 && response[6] == (uint8_t) cell_num && response[7] == 0x00){
    Serial.println("Configure BMS Cell was succesful.");
    return true;
  }
  

  return true;
}

bool AntBMS::parseSave(const uint8_t* response, int length){
  if (!validateFrame(response, length, 12)) return false;
  
  if (debugMode_) {
    Serial.printf("Settings have been saved : Cmd=0x%02X, DataLength=%d\n", 
                  response[2], response[5]);
  }
  
  // Check Frame
  if(response[2] == 0x61 && response[5] == 0x02 && response[6] == 0x01 && response[7] == 0x00){
    Serial.println("Settings saved successfully!");
    return true;
  }
  

  return true;
}


/**
 * Funktion zur Konfiguration der BMS-Zellanzahl
 * Fragt den Benutzer, ob die Anzahl der Batteriezellen geändert werden soll
 * Bei 'y': Eingabe der gewünschten Zellanzahl (10-24) mit 3 Versuchen
 * Bei 'n': Anzeige der aktuellen BMS-Schutzparameter
 */
void AntBMS::configureBMSCells() {
    Serial.println("Soll die Anzahl Batteriezellen geändert werden?");
    Serial.println("Bitte 'y' oder 'n' eingeben.");
    
    // Warten auf Eingabe
    while(Serial.available() == 0) {
        delay(10);
    }
    
    char input = Serial.read();
    
    
    if(input == 'y' || input == 'Y') {
        Serial.println("Bitte die gewünschte Anzahl der Zellen für das BMS eingeben (min. 10 und max. 24):");
        uint8_t cell_num = Serial.parseInt();
        Serial.print("Die Eingabe war: ");
        Serial.println(cell_num);
        
        if(10 <= cell_num && cell_num <= 24) {

            //Debug-Ausgabe der Hexwerte
            Serial.print("Dezimal: ");
            Serial.print(cell_num);
            Serial.print(" -> Hex: 0x");
            if(cell_num < 0x10) Serial.print("0");
            Serial.print(cell_num, HEX);
            // Serial.print(" (High: 0x");
            
            // 7E A1 22 9A 00 02 cell_num_low cell_num_high CRC_L CRC_H AA 55
            bool success = buildAndSendCommandBMSCell(AntBMSProtocol::PARAM_SET, 
                                            AntBMSProtocol::ADR_CELL_NUM,
                                            AntBMSProtocol::LENGTH_CELL_NUM, cell_num);
            
            
            Serial.print("Zellanzahl erfolgreich auf ");
            Serial.print(cell_num);
            Serial.println(" gesetzt.");                                

            if(SaveApply()){
              Serial.println("Settings could be saved!");
            } else {
              Serial.println("Settings could not be saved!");
            }
            
        } else if(cell_num < 10 || cell_num > 24) {
            Serial.println("Anzahl der Zellen konnte nicht eingegeben werden.");
            Serial.println("Bitte nochmal versuchen.");
            
            int retry_count = 0;
            const int max_retries = 3;
            bool valid_input = false;
            
            while(retry_count < max_retries && !valid_input) {
                Serial.print("Versuch ");
                Serial.print(retry_count + 1);
                Serial.print(" von ");
                Serial.print(max_retries);
                Serial.println(": Bitte die gewünschte Anzahl der Zellen für das BMS eingeben (min. 10 und max. 24):");
                
                // Warten auf Eingabe
                while(Serial.available() == 0) {
                    delay(10); // Kurze Pause, um CPU-Last zu reduzieren
                }
                
                cell_num = Serial.parseInt();
                
                Serial.print("Die Eingabe war: ");
                Serial.println(cell_num);

                if(10 <= cell_num && cell_num <= 24) {
                    // Konvertierung in 2-Byte Hexadezimalwert
                    uint16_t cell_num_hex = (uint16_t)cell_num;
                    
                    
                    // Debug-Ausgabe der Hexwerte
                    Serial.print("Dezimal: ");
                    Serial.print(cell_num);
                    Serial.print(" -> Hex: 0x");
                    if(cell_num_hex < 0x10) Serial.print("0");
                    Serial.print(cell_num_hex, HEX);
                    // Serial.print(" (High: 0x");
                  
                    
                    // 7E A1 22 9A 00 02 cell_num_low cell_num_high CRC_L CRC_H AA 55
                    Serial.print("Zellanzahl erfolgreich auf ");
                    Serial.print(cell_num);
                    Serial.println(" gesetzt.");
                    valid_input = true;

                    // 7E A1 22 9A 00 02 cell_num_low cell_num_high CRC_L CRC_H AA 55
                    bool success = buildAndSendCommandBMSCell(AntBMSProtocol::PARAM_SET, 
                                            AntBMSProtocol::ADR_CELL_NUM,
                                            AntBMSProtocol::LENGTH_CELL_NUM, cell_num_hex);
            
                    if(success){
                      Serial.println("TEST.");
                    } else {
                      Serial.println("nicht erfolgt!");
                    }
                    
                    Serial.print("Zellanzahl erfolgreich auf ");
                    Serial.print(cell_num);
                    Serial.println(" gesetzt.");                                

                    if(SaveApply()){
                      Serial.println("Settings could be saved!");
                    } else {
                      Serial.println("Settings could not be saved!");
                    }
                } else {
                    retry_count++;
                    Serial.print("Ungültige Eingabe: ");
                    Serial.print(cell_num);
                    Serial.println(". Zahl muss zwischen 10 und 24 liegen.");
                    
                    if(retry_count >= max_retries) {
                        Serial.println("Maximale Anzahl Versuche erreicht. Vorgang abgebrochen.");
                        Serial.println("Das BMS behält die aktuelle Zellkonfiguration bei.");
                    }
                }
            }
        }
        
    } else if(input == 'n' || input == 'N') {
        Serial.println("Das BMS hat folgende Schutzparameter:");
        
        // Read all parameters
        if (readAllParams()) {
            printAllData();
            checkAllParamsSafety();
        }
    }
}



// Print methods implementation
void AntBMS::printStateData() const {
  if (!stateData_.valid) {
    Serial.println("No valid state data available");
    return;
  }
  
  Serial.println("\n=========== AntBMS STATE DATA ===========");
  Serial.printf("Pack Voltage: %.2f V\n", stateData_.packVoltage);
  Serial.printf("Current: %.2f A\n", stateData_.current);
  Serial.printf("Power: %.2f W\n", stateData_.packVoltage * stateData_.current);
  Serial.printf("SOC: %d%%\n", stateData_.stateOfCharge);
  Serial.printf("Capacity: %.3f/%.3f AH\n", stateData_.remainingAH, stateData_.physicalAH);
  
  Serial.printf("\nCell Voltages (Max: %.3fV, Min: %.3fV, Diff: %.3fV):\n", 
                stateData_.unitMax, stateData_.unitMin, stateData_.unitDiff);
  Serial.print("stateData_numCells = ");
  Serial.println(stateData_.numCells);
  for (int i = 0; i < stateData_.numCells; i++) {
    delay(500);
    Serial.printf("  Cell %2d: %.3f V\n", i + 1, stateData_.cellVoltages[i]);
  }
  
  Serial.printf("\nTemperatures: T1:%.1f°C T2:%.1f°C T3:%.1f°C T4:%.1f°C MOS:%.1f°C PCB:%.1f°C\n",
                stateData_.temperature_T1, stateData_.temperature_T2, stateData_.temperature_T3,
                stateData_.temperature_T4, stateData_.temperature_MOS, stateData_.temperature_PCB);
  
  Serial.printf("Status: Balance:%s ChargeMOS:%s DischargeMOS:%s\n",
                stateData_.balanceStatus ? "ON" : "OFF",
                stateData_.chargeMOS ? "ON" : "OFF", 
                stateData_.dischargeMOS ? "ON" : "OFF");
  
  Serial.println("========================================\n");
}

void AntBMS::printVoltageParams() const {
  if (!voltageParams_.valid) {
    Serial.println("No valid voltage parameters available");
    return;
  }
  
 Serial.println("\n============= SPANNUNGSPARAMETER =============");
  
  // Überspannungsschutz
  Serial.println("ÜBERSPANNUNGSSCHUTZ:");
  Serial.printf("  1. CellOVProt   (Einzelzell-Überspannungsschutz Stufe1):      %.3f V\n", voltageParams_.cellOVProt);
  Serial.printf("  2. CellOVRec    (Einzelzell-Überspannungs-Wiederherstellung): %.3f V\n", voltageParams_.cellOVRec);
  Serial.printf("  3. CellOVProt2  (Einzelzell-Überspannungsschutz Stufe 2):     %.3f V\n", voltageParams_.cellOVProt2);
  Serial.printf("  4. CellOVRec2   (Einzelzell-Überspannungs-Wiederherst. St.2): %.3f V\n", voltageParams_.cellOVRec2);
  Serial.printf("  5. PackOVProt   (Gesamtspannungs-Überspannungsschutz St.3):   %.1f V\n", voltageParams_.packOVProt);
  Serial.printf("  6. PackOVRec    (Gesamtspannungs-Überspannungs-Wiederherst.): %.1f V\n", voltageParams_.packOVRec);
  
  // Unterspannungsschutz
  Serial.println("\nUNTERSPANNUNGSSCHUTZ:");
  Serial.printf("  7. CellUVProt   (Einzelzell-Unterspannungsschutz Stufe 1):    %.3f V\n", voltageParams_.cellUVProt);
  Serial.printf("  8. CellUVRec    (Einzelzell-Unterspannungs-Wiederherstellung):%.3f V\n", voltageParams_.cellUVRec);
  Serial.printf("  9. CellUVProt2  (Einzelzell-Unterspannungsschutz Stufe 2):    %.3f V\n", voltageParams_.cellUVProt2);
  Serial.printf(" 10. CellUVRec2   (Einzelzell-Unterspannungs-Wiederherst. St.2):%.3f V\n", voltageParams_.cellUVRec2);
  Serial.printf(" 11. PackUVProt   (Gesamtspannungs-Unterspannungsschutz St.3):  %.1f V\n", voltageParams_.packUVProt);
  Serial.printf(" 12. PackUVRec    (Gesamtspannungs-Unterspannungs-Wiederherst.):%.1f V\n", voltageParams_.packUVRec);
  
  // Spannungsdifferenzschutz
  Serial.println("\nSPANNUNGSDIFFERENZSCHUTZ:");
  Serial.printf(" 13. CellDiffProt (Zellspannungsdifferenz-Schutz):              %.3f V\n", voltageParams_.cellDiffProt);
  Serial.printf(" 14. CellDiffRec  (Zellspannungsdifferenz-Wiederherstellung):   %.3f V\n", voltageParams_.cellDiffRec);
  
  // Alarmparameter (Spannungsalarm-Parameter)
  Serial.println("\nSPANNUNGSALARM-PARAMETER:");
  Serial.printf(" 15. CellOVWarn    (Einzelzell-Überspannungswarnung):           %.3f V\n", voltageParams_.cellOVWarn);
  Serial.printf("     CellOVWarnRec (Einzelzell-Überspannungswarnung-Wiederherst.):%.3f V\n", voltageParams_.cellOVWarnRec);
  Serial.printf("     PackOVWarn    (Gesamtspannungs-Überspannungswarnung):      %.1f V\n", voltageParams_.packOVWarn);
  Serial.printf("     PackOVWarnRec (Gesamtspannungs-Überspannungswarnung-Wieder.):%.1f V\n", voltageParams_.packOVWarnRec);
  Serial.printf("     CellUVWarn    (Einzelzell-Unterspannungswarnung):          %.3f V\n", voltageParams_.cellUVWarn);
  Serial.printf("     CellUVWarnRec (Einzelzell-Unterspannungswarnung-Wiederherst.):%.3f V\n", voltageParams_.cellUVWarnRec);
  Serial.printf("     PackUVWarn    (Gesamtspannungs-Unterspannungswarnung):     %.1f V\n", voltageParams_.packUVWarn);
  Serial.printf("     PackUVWarnRec (Gesamtspannungs-Unterspannungswarnung-Wieder.):%.1f V\n", voltageParams_.packUVWarnRec);
  Serial.printf("     CellDiffWarn  (Zellspannungsdifferenz-Warnung):            %.3f V\n", voltageParams_.cellDiffWarn);
  Serial.printf("     CellDiffWarnRec (Zellspannungsdifferenz-Warnung-Wiederherst.):%.3f V\n", voltageParams_.cellDiffWarnRec);
  
  Serial.println("==============================================\n");
}


void AntBMS::printTemperatureParams() const {
  if (!temperatureParams_.valid) {
    Serial.println("No valid temperature parameters available");
    return;
  }
  
 Serial.println("\n============ TEMPERATURPARAMETER ============");
  
  // Batterietemperatur-Überhitzungsschutz
  Serial.println("BATTERIETEMPERATUR-ÜBERHITZUNGSSCHUTZ:");
  Serial.printf("  1. CHGHTProt    (Überhitzungsschutz beim Laden):               %.0f°C\n", temperatureParams_.chgHTProt);
  Serial.printf("  2. CHGHTRec     (Überhitzungsschutz-Wiederherstellung Laden):  %.0f°C\n", temperatureParams_.chgHTRec);
  Serial.printf("  3. DisCHGHTProt (Überhitzungsschutz beim Entladen):            %.0f°C\n", temperatureParams_.disCHGHTProt);
  Serial.printf("  4. DisCHGHTRec  (Überhitzungsschutz-Wiederherstellung Entlad.):%.0f°C\n", temperatureParams_.disCHGHTRec);
  
  // MOS-Temperaturschutz
  Serial.println("\nMOS-TEMPERATURSCHUTZ:");
  Serial.printf("  5. MOSHTProt    (MOS-Überhitzungsschutz):                      %.0f°C\n", temperatureParams_.mosHTProt);
  Serial.printf("  6. MOSHTRec     (MOS-Überhitzungsschutz-Wiederherstellung):    %.0f°C\n", temperatureParams_.mosHTRec);
  
  // Batterietemperatur-Kälteschutz
  Serial.println("\nBATTERIETEMPERATUR-KÄLTESCHUTZ:");
  Serial.printf("  7. CHGLTProt    (Kälteschutz beim Laden):                      %.0f°C\n", temperatureParams_.chgLTProt);
  Serial.printf("  8. CHGLTRec     (Kälteschutz-Wiederherstellung Laden):        %.0f°C\n", temperatureParams_.chgLTRec);
  Serial.printf("  9. DisCHGLTProt (Kälteschutz beim Entladen):                   %.0f°C\n", temperatureParams_.disCHGLTProt);
  Serial.printf(" 10. DisCHGLTRec  (Kälteschutz-Wiederherstellung Entladen):     %.0f°C\n", temperatureParams_.disCHGLTRec);
  
  // Temperatur-Alarmparameter
  Serial.println("\nTEMPERATUR-ALARMPARAMETER:");
  Serial.printf(" 11. CHGHTWarn     (Überhitzungswarnung beim Laden):             %.0f°C\n", temperatureParams_.chgHTWarn);
  Serial.printf("     CHGHTWarnRec  (Überhitzungswarnung-Wiederherstellung Laden):%.0f°C\n", temperatureParams_.chgHTWarnRec);
  Serial.printf("     DisCHGHTWarn  (Überhitzungswarnung beim Entladen):          %.0f°C\n", temperatureParams_.disCHGHTWarn);
  Serial.printf("     DisCHGHTWarnRec (Überhitzungswarnung-Wiederherst. Entlad.): %.0f°C\n", temperatureParams_.disCHGHTWarnRec);
  Serial.printf("     MOSHTWarn     (MOS-Überhitzungswarnung):                    %.0f°C\n", temperatureParams_.mosHTWarn);
  Serial.printf("     MOSHTWarnRec  (MOS-Überhitzungswarnung-Wiederherstellung):  %.0f°C\n", temperatureParams_.mosHTWarnRec);
  Serial.printf("     CHGLTWarn     (Kälteschutzwarnung beim Laden):              %.0f°C\n", temperatureParams_.chgLTWarn);
  Serial.printf("     CHGLTWarnRec  (Kälteschutzwarnung-Wiederherstellung Laden): %.0f°C\n", temperatureParams_.chgLTWarnRec);
  Serial.printf("     DisCHGLTWarn  (Kälteschutzwarnung beim Entladen):           %.0f°C\n", temperatureParams_.disCHGLTWarn);
  Serial.printf("     DisCHGLTWarnRec (Kälteschutzwarnung-Wiederherst. Entlad.):  %.0f°C\n", temperatureParams_.disCHGLTWarnRec);
  
  Serial.println("==============================================\n");
}

void AntBMS::printCurrentParams() const {
  if (!currentParams_.valid) {
    Serial.println("No valid current parameters available");
    return;
  }
  
 Serial.println("\n============= STROMPARAMETER =============");
  
  // Ladestromschutz
  Serial.println("LADESTROM-ÜBERSTROMSCHUTZ:");
  Serial.printf("  1. CHGOCProt     (Ladestrom-Überstromschutz):                  %.1f A\n", currentParams_.chgOCProt);
  Serial.printf("     CHGOCDelay    (Ladestrom-Überstromschutz-Verzögerung):     %d S\n", currentParams_.chgOCDelay);
  
  // Entladestromschutz
  Serial.println("\nENTLADESTROM-ÜBERSTROMSCHUTZ:");
  Serial.printf("  2. DisCHGOCProt  (Entladestrom-Überstromschutz Stufe 1):       %.1f A\n", currentParams_.disCHGOCProt);
  Serial.printf("     DisCHGOCDelay (Entladestrom-Überstromschutz-Verzögerung St.1): %d S\n", currentParams_.disCHGOCDelay);
  Serial.printf("  3. DisCHGOCProt2 (Entladestrom-Überstromschutz Stufe 2):       %.1f A\n", currentParams_.disCHGOCProt2);
  Serial.printf("     DisCHGOCDelay2(Entladestrom-Überstromschutz-Verzögerung St.2): %d MS\n", currentParams_.disCHGOCDelay2);
  
  // Kurzschlussschutz
  Serial.println("\nKURZSCHLUSSCHUTZ:");
  Serial.printf("  4. SCProt        (Kurzschlussschutz):                          %.0f A\n", currentParams_.scProt);
  Serial.printf("     SCDelay       (Kurzschlussschutz-Verzögerung):             %d US\n", currentParams_.scDelay);
  
  // Überstrom-Alarmparameter
  Serial.println("\nÜBERSTROM-ALARMPARAMETER:");
  Serial.printf("  5. CHGOCWarn     (Ladestrom-Überstromwarnung):                 %.1f A\n", currentParams_.chgOCWarn);
  Serial.printf("     CHGOCWarnRec  (Ladestrom-Überstromwarnung-Wiederherstellung): %.1f A\n", currentParams_.chgOCWarnRec);
  Serial.printf("     DisCHGOCWarn  (Entladestrom-Überstromwarnung):              %.1f A\n", currentParams_.disCHGOCWarn);
  Serial.printf("     DisCHGOCWarnRec(Entladestrom-Überstromwarnung-Wiederherst.): %.1f A\n", currentParams_.disCHGOCWarnRec);
  
  // SOC-Ladezustandsüberwachung
  Serial.println("\nSOC-LADEZUSTANDSÜBERWACHUNG:");
  Serial.printf("  6. SOCLowLV1Warn (SOC-Ladezustandsüberwachung Level 1):        %d %%\n", currentParams_.socLowLV1Warn);
  Serial.printf("     SOCLowLV2Warn (SOC-Ladezustandsüberwachung Level 2):        %d %%\n", currentParams_.socLowLV2Warn);
  
  Serial.println("==========================================\n");
}

void AntBMS::printBalanceParams() const {
  if (!balanceParams_.valid) {
    Serial.println("No valid balance parameters available");
    return;
  }
  
  Serial.println("\n============ BALANCING-PARAMETER ============");
  
  // Spannungsbasierte Balancing-Parameter
  Serial.println("SPANNUNGSBASIERTE BALANCING-PARAMETER:");
  Serial.printf("  1. BalLimitV    (Zell-Balancing-Grenzspannung):            %.3f V\n", balanceParams_.balLimitV);
  Serial.printf("  2. BalStartV    (Lade-Balancing-Startspannung):            %.3f V\n", balanceParams_.balStartV);
  
  // Spannungsdifferenz-Parameter
  Serial.println("\nSPANNUNGSDIFFERENZ-PARAMETER:");
  Serial.printf("  3. BalDiffOn    (Balancing-Einschalt-Spannungsdifferenz):  %.3f V\n", balanceParams_.balDiffOn);
  Serial.printf("  4. BalDiffOff   (Balancing-Ausschalt-Spannungsdifferenz):  %.3f V\n", balanceParams_.balDiffOff);
  
  // Strom-Parameter
  Serial.println("\nSTROM-PARAMETER:");
  Serial.printf("  5. BalCur       (Balancing-Strom/Effektivität):            %d N\n", balanceParams_.balCur);
  Serial.printf("  6. BalChgCur    (Balancing-Ladestrom-Grenze):              %d A\n", balanceParams_.balChgCur);
  
  Serial.println("==============================================\n");
}

void AntBMS::printAllData() const {
  printStateData();
  printVoltageParams();
  printTemperatureParams();
  printCurrentParams();
  printBalanceParams();
}

// Safety check implementations
bool AntBMS::checkVoltageParamsSafety() const {
  if (!voltageParams_.valid) return false;
  
  bool safe = true;
  
  if (voltageParams_.cellOVProt > 4.3 || voltageParams_.cellOVProt < 4.0) {
    Serial.printf("⚠️  WARNING: Unusual CellOVProt: %.3fV\n", voltageParams_.cellOVProt);
    safe = false;
  }
  
  if (voltageParams_.cellUVProt < 2.5 || voltageParams_.cellUVProt > 3.2) {
    Serial.printf("⚠️  WARNING: Unusual CellUVProt: %.3fV\n", voltageParams_.cellUVProt);
    safe = false;
  }
  
  if (safe) Serial.println("✅ Voltage parameters are within safe range");
  return safe;
}

bool AntBMS::checkTemperatureParamsSafety() const {
  if (!temperatureParams_.valid) return false;
  
  bool safe = true;
  
  if (temperatureParams_.chgHTProt > 70 || temperatureParams_.chgHTProt < 40) {
    Serial.printf("⚠️  WARNING: Unusual CHGHTProt: %.0f°C\n", temperatureParams_.chgHTProt);
    safe = false;
  }
  
  if (safe) Serial.println("✅ Temperature parameters are within safe range");
  return safe;
}

bool AntBMS::checkCurrentParamsSafety() const {
  if (!currentParams_.valid) return false;
  
  bool safe = true;
  
  if (currentParams_.chgOCProt > 200 || currentParams_.chgOCProt < 10) {
    Serial.printf("⚠️  WARNING: Unusual CHGOCProt: %.1fA\n", currentParams_.chgOCProt);
    safe = false;
  }
  
  if (safe) Serial.println("✅ Current parameters are within safe range");
  return safe;
}

bool AntBMS::checkBalanceParamsSafety() const {
  if (!balanceParams_.valid) return false;
  
  bool safe = true;
  
  if (balanceParams_.balLimitV > 4.3 || balanceParams_.balLimitV < 4.0) {
    Serial.printf("⚠️  WARNING: Unusual BalLimitV: %.3fV\n", balanceParams_.balLimitV);
    safe = false;
  }
  
  if (safe) Serial.println("✅ Balance parameters are within safe range");
  return safe;
}

bool AntBMS::checkAllParamsSafety() const {
  bool allSafe = true;
  allSafe &= checkVoltageParamsSafety();
  allSafe &= checkTemperatureParamsSafety();
  allSafe &= checkCurrentParamsSafety();
  allSafe &= checkBalanceParamsSafety();
  return allSafe;
}

#endif // ANTBMS_H


// Create AntBMS instance
AntBMS bms(20, 21); // RX Pin 20, TX Pin 21

void setup() {
  Serial.begin(115200);
  
  // Initialize BMS communication
  if (bms.begin()) {
    bms.setDebugMode(true);
    Serial.println("BMS erfolgreich initialisiert.");
    //bms.setDebugMode(true);
  } else {
    Serial.println("BMS Initialisierung fehlgeschlagen.");
  }

  // // Read all parameters
  // if (bms.readAllParams()) {
  //   bms.printAllData();
  //   bms.checkAllParamsSafety();
  // }

  // // Anzahl der Batteriezellen setten 
  // bms.configureBMSCells();

}


void loop() {

  char input = Serial.read();
  Serial.print("Eingabe war: ");
  Serial.println(input);
 
  // Warten auf Eingabe
  while(Serial.available() == 0) {
        delay(10);
  }
 
  if (input == 'r' || 'R'){
       Serial.println("Soll das BMS auf Werkeinstellungen zurückgesetzt werden?");
       Serial.println("Wenn ja, bitte 'y' eingeben sonstige Eingaben werden nicht berücksichtigt.");
      
      char input2 = Serial.read();
      Serial.print("Eingabe war: ");
      Serial.println(input2);
      if(input2 == 'y' || input2 == 'Y') {
          
          // Factory Reset
          bms.FactoryReset();

          delay(500);

          // Anzahl der Batteriezellen setten 
          bms.configureBMSCells(); 
      }

  }

 


  // Read state data
  if (bms.readStateData()) {
    bms.printStateData();
  }
  
  // Read all parameters
  //if (bms.readAllParams()) {
  //  bms.printAllData();
 //   bms.checkAllParamsSafety();
 // }
  // Serial.println("Um Ladevorgang zu starten, bitte 'c' eingeben.");
  // Serial.println("Um Ladevorgang zu stoppen, bitte 'x' eingeben.");
  // Serial.println("Um Batterieenergie zu nutzen, bitte 'd' eingeben.");
  // Serial.println("Um Battiernutzung zu unterbrechen, bitte 'f' eingeben.");

  //Serial.println("Bitte '' eingeben, um das BMS zu resetten.");

  // if (Serial.available()) {
  //   char input = Serial.read();
  //   //input.trim();
  //   Serial.println(input);
  //   if (bms.setControl(input)) {
  //   //bms.printStateData();
    
  // }
  // } 


  delay(1000);
}
