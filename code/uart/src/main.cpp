// =============================================================================
// Folgender Code ist für das BMS ANT-BLE24BHUB angefertigt
// Kommunikation zwischen ANT-BMS und ESP32C3 Mini über UART-Protokoll
// Das BMS ist mit 11 gleichgroßen 1kΩ Widerständen verbunden (simuliert 11 Batteriezellen in Reihenschaltung)
// =============================================================================

#ifndef ANTBMS_H
#define ANTBMS_H

#include <Arduino.h>
#include <HardwareSerial.h>

// =============================================================================
// AntBMS Protocol Constants
// =============================================================================
namespace AntBMSProtocol {
  // START- und ENDBYTES
  const uint8_t START1 = 0x7E;
  const uint8_t START2 = 0xA1; 
  const uint8_t END1 = 0xAA;
  const uint8_t END2 = 0x55;
  
  // Funktioncodes für verschiedene Operationen
  const uint8_t STATE = 0x01;
  const uint8_t PARAM_READ = 0x02;
  const uint8_t PARAM_SET = 0x22;
  const uint8_t CONTROL_SET = 0x51;
  
  // Registeradresse STATE
  const uint16_t ADR_STATE = 0x0000;
  
  // Registeradresse PARAMETERCONFIG
  const uint16_t ADR_PARAM_VOLT = 0x0000;
  const uint16_t ADR_PARAM_TEMP = 0x0038;
  const uint16_t ADR_PARAM_CURRENT = 0x0068;
  const uint16_t ADR_PARAM_BALANCE = 0x008C;

  
  // Registeradresse CONTROL SET
  const uint16_t ADR_CON_CHG_MOS_ON = 0x0006;     // Lade-MOS einschalten
  const uint16_t ADR_CON_CHG_MOS_OFF = 0x0004;    // Lade-MOS ausschalten
  const uint16_t ADR_CON_DIS_MOS_ON = 0x0003;     // Entlade-MOS einschalten
  const uint16_t ADR_CON_DIS_MOS_OFF = 0x0001;    // Entlade-MOS ausschalten


  // Adresse um die Zellanzahl zu setten
  const uint16_t ADR_CELL_NUM = 0x009A;
  
  // Registeradresse für SaveApply
  const uint16_t ADR_CON_SAVE = 0x0007;

  // Registeradresse für FactoryReset (Werkeinstellungen)
  const uint16_t ADR_F_RESET = 0x000C;


  // Registeradresse für Balance ON und OFF
  const uint16_t ADR_BAL_ON = 0x000D;
  const uint16_t ADR_BAL_OFF = 0x000E;

  // Datenlängen
  const uint8_t LENGTH_STATE = 0xBE;
  const uint8_t LENGTH_PARAM_VOLT = 0x34;
  const uint8_t LENGTH_PARAM_TEMP = 0x2C;
  const uint8_t LENGTH_PARAM_CURRENT = 0x20;
  const uint8_t LENGTH_PARAM_BALANCE = 0x0C;
  const uint8_t LENGTH_CONTROL = 0x00;

  
  // Datenlänge um die Zellanzahl zu setten
  const uint8_t LENGTH_CELL_NUM = 0x02;
  

}

// =============================================================================
// Datenstrukturen für BMS-Informationen
// =============================================================================

// Struct für BMS-Zustandsdaten 
struct BMSStateData {
  // Zelldaten
  int numCells = 10;          // Default-Wert
  float cellVoltages[24];     // Default-Datenlänge
  
  float unitMax, unitMin, unitDiff, avgVoltage;
  
  // Temperaturen
  float temperature_T1, temperature_T2, temperature_T3, temperature_T4;
  float temperature_MOS, temperature_PCB;
  
  // Gesamtbatterie-Daten
  float packVoltage;
  float current;
  uint16_t stateOfCharge;
  
  // Status
  bool balanceStatus, chargeMOS, dischargeMOS;
  
  // Kapazität
  float physicalAH, remainingAH;
  float totalDischargeAH, totalChargeAH;
  
  // Runtime
  uint32_t runtimeSeconds;
  uint16_t totalCycles;
  
  bool valid;
  
  BMSStateData() : valid(false), numCells(0) {}
};

// Struct für Spannungsschutzparameter
struct BMSVoltageParams {
  // Schutzparameter Spannung
  float cellOVProt, cellOVRec, cellOVProt2, cellOVRec2;
  float packOVProt, packOVRec;
  float cellUVProt, cellUVRec, cellUVProt2, cellUVRec2;
  float packUVProt, packUVRec;
  float cellDiffProt, cellDiffRec;
  
  // Warnparameter
  float cellOVWarn, cellOVWarnRec, packOVWarn, packOVWarnRec;
  float cellUVWarn, cellUVWarnRec, packUVWarn, packUVWarnRec;
  float cellDiffWarn, cellDiffWarnRec;
  
  bool valid;
  
  BMSVoltageParams() : valid(false) {}
};

// Struct für Temperaturschutzparameter
struct BMSTemperatureParams {
  // Schutzparameter
  float chgHTProt, chgHTRec, disCHGHTProt, disCHGHTRec;
  float mosHTProt, mosHTRec;
  float chgLTProt, chgLTRec, disCHGLTProt, disCHGLTRec;
  
  // Warnparameter
  float chgHTWarn, chgHTWarnRec, disCHGHTWarn, disCHGHTWarnRec;
  float mosHTWarn, mosHTWarnRec;
  float chgLTWarn, chgLTWarnRec, disCHGLTWarn, disCHGLTWarnRec;
  
  bool valid;
  
  BMSTemperatureParams() : valid(false) {}
};

// Struct für Stromschutzparameter
struct BMSCurrentParams {
  // Schutzparameter
  float chgOCProt, disCHGOCProt, disCHGOCProt2, scProt;
  uint16_t chgOCDelay, disCHGOCDelay, disCHGOCDelay2, scDelay;
  
  // Warnparameter
  float chgOCWarn, chgOCWarnRec, disCHGOCWarn, disCHGOCWarnRec;
  
  // SOC-Parameter 
  uint16_t socLowLV1Warn, socLowLV2Warn;
  
  bool valid;
  
  BMSCurrentParams() : valid(false) {}
};

// Struct für Balancing-Parameter
struct BMSBalanceParams {
  float balLimitV, balStartV;     // Spannugnsgrenzen für Balancing
  float balDiffOn, balDiffOff;    // Spannungsdifferenz für Ein-/Ausschalten
  uint16_t balCur, balChgCur;     // Balancing-Strom-Parameter
  
  bool valid;
  
  BMSBalanceParams() : valid(false) {}
};

// =============================================================================
// Haupt-AntBMS-Klasse für die Kommunikation
// =============================================================================

class AntBMS {
public:
  // Initialisierung der Schnittstelle mit den Pins und Baudrate
  AntBMS(uint8_t rxPin = 20, uint8_t txPin = 21, uint32_t baudRate = 19200);
  
  // Initialisierung und Beendigung der Kommunikation
  bool begin();
  void end();
  
  // Methoden zum Lesen verschiedener Datentypen
  bool readStateData();               // Statusinformationen lesen
  bool readVoltageParams();           // Spannungsparameter lesen
  bool readTemperatureParams();       // Temperaturparameter lesen
  bool readCurrentParams();           // Stromparameter lesen
  bool readBalanceParams();           // Balancing-Parameter lesen
  bool readAllParams();               // Alle Parameter auf einmal lesen
  

  // Steuerung
  bool SaveApply();                   // SaveApply
  bool AutoBalON();                   // AutoBalance ON
  bool AutoBalOFF();                  // AutoBalance OFF
  bool FactoryReset();                // Factory Reset

  // Funktionsdeklaration MOS-Steuerung
  bool setControl(const char input);
  
  
  // Methoden für Datenzugriff (nur lesend)
  const BMSStateData& getStateData() const { return stateData_; }
  const BMSVoltageParams& getVoltageParams() const { return voltageParams_; }
  const BMSTemperatureParams& getTemperatureParams() const { return temperatureParams_; }
  const BMSCurrentParams& getCurrentParams() const { return currentParams_; }
  const BMSBalanceParams& getBalanceParams() const { return balanceParams_; }
  
  // Ausgabemethoden für verschiedene Datentypen
  void printStateData() const;
  void printVoltageParams() const;
  void printTemperatureParams() const;
  void printCurrentParams() const;
  void printBalanceParams() const;
  void printAllData() const;

  // Anzahl der Zellen konfigurieren
  void configureBMSCells();
  
  // Sicherheitsprüfungen für Parameter
  bool checkVoltageParamsSafety() const;
  bool checkTemperatureParamsSafety() const;
  bool checkCurrentParamsSafety() const;
  bool checkBalanceParamsSafety() const;
  bool checkAllParamsSafety() const;
  
  // Konfigurationsmethoden
  void setTimeout(unsigned long timeout) { timeout_ = timeout; }      // Timeout setzen
  void setDebugMode(bool debug) { debugMode_ = debug; }               // Debug-Modus aktivieren
  
  // Status
  bool isConnected() const { return connected_; }                     // Verbindungsstatus
  unsigned long getLastReadTime() const { return lastReadTime_; }     // Letzte Lesezeit
  
private:
  // Hardware konfiguration
  uint8_t rxPin_, txPin_;                 // UART-Pins
  uint32_t baudRate_;                     // Übertragungsgeschwindigkeit
  HardwareSerial* serial_;                // Zeiger auf serielle Schnittstelle
  bool connected_;                        // Verbindungsstatus
  unsigned long timeout_;                 // Timeout für Antworten
  bool debugMode_;                        // Debug-Modus aktiviert
  unsigned long lastReadTime_;            // Zeitstempel der letzten Datenübertragung
  
  // Datenspeicher für BMS-Informationen
  BMSStateData stateData_;
  BMSVoltageParams voltageParams_;
  BMSTemperatureParams temperatureParams_;
  BMSCurrentParams currentParams_;
  BMSBalanceParams balanceParams_;
  
  // Kommunikationspuffer
  uint8_t rxBuffer_[256];                 // Empfangspuffer
  int rxIndex_;                           // Index im Empfangspuffer
  
  
  // Hilfsmethoden für Datenverarbeitung
  uint16_t calculateCRC16_TX(const uint8_t* data, int length) const;    // CRC für gesendete Daten
  uint16_t calculateCRC16_RX(const uint8_t* data, int length) const;    // CRC für empfangene Daten
  uint16_t readUint16LE(const uint8_t* data, int offset) const;         // 16-Bit Wert lesen (Little Endian)
  uint32_t readUint32LE(const uint8_t* data, int offset) const;         // 32-Bit Wert lesen (Little Endian)
  void printHex(const uint8_t* data, int length) const;                 // Hexadezimale Ausgabe für Debug
  
  // Kommunikationsmethoden
  bool sendCommand(const uint8_t* command, int cmdLength, uint8_t* response, int* respLength);
  bool buildAndSendCommand(uint8_t functionCode, uint16_t address, uint8_t dataLength);

  // Zellanzahl-Konfiguration
  bool buildAndSendCommandBMSCell(uint8_t functionCode, uint16_t address, uint8_t dataLength, uint8_t cell_num);
  
  // Parser-Methoden
  bool parseStateResponse(const uint8_t* response, int length);
  bool parseVoltageParams(const uint8_t* response, int length);
  bool parseTemperatureParams(const uint8_t* response, int length);
  bool parseCurrentParams(const uint8_t* response, int length);
  bool parseBalanceParams(const uint8_t* response, int length);

  // Parser für Steuerung
  bool parseSetConResponse(const uint8_t* response, int length, const char input);

  // Parser für Zellanzahl-Konfiguration
  bool parseConfigureBMSCells(const uint8_t* response, int length, uint16_t cell_num);
  
  // Parser für Einstellungen speichern
  bool parseSave(const uint8_t* response, int length);

  // Parser für Balancing-Steuerung
  bool parseAutoBalON(const uint8_t* response, int length);
  bool parseAutoBalOFF(const uint8_t* response, int length);

 // Validierungsmethoden für empfangene Daten
  bool validateFrame(const uint8_t* response, int length, int expectedMinLength) const;
  bool validateCRC(const uint8_t* response, int length) const;
  bool validateSetControl(const uint8_t* response, int length) const;
};

// =============================================================================
// Implementierung 
// =============================================================================

//initialisiert alle Variablen mit Standardwerten
AntBMS::AntBMS(uint8_t rxPin, uint8_t txPin, uint32_t baudRate)
  : rxPin_(rxPin), txPin_(txPin), baudRate_(baudRate), 
    serial_(nullptr), connected_(false), timeout_(1000), 
    debugMode_(false), lastReadTime_(0), rxIndex_(0) {
  serial_ = new HardwareSerial(1);
}

// Initialisierung der seriellen Kommunikation
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

// Beendigung der Kommunikation
void AntBMS::end() {
  if (serial_) {
    serial_->end();
    connected_ = false;
  }
}

// Zustandsdaten vom BMS abrufen
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

// Einstellungen speichern
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

// Werksreset durchführen
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

// Automatisches Balancing einschalten
bool AntBMS::AutoBalON() {
  if (!connected_) return false;
  
  if (debugMode_) Serial.println("AutoBalance ON...");
  
  bool success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_BAL_ON, 
                                    AntBMSProtocol::LENGTH_CONTROL);
  if (success) {
    lastReadTime_ = millis();
  }
  return success;
}

// Automatisches Balancing ausschalten
bool AntBMS::AutoBalOFF() {
  if (!connected_) return false;
  
  if (debugMode_) Serial.println("AutoBalance OFF...");
  
  bool success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_BAL_OFF, 
                                    AntBMSProtocol::LENGTH_CONTROL);
  if (success) {
    lastReadTime_ = millis();
  }
  return success;
}

// Spannungsparameter lesen
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

// Temperaturparameter lesen
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

// Stromparameter lesen
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

// Balancing-Parameter lesen
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

// Alle Parameter nacheinander lesen
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

// MOS-Steuerung basierend auf Benutzereingabe
bool AntBMS::setControl(const char input) {
  if (!connected_) return false;
  
  if (debugMode_) Serial.println("Set Charge Mode...");
  
  bool success;

  switch (input)
  {
  case 'c':     // Laden einschalten
    success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_CON_CHG_MOS_ON, 
                                    AntBMSProtocol::LENGTH_CONTROL);
    Serial.println("Ladevorgang gestartet.");
    break;
  case 'x':     // Laden ausschalten
    success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_CON_CHG_MOS_OFF, 
                                    AntBMSProtocol::LENGTH_CONTROL);
    Serial.println("Ladevorgang gestoppt.");
    break;
  case 'd':     // Entladen einschalten
    success = buildAndSendCommand(AntBMSProtocol::CONTROL_SET, 
                                    AntBMSProtocol::ADR_CON_DIS_MOS_ON, 
                                    AntBMSProtocol::LENGTH_CONTROL);
    Serial.println("Batterie steht für die Nutzung zur Verfügung.");
    break;
  case 'f':     // Entladen ausschalten
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


// Hilfsmethoden-Implementierung
// CRC16-Berechnung für gesendete Daten
uint16_t AntBMS::calculateCRC16_TX(const uint8_t* data, int length) const {
  uint16_t crc = 0xFFFF;            // Startert
  
  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos];     // XOR mit aktuellem Byte
    
    // 8 Bit-Shifts für CRC-Polynom
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;      // Modbus-Polynom    
      } else {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}

// CRC16-Berechnung für empfangene Daten
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

// 16-Bit Wert im Little-Endian-Format lesen
uint16_t AntBMS::readUint16LE(const uint8_t* data, int offset) const {
  return data[offset] | (data[offset + 1] << 8);
}

// 32-Bit Wert im Little-Endian-Format lesen
uint32_t AntBMS::readUint32LE(const uint8_t* data, int offset) const {
  return data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16) | (data[offset + 3] << 24);
}


// Hexadezimale Ausgabe für Debug
void AntBMS::printHex(const uint8_t* data, int length) const {
  if (!debugMode_) return;
  
  for (int i = 0; i < length; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// Standardkommando erstellen und senden
bool AntBMS::buildAndSendCommand(uint8_t functionCode, uint16_t address, uint8_t dataLength) {
  // Datenarray für CRC-Berechnung (ohne Start- und Endbytes)
  uint8_t data[5] = {
    AntBMSProtocol::START2,
    functionCode,
    (uint8_t)(address & 0xFF),        // Niedrigwertiges Byte der Adresse
    (uint8_t)(address >> 8),          // Höherwertiges Byte der Adresse
    dataLength
  };
  
  // CRC berechnen und in Low/High-Bytes aufteilen
  uint16_t crc = calculateCRC16_TX(data, 5);
  uint8_t crcLow = crc & 0xFF;
  uint8_t crcHigh = (crc >> 8) & 0xFF;
  
  // Vollständiges Kommando zusammenstellen
  uint8_t command[10] = {
    AntBMSProtocol::START1, AntBMSProtocol::START2,       // Startbytes
    functionCode,
    (uint8_t)(address & 0xFF), (uint8_t)(address >> 8),   // Adresse
    dataLength,
    crcLow, crcHigh,                                      // CRC-Prüfsumme
    AntBMSProtocol::END1, AntBMSProtocol::END2            // Endbytes
  };
  
  uint8_t response[256];    // Puffer für Antwort
  int responseLength = 0;
  
  if (sendCommand(command, 10, response, &responseLength)) {
    // Bestimme welcher Parser verwendet werden soll basierend auf Funktionscode und Adresse
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
        Serial.println("Werksreset wurde erfolgreich durchgeführt!");
        return true;
      } else if (address == AntBMSProtocol::ADR_BAL_ON){
        return parseAutoBalON(response, responseLength);
      } else if (address == AntBMSProtocol::ADR_BAL_OFF){
        return parseAutoBalOFF(response, responseLength);
      }
    }
  }
  return false;
}

// Spezielles Kommando für Zellanzahl-Konfiguration
bool AntBMS::buildAndSendCommandBMSCell(uint8_t functionCode, uint16_t address, uint8_t dataLength, uint8_t cell_num) {

  uint8_t response[256];
  int responseLength = 0;

  // Datenarray für CRC-Berechnung (inklusive Zellanzahl)
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
  
  // Vollständiges Kommando für Zellkonfiguration
  uint8_t commandCell[12] = {
    AntBMSProtocol::START1, AntBMSProtocol::START2,
    functionCode,
    (uint8_t)(address & 0xFF), (uint8_t)(address >> 8),
    dataLength, 
    cell_num, 0x00,
    crcLow, crcHigh,
    AntBMSProtocol::END1, AntBMSProtocol::END2
  };

  Serial.println("Sende Zellkonfiguration...");

  if (sendCommand(commandCell, 12, response, &responseLength)){
        return parseConfigureBMSCells(response, responseLength, cell_num);
  }
  
}

// Kommando senden und auf Antwort warten
bool AntBMS::sendCommand(const uint8_t* command, int cmdLength, uint8_t* response, int* respLength) {
  if (!serial_ || !connected_) return false;
  
  // Lösche wartende Daten im Empfangspuffer
  while (serial_->available()) {
    serial_->read();
  }
  
  // Sende Kommando
  serial_->write(command, cmdLength);
  
  if (debugMode_) {
    Serial.print("Sent: ");
    printHex(command, cmdLength);
  }
  
  // Warte auf Antwort
  rxIndex_ = 0;
  memset(rxBuffer_, 0, sizeof(rxBuffer_));
  unsigned long startTime = millis();
  bool responseEnded = false;
  
  // Empfange Daten bis Timeout oder vollständige Antwort
  while (millis() - startTime < timeout_ && !responseEnded) {
    if (serial_->available() > 0) {
      rxBuffer_[rxIndex_] = serial_->read();
      rxIndex_++;
      startTime = millis();         // Reset Timeout bei neuen Daten
      
      if (rxIndex_ >= sizeof(rxBuffer_)) {
        if (debugMode_) Serial.println("Warnung: Pufferüberlauf verhindert");
        break;
      }
      
      // Prüfe auf Ende des Rahmens (AA 55)
      if (rxIndex_ >= 2 && rxBuffer_[rxIndex_-2] == AntBMSProtocol::END1 && 
          rxBuffer_[rxIndex_-1] == AntBMSProtocol::END2) {
        responseEnded = true;
      }
    }
    else if (rxIndex_ > 0 && millis() - startTime > 100) {
      responseEnded = true;       // Timeout für unvollständige Antworten
    }
  }
  
  // Verarbeite empfangene Daten
  if (rxIndex_ > 0) {
    memcpy(response, rxBuffer_, rxIndex_);
    *respLength = rxIndex_;
    
    if (debugMode_) {
      Serial.print("Empfangen: ");
      printHex(response, *respLength);
    }
    
    return validateCRC(response, *respLength);
  }
  
  if (debugMode_) Serial.println("Keine Antwort empfangen");
  return false;
}

// Framevalidierung - prüft Struktur und Länge
bool AntBMS::validateFrame(const uint8_t* response, int length, int expectedMinLength) const {
  if (length < expectedMinLength || 
      response[0] != AntBMSProtocol::START1 || 
      response[1] != AntBMSProtocol::START2) {
    if (debugMode_) {
      Serial.printf("Ungültiges Frame: Länge=%d, header=0x%02X%02X\n", 
                    length, response[0], response[1]);
    }
    return false;
  }
  
  if (response[length-2] != AntBMSProtocol::END1 || 
      response[length-1] != AntBMSProtocol::END2) {
    if (debugMode_) {
      Serial.printf("Ungültige Endmarkierungen: 0x%02X%02X (erwartet 0xAA55)\n", 
                    response[length-2], response[length-1]);
    }
    return false;
  }
  
  return true;
}

// CRC-Validierung für empfangene Daten
bool AntBMS::validateCRC(const uint8_t* response, int length) const {
  if (length < 8) return false;
  
  uint8_t dataLength = response[5];
  uint16_t calculatedCRC = calculateCRC16_RX(response, dataLength + 6);
  
  uint8_t receivedCRC_Low = response[dataLength + 6];
  uint8_t receivedCRC_High = response[dataLength + 7];
  uint16_t receivedCRC = (receivedCRC_High << 8) | receivedCRC_Low;
  
  if (debugMode_) {
    Serial.printf("Empfangene CRC-16_RX: 0x%04X\n", receivedCRC);
  }
  
  if (calculatedCRC == receivedCRC) {
    if (debugMode_) Serial.println("CRC-Check erfolgreich - CRC korrekt");
    return true;
  } else {
    if (debugMode_) {
      Serial.printf("CRC-Check fehlgeschlagen - CRC falsch (Berechnet: 0x%04X, Empfangen: 0x%04X)\n", 
                    calculatedCRC, receivedCRC);
    }
    return false;
  }
}

// Parser-Implementierungen

// Parse State Antwort
bool AntBMS::parseStateResponse(const uint8_t* response, int length) {
  if (!validateFrame(response, length, 146)) return false;
  
  stateData_ = BMSStateData(); // Reset
  
  if (debugMode_) {
    Serial.printf("Gültige AntBMS Zustandsantwort: Cmd=0x%02X, Datalänge=%d\n", 
                  response[2], response[5]);
  }
  
  // Anzahl der Zellen aus Byte 9 lesen
  int numCells =response[9];
  Serial.print("numCell ist: ");
  Serial.println(numCells);
  
  // Parse Zellspannungen (Bytes 34-55 bzw. je nach Zellanzahl länger oder kürzer)
  stateData_.numCells = numCells;
  for (int i = 0; i < numCells ; i++) {
    uint16_t rawVoltage = readUint16LE(response, 34 + i * 2);
    stateData_.cellVoltages[i] = rawVoltage * 0.001;      // Umwandlung in Volt
  }
  
  // Parse Temperaturen (nach den Zellspannungen)
  stateData_.temperature_T1 = readUint16LE(response, 34 + numCells*2);
  stateData_.temperature_T2 = readUint16LE(response, 36 + numCells*2);
  stateData_.temperature_T3 = readUint16LE(response, 38 + numCells*2);
  stateData_.temperature_T4 = readUint16LE(response, 40 + numCells*2);
  stateData_.temperature_MOS = readUint16LE(response, 42 + numCells*2);
  stateData_.temperature_PCB = readUint16LE(response, 44 + numCells*2);
  
  // Parse Gesamtbatterie-Daten
  stateData_.packVoltage = readUint16LE(response, 46 + numCells*2) * 0.01;
  stateData_.current = (int16_t)readUint16LE(response, 48 + numCells*2);
  stateData_.stateOfCharge = readUint16LE(response, 50 + numCells*2);
  
  // Parse Systemstatus
  stateData_.balanceStatus = (response[53 + numCells*2] == 0x01);
  stateData_.chargeMOS = (response[54 + numCells*2] == 0x01);
  stateData_.dischargeMOS = (response[55 + numCells*2] == 0x01);
  
  // Parse Kapazitäten (in Amperestunden)
  stateData_.physicalAH = readUint32LE(response, 58 + numCells*2) * 0.000001;
  stateData_.remainingAH = readUint32LE(response, 62 + numCells*2) * 0.000001;
  stateData_.runtimeSeconds = readUint32LE(response, 74 + numCells*2);
  
  // Parse Spanungsgrenzen
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

// Parse Spannungsparameter-Antwort
bool AntBMS::parseVoltageParams(const uint8_t* response, int length) {
  if (!validateFrame(response, length, 68)) return false;
  
  voltageParams_ = BMSVoltageParams();        // Zurücksetzen
  
  // Parse Schutzparameter (alle Werte in Volt)
  voltageParams_.cellOVProt = readUint16LE(response, 6) * 0.001;     
  voltageParams_.cellOVRec = readUint16LE(response, 8) * 0.001;       
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
  
  // Parse Warnparameter
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

// Parse Temperaturparameter-Antwort
bool AntBMS::parseTemperatureParams(const uint8_t* response, int length) {
  if (!validateFrame(response, length, 60)) return false;
  
  temperatureParams_ = BMSTemperatureParams(); // Reset
  
  // Parse Schutzparameter (alle Werte in Grad Celsius)
  temperatureParams_.chgHTProt = readUint16LE(response, 6);
  temperatureParams_.chgHTRec = readUint16LE(response, 8);
  temperatureParams_.disCHGHTProt = readUint16LE(response, 10);
  temperatureParams_.disCHGHTRec = readUint16LE(response, 12);
  temperatureParams_.mosHTProt = readUint16LE(response, 14);
  temperatureParams_.mosHTRec = readUint16LE(response, 16);
  
  // Kälteschutz (kann negative Werte haben)
  temperatureParams_.chgLTProt = (int16_t)readUint16LE(response, 18);
  temperatureParams_.chgLTRec = readUint16LE(response, 20);
  temperatureParams_.disCHGLTProt = (int16_t)readUint16LE(response, 22);
  temperatureParams_.disCHGLTRec = (int16_t)readUint16LE(response, 24);
  
  // Parse Warnparameter
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

// Parse Stromparameter-Antwort
bool AntBMS::parseCurrentParams(const uint8_t* response, int length) {
  if (!validateFrame(response, length, 48)) return false;
  
  currentParams_ = BMSCurrentParams();        // Zurücksetzen
  
  // Parse Schutzparameter
  currentParams_.chgOCProt = readUint16LE(response, 6) * 0.1;
  currentParams_.chgOCDelay = readUint16LE(response, 8);
  currentParams_.disCHGOCProt = readUint16LE(response, 10) * 0.1;
  currentParams_.disCHGOCDelay = readUint16LE(response, 12);
  currentParams_.disCHGOCProt2 = readUint16LE(response, 14) * 0.1;
  currentParams_.disCHGOCDelay2 = readUint16LE(response, 16);
  currentParams_.scProt = readUint16LE(response, 18);
  currentParams_.scDelay = readUint16LE(response, 20);
  
  // Parse Warnparameter
  currentParams_.chgOCWarn = readUint16LE(response, 26) * 0.1;
  currentParams_.chgOCWarnRec = readUint16LE(response, 28) * 0.1;
  currentParams_.disCHGOCWarn = readUint16LE(response, 30) * 0.1;
  currentParams_.disCHGOCWarnRec = readUint16LE(response, 32) * 0.1;
  
  // Parse SOC-Parameter (Ladezustand in Prozent)
  currentParams_.socLowLV1Warn = readUint16LE(response, 34);
  currentParams_.socLowLV2Warn = readUint16LE(response, 36);
  
  currentParams_.valid = true;
  return true;
}

// Parse Balancing-Parameter-Antwort
bool AntBMS::parseBalanceParams(const uint8_t* response, int length) {
  if (!validateFrame(response, length, 28)) return false;
  
  balanceParams_ = BMSBalanceParams();          // Reset
  
  // Parse Balancing-Parameter
  balanceParams_.balLimitV = readUint16LE(response, 6) * 0.001;       // Grenzspannung in Volt
  balanceParams_.balStartV = readUint16LE(response, 8) * 0.001;       // Startspannung
  balanceParams_.balDiffOn = readUint16LE(response, 10) * 0.001;      // Einschalt-Differenz
  balanceParams_.balDiffOff = readUint16LE(response, 12) * 0.001;     // Ausschalt-Differenz
  balanceParams_.balCur = readUint16LE(response, 14);                 // Balancing-Strom
  balanceParams_.balChgCur = readUint16LE(response, 16);              // Lade-Grenzstrom
  
  balanceParams_.valid = true;
  return true;
}

// Parse Steuerungskommando-Antwort
bool AntBMS::parseSetConResponse(const uint8_t* response, int length, const char input) {
  if (!validateFrame(response, length, 12)) return false;
  
  if (debugMode_) {
    Serial.printf("Gültige AntBMS Steuerungsantwort: Cmd=0x%02X, Datalänge=%d\n", 
                  response[2], response[5]);
  }
  
  // Prüfe Framestruktur für erfolgreiche Steuerung
  if(response[2] == 0x61 && response[5] == 0x02 && response[6] == 0x01){
    Serial.println("Steuerungskommando erfolgreich validiert.");
    return true;
  }
  

  return true;
}

// Parse Zellkonfiguration-Antwort
bool AntBMS::parseConfigureBMSCells(const uint8_t* response, int length, uint16_t cell_num){
  if (!validateFrame(response, length, 12)) return false;
  
  if (debugMode_) {
    Serial.printf("Zellkonfiguration-Antwort: Cmd=0x%02X, Datalänge=%d\n", 
                  response[2], response[5]);
  }
  
  // Prüfe erfolgreiche Zellkonfiguration
  if(response[2] == 0x42 && response[5] == 0x02 && response[6] == (uint8_t) cell_num && response[7] == 0x00){
    Serial.println("BMS-Zellkonfiguration war erfolgreich.");
    return true;
  }
  

  return true;
}

// Parse Speichern-Antwort
bool AntBMS::parseSave(const uint8_t* response, int length){
  if (!validateFrame(response, length, 12)) return false;
  
  if (debugMode_) {
    Serial.printf("Einstellungen wurden gespeichert: Cmd=0x%02X, Datalänge=%d\n", 
                  response[2], response[5]);
  }
  
  // Prüfe erfolgreiche Speicherung
  if(response[2] == 0x61 && response[5] == 0x02 && response[6] == 0x01 && response[7] == 0x00){
    Serial.println("Einstellungen erfolgreich gespeichert!");
    return true;
  }
  

  return true;
}

// Parse Balancing EIN-Antwort
bool AntBMS::parseAutoBalON(const uint8_t* response, int length){
  if (!validateFrame(response, length, 12)) return false;
  
  if (debugMode_) {
    Serial.printf("AutoBalancing EIN: Cmd=0x%02X, Datenlänge=%d\n", 
                  response[2], response[5]);
  }
  
  // Prüfe erfolgreiche Aktivierung
  if(response[2] == 0x61 && response[5] == 0x02 && response[6] == 0x01 && response[7] == 0x00){
    Serial.println("AutoBalance ist eingeschaltet!");
    return true;
  }
  

  return true;
}

// Parse Balancing AUS-Antwort
bool AntBMS::parseAutoBalOFF(const uint8_t* response, int length){
  if (!validateFrame(response, length, 12)) return false;
  
  if (debugMode_) {
    Serial.printf("AutoBalancing AUS: Cmd=0x%02X, Datalänge=%d\n", 
                  response[2], response[5]);
  }
  
  // Prüfe erfolgreiche Deaktivierung
  if(response[2] == 0x61 && response[5] == 0x02 && response[6] == 0x01 && response[7] == 0x00){
    Serial.println("AutoBalance ist ausgeschaltet!");
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
            // Debug-Ausgabe der Hexwerte
            Serial.print("Dezimal: ");
            Serial.print(cell_num);
            Serial.print(" -> Hex: 0x");
            if(cell_num < 0x10) Serial.print("0");
            Serial.print(cell_num, HEX);
            
            // Sende Zellkonfigurationskommando
            // 7E A1 22 9A 00 02 cell_num_low cell_num_high CRC_L CRC_H AA 55
            bool success = buildAndSendCommandBMSCell(AntBMSProtocol::PARAM_SET, 
                                            AntBMSProtocol::ADR_CELL_NUM,
                                            AntBMSProtocol::LENGTH_CELL_NUM, cell_num);
            
            
            Serial.print("Zellanzahl erfolgreich auf ");
            Serial.print(cell_num);
            Serial.println(" gesetzt.");                                

            // Speichere die Einstellungen
            if(SaveApply()){
              Serial.println("Einstellungen konnten gespeichert werden!");
            } else {
              Serial.println("Einstellungen konnten nicht gespeichert werden!");
            }
            
        } else if(cell_num < 10 || cell_num > 24) {
            Serial.println("Anzahl der Zellen konnte nicht eingegeben werden.");
            Serial.println("Bitte nochmal versuchen.");
            
            int retry_count = 0;
            const int max_retries = 3;
            bool valid_input = false;
            
            // Bis zu 3 Wiederholungsversuche
            while(retry_count < max_retries && !valid_input) {
                Serial.print("Versuch ");
                Serial.print(retry_count + 1);
                Serial.print(" von ");
                Serial.print(max_retries);
                Serial.println(": Bitte die gewünschte Anzahl der Zellen für das BMS eingeben (min. 10 und max. 24):");
                
                // Warten auf Eingabe
                while(Serial.available() == 0) {
                    delay(10); 
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
                    
                    Serial.print("Zellanzahl erfolgreich auf ");
                    Serial.print(cell_num);
                    Serial.println(" gesetzt.");
                    valid_input = true;

                    // Sende Zellkonfigurationskommando
                    bool success = buildAndSendCommandBMSCell(AntBMSProtocol::PARAM_SET, 
                                            AntBMSProtocol::ADR_CELL_NUM,
                                            AntBMSProtocol::LENGTH_CELL_NUM, cell_num_hex);
            
                    if(success){
                      Serial.println("Konfiguration erfolgreich.");
                    } else {
                      Serial.println("Konfiguration nicht erfolgreich!");
                    }
                    
                    Serial.print("Zellanzahl erfolgreich auf ");
                    Serial.print(cell_num);
                    Serial.println(" gesetzt.");    

                    // Speichere die Einstellungen
                    if(SaveApply()){
                      Serial.println("Einstellungen konnten gespeichert werden!");
                    } else {
                      Serial.println("Einstellungen konnten nicht gespeichert werden!");
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
        
        // Lese alle Parameter und zeige sie an
        if (readAllParams()) {
            printAllData();
            checkAllParamsSafety();
        }
    }
}

/////// Ausgabemethoden-Implementierung ///////

// Ausgabe der Zustandsdaten
void AntBMS::printStateData() const {
  if (!stateData_.valid) {
    Serial.println("Keine gültigen Zustandsdaten verfügbar");
    return;
  }
  
  Serial.println("\n=========== AntBMS STATUS-INFORMATIONEN ===========");
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
    delay(500);     // Kurze Verzögerung für bessere Lesbarkeit
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

// Ausgabe der Spannungsparameter
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

// Ausgabe der Temperaturparameter
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

// Ausgabe der Stromparameter
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

// Ausgabe der Balancing-Parameter
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

// Ausgabe aller Daten
void AntBMS::printAllData() const {
  printStateData();
  printVoltageParams();
  printTemperatureParams();
  printCurrentParams();
  printBalanceParams();
}

///// Sicherheitsprüfungen-Implementierung /////

// Prüfung der Spannungsparameter auf sichere Bereiche
bool AntBMS::checkVoltageParamsSafety() const {
  if (!voltageParams_.valid) return false;
  
  bool safe = true;
  // Prüfe Überspannungsschutz (typisch 4.0-4.3V für Li-Ion)
  if (voltageParams_.cellOVProt > 4.3 || voltageParams_.cellOVProt < 4.0) {
    Serial.printf("WARNUNG: Ungewöhnlicher CellOVProt: %.3fV\n", voltageParams_.cellOVProt);
    safe = false;
  }
  
  // Prüfe Unterspannungsschutz (typisch 2.5-3.2V für Li-Ion)
  if (voltageParams_.cellUVProt < 2.5 || voltageParams_.cellUVProt > 3.2) {
    Serial.printf("WARNUNG: Ungewöhnlicher CellUVProt: %.3fV\n", voltageParams_.cellUVProt);
    safe = false;
  }
  
  // Spannungsparameter liegen im sicheren Bereich
  if (safe) Serial.println("Voltage parameters are within safe range");
  return safe;
}

// Prüfung der Temperaturparameter
bool AntBMS::checkTemperatureParamsSafety() const {
  if (!temperatureParams_.valid) return false;
  
  bool safe = true;

  // Prüfe Überhitzungsschutz (typisch 40-70°C)
  if (temperatureParams_.chgHTProt > 70 || temperatureParams_.chgHTProt < 40) {
    Serial.printf("WARNUNG: Ungewöhnlicher CHGHTProt: %.0f°C\n", temperatureParams_.chgHTProt);
    safe = false;
  }
  
  if (safe) Serial.println("Temperaturparameter liegen im sicheren Bereich");
  return safe;
}

// Prüfung der Stromparameter
bool AntBMS::checkCurrentParamsSafety() const {
  if (!currentParams_.valid) return false;
  
  bool safe = true;
  
  // Prüfe Ladestromschutz (typisch 10-200A je nach Batterie)
  if (currentParams_.chgOCProt > 200 || currentParams_.chgOCProt < 10) {
    Serial.printf(" WARNUNG: Ungewöhnlicher CHGOCProt: %.1fA\n", currentParams_.chgOCProt);
    safe = false;
  }
  
  if (safe) Serial.println("Stromparameter liegen im sicheren Bereich");
  return safe;
}

// Prüfung der Balancing-Parameter
bool AntBMS::checkBalanceParamsSafety() const {
  if (!balanceParams_.valid) return false;
  
  bool safe = true;
  
  // Prüfe Balancing-Grenzspannung (typisch 4.0-4.3V)
  if (balanceParams_.balLimitV > 4.3 || balanceParams_.balLimitV < 4.0) {
    Serial.printf("WARNUNG: Ungewöhnlicher BalLimitV: %.3fV\n", balanceParams_.balLimitV);
    safe = false;
  }
  
  if (safe) Serial.println("Balancing-Parameter liegen im sicheren Bereich");
  return safe;
}

// Prüfung aller Parameter auf Sicherheit
bool AntBMS::checkAllParamsSafety() const {
  bool allSafe = true;
  allSafe &= checkVoltageParamsSafety();
  allSafe &= checkTemperatureParamsSafety();
  allSafe &= checkCurrentParamsSafety();
  allSafe &= checkBalanceParamsSafety();
  return allSafe;
}

#endif // ANTBMS_H

// =============================================================================
// HAUPTPROGRAMM - Setup und Loop
// =============================================================================

// Erstelle AntBMS-Instanz mit RX Pin 20, TX Pin 21
AntBMS bms(20, 21); 

void setup() {
  Serial.begin(115200); 

 // Warten auf Eingabe
  while(Serial.available() == 0) {
        delay(10);
  }
 

  // Initialisiere BMS-Kommunikation
  if (bms.begin()) {
    // bms.setDebugMode(true);     // Aktiviere Debug-Modus für detaillierte Ausgaben
    Serial.println("BMS erfolgreich initialisiert.");
  
  } else {
    Serial.println("BMS Initialisierung fehlgeschlagen.");
  }


  // Read all parameters
  if (bms.readAllParams()) {
    bms.printAllData();
    bms.checkAllParamsSafety();
  }

  // Anzahl der Batteriezellen setten 
  bms.configureBMSCells();

}


void loop() {

  Serial.print("Eingabe war: ");
  Serial.flush(); // Puffer leeren, um alte Eingaben zu entfernen
  char input = Serial.read();
  Serial.println(input);
 
  // Warten auf Eingabe
  while(Serial.available() == 0) {
        delay(10);
  }
 
  if (input == 'r' || 'R'){
      Serial.println("Soll das BMS auf Werkeinstellungen zurückgesetzt werden?");
      Serial.print("Eingabe war: ");
      Serial.println("Wenn ja, bitte 'y' eingeben sonstige Eingaben werden nicht berücksichtigt.");
      
      
      char input2 = Serial.read();
      Serial.println(input2);
      if(input2 == 'y' || input2 == 'Y') {
          
          // Führe Werksreset durch
          bms.FactoryReset();

          delay(500);

          // Konfiguriere Anzahl der Batteriezellen neu
          bms.configureBMSCells(); 
      }

  } else if(input == 'b' || 'B'){
       // Balancing-Zustand ändern
       Serial.println("Soll das Balancing-Zustand geändert werden?");
       Serial.println("Wenn ja, bitte 'y' eingeben sonstige Eingaben werden nicht berücksichtigt.");
      
      char input2 = Serial.read();
      Serial.print("Eingabe war: ");
      Serial.println(input2);
      if(input2 == '0') {
          
          // AutoBalance ausschalten
          bms.AutoBalOFF();
          delay(500);
      
      } else if(input2 == '1'){
          
        // AutoBalance einschalten
          bms.AutoBalON();
          delay(500);
      } else{
        Serial.println("Keine passende Eingabe. Zustand nicht geändert!");
      }

  } 

  // Lese aktuelle Zustandsdaten vom BMS
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
