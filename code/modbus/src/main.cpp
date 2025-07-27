/**
 * ANT-BMS MODBUS Monitor
 *
 * Kommunikation mit dem ANT-BMS über MODBUS RTU.
 *
 * Hardware: ESP32C3 Mini wemos lolin
 * BMS: ANT-BLE 24BHUB (simuliert mit 11x 1kΩ Widerständen)
 * Protokoll: MODBUS RTU
 * 
 * 
 * date: 2025-03-23
**/

#include <Arduino.h>
#include <HardwareSerial.h>

// GPIO-Pins für UART-Kommunikation mit dem ANT-BMS definieren
#define BMS_RX_PIN 20 // GPIO20 als RX (Empfang vom BMS)
#define BMS_TX_PIN 21 // GPIO21 als TX (Senden zum BMS)

// Hardware Serial für BMS-Kommunikation verwenden
HardwareSerial BMSSerial(1); // UART1 verwenden

// Puffer für empfangene Daten
uint8_t rxBuffer[256];  // Empfangspuffer mit 256 Bytes
int rxIndex = 0;        // Index für aktuelle Position im Empfangspuffer

// Datenspeicher für BMS-Werte
uint16_t cellVoltages[11];  // Array für Spannungen aller 11 Zellen
uint16_t totalVoltage = 0;  // Gesamtspannung des Batteriepakets

// Wake-Up-Sequenz vor MODBUS-Kommandos senden
// Das BMS benötigt diese Sequenz um aus dem Energiesparmodus aufzuwachen
void wakeUpBMS()
{
  // Standard Wake-Up-Sequenz für ANT-BMS
  uint8_t wakeSeq[] = {0xAA, 0x55, 0xAA, 0xFF};
  
  // Wake-Up-Bytes einzeln senden
  for (int i = 0; i < 4; i++)
  {
    BMSSerial.write(wakeSeq[i]);
  }
  
  // Kurze Pause nach Wake-Up
  delay(50);

  // Alle noch vorhandenen Daten im Empfangspuffer löschen
  while (BMSSerial.available())
  {
    BMSSerial.read();
  }
}

// MODBUS CRC16 Prüfsumme berechnen
// CRC16 dient zur Fehlererkennung bei der MODBUS-Kommunikation
uint16_t calculateCRC16(uint8_t *data, int length)
{
  uint16_t crc = 0xFFFF;  // CRC-Startwert für MODBUS
  
  // Über alle Datenbytes iterieren
  for (int i = 0; i < length; i++)
  {
    crc ^= data[i];  // XOR mit aktuellem Datenbyte
    
    // 8 Bits des aktuellen Bytes verarbeiten
    for (int j = 0; j < 8; j++)
    {
      if (crc & 0x0001)  // Wenn niedrigstes Bit gesetzt ist
      {
        crc >>= 1;       // Rechts verschieben
        crc ^= 0xA001;   // XOR mit MODBUS-Polynom
      }
      else
      {
        crc >>= 1;       // Nur rechts verschieben
      }
    }
  }
  return crc;
}

// Funktion zum Schreiben in ein einzelnes Register (MODBUS Funktionscode 0x06)
// Ermöglicht das Setzen von Konfigurationswerten im BMS
// HINWEIS: Diese Funktion ist implementiert, funktioniert aber nicht, da das ANT-BMS 
// nicht auf Schreibkommandos reagiert (vermutlich schreibgeschützt) 
bool writeSingleRegister(uint16_t registerAddress, uint16_t value)
{
  // MODBUS RTU Kommando aufbauen (8 Bytes gesamt)
  uint8_t modbusCmd[8];

  modbusCmd[0] = 0x01;  // Geräteadresse (Slave-ID des BMS)
  modbusCmd[1] = 0x06;  // Funktionscode für "Schreiben in ein einzelnes Register"
  modbusCmd[2] = (registerAddress >> 8) & 0xFF; // Registeradresse High-Byte
  modbusCmd[3] = registerAddress & 0xFF;        // Registeradresse Low-Byte
  modbusCmd[4] = (value >> 8) & 0xFF;   // Wert High-Byte
  modbusCmd[5] = value & 0xFF;          // Wert Low-Byte
  
  // CRC16-Prüfsumme für die ersten 6 Bytes berechnen
  uint16_t crc = calculateCRC16(modbusCmd, 6);
  modbusCmd[6] = crc & 0xFF;        // CRC Low-Byte
  modbusCmd[7] = (crc >> 8) & 0xFF; // CRC High-Byte

  // Kommando für Debugging ausgeben
  Serial.print("Sending: ");
  for (int i = 0; i < 8; i++)
  {
    if (modbusCmd[i] < 16)
      Serial.print("0");  // Führende Null für einstellige Hex-Werte
    Serial.print(modbusCmd[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Empfangspuffer leeren vor dem Senden
  while (BMSSerial.available())
  {
    BMSSerial.read();
  }

  // MODBUS-Kommando byte-weise senden
  for (int i = 0; i < 8; i++)
  {
    BMSSerial.write(modbusCmd[i]);
  }

  // Auf Antwort warten mit Timeout-Überwachung
  rxIndex = 0;
  memset(rxBuffer, 0, sizeof(rxBuffer));  // Empfangspuffer zurücksetzen

  unsigned long startTime = millis();  // Zeitstempel für Timeout
  bool responseEnded = false;          // Flag für vollständige Antwort

  // Antwort empfangen (max. 500ms Timeout)
  while (millis() - startTime < 500 && !responseEnded)
  {
    if (BMSSerial.available() > 0)
    {
      rxBuffer[rxIndex] = BMSSerial.read();  // Byte empfangen
      rxIndex++;
      startTime = millis(); // Timeout zurücksetzen bei neuen Daten

      // Pufferüberlauf verhindern
      if (rxIndex >= sizeof(rxBuffer))
      {
        break;
      }
    }
    else if (rxIndex > 0 && millis() - startTime > 50)
    {
      // Nachricht ist vollständig nach 50ms keine neuen Daten
      responseEnded = true;
    }
  }

  // Empfangene Antwort ausgeben
  Serial.print("Response: ");
  for (int i = 0; i < rxIndex; i++)
  {
    if (rxBuffer[i] < 16)
      Serial.print("0");
    Serial.print(rxBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Prüfen ob Antwort vom korrekten Gerät
  if (rxBuffer[0] != modbusCmd[0])
  {
    Serial.println("Incorrect device address in response");
    return false;
  }

  // Auf MODBUS-Fehlermeldung prüfen (Funktionscode mit gesetztem Bit 7)
  if (rxBuffer[1] & 0x80)
  {
    Serial.print("MODBUS Exception: ");
    Serial.println(rxBuffer[2], HEX);
    return false;
  }

  return true;  
}

// Zellspannungen vom BMS lesen (MODBUS Funktionscode 0x03)
// Liest die Spannungen aller 11 simulierten Batteriezellen
bool readCellVoltages()
{
  Serial.println("Lese Zellspannungen...");

  // MODBUS RTU Kommando zum Lesen von Registern aufbauen
  uint8_t modbusCmd[8];

  modbusCmd[0] = 0x01;  // Geräteadresse (des BMS)
  modbusCmd[1] = 0x03;  // Funktionscode für "Register lesen"
  modbusCmd[2] = 0x00;  // Startregister High-Byte
  modbusCmd[3] = 0x09;  // Startregister Low-Byte (Zelle 1 beginnt bei Adresse 9)
  modbusCmd[4] = 0x00;  // Anzahl Register High-Byte
  modbusCmd[5] = 0x0B;  // Anzahl Register Low-Byte (11 Zellen = 11 Register)
  
  // CRC16-Prüfsumme berechnen
  uint16_t crc = calculateCRC16(modbusCmd, 6);
  modbusCmd[6] = crc & 0xFF;        // CRC Low-Byte
  modbusCmd[7] = (crc >> 8) & 0xFF; // CRC High-Byte

  // Empfangspuffer vor Senden leeren
  while (BMSSerial.available())
  {
    BMSSerial.read();
  }

  // MODBUS-Kommando senden
  for (int i = 0; i < 8; i++)
  {
    BMSSerial.write(modbusCmd[i]);
  }

  // Auf Antwort warten mit Timeout-Behandlung
  rxIndex = 0;
  memset(rxBuffer, 0, sizeof(rxBuffer));

  unsigned long startTime = millis();
  bool responseEnded = false;

  // Antwort empfangen (max. 500ms Timeout)
  while (millis() - startTime < 500 && !responseEnded)
  {
    if (BMSSerial.available() > 0)
    {
      rxBuffer[rxIndex] = BMSSerial.read();
      rxIndex++;
      startTime = millis(); // Timeout bei neuen Daten zurücksetzen

      // Pufferüberlauf verhindern
      if (rxIndex >= sizeof(rxBuffer))
      {
        break;
      }
    }
    else if (rxIndex > 0 && millis() - startTime > 50)
    {
      // Nachricht vollständig nach 50ms Stille
      responseEnded = true;
    }
  }

  // Antwort auf Zellspannungen parsen
  if (rxBuffer[1] == 0x03 && rxBuffer[2] >= 22) 
  { 
    // Funktionscode 0x03 und mindestens 22 Datenbytes (11 Zellen * 2 Bytes)
    for (int i = 0; i < 11; i++) 
    {
      // Jede Zellspannung aus 2 Bytes zusammensetzen (Big-Endian)
      cellVoltages[i] = (rxBuffer[3 + i * 2] << 8) | rxBuffer[4 + i * 2];
    }

    // Zellspannungen formatiert ausgeben
    Serial.println("Zellspannungen:");
    for (int i = 0; i < 11; i++) 
    {
      Serial.print("Zelle ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(cellVoltages[i]);
      Serial.print(" mV (");
      Serial.print(cellVoltages[i] / 1000.0, 3);  // Umrechnung in Volt mit 3 Dezimalstellen
      Serial.println(" V)");
    }

    return true;  
  }
  else
  {
    Serial.println("Ungültiges Zellspannungs-Antwortformat");
    return false;  
  }
}

// Gesamtspannung vom BMS lesen (MODBUS Funktionscode 0x03)
// Liest die Gesamtspannung des Batteriepacks (Summe aller Zellen)
bool readTotalVoltage()
{
  Serial.println("Lese Gesamtspannung...");

  // MODBUS RTU Kommando zum Lesen der Gesamtspannung
  uint8_t modbusCmd[8];

  modbusCmd[0] = 0x01;  // Geräteadresse (des BMS)
  modbusCmd[1] = 0x03;  // Funktionscode für "Register lesen"
  modbusCmd[2] = 0x00;  // Registeradresse High-Byte
  modbusCmd[3] = 0x00;  // Registeradresse Low-Byte (Gesamtspannung bei Adresse 0)
  modbusCmd[4] = 0x00;  // Anzahl Register High-Byte
  modbusCmd[5] = 0x01;  // Anzahl Register Low-Byte (1 Register für Gesamtspannung)
  
  // CRC16-Prüfsumme berechnen
  uint16_t crc = calculateCRC16(modbusCmd, 6);
  modbusCmd[6] = crc & 0xFF;        // CRC Low-Byte
  modbusCmd[7] = (crc >> 8) & 0xFF; // CRC High-Byte

  // Empfangspuffer vor Senden leeren
  while (BMSSerial.available())
  {
    BMSSerial.read();
  }

  // MODBUS-Kommando senden
  for (int i = 0; i < 8; i++)
  {
    BMSSerial.write(modbusCmd[i]);
  }

  // Auf Antwort warten mit Timeout-Behandlung
  rxIndex = 0;
  memset(rxBuffer, 0, sizeof(rxBuffer));

  unsigned long startTime = millis();
  bool responseEnded = false;

  // Antwort empfangen (max. 500ms Timeout)
  while (millis() - startTime < 500 && !responseEnded)
  {
    if (BMSSerial.available() > 0)
    {
      rxBuffer[rxIndex] = BMSSerial.read();
      rxIndex++;
      startTime = millis(); // Timeout bei neuen Daten zurücksetzen

      // Pufferüberlauf verhindern
      if (rxIndex >= sizeof(rxBuffer))
      {
        break;
      }
    }
    else if (rxIndex > 0 && millis() - startTime > 50)
    {
      // Nachricht vollständig nach 50ms Stille
      responseEnded = true;
    }
  }

  // Antwort auf Gesamtspannung parsen
  if (rxBuffer[1] == 0x03 && rxBuffer[2] >= 2)
  {
    // Funktionscode 0x03 und mindestens 2 Datenbytes
    // Gesamtspannung aus 2 Bytes zusammensetzen (Big-Endian)
    totalVoltage = (rxBuffer[3] << 8) | rxBuffer[4];

    // Gesamtspannung formatiert ausgeben
    Serial.print("Gesamtspannung: ");
    Serial.print(totalVoltage*10);    // Wert * 10 für mV-Anzeige
    Serial.print(" mV (");
    Serial.print(totalVoltage / 100.0, 3);  // Umrechnung in Volt mit 3 Dezimalstellen
    Serial.println(" V)");

    return true;  // Erfolgreiche Datenauswertung
  }
  else
  {
    Serial.println("Ungültiges Gesamtspannungs-Antwortformat");
    return false;  // Fehlerhafte Antwort
  }
}


void setup()
{
  // Debug-Serial für Ausgabe auf Monitor initialisieren
  Serial.begin(9600);
  Serial.println("\n\n===== ANT-BMS MODBUS Communication =====");
  
  // BMS-Serial für MODBUS-Kommunikation initialisieren
  BMSSerial.begin(19200, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);

  // Kurze Pause um sicherzustellen dass alles bereit ist
  delay(1000);
}


void loop()
{
  Serial.println("\n======== BMS OPERATIONEN ========");

  // BMS vor Kommandos aufwecken (wichtig für Energiesparmodus)
  wakeUpBMS();
  
  // Aktuelle Zellspannungen lesen
  readCellVoltages();
  delay(100);  // Kurze Pause zwischen Kommandos
  
  // Gesamtspannung lesen
  readTotalVoltage();
  delay(100);  // Kurze Pause zwischen Kommandos
  
  // Werte erneut lesen um Änderungen zu verifizieren
  readCellVoltages();
  delay(100);  // Kurze Pause zwischen Kommandos
  
  // Gesamtspannung erneut lesen
  readTotalVoltage();
  
  Serial.println("\n=================================\n");
  
  // 5 Sekunden warten vor nächstem Zyklus
  delay(5000);
}