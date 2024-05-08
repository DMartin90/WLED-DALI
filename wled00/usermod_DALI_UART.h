#pragma once
#include <Arduino.h>
#include <wled.h>
#include <HardwareSerial.h>

struct FrameFormat {
    uint8_t deviceTypePresent : 1;
    uint8_t separateAddressBytes : 1;
    uint8_t opcodeCount : 3;  // Number of opcodes, 0 to 7
    uint8_t dtrCount : 2;     // Number of DTR bytes, 0 to 3
    uint8_t reserved : 1;
};

struct ADUData {
    byte transactionType;
    byte sourceAddress;
    FrameFormat frameFormat;
    byte deviceType;
    byte address;
    byte opcode;
    byte dtr[3];  // Maximum of 3 DTR bytes
};

class daliuartusermod: public Usermod {
private:

    HardwareSerial daliSerial{2}; // Use UART 2
    byte packetBuffer[263];
    int bytesRead = 0;
    ADUData aduData;

public:
    void setup() {
        daliSerial.begin(115200, SERIAL_8N1);
        Serial.begin(115200);
    }

    void loop() {
        while (daliSerial.available()) {
            byte incomingByte = daliSerial.read();
            if (bytesRead == 0 && incomingByte != 0x68) {
                continue;  // Ignore bytes until start byte is found
            }
            packetBuffer[bytesRead++] = incomingByte;

            if (bytesRead > 2 && !isForwardFrame(packetBuffer[2])) {
                bytesRead = 0;  // Reset if it's a backward frame
                continue;
            }

            if (bytesRead > 4 && bytesRead == packetBuffer[3] + 5) {
                // If we've received the whole packet
                byte expectedChecksum = packetBuffer[bytesRead - 1];
                if (validateChecksum(&packetBuffer[5], packetBuffer[3], expectedChecksum)) {
                    // If checksum is valid
                    processADU(&packetBuffer[5], packetBuffer[3]);  // Process the ADU
                } else {
                    Serial.println("Checksum error");
                }
                bytesRead = 0;  // Reset for the next packet
            }
        }
    }

    bool isForwardFrame(byte flags) {
        return (flags & 0x80) == 0;  // Check if bit 7 is 0
    }

    bool validateChecksum(const byte* adu, int aduLength, byte expectedChecksum) {
        uint8_t crc = 0x00;  // Initial CRC value
        uint8_t poly = 0xD5;  // Polynomial for CRC-8/DVB-S2

        for (int i = 0; i < aduLength; i++) {
            crc ^= adu[i];
            for (uint8_t j = 8; j > 0; j--) {
                if (crc & 0x80)
                    crc = (crc << 1) ^ poly;
                else
                    crc <<= 1;
            }
        }
        return crc == expectedChecksum;
    }

    void processADU(const byte* adu, byte aduLength) {
        Serial.println("Received ADU:");
        byte address = adu[1];  // Assuming address is at index 1
        byte opcode = adu[2];  // Assuming opcode is at index 2

        if (shouldExecuteCommand(address)) {
            executeWledCommand(opcode);
        }
    }

    bool shouldExecuteCommand(byte address) {
        if (address == 0xFF) {  // Check if the address is broadcast
            Serial.println("Broadcast address detected. Command will be executed.");
            return true;
        } else {
            Serial.print("Address ");
            Serial.print(address, HEX);
            Serial.println(" detected. Command will be executed (for now).");
            return true;  // Temporarily executing all commands for simplicity
        }
    }

    void executeWledCommand(byte opcode) {
        switch (opcode) {
            case 0x05: // RECALL MAX LEVEL
                bri = 255;  // Set WLED brightness to maximum
                colorUpdated(0);
                Serial.println("Set brightness to maximum.");
                break;
            case 0x06: // RECALL MIN LEVEL
                bri = 1;  // Set WLED brightness to minimum
                colorUpdated(0);
                Serial.println("Set brightness to minimum.");
                break;
            case 0x00: // OFF
                bri = 0;  // Turn off WLED
                colorUpdated(0);
                Serial.println("Turned off lights.");
                break;
            default:
                Serial.print("Opcode 0x");
                Serial.print(opcode, HEX);
                Serial.println(" not recognized or not implemented.");
                break;
        }
    };
};

/*extern "C" void registerMod(Usermod **um)
{
    *um = new DaliUARTUserMod();
}*/