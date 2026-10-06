#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#define MCP1_CS   25
#define MCP2_CS   32
#define MCP3_CS   33

#define MCP_MOSI 14
#define MCP_MISO 26
#define MCP_SCK  27

// ============================================================================
// SPI
// ============================================================================

SPIClass mcpSPI(VSPI);

SPISettings mcpSettings(
    500000,
    MSBFIRST,
    SPI_MODE0
);

// ============================================================================
// BLE
// ============================================================================

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;

// ============================================================================
// MEASUREMENT PARAMETERS
// ============================================================================

#define NUM_CHANNELS 24

// Number of measurements used for the average
#define NUM_MEASUREMENTS 16

// Accumulator
uint32_t accumulated[NUM_CHANNELS];

// Final averaged measurements
uint16_t average[NUM_CHANNELS];

// Temporary measurement
uint16_t measurement[NUM_CHANNELS];

// Frame counter
uint32_t frameNumber = 0;


// ============================================================================
// ADC LECTURE
// ============================================================================

uint16_t readMCP3208(uint8_t channel, uint8_t adc_sel)
{
    if (channel > 7)
        return 0;

    uint8_t tx[3];
    uint8_t rx[3];

    tx[0] = 0x06 | ((channel & 0x04) >> 2);
    tx[1] = (channel & 0x03) << 6;
    tx[2] = 0x00;

    mcpSPI.beginTransaction(mcpSettings);

    digitalWrite(adc_sel, LOW);

    mcpSPI.transferBytes(tx, rx, 3);

    digitalWrite(adc_sel, HIGH);

    mcpSPI.endTransaction();

    return ((rx[1] & 0x0F) << 8) | rx[2];
}


// ============================================================================
// READ ALL 24 CHANNELS
// ============================================================================

void readAllChannels(uint16_t values[NUM_CHANNELS])
{
    uint8_t adc_cs[3] = {
        MCP1_CS,
        MCP2_CS,
        MCP3_CS
    };

    uint8_t index = 0;

    for (uint8_t adc = 0; adc < 3; adc++)
    {
        for (uint8_t channel = 0; channel < 8; channel++)
        {
            values[index] = readMCP3208(
                channel,
                adc_cs[adc]
            );

            index++;
        }
    }
}


// ============================================================================
// CALCULATE AVERAGE OF 24 CHANNELS
// ============================================================================

void measureAverage()
{
    // Reset accumulators
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        accumulated[i] = 0;
    }

    // Take NUM_MEASUREMENTS complete measurements
    for (uint32_t n = 0; n < NUM_MEASUREMENTS; n++)
    {
        // Read all 24 channels
        readAllChannels(measurement);

        // Accumulate the 24 channels
        for (uint8_t i = 0; i < NUM_CHANNELS; i++)
        {
            accumulated[i] += measurement[i];
        }
    }

    // Calculate average
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        average[i] =
            accumulated[i] / NUM_MEASUREMENTS;
    }
}


// ============================================================================
// SEND VECTOR THROUGH BLE
// ============================================================================

// ============================================================================
// SEND VECTOR THROUGH BLE
// ============================================================================

void sendMeasurementsBLE()
{
    /*
     * Each ADC measurement is 12 bits.
     * uint16_t is therefore enough to transmit the average.
     *
     * 24 channels × 2 bytes = 48 bytes
     */

    uint8_t packet[NUM_CHANNELS * sizeof(uint16_t)];

    memcpy(
        packet,
        average,
        sizeof(packet)
    );

    pCharacteristic->setValue(
        packet,
        sizeof(packet)
    );

    pCharacteristic->notify();

    frameNumber++;
}


// ============================================================================
// BLE SETUP
// ============================================================================

void setupBLE()
{
    BLEDevice::init("MiniCam");

    BLEServer *pServer =
        BLEDevice::createServer();

    BLEService *pService =
        pServer->createService(SERVICE_UUID);

    pCharacteristic =
        pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY
        );

    pCharacteristic->addDescriptor(
        new BLE2902()
    );

    pService->start();

    BLEAdvertising *pAdvertising =
        BLEDevice::getAdvertising();

    pAdvertising->addServiceUUID(
        SERVICE_UUID
    );

    pAdvertising->setScanResponse(true);

    BLEDevice::startAdvertising();

    Serial.println("BLE started");
    Serial.println("Waiting for connection...");
}


// ============================================================================
// SETUP
// ============================================================================

void setup()
{
    Serial.begin(115200);

    // --------------------------------------------------
    // ADC CHIP SELECT
    // --------------------------------------------------

    pinMode(MCP1_CS, OUTPUT);
    pinMode(MCP2_CS, OUTPUT);
    pinMode(MCP3_CS, OUTPUT);

    digitalWrite(MCP1_CS, HIGH);
    digitalWrite(MCP2_CS, HIGH);
    digitalWrite(MCP3_CS, HIGH);

    // --------------------------------------------------
    // SPI
    // --------------------------------------------------

    mcpSPI.begin(
        MCP_SCK,
        MCP_MISO,
        MCP_MOSI,
        -1
    );

    // --------------------------------------------------
    // BLE
    // --------------------------------------------------

    setupBLE();

    Serial.println("MiniCam ready");
}


// ============================================================================
// LOOP
// ============================================================================

void loop()
{
    // --------------------------------------------------
    // Acquire 24-channel averaged measurement
    // --------------------------------------------------

    measureAverage();

    // --------------------------------------------------
    // Send vector through BLE
    // --------------------------------------------------

    sendMeasurementsBLE();

    // --------------------------------------------------
    // Optional: print values for debugging
    // --------------------------------------------------

    Serial.print("Frame ");
    Serial.println(frameNumber);

    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        Serial.print(average[i]);

        if (i < NUM_CHANNELS - 1)
            Serial.print(", ");
    }

    Serial.println();

    // Small delay
    delay(10);
}