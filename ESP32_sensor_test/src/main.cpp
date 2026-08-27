#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define DEVICE_NAME "ESP32_BW_TEST"

#define SERVICE_UUID        "0000fff0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000fff1-0000-1000-8000-00805f9b34fb"

HardwareSerial STM32UART(2);

static const int UART_RX_PIN = 16;
static const int UART_TX_PIN = 17;

volatile unsigned long ble_write_count = 0;
volatile unsigned long ble_byte_count = 0;

class RxCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic) override
    {
        std::string value = pCharacteristic->getValue();

        if (value.empty())
        {
            return;
        }

        ble_write_count++;
        ble_byte_count += value.size();

        STM32UART.write(
            (const uint8_t *)value.data(),
            value.size()
        );
    }
};

class ServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer) override
    {
        Serial.println("PC BLE connected");
    }

    void onDisconnect(BLEServer *pServer) override
    {
        Serial.println("PC BLE disconnected");
        delay(100);
        BLEDevice::startAdvertising();
        Serial.println("Advertising restarted");
    }
};

void setup()
{
    Serial.begin(115200);

    STM32UART.begin(
        115200,
        SERIAL_8N1,
        UART_RX_PIN,
        UART_TX_PIN
    );

    Serial.println();
    Serial.println("========================================");
    Serial.println("PC BLE -> ESP32 -> STM32 Forwarder");
    Serial.println("========================================");
    Serial.println("UART TX: GPIO17 -> STM32 PA10");
    Serial.println("UART: 115200 8N1");

    BLEDevice::init(DEVICE_NAME);

    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    BLEService *service = server->createService(SERVICE_UUID);

    BLECharacteristic *characteristic =
        service->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR
        );

    characteristic->setCallbacks(new RxCallbacks());

    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->start();

    Serial.println("BLE advertising: ESP32_BW_TEST");
    Serial.println("Waiting for PC...");
}

void loop()
{
    static unsigned long last_report = 0;

    if (millis() - last_report >= 5000)
    {
        last_report = millis();

        Serial.print("BLE writes=");
        Serial.print(ble_write_count);

        Serial.print(" | forwarded bytes=");
        Serial.println(ble_byte_count);
    }

    delay(1);
}
