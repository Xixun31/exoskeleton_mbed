#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>


// ============================================================
// 感測器設定
// ============================================================

static BLEUUID SERVICE_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static BLEUUID NOTIFY_UUID ("0000fff1-0000-1000-8000-00805f9b34fb");


// 感測器名稱
static const char* SENSOR_NAME = "NB-FF2405103703";


// ============================================================
// UART 設定
//
// ESP32 TX -> STM32 RX
// ============================================================

HardwareSerial SensorUART(2);

const int UART_RX_PIN = 16;   // 這次其實不用
const int UART_TX_PIN = 17;

const int UART_BAUD = 115200;


// ============================================================
// BLE
// ============================================================

BLEAdvertisedDevice* targetDevice = nullptr;
BLEClient* client = nullptr;

bool connected = false;


// ============================================================
// 39-byte buffer
// ============================================================

uint8_t frameBuffer[39];

size_t frameIndex = 0;


// ============================================================
// 統計
// ============================================================

unsigned long goodFrames = 0;
unsigned long badFrames = 0;


// ============================================================
// checksum
//
// byte[38] = byte[0] ~ byte[37] 加總低8位
// ============================================================

bool checkFrame(uint8_t* frame)
{
    uint16_t sum = 0;

    for (int i = 0; i < 38; i++)
    {
        sum += frame[i];
    }

    uint8_t checksum = sum & 0xFF;

    return checksum == frame[38];
}


// ============================================================
// 處理一個完整39-byte frame
// ============================================================

void processFrame()
{
    if (checkFrame(frameBuffer))
    {
        goodFrames++;

        // ----------------------------------------------------
        // 一次送完整39 bytes給STM32
        // ----------------------------------------------------

        SensorUART.write(frameBuffer, 39);


        // Debug
        Serial.print("GOOD #");
        Serial.print(goodFrames);

        Serial.print("  ");

        for (int i = 0; i < 39; i++)
        {
            if (frameBuffer[i] < 0x10)
                Serial.print("0");

            Serial.print(
                frameBuffer[i],
                HEX
            );

            Serial.print(" ");
        }

        Serial.println();
    }
    else
    {
        badFrames++;

        Serial.print("BAD checksum #");
        Serial.println(badFrames);
    }
}


// ============================================================
// BLE Notify callback
// ============================================================

void notifyCallback(
    BLERemoteCharacteristic* characteristic,
    uint8_t* data,
    size_t length,
    bool isNotify
)
{
    for (size_t i = 0; i < length; i++)
    {
        uint8_t b = data[i];


        // ====================================================
        // 還沒開始收frame
        // 尋找 0xAA
        // ====================================================

        if (frameIndex == 0)
        {
            if (b == 0xAA)
            {
                frameBuffer[0] = b;

                frameIndex = 1;
            }

            continue;
        }


        // ====================================================
        // 第二byte應為01或02
        // ====================================================

        if (frameIndex == 1)
        {
            if (b == 0x01 ||
                b == 0x02)
            {
                frameBuffer[1] = b;

                frameIndex = 2;
            }
            else
            {
                // 如果又看到AA
                // 保留當新frame開頭
                if (b == 0xAA)
                {
                    frameBuffer[0] = 0xAA;

                    frameIndex = 1;
                }
                else
                {
                    frameIndex = 0;
                }
            }

            continue;
        }


        // ====================================================
        // frame內其他byte
        // ====================================================

        frameBuffer[frameIndex] = b;

        frameIndex++;


        // ====================================================
        // 湊滿39 bytes
        // ====================================================

        if (frameIndex == 39)
        {
            processFrame();

            frameIndex = 0;
        }
    }
}


// ============================================================
// Scan callback
// ============================================================

class MyAdvertisedDeviceCallbacks:
    public BLEAdvertisedDeviceCallbacks
{
    void onResult(
        BLEAdvertisedDevice advertisedDevice
    )
    {
        String name =
            advertisedDevice.getName().c_str();


        if (name == SENSOR_NAME)
        {
            Serial.println();
            Serial.println("找到感測器");

            Serial.print("Name: ");
            Serial.println(name);

            Serial.print("Address: ");
            Serial.println(
                advertisedDevice
                .getAddress()
                .toString()
                .c_str()
            );


            BLEDevice::getScan()->stop();


            targetDevice =
                new BLEAdvertisedDevice(
                    advertisedDevice
                );
        }
    }
};


// ============================================================
// 連感測器
// ============================================================

bool connectSensor()
{
    if (targetDevice == nullptr)
    {
        return false;
    }


    Serial.println("Connecting sensor...");


    client =
        BLEDevice::createClient();


    if (!client->connect(targetDevice))
    {
        Serial.println("Connect failed");

        return false;
    }


    Serial.println("BLE connected");


    // ========================================================
    // 找FFF0 service
    // ========================================================

    BLERemoteService* service =
        client->getService(
            SERVICE_UUID
        );


    if (service == nullptr)
    {
        Serial.println(
            "找不到 FFF0 service"
        );

        client->disconnect();

        return false;
    }


    Serial.println(
        "找到 FFF0 service"
    );


    // ========================================================
    // 找FFF1 characteristic
    // ========================================================

    BLERemoteCharacteristic* notifyChar =
        service->getCharacteristic(
            NOTIFY_UUID
        );


    if (notifyChar == nullptr)
    {
        Serial.println(
            "找不到 FFF1 characteristic"
        );

        client->disconnect();

        return false;
    }


    Serial.println(
        "找到 FFF1 characteristic"
    );


    // ========================================================
    // 訂閱 Notify
    // ========================================================

    if (notifyChar->canNotify())
    {
        notifyChar->registerForNotify(
            notifyCallback
        );

        Serial.println(
            "Notify subscribed"
        );
    }
    else
    {
        Serial.println(
            "FFF1 不支援 Notify"
        );

        return false;
    }


    connected = true;

    return true;
}


// ============================================================
// setup
// ============================================================

void setup()
{
    Serial.begin(115200);


    delay(1000);


    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "Sensor -> BLE -> ESP32 -> STM32"
    );

    Serial.println(
        "================================"
    );


    // ========================================================
    // UART2
    // ========================================================

    SensorUART.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART_RX_PIN,
        UART_TX_PIN
    );


    Serial.println(
        "UART2 ready: 115200"
    );


    // ========================================================
    // BLE初始化
    // ========================================================

    BLEDevice::init("");


    BLEScan* scan =
        BLEDevice::getScan();


    scan->setAdvertisedDeviceCallbacks(
        new MyAdvertisedDeviceCallbacks()
    );


    scan->setActiveScan(true);


    Serial.println(
        "Scanning sensor..."
    );


    // 掃描10秒
    scan->start(
        10,
        false
    );


    // ========================================================
    // 找到後連線
    // ========================================================

    if (targetDevice != nullptr)
    {
        connectSensor();
    }
    else
    {
        Serial.println(
            "找不到感測器"
        );
    }
}


// ============================================================
// loop
// ============================================================

void loop()
{
    // ========================================================
    // 斷線處理
    // ========================================================

    if (connected)
    {
        if (!client->isConnected())
        {
            connected = false;

            Serial.println(
                "BLE disconnected"
            );
        }
    }


    delay(10);
}