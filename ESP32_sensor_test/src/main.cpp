#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>

// ============================================================
// 兩顆足壓感測器設定
// ============================================================
static BLEUUID SERVICE_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static BLEUUID NOTIFY_UUID ("0000fff1-0000-1000-8000-00805f9b34fb");

// 左腳 BLE 感測器
static const char* LEFT_SENSOR_NAME = "NB-FF2405103703";
// 右腳 BLE 感測器
static const char* RIGHT_SENSOR_NAME = "NB-FF240510831F";

// ============================================================
// UART: ESP32 -> STM32
// GPIO17 TX -> STM32 PA10 RX
// GND -> GND
// ============================================================
HardwareSerial SensorUART(2);

const int UART_RX_PIN = 16;   // 這次不使用
const int UART_TX_PIN = 17;
const int UART_BAUD = 115200;

// ============================================================
// BLE 裝置與 Client
// ============================================================
BLEAdvertisedDevice* targetDevice1 = nullptr;
BLEAdvertisedDevice* targetDevice2 = nullptr;

BLEClient* client1 = nullptr;
BLEClient* client2 = nullptr;

bool connected1 = false;
bool connected2 = false;

// ============================================================
// 每顆感測器各自一個 39-byte 重組 buffer
// 避免兩顆 BLE notify 交錯時互相污染
// ============================================================
uint8_t frameBuffer1[39];
uint8_t frameBuffer2[39];

size_t frameIndex1 = 0;
size_t frameIndex2 = 0;

unsigned long goodFrames1 = 0;
unsigned long badFrames1 = 0;
unsigned long goodFrames2 = 0;
unsigned long badFrames2 = 0;

// ============================================================
// checksum
// byte[38] = byte[0]~byte[37] 加總後低 8 位
// ============================================================
bool checkFrame(const uint8_t* frame)
{
    uint16_t sum = 0;

    for (int i = 0; i < 38; i++)
    {
        sum += frame[i];
    }

    return ((uint8_t)(sum & 0xFF)) == frame[38];
}

// ============================================================
// 將完整 39-byte frame 送給 STM32
// frame 本身的 byte[1] 已含左右腳 ID：0x01 / 0x02
// ============================================================
void processFrame(
    uint8_t* frame,
    int sensorNo,
    unsigned long& goodFrames,
    unsigned long& badFrames)
{
    if (!checkFrame(frame))
    {
        badFrames++;
        Serial.printf("[Sensor %d] BAD checksum #%lu\n", sensorNo, badFrames);
        return;
    }

    goodFrames++;

    // 原封不動送給 STM32，由 STM32 依 frame[1] 判斷 LEFT / RIGHT
    SensorUART.write(frame, 39);

    // 為了避免大量 Serial.print 影響 BLE 接收，不每包印 39 bytes。
    // 每 100 包回報一次即可。
    if (goodFrames % 100 == 0)
    {
        const char* foot = "UNKNOWN";
        if (frame[1] == 0x01) foot = "LEFT";
        else if (frame[1] == 0x02) foot = "RIGHT";

        Serial.printf(
            "[Sensor %d] GOOD=%lu  Foot=%s  ID=0x%02X\n",
            sensorNo,
            goodFrames,
            foot,
            frame[1]
        );
    }
}

// ============================================================
// 通用 byte parser
// 每顆 sensor 使用自己的 buffer / index
// ============================================================
void feedNotifyBytes(
    uint8_t* data,
    size_t length,
    uint8_t* frameBuffer,
    size_t& frameIndex,
    int sensorNo,
    unsigned long& goodFrames,
    unsigned long& badFrames)
{
    for (size_t i = 0; i < length; i++)
    {
        uint8_t b = data[i];

        // 等待 frame 開頭 0xAA
        if (frameIndex == 0)
        {
            if (b == 0xAA)
            {
                frameBuffer[0] = b;
                frameIndex = 1;
            }
            continue;
        }

        // 第二 byte 必須是左右腳 ID 0x01 / 0x02
        if (frameIndex == 1)
        {
            if (b == 0x01 || b == 0x02)
            {
                frameBuffer[1] = b;
                frameIndex = 2;
            }
            else if (b == 0xAA)
            {
                frameBuffer[0] = 0xAA;
                frameIndex = 1;
            }
            else
            {
                frameIndex = 0;
            }
            continue;
        }

        frameBuffer[frameIndex++] = b;

        if (frameIndex == 39)
        {
            processFrame(
                frameBuffer,
                sensorNo,
                goodFrames,
                badFrames
            );

            frameIndex = 0;
        }
    }
}

// ============================================================
// LEFT notify callback
// ============================================================
void notifyCallback1(
    BLERemoteCharacteristic* characteristic,
    uint8_t* data,
    size_t length,
    bool isNotify)
{
    feedNotifyBytes(
        data,
        length,
        frameBuffer1,
        frameIndex1,
        1,
        goodFrames1,
        badFrames1
    );
}

// ============================================================
// RIGHT notify callback
// ============================================================
void notifyCallback2(
    BLERemoteCharacteristic* characteristic,
    uint8_t* data,
    size_t length,
    bool isNotify)
{
    feedNotifyBytes(
        data,
        length,
        frameBuffer2,
        frameIndex2,
        2,
        goodFrames2,
        badFrames2
    );
}

// ============================================================
// BLE scan callback
// 同時找兩個指定名稱
// ============================================================
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice) override
    {
        String name = advertisedDevice.getName().c_str();

        if (name == LEFT_SENSOR_NAME && targetDevice1 == nullptr)
        {
            targetDevice1 = new BLEAdvertisedDevice(advertisedDevice);

            Serial.println();
            Serial.println("找到 LEFT");
            Serial.print("Name: ");
            Serial.println(name);
            Serial.print("Address: ");
            Serial.println(advertisedDevice.getAddress().toString().c_str());
        }
        else if (name == RIGHT_SENSOR_NAME && targetDevice2 == nullptr)
        {
            targetDevice2 = new BLEAdvertisedDevice(advertisedDevice);

            Serial.println();
            Serial.println("找到 RIGHT");
            Serial.print("Name: ");
            Serial.println(name);
            Serial.print("Address: ");
            Serial.println(advertisedDevice.getAddress().toString().c_str());
        }

        // 兩顆都找到後就提早停止掃描
        if (targetDevice1 != nullptr && targetDevice2 != nullptr)
        {
            BLEDevice::getScan()->stop();
        }
    }
};

// ============================================================
// 連線並訂閱 notify
// ============================================================
bool connectSensor(
    BLEAdvertisedDevice* targetDevice,
    BLEClient*& client,
    void (*notifyCallback)(BLERemoteCharacteristic*, uint8_t*, size_t, bool),
    const char* side)
{
    if (targetDevice == nullptr)
    {
        Serial.printf("%s 尚未找到，無法連線\n", side);
        return false;
    }

    Serial.printf("Connecting %s...\n", side);

    client = BLEDevice::createClient();

    if (!client->connect(targetDevice))
    {
        Serial.printf("%s connect failed\n", side);
        return false;
    }

    Serial.printf("%s BLE connected\n", side);

    BLERemoteService* service = client->getService(SERVICE_UUID);

    if (service == nullptr)
    {
        Serial.printf("%s 找不到 FFF0 service\n", side);
        client->disconnect();
        return false;
    }

    Serial.printf("%s 找到 FFF0 service\n", side);

    BLERemoteCharacteristic* notifyChar =
        service->getCharacteristic(NOTIFY_UUID);

    if (notifyChar == nullptr)
    {
        Serial.printf("%s 找不到 FFF1 characteristic\n", side);
        client->disconnect();
        return false;
    }

    Serial.printf("%s 找到 FFF1 characteristic\n", side);

    if (!notifyChar->canNotify())
    {
        Serial.printf("%s FFF1 不支援 Notify\n", side);
        client->disconnect();
        return false;
    }

    notifyChar->registerForNotify(notifyCallback);
    Serial.printf("%s Notify subscribed\n", side);

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
    Serial.println("========================================");
    Serial.println("Dual Foot Sensor -> BLE -> ESP32 -> STM32");
    Serial.println("========================================");

    SensorUART.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART_RX_PIN,
        UART_TX_PIN
    );

    Serial.println("UART2 ready: GPIO17 TX -> STM32 PA10, 115200");

    BLEDevice::init("");

    BLEScan* scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    scan->setActiveScan(true);

    Serial.println();
    Serial.println("Scanning two sensors...");
    Serial.print("LEFT: ");
    Serial.println(LEFT_SENSOR_NAME);
    Serial.print("RIGHT: ");
    Serial.println(RIGHT_SENSOR_NAME);

    // 最多掃描 15 秒；兩顆都找到時 callback 會提早 stop
    scan->start(15, false);

    Serial.println();
    Serial.println("Scan finished");

    if (targetDevice1 == nullptr)
        Serial.println("LEFT NOT FOUND");

    if (targetDevice2 == nullptr)
        Serial.println("RIGHT NOT FOUND");

    // 分別建立兩個 BLE Client
    if (targetDevice1 != nullptr)
        connected1 = connectSensor(targetDevice1, client1, notifyCallback1, "LEFT");

    if (targetDevice2 != nullptr)
        connected2 = connectSensor(targetDevice2, client2, notifyCallback2, "RIGHT");

    Serial.println();
    Serial.println("========================================");
    Serial.printf("LEFT connected: %s\n", connected1 ? "YES" : "NO");
    Serial.printf("RIGHT connected: %s\n", connected2 ? "YES" : "NO");
    Serial.println("Waiting for pressure notifications...");
    Serial.println("========================================");
}

// ============================================================
// loop
// ============================================================
void loop()
{
    if (connected1 && client1 != nullptr && !client1->isConnected())
    {
        connected1 = false;
        Serial.println("LEFT BLE disconnected");
    }

    if (connected2 && client2 != nullptr && !client2->isConnected())
    {
        connected2 = false;
        Serial.println("RIGHT BLE disconnected");
    }

    // 每 5 秒印一次統計，不會每包洗版
    static unsigned long lastReport = 0;

    if (millis() - lastReport >= 5000)
    {
        lastReport = millis();

        Serial.printf(
            "[STAT] LEFT good=%lu bad=%lu | RIGHT good=%lu bad=%lu\n",
            goodFrames1,
            badFrames1,
            goodFrames2,
            badFrames2
        );
    }

    delay(10);
}
