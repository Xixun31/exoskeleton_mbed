#include "mbed.h"

DigitalOut sck(PB_0);
DigitalIn  dout(PB_1);
Timer t;

const int SAMPLES = 10; // 平均幾筆，降低雜訊

// 讀一筆 24-bit 原始值（gain 128, channel A）
long read_raw()
{
    while (dout.read() == 1); // 等資料就緒

    long value = 0;
    for (int i = 0; i < 24; i++) {
        sck = 1;
        value = (value << 1) | dout.read();
        sck = 0;
    }
    sck = 1; sck = 0; // 第 25 個脈衝：設定下次為 gain128, channel A

    if (value & 0x800000) value |= 0xFF000000; // 24-bit 補數轉 32-bit 負數
    return value;
}

long average_raw(int n)
{
    long sum = 0;
    for (int i = 0; i < n; i++) sum += read_raw();
    return sum / n;
}

int main()
{
    sck = 0;
    t.start();

    printf("HX711 warm-up...\r\n");
    for (int i = 0; i < 20; i++) read_raw(); // 先丟棄前幾筆不穩定的資料

    // ---- 驗證目前實際的取樣速率 ----
    const int RATE_TEST_SAMPLES = 80;
    t.reset();
    long t0 = t.read_ms();
    for (int i = 0; i < RATE_TEST_SAMPLES; i++) read_raw();
    long elapsed_ms = t.read_ms() - t0;
    float measured_sps = RATE_TEST_SAMPLES * 1000.0f / elapsed_ms;
    printf("Measured SPS = %.1f  (elapsed = %ld ms for %d samples)\r\n",
           measured_sps, elapsed_ms, RATE_TEST_SAMPLES);

    // ---- 1. 歸零（相當於 tare） ----
    printf("Taring, keep load cell empty...\r\n");
    long tareOffset = average_raw(SAMPLES);
    printf("Tare offset = %ld\r\n", tareOffset);

    // ---- 2. 校正（相當於 Calibration.ino） ----
    printf("Place known mass now, then press any key + Enter...\r\n");
    getchar();

    long rawWithMass = average_raw(SAMPLES);
    float known_mass_g = 1000.0f; // 改成你實際放的已知重量（克）
    float calFactor = (rawWithMass - tareOffset) / known_mass_g;
    printf("rawWithMass = %ld, calFactor = %.4f\r\n", rawWithMass, calFactor);
    printf("请把 calFactor 记下来，之后直接写死在正式程式里\r\n");

    // ---- 3. 持續讀值（相當於 Read_1x_load_cell.ino） ----
    printf("Remove the mass, start continuous reading...\r\n");
    while (true) {
        long raw = read_raw();
        float grams = (raw - tareOffset) / calFactor;
        printf("raw=%ld  weight=%.1f g\r\n", raw, grams);
    }
}