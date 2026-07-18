#include <cstdio>
#include <cstdint>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ---------- Configuration ----------
#define I2C_PORT      i2c0
#define I2C_SDA_PIN   16          // GP16 -> SDA
#define I2C_SCL_PIN   17          // GP17 -> SCL
#define I2C_BAUD      400000      // 400 kHz (MS5611 max)

#define MS5611_ADDR   0x77        // change to 0x76 if the sensor doesn't respond

// Local sea-level pressure in hPa. 1013.25 = standard atmosphere.
// Set this to your local QNH for an accurate absolute altitude.
#define SEA_LEVEL_HPA 1013.25

// ---------- MS5611 commands ----------
static const uint8_t CMD_RESET    = 0x1E;
static const uint8_t CMD_CONV_D1  = 0x48;   // pressure,    OSR = 4096
static const uint8_t CMD_CONV_D2  = 0x58;   // temperature, OSR = 4096
static const uint8_t CMD_ADC_READ = 0x00;
static const uint8_t CMD_PROM_RD  = 0xA0;   // base of the 8-word PROM

// Calibration coefficients C1..C6 (index 0 unused)
static uint16_t C[7];

// Write a single command byte. Returns true if the sensor ACKed.
static bool ms5611_cmd(uint8_t cmd) {
    return i2c_write_blocking(I2C_PORT, MS5611_ADDR, &cmd, 1, false) == 1;
}

// Read one 16-bit PROM word (index 0..7).
static uint16_t ms5611_prom(uint8_t index) {
    uint8_t cmd = CMD_PROM_RD + index * 2;
    uint8_t buf[2] = {0};
    i2c_write_blocking(I2C_PORT, MS5611_ADDR, &cmd, 1, true); // repeated start
    i2c_read_blocking(I2C_PORT, MS5611_ADDR, buf, 2, false);
    return (uint16_t)(buf[0] << 8 | buf[1]);
}

// Start a conversion, wait, then read the 24-bit result.
static uint32_t ms5611_adc(uint8_t conv_cmd) {
    ms5611_cmd(conv_cmd);
    sleep_ms(10);                     // OSR 4096 needs ~9 ms
    uint8_t cmd = CMD_ADC_READ;
    uint8_t buf[3] = {0};
    i2c_write_blocking(I2C_PORT, MS5611_ADDR, &cmd, 1, true);
    i2c_read_blocking(I2C_PORT, MS5611_ADDR, buf, 3, false);
    return (uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | buf[2];
}

int main() {
    stdio_init_all();
    sleep_ms(2000);                   // give USB serial time to enumerate

    // Set up I2C0 on GP16/GP17
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);         // backup pull-ups (breakout usually has its own)
    gpio_pull_up(I2C_SCL_PIN);

    printf("\nMS5611 altimeter\n");

    // Reset and load the factory calibration
    if (!ms5611_cmd(CMD_RESET)) {
        printf("No response at 0x%02X - check wiring / try 0x76.\n", MS5611_ADDR);
    }
    sleep_ms(5);

    for (uint8_t i = 1; i <= 6; i++) {
        C[i] = ms5611_prom(i);
        printf("C%u = %u\n", i, C[i]);
    }

    while (true) {
        uint32_t D1 = ms5611_adc(CMD_CONV_D1);   // raw pressure
        uint32_t D2 = ms5611_adc(CMD_CONV_D2);   // raw temperature

        // --- First-order temperature compensation (datasheet) ---
        int32_t dT   = (int32_t)D2 - ((int32_t)C[5] << 8);
        int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * C[6]) >> 23);

        int64_t OFF  = ((int64_t)C[2] << 16) + (((int64_t)C[4] * dT) >> 7);
        int64_t SENS = ((int64_t)C[1] << 15) + (((int64_t)C[3] * dT) >> 8);

        // --- Second-order compensation (better accuracy below 20 C) ---
        if (TEMP < 2000) {
            int64_t d     = TEMP - 2000;
            int64_t T2    = ((int64_t)dT * dT) >> 31;
            int64_t OFF2  = (5 * d * d) >> 1;
            int64_t SENS2 = (5 * d * d) >> 2;
            if (TEMP < -1500) {
                int64_t e = TEMP + 1500;
                OFF2  += 7 * e * e;
                SENS2 += (11 * e * e) >> 1;
            }
            TEMP -= (int32_t)T2;
            OFF  -= OFF2;
            SENS -= SENS2;
        }

        int32_t P = (int32_t)((((int64_t)D1 * SENS >> 21) - OFF) >> 15);

        double tempC   = TEMP / 100.0;    // deg C
        double presHpa = P / 100.0;       // hPa (mbar)

        // Barometric formula -> metres above sea level
        double altM = 44330.0 * (1.0 - pow(presHpa / SEA_LEVEL_HPA, 0.1902949));

        printf("T=%.2f C  P=%.2f hPa  Alt=%.2f m\n", tempC, presHpa, altM);
        sleep_ms(500);
    }
}