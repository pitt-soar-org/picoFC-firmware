#include <cstdio>
#include <cstdint>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

class MS5611 {
public:
    MS5611(i2c_inst_t* i2c, uint8_t addr) : i2c_(i2c), addr_(addr) {}

    // One-time setup: reset + load factory calibration. Blocks, but runs once.
    bool begin() {
        if (!writeCmd(CMD_RESET)) return false;
        sleep_ms(5);                        // PROM reload
        for (uint8_t i = 1; i <= 6; i++)
            C_[i] = readProm(i);
        state_ = START_D1;
        return true;
    }

    // Call every loop iteration. Returns true on the pass a new reading lands.
    bool update() {
        switch (state_) {
        case START_D1:
            writeCmd(CMD_CONV_D1);               // kick off pressure conversion
            ready_ = make_timeout_time_ms(CONV_MS);
            state_ = WAIT_D1;
            break;

        case WAIT_D1:
            if (time_reached(ready_)) {          // conversion finished?
                D1_ = readAdc();
                writeCmd(CMD_CONV_D2);           // kick off temperature conversion
                ready_ = make_timeout_time_ms(CONV_MS);
                state_ = WAIT_D2;
            }
            break;

        case WAIT_D2:
            if (time_reached(ready_)) {
                D2_ = readAdc();
                compute();
                state_ = START_D1;               // loop into the next reading
                return true;                     // <-- fresh data this pass
            }
            break;
        }
        return false;
    }

    double temperatureC() const { return tempC_; }
    double pressureHpa()  const { return presHpa_; }
    double altitudeM(double seaLevelHpa = 1013.25) const {
        return 44330.0 * (1.0 - pow(presHpa_ / seaLevelHpa, 0.1902949));
    }

private:
    static constexpr uint8_t  CMD_RESET    = 0x1E;
    static constexpr uint8_t  CMD_CONV_D1  = 0x48;   // pressure,    OSR 4096
    static constexpr uint8_t  CMD_CONV_D2  = 0x58;   // temperature, OSR 4096
    static constexpr uint8_t  CMD_ADC_READ = 0x00;
    static constexpr uint8_t  CMD_PROM_RD  = 0xA0;
    static constexpr uint32_t CONV_MS      = 10;     // > 9.04 ms max @ OSR 4096

    bool writeCmd(uint8_t cmd) {
        return i2c_write_blocking(i2c_, addr_, &cmd, 1, false) == 1;
    }
    uint16_t readProm(uint8_t index) {
        uint8_t cmd = CMD_PROM_RD + index * 2, buf[2] = {0};
        i2c_write_blocking(i2c_, addr_, &cmd, 1, true);
        i2c_read_blocking(i2c_, addr_, buf, 2, false);
        return (uint16_t)(buf[0] << 8 | buf[1]);
    }
    uint32_t readAdc() {
        uint8_t cmd = CMD_ADC_READ, buf[3] = {0};
        i2c_write_blocking(i2c_, addr_, &cmd, 1, true);
        i2c_read_blocking(i2c_, addr_, buf, 3, false);
        return (uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | buf[2];
    }
    void compute() {
        int32_t dT   = (int32_t)D2_ - ((int32_t)C_[5] << 8);
        int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * C_[6]) >> 23);
        int64_t OFF  = ((int64_t)C_[2] << 16) + (((int64_t)C_[4] * dT) >> 7);
        int64_t SENS = ((int64_t)C_[1] << 15) + (((int64_t)C_[3] * dT) >> 8);
        if (TEMP < 2000) {
            int64_t d = TEMP - 2000;
            int64_t T2 = ((int64_t)dT * dT) >> 31;
            int64_t OFF2 = (5 * d * d) >> 1;
            int64_t SENS2 = (5 * d * d) >> 2;
            if (TEMP < -1500) {
                int64_t e = TEMP + 1500;
                OFF2  += 7 * e * e;
                SENS2 += (11 * e * e) >> 1;
            }
            TEMP -= (int32_t)T2; OFF -= OFF2; SENS -= SENS2;
        }
        int32_t P = (int32_t)((((int64_t)D1_ * SENS >> 21) - OFF) >> 15);
        tempC_   = TEMP / 100.0;
        presHpa_ = P / 100.0;
    }

    i2c_inst_t*     i2c_;
    uint8_t         addr_;
    uint16_t        C_[7] = {0};
    uint32_t        D1_ = 0, D2_ = 0;
    double          tempC_ = 0, presHpa_ = 0;
    enum State { START_D1, WAIT_D1, WAIT_D2 } state_ = START_D1;
    absolute_time_t ready_;
};

// ---------- Configuration ----------
#define I2C_PORT     i2c0
#define I2C_SDA_PIN  16
#define I2C_SCL_PIN  17
#define I2C_BAUD     400000
#define MS5611_ADDR  0x77          // change to 0x76 if the sensor doesn't respond

MS5611 baro(I2C_PORT, MS5611_ADDR);

int main() {
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("\nMS5611 altimeter (non-blocking)\n");
    if (!baro.begin())
        printf("No response at 0x%02X - check wiring / try 0x76.\n", MS5611_ADDR);

    absolute_time_t next_print = make_timeout_time_ms(500);

    while (true) {
        if (baro.update()) {
            // Runs once per new sample. Good spot for a filter, logging, etc.
        }

        // --- your other work goes here; it runs every pass, nothing blocks ---

        // Print the latest reading at 2 Hz, on its own timer
        if (time_reached(next_print)) {
            next_print = delayed_by_ms(next_print, 500);
            printf("T=%.2f C  P=%.2f hPa  Alt=%.2f m\n",baro.temperatureC(), baro.pressureHpa(), baro.altitudeM());
        }
    }
}