#include "dht11.h"

#include <furi.h>
#include <furi_hal_cortex.h>
#include <furi_hal_gpio.h>
#include <furi_hal_resources.h>

#include <stddef.h>
#include <stdint.h>

#define DHT11_DATA_PIN (&gpio_ext_pc0)

static bool wait_for_level(bool level, uint32_t timeout_us) {
    for(uint32_t elapsed = 0; elapsed < timeout_us; elapsed++) {
        if(furi_hal_gpio_read(DHT11_DATA_PIN) == level) {
            return true;
        }
        furi_hal_cortex_delay_us(1);
    }

    return furi_hal_gpio_read(DHT11_DATA_PIN) == level;
}

void dht11_release(void) {
    furi_hal_gpio_init(DHT11_DATA_PIN, GpioModeInput, GpioPullUp, GpioSpeedLow);
}

static bool dht11_read_once(Dht11Reading* reading) {
    uint8_t data[5] = {0};
    bool success = false;

    furi_hal_gpio_init(
        DHT11_DATA_PIN, GpioModeOutputOpenDrain, GpioPullUp, GpioSpeedVeryHigh);
    furi_hal_gpio_write(DHT11_DATA_PIN, false);
    furi_delay_ms(18);

    FURI_CRITICAL_ENTER();
    furi_hal_gpio_write(DHT11_DATA_PIN, true);
    furi_hal_cortex_delay_us(30);

    if(!wait_for_level(false, 100) || !wait_for_level(true, 100) ||
       !wait_for_level(false, 100)) {
        goto cleanup;
    }

    for(size_t byte = 0; byte < sizeof(data); byte++) {
        for(uint8_t bit = 0; bit < 8; bit++) {
            if(!wait_for_level(true, 100)) {
                goto cleanup;
            }

            furi_hal_cortex_delay_us(40);
            data[byte] <<= 1;
            if(furi_hal_gpio_read(DHT11_DATA_PIN)) {
                data[byte] |= 1;
            }

            if(!wait_for_level(false, 100)) {
                goto cleanup;
            }
        }
    }

    if((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) {
        goto cleanup;
    }

    float temperature = (float)data[2] + (float)(data[3] & 0x7FU) / 10.0f;
    if(data[3] & 0x80U) {
        temperature = -temperature;
    }

    reading->temperature_c = temperature;
    reading->humidity_percent = (float)data[0] + (float)data[1] / 10.0f;
    success = true;

cleanup:
    FURI_CRITICAL_EXIT();
    dht11_release();
    return success;
}

bool dht11_read(Dht11Reading* reading) {
    furi_check(reading);
    return dht11_read_once(reading);
}
