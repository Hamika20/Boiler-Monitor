#pragma once

#include <stdbool.h>

typedef struct {
    float temperature_c;
    float humidity_percent;
} Dht11Reading;

bool dht11_read(Dht11Reading* reading);
void dht11_release(void);
