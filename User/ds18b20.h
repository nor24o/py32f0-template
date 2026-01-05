#ifndef DS18B20_H
#define DS18B20_H

#include "py32f002b_hal.h"

/* ==========================================
   DS18B20 CONFIGURATION
   ========================================== */
/* Define your DS18B20 pin here */
#define DS18B20_PORT    GPIOA
#define DS18B20_PIN     GPIO_PIN_0

/* Resolution (9-12 bits) - affects conversion time */
typedef enum {
    DS18B20_RESOLUTION_9BIT  = 0x1F,  /* 93.75ms conversion */
    DS18B20_RESOLUTION_10BIT = 0x3F,  /* 187.5ms conversion */
    DS18B20_RESOLUTION_11BIT = 0x5F,  /* 375ms conversion */
    DS18B20_RESOLUTION_12BIT = 0x7F   /* 750ms conversion (default) */
} DS18B20_Resolution_t;

/* Error codes */
#define DS18B20_OK              0
#define DS18B20_ERROR_NO_DEVICE 1
#define DS18B20_ERROR_CRC       2
#define DS18B20_ERROR_TIMEOUT   3

/* ==========================================
   PUBLIC FUNCTIONS
   ========================================== */
uint8_t DS18B20_Init(DS18B20_Resolution_t resolution);
uint8_t DS18B20_StartConversion(void);
int16_t DS18B20_ReadTemperature(void);
float DS18B20_ReadTemperatureFloat(void);
uint8_t DS18B20_GetROM(uint8_t *rom);
uint8_t DS18B20_IsPresent(void);
void DS18B20_SetResolution(DS18B20_Resolution_t resolution);
DS18B20_Resolution_t DS18B20_GetResolution(void);

#endif /* DS18B20_H */