#include "ds18b20.h"
#include <string.h>

/* ==========================================
   DIRECT REGISTER MACROS (FAST IO)
   ========================================== */
/* ASSUMPTION: DS18B20 is on GPIOA, PIN 0. */
/* If you change the pin, you MUST update these macros! */
#define DS_PORT      GPIOA
#define DS_PIN_MASK  (1UL << 0)   /* Pin 0 Mask */
#define DS_PIN_POS   0            /* Pin 0 Position */

/* MODER Register: 2 bits per pin.
   00 = Input, 01 = Output.
   For Pin 0: Mask is Bits [1:0] (value 3).
*/
static inline void DS_Pin_Output(void) {
    /* Clear bits (make input) then Set bit 0 (make output) */
    DS_PORT->MODER = (DS_PORT->MODER & ~(3UL << (DS_PIN_POS * 2))) | (1UL << (DS_PIN_POS * 2));
}

static inline void DS_Pin_Input(void) {
    /* Clear bits (make input) */
    DS_PORT->MODER = (DS_PORT->MODER & ~(3UL << (DS_PIN_POS * 2)));
}

static inline void DS_Pin_Low(void) {
    /* Write to Bit Set/Reset Register. Upper 16 bits are Reset (Low). */
    DS_PORT->BSRR = (DS_PIN_MASK << 16U); 
}

/* ==========================================
   TIMING FUNCTIONS (8MHz TUNED)
   ========================================== */
static void DS18B20_DelayUs(uint16_t us) {
    /* * TUNED FOR 8MHz DEFAULT CLOCK
     * 1 us = 8 cycles.
     * Loop overhead + decrement is approx 4 cycles.
     * Multiplier of 2 is close enough for 1-Wire slots.
     */
    volatile uint32_t count = us * 2; 
    
    /* Minimum delay for 1us to avoid 0 count */
    if (count == 0 && us > 0) count = 1;

    while(count--) {
        __NOP();
    }
}

/* ==========================================
   1-WIRE PROTOCOL
   ========================================== */
static uint8_t DS18B20_Reset(void) {
    uint8_t presence = 0;
    
    /* Disable IRQs to protect strict timing */
    __disable_irq();

    DS_Pin_Output();
    DS_Pin_Low();
    DS18B20_DelayUs(500); /* Drive Low > 480us */
    
    DS_Pin_Input();       /* Release Bus */
    DS18B20_DelayUs(70);  /* Wait for sensor to pull low */
    
    /* Check if line is Low (Presence Pulse) */
    if (!(DS_PORT->IDR & DS_PIN_MASK)) {
        presence = 1; 
    }
    
    /* Wait for end of slot */
    DS18B20_DelayUs(420); 
    
    __enable_irq();
    return presence;
}

static void DS18B20_WriteBit(uint8_t bit) {
    __disable_irq();
    
    DS_Pin_Output();
    DS_Pin_Low();
    
    if (bit) {
        /* Write 1: Low for <15us (we do ~6us), then release */
        DS18B20_DelayUs(6);
        DS_Pin_Input(); 
        DS18B20_DelayUs(64);
    } else {
        /* Write 0: Low for >60us (we do ~65us), then release */
        DS18B20_DelayUs(65);
        DS_Pin_Input();
        DS18B20_DelayUs(5);
    }
    
    __enable_irq();
}

static uint8_t DS18B20_ReadBit(void) {
    uint8_t bit = 0;
    
    __disable_irq();
    
    DS_Pin_Output();
    DS_Pin_Low();
    DS18B20_DelayUs(3);   /* Hold Low brief (<15us) */
    
    DS_Pin_Input();       /* Release */
    DS18B20_DelayUs(10);  /* Wait to sample (must be within 15us of start) */
    
    if (DS_PORT->IDR & DS_PIN_MASK) {
        bit = 1;
    }
    
    DS18B20_DelayUs(50);  /* Finish slot */
    
    __enable_irq();
    return bit;
}

static void DS18B20_WriteByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        DS18B20_WriteBit(data & 0x01);
        data >>= 1;
    }
}

static uint8_t DS18B20_ReadByte(void) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        data >>= 1;
        if (DS18B20_ReadBit()) data |= 0x80;
    }
    return data;
}

/* ==========================================
   PUBLIC FUNCTIONS
   ========================================== */
uint8_t DS18B20_Init(DS18B20_Resolution_t resolution) {
    /* 1. Enable Clock for GPIOA */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* 2. Configure Pin initially as Input + PullUp */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0; // PA0
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    HAL_Delay(5); /* Stabilize */

    /* 3. Reset Check */
    if (!DS18B20_Reset()) {
        return 1; // Error
    }
    
    /* 4. Configure Resolution */
    DS18B20_WriteByte(0xCC); // Skip ROM
    DS18B20_WriteByte(0x4E); // Write Scratchpad
    DS18B20_WriteByte(0x00); // TH
    DS18B20_WriteByte(0x00); // TL
    DS18B20_WriteByte(resolution); // Config Register
    
    return 0; // OK
}

uint8_t DS18B20_StartConversion(void) {
    if (!DS18B20_Reset()) return 1;
    DS18B20_WriteByte(0xCC); // Skip ROM
    DS18B20_WriteByte(0x44); // Convert
    return 0;
}

int16_t DS18B20_ReadTemperature(void) {
    uint8_t low, high;
    if (!DS18B20_Reset()) return 0; // Return 0 on error
    
    DS18B20_WriteByte(0xCC); // Skip ROM
    DS18B20_WriteByte(0xBE); // Read Scratchpad
    
    low = DS18B20_ReadByte();
    high = DS18B20_ReadByte();
    
    DS18B20_Reset(); // Stop reading
    
    int16_t val = (high << 8) | low;
    /* Resolution calc: 0.0625 degrees per bit */
    /* Return in hundredths (e.g. 25.50C -> 2550) */
    return (val * 625) / 100;
}

/* Stubs for compatibility */
void DS18B20_SetResolution(DS18B20_Resolution_t resolution) {}
uint8_t DS18B20_GetROM(uint8_t *rom) { return 0; }