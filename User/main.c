#include "py32f002b_hal.h"
#include "ds18b20.h" /* <--- Uses your existing library */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ==========================================
   CONFIGURATION
   ========================================== */
#define DEBUG_MODE 0 /* 0 = Low Power (<2uA), 1 = UART Debug */
#define MY_ID 101


/* ==========================================
   MODULE SELECTOR
   Uncomment ONLY ONE of the lines below
   ========================================== */
#define USE_LLCC68 1 /* Use this for your LLCC68 module */
// #define USE_SX1262   1   /* Use this for your SX1262 module */


/* ==========================================
   PINOUT
   ========================================== */
/* Power Pin (Controls Radio AND DS18B20 VCC if wired through it) */
#define LORA_PWR_PIN GPIO_PIN_3 /* PB3 */
#define LORA_PWR_PORT GPIOB

#define LORA_NSS_PIN GPIO_PIN_0 /* PB0 */
#define LORA_NSS_PORT GPIOB
#define LORA_RST_PIN GPIO_PIN_1 /* PA1 */
#define LORA_RST_PORT GPIOA
#define LORA_BUSY_PIN GPIO_PIN_1 /* PC1 */
#define LORA_BUSY_PORT GPIOC
#define LORA_DIO1_PIN GPIO_PIN_4 /* PA4 */
#define LORA_DIO1_PORT GPIOA

#define SPI_SCK_PIN GPIO_PIN_5 /* PA5 */
#define SPI_SCK_PORT GPIOA
#define SPI_MISO_PIN GPIO_PIN_6 /* PA6 */
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN GPIO_PIN_7 /* PA7 */
#define SPI_MOSI_PORT GPIOA

#define UART_PORT GPIOB
#define UART_TX_PIN GPIO_PIN_5
#define UART_RX_PIN GPIO_PIN_4

/* Battery ADC */
#define BATT_PIN GPIO_PIN_3 /* Check your ADC pin mapping */
#define BATT_PORT GPIOA
#define BATT_ADC_CHANNEL ADC_CHANNEL_1
#define BATT_DIVIDER_RATIO 2.4

/* ==========================================
   GLOBALS
   ========================================== */
UART_HandleTypeDef UartHandle;
LPTIM_HandleTypeDef hlptim;
ADC_HandleTypeDef hadc;
volatile uint8_t dio1_fired = 0;

struct __attribute__((packed)) DataPacket
{
    uint8_t packetID;
    int16_t temperature;
    uint16_t batteryMilliVolts;
};
struct DataPacket myData;

/* ==========================================
   PROTOTYPES
   ========================================== */
void System_Init_Once(void);
void Peripherals_Init(void);
void Enter_Low_Power(uint16_t ticks);
void Safe_GPIO_Parking(void);
void Serial_Print(const char *format, ...);
uint16_t ADC_Read_Battery(void);

/* LoRa */
void SX_Init(void);
void SX_PowerOn(void);
void SX_PowerOff(void);
void SX_Send(uint8_t *payload, uint8_t len);
void SX_WriteCmd(uint8_t cmd, uint8_t *data, uint8_t len);
void SX_WaitBusy(void);
uint8_t SoftSPI_TxRx(uint8_t data);

/* ==========================================
   INTERRUPT HANDLERS (Merged)
   ========================================== */
void NMI_Handler(void) {}
void HardFault_Handler(void)
{
    while (1)
        ;
}
void SVC_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }

void LPTIM1_IRQHandler(void)
{
    HAL_LPTIM_IRQHandler(&hlptim);
}

void EXTI4_15_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(LORA_DIO1_PIN) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(LORA_DIO1_PIN);
        dio1_fired = 1;
    }
}

/* ==========================================
   MAIN LOOP
   ========================================== */
int main(void)
{
    /* 1. Cold Start */
    HAL_Init();
    System_Init_Once();
    HAL_Delay(500); // Stabilization

    myData.packetID = 0;

    /* 2. Boot Packet (Immediate Logic) */
    Peripherals_Init();

    /* Init Sensor for first time */
    DS18B20_Init(DS18B20_RESOLUTION_12BIT);

    myData.batteryMilliVolts = ADC_Read_Battery();
    myData.temperature = 0; // Dummy for boot

    /* Send Hello */
    uint8_t buffer[32];
    uint8_t idx = 0;

    SX_PowerOn();
    SX_Init();

   // buffer[idx++] = '<';
   // idx += sprintf((char *)&buffer[idx], "%d", MY_ID);
   // buffer[idx++] = '>';
   // memcpy(&buffer[idx], &myData, sizeof(myData));
   // idx += sizeof(myData);
//
   // SX_Send(buffer, idx);
   // SX_PowerOff();

    /* 3. Main State Machine */
    while (1)
    {
        /* --- STATE 1: WAKE & CONVERT --- */
        HAL_Init();         /* Restore Clock */
        Peripherals_Init(); /* Restore GPIOs */

        /* Power Up (Radio + Sensor VCC) */
        HAL_GPIO_WritePin(LORA_PWR_PORT, LORA_PWR_PIN, GPIO_PIN_SET);
        HAL_Delay(50);

        /* Re-Init Library (Required because we cut power/parked pins) */
        DS18B20_Init(DS18B20_RESOLUTION_12BIT);

        /* Start Conversion */
        DS18B20_StartConversion();

        /* Sleep during conversion (800ms for 12-bit) */
        HAL_Delay(800);

        /* --- STATE 2: READ & SEND --- */
        HAL_Init();
        Peripherals_Init();

        /* Read Result using YOUR Library */
        myData.temperature = DS18B20_ReadTemperature();

        myData.batteryMilliVolts = ADC_Read_Battery();
        myData.packetID++;

        /* Send LoRa */
        SX_PowerOn();
        SX_Init();

        idx = 0;
        buffer[idx++] = '<';
        idx += sprintf((char *)&buffer[idx], "%d", MY_ID);
        buffer[idx++] = '>';
        memcpy(&buffer[idx], &myData, sizeof(myData));
        idx += sizeof(myData);

        Serial_Print("[TX] Pkt %d Temp %d\r\n", myData.packetID, myData.temperature);
        SX_Send(buffer, idx);

        /* --- STATE 3: DEEP SLEEP --- */
        Enter_Low_Power(0xFFFF);
    }
}

/* ==========================================
   LOW POWER & SYSTEM
   ========================================== */
void System_Init_Once(void)
{
    __HAL_RCC_LPTIM_CLK_ENABLE();
    hlptim.Instance = LPTIM1;
    hlptim.Init.Prescaler = LPTIM_PRESCALER_DIV128;
    hlptim.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
    HAL_LPTIM_Init(&hlptim);
}

void Peripherals_Init(void)
{
    GPIO_InitTypeDef init = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
#if DEBUG_MODE
    __HAL_RCC_USART1_CLK_ENABLE();
#endif

    /* Radio Power */
    init.Pin = LORA_PWR_PIN;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(LORA_PWR_PORT, &init);

    /* SPI */
    init.Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI_SCK_PORT, &init);

    init.Pin = SPI_MISO_PIN;
    init.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(SPI_MISO_PORT, &init);

    /* LoRa Control */
    init.Pin = LORA_NSS_PIN;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(LORA_NSS_PORT, &init);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);

    init.Pin = LORA_RST_PIN;
    HAL_GPIO_Init(LORA_RST_PORT, &init);

    init.Pin = LORA_BUSY_PIN;
    init.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(LORA_BUSY_PORT, &init);

    init.Pin = LORA_DIO1_PIN;
    init.Mode = GPIO_MODE_IT_RISING;
    init.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(LORA_DIO1_PORT, &init);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

    /* ADC */
    init.Pin = BATT_PIN;
    init.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(BATT_PORT, &init);

/* UART */
#if DEBUG_MODE
    UartHandle.Instance = USART1;
    UartHandle.Init.BaudRate = 9600;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits = UART_STOPBITS_1;
    UartHandle.Init.Parity = UART_PARITY_NONE;
    UartHandle.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&UartHandle);

    GPIO_InitTypeDef uart_gpio = {0};
    uart_gpio.Pin = UART_TX_PIN | UART_RX_PIN;
    uart_gpio.Mode = GPIO_MODE_AF_PP;
    uart_gpio.Pull = GPIO_PULLUP;
    uart_gpio.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(UART_PORT, &uart_gpio);
#endif
}

void Safe_GPIO_Parking(void)
{
    /* Park all pins to Analog to stop 80uA leakage */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_All;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Enter_Low_Power(uint16_t ticks)
{
    /* 1. Shutdown Radio & UART */
    SX_PowerOff();
#if DEBUG_MODE
    HAL_UART_DeInit(&UartHandle);
#endif

    /* 2. Kill ADC */
    if (hadc.Instance != NULL)
    {
        HAL_ADC_DeInit(&hadc);
        __HAL_RCC_ADC_CLK_DISABLE();
    }

    /* 3. Park GPIOs */
    Safe_GPIO_Parking();

    /* 4. Setup Wakeup Timer */
    HAL_LPTIM_SetOnce_Start_IT(&hlptim, ticks);

    /* 5. Enter Stop Mode */
    HAL_SuspendTick();
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_All);

    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* 6. Wakeup Restoration */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    HAL_ResumeTick();
    __HAL_LPTIM_CLEAR_FLAG(&hlptim, LPTIM_FLAG_ARRM);
}

/* ==========================================
   LORA DRIVER (YOUR CONFIGURATION)
   ========================================== */
void SX_Init(void)
{
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
    SX_WaitBusy();

    uint8_t buf[8];
    buf[0] = 0x00;
    SX_WriteCmd(0x80, buf, 1); // Standby
    buf[0] = 0x01;
    SX_WriteCmd(0x8A, buf, 1); // LoRa



    #if defined(USE_SX1262)
    /* ==========================================
       SX1262 CONFIGURATION (TCXO + LDO)
       ========================================== */
    Serial_Print("[CFG] Init for SX1262 (TCXO)\r\n");

    /* 1. Enable TCXO (Required for many SX1262 modules) */
    /* 1.8V (0x02), Delay 5ms */
    buf[0] = 0x02;
    buf[1] = 0x00;
    buf[2] = 0x01;
    buf[3] = 0x40;
    SX_WriteCmd(0x97, buf, 4);

    /* 2. Regulator -> LDO Mode */
    buf[0] = 0x00;
    SX_WriteCmd(0x96, buf, 1);

    /* 3. Enable DIO2 (RF Switch) */
    buf[0] = 0x01;
    SX_WriteCmd(0x9D, buf, 1);

#elif defined(USE_LLCC68)
    /* ==========================================
       LLCC68 CONFIGURATION (XTAL + DCDC)
       ========================================== */
    Serial_Print("[CFG] Init for LLCC68 (XTAL)\r\n");

    /* 1. TCXO OFF (Use Internal Crystal Oscillator) */
    /* We DO NOT send command 0x97. This defaults to XTAL mode. */

    /* 2. Regulator -> Default (DCDC) */
    /* We DO NOT send command 0x96. Defaults to DCDC. */

    /* 3. DIO2 (RF Switch) - Optional but usually safe */
    /* If this fails, comment it out, but usually LLCC68 needs it too. */
    buf[0] = 0x01;
    SX_WriteCmd(0x9D, buf, 1);

#else
#error "Please #define USE_LLCC68 or USE_SX1262 at the top!"
#endif


    ///* YOUR FREQUENCY (868.0 MHz) */
    //buf[0] = 0x36;
    //buf[1] = 0x40;
    //buf[2] = 0x00;
    //buf[3] = 0x00;
    //SX_WriteCmd(0x86, buf, 4);

    /* Set Frequency to 868.100 MHz
       Calculation: 868100000 * (2^25 / 32000000) = 0x36419999 */
    buf[0] = 0x36;
    buf[1] = 0x41;
    buf[2] = 0x99;
    buf[3] = 0x99;
    SX_WriteCmd(0x86, buf, 4);

    buf[0] = 0x04;
    buf[1] = 0x07;
    buf[2] = 0x00;
    buf[3] = 0x01;
    SX_WriteCmd(0x95, buf, 4);

    buf[0] = 22;
    buf[1] = 0x04;
    SX_WriteCmd(0x8E, buf, 2);

    buf[0] = 0x00;
    buf[1] = 0x00;
    SX_WriteCmd(0x8F, buf, 2);

    buf[0] = 0x09;
    buf[1] = 0x04;
    buf[2] = 0x01;
    buf[3] = 0x00;
    SX_WriteCmd(0x8B, buf, 4);

    buf[0] = 0x00;
    buf[1] = 0x08;
    buf[2] = 0x00;
    buf[3] = 0xFF;
    buf[4] = 0x01;
    buf[5] = 0x00;
    SX_WriteCmd(0x8C, buf, 6);
}

void SX_Send(uint8_t *payload, uint8_t len)
{
    uint8_t buf[8];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    SX_WriteCmd(0x02, buf, 2);
    SX_WaitBusy();

    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    SoftSPI_TxRx(0x0E);
    SoftSPI_TxRx(0x00);
    for (int i = 0; i < len; i++)
        SoftSPI_TxRx(payload[i]);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);

    buf[0] = 0x00;
    buf[1] = 0x08;
    buf[2] = 0x00;
    buf[3] = len;
    buf[4] = 0x01;
    buf[5] = 0x00;
    SX_WriteCmd(0x8C, buf, 6);

    buf[0] = 0x02;
    buf[1] = 0x01;
    buf[2] = 0x02;
    buf[3] = 0x01;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 0x00;
    buf[7] = 0x00;
    SX_WriteCmd(0x08, buf, 8);

    buf[0] = 0x00;
    buf[1] = 0x00;
    buf[2] = 0x00;
    SX_WriteCmd(0x83, buf, 3);

    /* Wait for DIO1 (TxDone) */
    uint32_t t = 0;
    dio1_fired = 0;
    while (!dio1_fired)
    {
        t++;
        HAL_Delay(1);
        if (t > 3000)
            break;
    }
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    SX_WriteCmd(0x02, buf, 2);
}

void SX_PowerOn(void)
{
    HAL_GPIO_WritePin(LORA_PWR_PORT, LORA_PWR_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
}
void SX_PowerOff(void)
{
    uint8_t s = 0x00;
    SX_WriteCmd(0x84, &s, 1);
}

void SX_WriteCmd(uint8_t cmd, uint8_t *data, uint8_t len)
{
    SX_WaitBusy();
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    SoftSPI_TxRx(cmd);
    for (int i = 0; i < len; i++)
        SoftSPI_TxRx(data[i]);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
    SX_WaitBusy();
}

void SX_WaitBusy(void)
{
    uint32_t t = 0;
    while (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET)
    {
        t++;
        if (t > 500000)
            break;
    }
}

uint8_t SoftSPI_TxRx(uint8_t data)
{
    uint8_t rx = 0;
    for (int i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1;
        HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_SET);
        rx <<= 1;
        if (HAL_GPIO_ReadPin(SPI_MISO_PORT, SPI_MISO_PIN))
            rx |= 1;
        HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_RESET);
    }
    return rx;
}

uint16_t ADC_Read_Battery(void)
{
    uint32_t adc_val = 0;
    ADC_ChannelConfTypeDef sConfig = {0};

    /* 1. Enable Clocks */
    __HAL_RCC_ADC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE(); /* PA3 is on GPIOA */

    /* 2. Configure GPIO (PA3 as Analog) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = BATT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BATT_PORT, &GPIO_InitStruct);

    /* 3. ADC Reset & Init */
    __HAL_RCC_ADC_FORCE_RESET();
    __HAL_RCC_ADC_RELEASE_RESET();

    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = ENABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

    if (HAL_ADC_Init(&hadc) != HAL_OK)
        return 0;

    /* 4. Calibration */
    HAL_ADCEx_Calibration_Start(&hadc);

    /* 5. Configure Channel 2 (PA3) */
    sConfig.Channel = BATT_ADC_CHANNEL; // ADC_CHANNEL_2
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
        return 0;

    /* 6. Start & Read */
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
    {
        adc_val = HAL_ADC_GetValue(&hadc);
    }
    HAL_ADC_Stop(&hadc);

    /* 7. Cleanup */
    HAL_ADC_DeInit(&hadc);
    __HAL_RCC_ADC_CLK_DISABLE();

    /* Convert to Millivolts */
    uint32_t mv = ((uint32_t)adc_val * 3300) / 4095;
    return (uint16_t)(mv * BATT_DIVIDER_RATIO);
}

void Serial_Print(const char *format, ...)
{
#if DEBUG_MODE
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)buf, strlen(buf), 100);
#endif
}