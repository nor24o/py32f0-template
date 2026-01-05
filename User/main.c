#include "py32f002b_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ==========================================
   CONFIGURATION
   ========================================== */
/* 0 = LOW POWER MODE (< 2uA)
   1 = DEBUG MODE (1.9mA, UART ON) */
#define DEBUG_MODE 1

#ifndef FLASH_ACR_SLEEP_PD
#define FLASH_ACR_SLEEP_PD (0x1UL << 12U)
#endif

/* ==========================================
   WAKEUP CONFIGURATION
   ========================================== */
static uint8_t wakeup_count = 0;
#define WAKEUP_INTERVAL_MINUTES 1 /* Wake every 30 minutes */
#define SHORT_SLEEP_SECONDS 60    /* Short sleep interval */

/* ==========================================
   MODULE SELECTOR
   Uncomment ONLY ONE of the lines below
   ========================================== */
#define USE_LLCC68 1 /* Use this for your LLCC68 module */
// #define USE_SX1262   1   /* Use this for your SX1262 module */

/* ==========================================
   PINOUT
   ========================================== */
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

/* Sensors */
#define BATT_PIN GPIO_PIN_3 /* PB2 (ADC_IN0) */
#define BATT_PORT GPIOA
#define BATT_ADC_CHANNEL ADC_CHANNEL_1

/* Voltage Divider: (100k+100k)/100k = 2 */
/* Voltage divider: Connect between Battery+ and LoRa GND (switched) */
#define BATT_R1 100000 /* 100kΩ */
#define BATT_R2 100000 /* 100kΩ */

#define BATT_DIVIDER_RATIO 2.4
#define VREF_MV 3300

#define DS18_PIN GPIO_PIN_2 /* PA3 */
#define DS18_PORT GPIOB

#define SWCLK_PIN GPIO_PIN_2 /* PA2 */
#define SWDIO_PIN GPIO_PIN_6 /* PB6 */

UART_HandleTypeDef UartHandle;
LPTIM_HandleTypeDef hlptim;
ADC_HandleTypeDef hadc;
volatile uint8_t dio1_fired = 0;

#define MY_ID 101
struct __attribute__((packed)) DataPacket
{
    uint8_t packetID;
    int16_t temperature;
    uint16_t batteryMilliVolts;
};
struct DataPacket myData;

/* Prototypes */
void System_Init_Once(void);
void Peripherals_Init(void);
void Enter_Low_Power(void);
void Serial_Print(const char *format, ...);
void APP_ErrorHandler(void);
uint16_t ADC_Read_Battery(void);
/* LoRa */
void SX_Init(void);
void SX_PowerOn(void);
void SX_PowerOff(void);
void SX_Send(uint8_t *payload, uint8_t len);
void SX_WriteCmd(uint8_t cmd, uint8_t *data, uint8_t len);
void SX_ReadReg(uint16_t address, uint8_t *data, uint8_t len);
uint16_t SX_GetIrqStatus(void);
void SX_WaitBusy(void);
uint8_t SoftSPI_TxRx(uint8_t data);

void SX1262_DIO1_Callback(void) { dio1_fired = 1; }

/* Error Handler: Blinks Fast if crashed */
void APP_ErrorHandler(void)
{
    while (1)
    {
        HAL_GPIO_TogglePin(LORA_PWR_PORT, LORA_PWR_PIN);
        for (volatile int i = 0; i < 10000; i++)
            ;
    }
}

/* ==========================================
   MAIN LOOP
   ========================================== */
int main(void)
{
    HAL_Init();

    /* 1. Global System Init (Clocks, LPTIM) - Run ONCE */
    System_Init_Once();

    /* 2. Visual Boot Blink */
    if (DEBUG_MODE)
    {
        Peripherals_Init(); /* Need GPIOs for blink */
        Serial_Print("\r\n[BOOT] Active\r\n");
        for (int i = 0; i < 6; i++)
        {
            // HAL_GPIO_TogglePin(LORA_PWR_PORT, LORA_PWR_PIN);
            HAL_Delay(1000);
        }
    }

    myData.packetID = 0;
    myData.temperature = 2500;
    myData.batteryMilliVolts = ADC_Read_Battery();

    while (1)
    {
        /* Check if it's time to send (only on wakeup_count == 0) */
        if (wakeup_count == 0)
        {
#if !DEBUG_MODE
            /* WAKEUP: Re-enable GPIOs/SPI/UART/Radio only */
            Peripherals_Init();
#endif

            /* Activate Radio */
            SX_PowerOn();
            SX_Init();

            myData.packetID++;
            uint8_t buffer[32];
            uint8_t idx = 0;
            myData.batteryMilliVolts = ADC_Read_Battery();
            buffer[idx++] = '<';
            idx += sprintf((char *)&buffer[idx], "%d", MY_ID);
            buffer[idx++] = '>';
            memcpy(&buffer[idx], &myData, sizeof(myData));
            idx += sizeof(myData);

            Serial_Print("[TX] Pkt %d...", myData.packetID);
            SX_Send(buffer, idx);
        }

#if DEBUG_MODE
        /* DEBUG: Stay awake */
        if (wakeup_count == 0)
        {
            Serial_Print(" Waiting (5s)...\r\n");
        }
        HAL_Delay(5000);
#else
        /* LOW POWER: Deep Sleep */
        Enter_Low_Power();
#endif
    }
}

/* ==========================================
   LOW POWER LOGIC (FIXED)
   ========================================== */
void Enter_Low_Power(void)
{
    /* 1. Shutdown Radio */
    SX_PowerOff();

    /* 2. Kill UART Driver */
    HAL_UART_DeInit(&UartHandle);

    /* 3. SET EVERY PIN TO ANALOG (High-Z) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* All Analog EXCEPT PB3 (Radio Power) */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 |
                          GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Keep PB3 Output Low (Radio OFF) */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = LORA_PWR_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, LORA_PWR_PIN, GPIO_PIN_RESET);

    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 4. START TIMER (1280 ticks ~= 5s) */
    /* NOTE: LPTIM is already Inited in main(), just start it */
    // HAL_LPTIM_SetOnce_Start_IT(&hlptim, 1280);
    /////////////////////////////////////////////////

    /* 4. Calculate sleep time based on wakeup count */
    uint32_t compare_value;

    /* Calculate how many wakeups needed for the interval */
    uint8_t wakeups_needed = (WAKEUP_INTERVAL_MINUTES * 60) / SHORT_SLEEP_SECONDS;

    if (wakeup_count >= wakeups_needed - 1)
    {
        /* Time to send data next wakeup */
        wakeup_count = 0;
        compare_value = (SHORT_SLEEP_SECONDS * 32768) / 128;
        //Serial_Print("[SLEEP] Next wakeup will SEND (count reset)\r\n");
    }
    else
    {
        /* Just sleep, don't send next time */
        wakeup_count++;
        compare_value = (SHORT_SLEEP_SECONDS * 32768) / 128;
        //Serial_Print("[SLEEP] Just sleeping (count: %d/%d)\r\n",wakeup_count, wakeups_needed);
    }

    /* Safety check - don't exceed max LPTIM value */
    if (compare_value > 65535)
    {
        compare_value = 65535;
    }

    HAL_LPTIM_SetOnce_Start_IT(&hlptim, compare_value);

    /* 5. Kill System Clocks */
    HAL_SuspendTick();
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    __HAL_RCC_USART1_CLK_DISABLE();

    /* 6. Flash Power Down */
    FLASH->ACR |= FLASH_ACR_SLEEP_PD;

    /* 7. Enter STOP Mode */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* --- SLEEPING --- */

    /* 8. WAKE UP */
    FLASH->ACR &= ~FLASH_ACR_SLEEP_PD;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    HAL_ResumeTick();
}

/* ==========================================
   INIT FUNCTIONS
   ========================================== */
void System_Init_Once(void)
{
    /* Only called ONCE at startup */
    __HAL_RCC_LPTIM_CLK_ENABLE();

    hlptim.Instance = LPTIM1;
    hlptim.Init.Prescaler = LPTIM_PRESCALER_DIV128;
    hlptim.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;

    if (HAL_LPTIM_Init(&hlptim) != HAL_OK)
    {
        APP_ErrorHandler();
    }
}

void Peripherals_Init(void)
{
    /* Called every wake cycle */
    GPIO_InitTypeDef init = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    init.Pin = LORA_PWR_PIN;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(LORA_PWR_PORT, &init);
    HAL_GPIO_WritePin(LORA_PWR_PORT, LORA_PWR_PIN, GPIO_PIN_RESET);

    init.Pin = LORA_NSS_PIN;
    HAL_GPIO_Init(LORA_NSS_PORT, &init);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);

    init.Pin = LORA_RST_PIN;
    HAL_GPIO_Init(LORA_RST_PORT, &init);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);

    init.Pin = LORA_BUSY_PIN;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LORA_BUSY_PORT, &init);

    init.Pin = LORA_DIO1_PIN;
    init.Mode = GPIO_MODE_IT_RISING;
    init.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(LORA_DIO1_PORT, &init);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

    init.Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI_SCK_PORT, &init);
    HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_RESET);

    init.Pin = SPI_MISO_PIN;
    init.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(SPI_MISO_PORT, &init);

    /* Sensors Analog */
    init.Pin = BATT_PIN;
    init.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(BATT_PORT, &init);

    UartHandle.Instance = USART1;
    UartHandle.Init.BaudRate = 9600;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits = UART_STOPBITS_1;
    UartHandle.Init.Parity = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&UartHandle);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (huart->Instance == USART1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();
        GPIO_InitStruct.Pin = UART_TX_PIN | UART_RX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
        HAL_GPIO_Init(UART_PORT, &GPIO_InitStruct);
    }
}

/* ==========================================
   LORA DRIVER (SX1262 Configuration)
   ========================================== */
void SX_Init(void)
{
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
    SX_WaitBusy();

    uint8_t buf[8];

    /* 1. STANDBY & PACKET TYPE (Common) */
    buf[0] = 0x00;
    SX_WriteCmd(0x80, buf, 1); // Standby RC
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

    /* ==========================================
       COMMON RF SETTINGS
       ========================================== */

    /* Frequency (868MHz) */
    buf[0] = 0x36;
    buf[1] = 0x40;
    buf[2] = 0x00;
    buf[3] = 0x00;
    SX_WriteCmd(0x86, buf, 4);

    /* PA CONFIG (+22dBm High Power) - Same for both */
    /* paDuty=0x04, hpMax=0x07, deviceSel=0x00, paLut=0x01 */
    buf[0] = 0x04;
    buf[1] = 0x07;
    buf[2] = 0x00;
    buf[3] = 0x01;
    SX_WriteCmd(0x95, buf, 4);

    /* TX Params (+22dBm) */
    buf[0] = 22;
    buf[1] = 0x04;
    SX_WriteCmd(0x8E, buf, 2);

    /* Buffer Base */
    buf[0] = 0x00;
    buf[1] = 0x00;
    SX_WriteCmd(0x8F, buf, 2);

    /* Modulation (SF9, BW125) */
    buf[0] = 0x09;
    buf[1] = 0x04;
    buf[2] = 0x01;
    buf[3] = 0x00;
    SX_WriteCmd(0x8B, buf, 4);

    /* Packet Params */
    buf[0] = 0x00;
    buf[1] = 0x08;
    buf[2] = 0x00;
    buf[3] = 0xFF;
    buf[4] = 0x01;
    buf[5] = 0x00;
    SX_WriteCmd(0x8C, buf, 6);

    Serial_Print("[CFG] Radio Config Done.\r\n");
}

void SX_PowerOn(void)
{
    HAL_GPIO_WritePin(LORA_PWR_PORT, LORA_PWR_PIN, GPIO_PIN_SET);
    HAL_Delay(20);

    /* Ensure NSS/RST are High when powering on */
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
}

void SX_PowerOff(void)
{
    /* Send Sleep Opcode (0x84)
       0x00 = Cold Start configuration (Lose config on sleep)
       0x04 = Warm Start (Keep config, only works if power stays connected) */
    uint8_t sleep = 0x00;
    SX_WriteCmd(0x84, &sleep, 1);
}

uint8_t SoftSPI_TxRx(uint8_t data)
{
    uint8_t rx = 0;
    for (int i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1;
        HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_SET);
        rx <<= 1;
        if (HAL_GPIO_ReadPin(SPI_MISO_PORT, SPI_MISO_PIN))
            rx |= 1;
    }
    HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_RESET);
    return rx;
}

void SX_Send(uint8_t *payload, uint8_t len)
{
    uint8_t buf[8];
    /* Set DIO2 as RF Switch Control (Optional, typically 0x00) */
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    SX_WriteCmd(0x02, buf, 2);
    SX_WaitBusy();

    /* Write Payload to Buffer */
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    SoftSPI_TxRx(0x0E);
    SoftSPI_TxRx(0x00);
    for (int i = 0; i < len; i++)
        SoftSPI_TxRx(payload[i]);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);

    /* RE-SET PACKET PARAMS (0x8C)
       We must update the payload length for the specific packet we are sending. */
    buf[0] = 0x00;
    buf[1] = 0x08;
    buf[2] = 0x00;
    buf[3] = len;
    buf[4] = 0x01;
    buf[5] = 0x00;
    SX_WriteCmd(0x8C, buf, 6);

    /* SET DIO IRQ PARAMS (0x08)
       -----------------------------------------------------
       Bytes 0-1: IRQ Mask (What to listen for)
       Bytes 2-3: DIO1 Mask (What triggers DIO1 pin)
       Bytes 4-5: DIO2 Mask (What triggers DIO2 pin)
       Bytes 6-7: DIO3 Mask (What triggers DIO3 pin)

       IRQ Bitmasks:
         0x0001 = TxDone
         0x0002 = RxDone
         0x0004 = PreambleDetected
         0x0200 = RxTimeout

       Here we Enable TxDone (0x0001) on DIO1. */
    buf[0] = 0x02;
    buf[1] = 0x01; /* Global IRQ Mask: TxDone(1) | RxTimeout(0x200) */
    buf[2] = 0x02;
    buf[3] = 0x01; /* DIO1 Mask: Same */
    buf[4] = 0x00;
    buf[5] = 0x00; /* DIO2 Mask: None */
    buf[6] = 0x00;
    buf[7] = 0x00; /* DIO3 Mask: None */
    SX_WriteCmd(0x08, buf, 8);

    /* SET TX (0x83) - START TRANSMISSION
       -----------------------------------------------------
       Bytes 0-2: Timeout (0 = No Timeout)
       Value is Timeout * 15.625us */
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
        {
            Serial_Print(" [TIMEOUT]");
            break;
        }
    }

    /* Clear IRQ Status */
    uint16_t irq = SX_GetIrqStatus();
    if (irq & 0x01)
        Serial_Print(" [OK]\r\n");
    else
        Serial_Print(" [ERR 0x%04X]\r\n", irq);

    /* Clear Interrupts (OpCode 0x02) */
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    SX_WriteCmd(0x02, buf, 2);
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

void SX_ReadReg(uint16_t address, uint8_t *data, uint8_t len)
{
    uint8_t tx[4] = {0x1D, (address >> 8) & 0xFF, address & 0xFF, 0x00};
    SX_WaitBusy();
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    for (int i = 0; i < 4; i++)
        SoftSPI_TxRx(tx[i]);
    for (int i = 0; i < len; i++)
        data[i] = SoftSPI_TxRx(0x00);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
}

uint16_t SX_GetIrqStatus(void)
{
    uint8_t rx[2];
    SX_WaitBusy();
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    SoftSPI_TxRx(0x12);
    SoftSPI_TxRx(0x00);
    SoftSPI_TxRx(0x00);
    SoftSPI_TxRx(0x00);
    rx[0] = SoftSPI_TxRx(0x00);
    rx[1] = SoftSPI_TxRx(0x00);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
    return (rx[0] << 8) | rx[1];
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