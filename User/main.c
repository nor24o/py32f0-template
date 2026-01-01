#include "py32f002b_hal.h"
#include <string.h>
#include <stdio.h>

/* ==========================================
   1. PIN DEFINITIONS (Current Working Layout)
   ========================================== */
/* LoRa Power (LDO) - Restored from your sketch */
#define LORA_PWR_PIN       GPIO_PIN_3   /* PB3 */
#define LORA_PWR_PORT      GPIOB

/* LoRa Control */
#define LORA_NSS_PIN       GPIO_PIN_4   /* PA4 */
#define LORA_NSS_PORT      GPIOA

#define LORA_RST_PIN       GPIO_PIN_1   /* PA1 */
#define LORA_RST_PORT      GPIOA

#define LORA_BUSY_PIN      GPIO_PIN_0   /* PA0 */
#define LORA_BUSY_PORT     GPIOA

#define LORA_DIO1_PIN      GPIO_PIN_3   /* PA3 - Interrupt */
#define LORA_DIO1_PORT     GPIOA

/* Handles */
UART_HandleTypeDef UartHandle;
LPTIM_HandleTypeDef hlptim;
SPI_HandleTypeDef hspi1;

/* Packet Data */
#define MY_ID  101
struct __attribute__((packed)) DataPacket {
  uint8_t status;
  uint8_t packetID;
  int16_t temperature;     
  uint16_t batteryMilliVolts; 
};
struct DataPacket myData;

/* Prototypes */
static void APP_SystemConfig(void);
static void APP_GpioConfig(void);
static void APP_SpiConfig(void);
static void APP_UartConfig(void);
static void APP_LptimConfig(void);
static void APP_LockUnusedPins(void);
void APP_ErrorHandler(void);

/* LoRa Driver Prototypes */
void SX_PowerOn(void);
void SX_PowerOff(void);
void SX_Init(void);
void SX_Send(uint8_t *payload, uint8_t len);
void SX_WriteCmd(uint8_t cmd, uint8_t* data, uint8_t len);
void SX_WaitBusy(void);
uint8_t SPI_TxRx(uint8_t data);

/* Interrupt Flag */
volatile uint8_t dio1_fired = 0;

void SX1262_DIO1_Callback(void)
{
  dio1_fired = 1;
}

int main(void)
{
  HAL_Init();
  
  /* Init Peripherals */
  APP_SystemConfig();
  APP_GpioConfig();
  APP_UartConfig();
  APP_SpiConfig();
  APP_LptimConfig();
  
  /* Lock Unused Pins (Save Power) */
  APP_LockUnusedPins();

  /* --- BOOT SAFETY (6s) --- */
  /* Blink Power Pin (PB3) to show life and allow debug attach */
  for(int i=0; i<12; i++) {
    HAL_GPIO_TogglePin(LORA_PWR_PORT, LORA_PWR_PIN);
    HAL_Delay(500);
  }
  
  /* Init Data */
  myData.packetID = 0;
  myData.temperature = 2500; // Fake 25.00C for now
  myData.batteryMilliVolts = 3300; 

  /* Start LPTIM (10s Cycle) */
  if (HAL_LPTIM_SetOnce_Start_IT(&hlptim, 2560) != HAL_OK) APP_ErrorHandler();

  while (1)
  {
    /* 1. POWER ON LORA */
    SX_PowerOn();
    
    /* 2. INIT LORA (Magic Sequence) */
    SX_Init();
    
    /* 3. PREPARE PACKET */
    myData.packetID++;
    uint8_t buffer[32];
    uint8_t idx = 0;
    
    buffer[idx++] = '<';
    idx += sprintf((char*)&buffer[idx], "%d", MY_ID);
    buffer[idx++] = '>';
    memcpy(&buffer[idx], &myData, sizeof(myData));
    idx += sizeof(myData);

    /* 4. SEND PACKET */
    char *log = "TX Sending...\r\n";
    HAL_UART_Transmit(&UartHandle, (uint8_t *)log, strlen(log), 100);
    
    SX_Send(buffer, idx);
    
    log = "TX Done. Sleeping.\r\n";
    HAL_UART_Transmit(&UartHandle, (uint8_t *)log, strlen(log), 100);

    /* 5. POWER OFF LORA */
    SX_PowerOff();

    /* 6. MCU DEEP SLEEP */
    /* Kill Debug & UART Clocks */
    HAL_SuspendTick();
    __HAL_RCC_USART1_CLK_DISABLE();
    __HAL_RCC_SPI1_CLK_DISABLE(); 
    
    /* Kill Debug Pins */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_2; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_6; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Sleep */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* 7. WAKE UP */
    HAL_ResumeTick();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    
    /* Restart Timer */
    HAL_LPTIM_SetOnce_Start_IT(&hlptim, 2560);
  }
}

/* ==========================================
   SX1262 DRIVER IMPLEMENTATION
   ========================================== */

void SX_PowerOn(void) {
    /* Turn on LDO/Power Gate */
    HAL_GPIO_WritePin(LORA_PWR_PORT, LORA_PWR_PIN, GPIO_PIN_SET);
    HAL_Delay(20); 
    
    /* Set Control Pins High */
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
}

void SX_PowerOff(void) {
    /* Send Sleep Opcode (Warm Start configuration) */
    uint8_t sleep[] = {0x84, 0x04}; 
    SX_WriteCmd(0x84, &sleep[1], 1); // Opcode is 1st byte

    /* Turn off LDO/Power Gate */
    HAL_GPIO_WritePin(LORA_PWR_PORT, LORA_PWR_PIN, GPIO_PIN_RESET);
    
    /* Set Control Pins Low to prevent current leak */
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
}

void SX_Init(void) {
    /* Hardware Reset Pulse */
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
    SX_WaitBusy();

    /* Configuration Sequence from your sketch */
    uint8_t buf[8];

    /* Standby RC */
    buf[0]=0x00; SX_WriteCmd(0x80, buf, 1); 
    
    /* Set Packet Type (LoRa = 0x01) */
    buf[0]=0x01; SX_WriteCmd(0x8A, buf, 1); 

    /* Set RF Frequency (868MHz example calc) */
    /* 0x36400000 = ~868MHz */
    buf[0]=0x36; buf[1]=0x40; buf[2]=0x00; buf[3]=0x00;
    SX_WriteCmd(0x86, buf, 4);

    /* PA Config (Output Power) */
    buf[0]=0x02; buf[1]=0x03; buf[2]=0x00; buf[3]=0x01; 
    SX_WriteCmd(0x95, buf, 4);

    /* Tx Params */
    buf[0]=14; buf[1]=0x04; 
    SX_WriteCmd(0x8E, buf, 2);

    /* Buffer Base Address */
    buf[0]=0x00; buf[1]=0x00;
    SX_WriteCmd(0x8F, buf, 2);

    /* Modulation Params (SF9, BW125, CR4/5) */
    buf[0]=0x09; buf[1]=0x04; buf[2]=0x01; buf[3]=0x00;
    SX_WriteCmd(0x8B, buf, 4);

    /* Packet Params */
    buf[0]=0x00; buf[1]=0x08; 
    buf[2]=0x00;              
    buf[3]=0xFF;              
    buf[4]=0x01;              
    buf[5]=0x00;              
    SX_WriteCmd(0x8C, buf, 6);
}

void SX_Send(uint8_t *payload, uint8_t len) {
    uint8_t buf[8];
    
    /* Clear IRQ */
    buf[0]=0xFF; buf[1]=0xFF; SX_WriteCmd(0x02, buf, 2);

    /* Write Buffer */
    SX_WaitBusy();
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    SPI_TxRx(0x0E); SPI_TxRx(0x00); /* Write to address 0 */
    for(int i=0; i<len; i++) SPI_TxRx(payload[i]);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);

    /* Packet Params Update Length */
    buf[0]=0x00; buf[1]=0x08; buf[2]=0x00; buf[3]=len; buf[4]=0x01; buf[5]=0x00;
    SX_WriteCmd(0x8C, buf, 6);

    /* Set DIO1 IRQ (Tx Done) */
    buf[0]=0x01; buf[1]=0x00; buf[2]=0x00; buf[3]=0x00; 
    buf[4]=0x00; buf[5]=0x00; buf[6]=0x00; buf[7]=0x00;
    SX_WriteCmd(0x08, buf, 8);

    /* Set Tx (Timeout = 0) */
    buf[0]=0x00; buf[1]=0x00; buf[2]=0x00;
    SX_WriteCmd(0x83, buf, 3);

    /* Wait for DIO1 (IRQ) or Timeout */
    uint32_t t = 0;
    while(HAL_GPIO_ReadPin(LORA_DIO1_PORT, LORA_DIO1_PIN) == GPIO_PIN_RESET) {
        t++;
        HAL_Delay(1);
        if(t > 2000) break; // 2s Timeout
    }
    
    /* Clear IRQ again */
    buf[0]=0xFF; buf[1]=0xFF; SX_WriteCmd(0x02, buf, 2);
}

void SX_WriteCmd(uint8_t cmd, uint8_t* data, uint8_t len) {
    SX_WaitBusy();
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    SPI_TxRx(cmd);
    for(int i=0; i<len; i++) SPI_TxRx(data[i]);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
    SX_WaitBusy();
}

void SX_WaitBusy(void) {
    uint32_t t = 0;
    while(HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
        t++; if(t>500000) break;
    }
}

uint8_t SPI_TxRx(uint8_t data) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 100);
    return rx;
}

/* ==========================================
   CONFIG FUNCTIONS
   ========================================== */

static void APP_LockUnusedPins(void) {
  /* Keep PB3 (Power) and Control Pins ACTIVE */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Port B: PB0, PB1, PB2, PB7 Unused. 
     USED: PB3(Pwr), PB4(Rx), PB5(Tx), PB6(Dbg) */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Port C: PC1 Unused. Used: PC0(Rst) */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void APP_GpioConfig(void) {
  GPIO_InitTypeDef  GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* POWER PIN (PB3) */
  GPIO_InitStruct.Pin = LORA_PWR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LORA_PWR_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LORA_PWR_PORT, LORA_PWR_PIN, GPIO_PIN_SET);

  /* NSS & RST */
  GPIO_InitStruct.Pin = LORA_NSS_PIN | LORA_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* BUSY */
  GPIO_InitStruct.Pin = LORA_BUSY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* DIO1 (IT) */
  GPIO_InitStruct.Pin = LORA_DIO1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

static void APP_SpiConfig(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) APP_ErrorHandler();
}

static void APP_UartConfig(void) {
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

static void APP_LptimConfig(void) {
  hlptim.Instance = LPTIM1;
  hlptim.Init.Prescaler = LPTIM_PRESCALER_DIV128; 
  hlptim.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  if (HAL_LPTIM_Init(&hlptim) != HAL_OK) APP_ErrorHandler();
}

static void APP_SystemConfig(void) {}

void APP_ErrorHandler(void) { while (1); }