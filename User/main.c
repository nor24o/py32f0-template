#include "py32f002b_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ==========================================
   CONFIGURATION
   ========================================== */
#define DEBUG_MODE 1

/* ==========================================
   PINOUT
   ========================================== */
/* Power */
#define LORA_PWR_PIN GPIO_PIN_3 /* PB3 */
#define LORA_PWR_PORT GPIOB

/* Control */
#define LORA_NSS_PIN GPIO_PIN_0 /* PB0 */
#define LORA_NSS_PORT GPIOB

#define LORA_RST_PIN GPIO_PIN_1 /* PA1 */
#define LORA_RST_PORT GPIOA

#define LORA_BUSY_PIN GPIO_PIN_1 /* PC1 */
#define LORA_BUSY_PORT GPIOC

#define LORA_DIO1_PIN GPIO_PIN_4 /* PA4 */
#define LORA_DIO1_PORT GPIOA

/* SPI PINS (MANUAL CONTROL) */
#define SPI_SCK_PIN GPIO_PIN_5 /* PA5 */
#define SPI_SCK_PORT GPIOA
#define SPI_MISO_PIN GPIO_PIN_6 /* PA6 */
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN GPIO_PIN_7 /* PA7 */
#define SPI_MOSI_PORT GPIOA

/* UART (PB4/PB5) */
#define UART_PORT GPIOB
#define UART_TX_PIN GPIO_PIN_5
#define UART_RX_PIN GPIO_PIN_4

UART_HandleTypeDef UartHandle;
LPTIM_HandleTypeDef hlptim;

volatile uint8_t dio1_fired = 0;

#define MY_ID 101
struct __attribute__((packed)) DataPacket
{
  uint8_t packetID;
  int16_t temperature;
  uint16_t batteryMilliVolts;
};
struct DataPacket myData;

/* --- PROTOTYPES --- */
void System_Init(void);
void Serial_Print(const char *format, ...);
void APP_ErrorHandler(void);

/* LoRa */
void SX_Init(void);
void SX_Send(uint8_t *payload, uint8_t len);
void SX_WriteCmd(uint8_t cmd, uint8_t *data, uint8_t len);
void SX_ReadReg(uint16_t address, uint8_t *data, uint8_t len);
uint16_t SX_GetIrqStatus(void);
void SX_WaitBusy(void);
uint8_t SoftSPI_TxRx(uint8_t data); /* NEW: Software SPI */

void SX1262_DIO1_Callback(void) { dio1_fired = 1; }
void APP_ErrorHandler(void)
{
  while (1)
    ;
}

/* ==========================================
   MAIN LOOP
   ========================================== */
int main(void)
{
  HAL_Init();
  System_Init();

  /* 1. Visual Boot */
  Serial_Print("\r\n[BOOT] SoftSPI Version\r\n");
  for (int i = 0; i < 4; i++)
  {
    HAL_GPIO_TogglePin(LORA_PWR_PORT, LORA_PWR_PIN);
    HAL_Delay(100);
  }

  /* 2. FORCE POWER ON */
  HAL_GPIO_WritePin(LORA_PWR_PORT, LORA_PWR_PIN, GPIO_PIN_SET);
  HAL_Delay(50);

  /* 3. DIAGNOSTIC: Check Busy */
  if (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET)
  {
    Serial_Print("[WARN] Radio BUSY High (Bad if reset complete)\r\n");
  }
  else
  {
    Serial_Print("[INFO] Radio BUSY Low (Ready)\r\n");
  }

  /* 4. SPI TEST: Read Sync Word */
  /* If this still returns 0xFFFF, check your soldering/wires! */
  uint8_t sync[2] = {0, 0};
  SX_ReadReg(0x0740, sync, 2);
  Serial_Print("[TEST] Read 0x0740: 0x%02X%02X (Expect: 0x1424)\r\n", sync[0], sync[1]);

  /* 5. Init Radio */
  SX_Init();

  myData.packetID = 0;
  myData.temperature = 2500;
  myData.batteryMilliVolts = 3300;

  while (1)
  {
    myData.packetID++;
    uint8_t buffer[32];
    uint8_t idx = 0;
    buffer[idx++] = '<';
    idx += sprintf((char *)&buffer[idx], "%d", MY_ID);
    buffer[idx++] = '>';
    memcpy(&buffer[idx], &myData, sizeof(myData));
    idx += sizeof(myData);

    Serial_Print("[TX] Pkt %d...", myData.packetID);
    SX_Send(buffer, idx);

    HAL_Delay(2000);
  }
}

/* ==========================================
   SOFTWARE SPI (BIT BANGING)
   ========================================== */
uint8_t SoftSPI_TxRx(uint8_t data)
{
  uint8_t rx = 0;
  for (int i = 0; i < 8; i++)
  {
    /* 1. Setup Data on MOSI (MSB First) */
    if (data & 0x80)
      HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_RESET);

    data <<= 1;

    /* 2. Clock Low (Setup) */
    HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_RESET);
    // __NOP(); __NOP(); // Small delay if needed

    /* 3. Clock High (Sample) */
    HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_SET);
    // __NOP(); __NOP();

    /* 4. Read MISO */
    rx <<= 1;
    if (HAL_GPIO_ReadPin(SPI_MISO_PORT, SPI_MISO_PIN) == GPIO_PIN_SET)
    {
      rx |= 0x01;
    }
  }
  /* Idle Low */
  HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_RESET);
  return rx;
}

/* ==========================================
   SYSTEM INIT
   ========================================== */
void System_Init(void)
{
  GPIO_InitTypeDef init = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* PWR (PB3) */
  init.Pin = LORA_PWR_PIN;
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LORA_PWR_PORT, &init);
  HAL_GPIO_WritePin(LORA_PWR_PORT, LORA_PWR_PIN, GPIO_PIN_SET);

  /* NSS (PB0) */
  init.Pin = LORA_NSS_PIN;
  HAL_GPIO_Init(LORA_NSS_PORT, &init);
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);

  /* RST (PA1) */
  init.Pin = LORA_RST_PIN;
  HAL_GPIO_Init(LORA_RST_PORT, &init);
  HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);

  /* BUSY (PC1) */
  init.Pin = LORA_BUSY_PIN;
  init.Mode = GPIO_MODE_INPUT;
  init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_BUSY_PORT, &init);

  /* DIO1 (PA4) */
  init.Pin = LORA_DIO1_PIN;
  init.Mode = GPIO_MODE_IT_RISING;
  init.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LORA_DIO1_PORT, &init);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* SPI PINS (MANUAL GPIO) */
  /* SCK (PA5), MOSI (PA7) -> OUTPUT */
  init.Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &init);
  HAL_GPIO_WritePin(GPIOA, SPI_SCK_PIN, GPIO_PIN_RESET); /* Idle Low */

  /* MISO (PA6) -> INPUT */
  init.Pin = SPI_MISO_PIN;
  init.Mode = GPIO_MODE_INPUT;
  init.Pull = GPIO_NOPULL; /* Radio drives this */
  HAL_GPIO_Init(GPIOA, &init);

  /* UART Config */
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
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/* ==========================================
   LORA DRIVER
   ========================================== */
void SX_Init(void)
{
  HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(20);

  Serial_Print("[CFG] Waiting for Busy...");
  SX_WaitBusy();
  Serial_Print("OK\r\n");

  uint8_t buf[8];
  buf[0] = 0x00;
  SX_WriteCmd(0x80, buf, 1);
  buf[0] = 0x01;
  SX_WriteCmd(0x8A, buf, 1);
  buf[0] = 0x36;
  buf[1] = 0x40;
  buf[2] = 0x00;
  buf[3] = 0x00;
  SX_WriteCmd(0x86, buf, 4);
  buf[0] = 0x02;
  buf[1] = 0x03;
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

  uint32_t t = 0;
  dio1_fired = 0;
  while (dio1_fired == 0)
  {
    t++;
    HAL_Delay(1);
    if (t > 3000)
    {
      Serial_Print(" [ERR] Timeout! ");
      break;
    }
  }

  uint16_t irq = SX_GetIrqStatus();
  if ((irq & 0x01) && (irq != 0xFFFF))
    Serial_Print(" [OK]\r\n");
  else
    Serial_Print(" [FAIL] IRQ:0x%04X\r\n", irq);

  buf[0] = 0xFF;
  buf[1] = 0xFF;
  SX_WriteCmd(0x02, buf, 2);
}

void SX_ReadReg(uint16_t address, uint8_t *data, uint8_t len)
{
  uint8_t tx[4];
  tx[0] = 0x1D;
  tx[1] = (address >> 8) & 0xFF;
  tx[2] = address & 0xFF;
  tx[3] = 0x00;
  SX_WaitBusy();
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
  SoftSPI_TxRx(tx[0]);
  SoftSPI_TxRx(tx[1]);
  SoftSPI_TxRx(tx[2]);
  SoftSPI_TxRx(tx[3]);
  for (int i = 0; i < len; i++)
    data[i] = SoftSPI_TxRx(0x00);
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
}

uint16_t SX_GetIrqStatus(void)
{
  uint8_t rx[2] = {0};
  SX_WaitBusy();
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
  SoftSPI_TxRx(0x12);
  SoftSPI_TxRx(0x00);
  SoftSPI_TxRx(0x00);
  SoftSPI_TxRx(0x00);
  rx[0] = SoftSPI_TxRx(0x00);
  rx[1] = SoftSPI_TxRx(0x00);
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
  return (uint16_t)((rx[0] << 8) | rx[1]);
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

void Serial_Print(const char *format, ...)
{
  char buf[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)buf, strlen(buf), 100);
}