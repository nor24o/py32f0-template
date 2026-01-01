#include "py32f002b_hal.h"
#include <string.h>

/* Private variables */
UART_HandleTypeDef UartHandle;
LPTIM_HandleTypeDef hlptim;

/* Private function prototypes */
static void APP_GpioConfig(void);
static void APP_UartConfig(void);
static void APP_LptimConfig(void);
static void APP_EnterDeepSleep(void);
void APP_ErrorHandler(void);

int main(void)
{
  HAL_Init();
  
  /* 1. Initialize Peripherals */
  APP_GpioConfig();
  APP_UartConfig();
  APP_LptimConfig();

  /* 2. SAFETY DELAY (6 Seconds) - CRITICAL */
  /* The debugger works during this time. After this loop, it dies. */
  for(int i = 0; i < 12; i++) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(500);
  }
  
  /* Ensure LED is OFF (Assuming Reset = OFF based on your note) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 

  /* 3. Start LPTIM (10 Seconds) */
  if (HAL_LPTIM_SetOnce_Start_IT(&hlptim, 2560) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  while (1)
  {
    /* --- SLEEP SEQUENCE --- */
    
    /* A. Suspend SysTick so it doesn't wake us up */
    HAL_SuspendTick();
    
    /* B. Disable UART Clock to save power */
    __HAL_RCC_USART1_CLK_DISABLE();

    /* C. KILL DEBUG PINS (PA2/PB6) to save ~60uA */
    /* We reconfigure them as Analog Input */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_2;       /* PA2 (SWC) */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_6;       /* PB6 (SWD) */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* D. Enter STOP mode */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    
    /* --- WAKEUP SEQUENCE --- */
    
    /* E. Resume SysTick */
    HAL_ResumeTick();

    /* F. Re-enable UART Clock */
    __HAL_RCC_USART1_CLK_ENABLE();
    
    /* Note: We do NOT restore Debug pins. 
       If you need to debug, hit the physical Reset button. */
    
    /* 4. DO WORK */
    
    /* Turn LED ON (Active High) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); 
    
    /* Send Message */
    char *msg = "WAKEUP: 10s passed\r\n";
    HAL_UART_Transmit(&UartHandle, (uint8_t *)msg, strlen(msg), 1000);
    
    /* Wait for blink */
    HAL_Delay(50);
    
    /* Turn LED OFF */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 

    /* 5. Restart Timer */
    HAL_LPTIM_SetOnce_Start_IT(&hlptim, 2560);
  }
}

static void APP_LptimConfig(void)
{
  hlptim.Instance = LPTIM1;
  hlptim.Init.Prescaler = LPTIM_PRESCALER_DIV128; 
  hlptim.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  
  if (HAL_LPTIM_Init(&hlptim) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

static void APP_UartConfig(void)
{
  UartHandle.Instance          = USART1;
  UartHandle.Init.BaudRate     = 9600;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&UartHandle);
}

static void APP_GpioConfig(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* 1. Configure LED (PA5) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* 2. Lock ALL Unused Pins to Analog (Including PA2/PB6 later) */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  /* Port A Unused: PA0, PA1, PA3, PA4, PA6, PA7 */
  /* NOTE: PA2 is left alone here, but killed inside the while(1) loop */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | \
                        GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Port B Unused: PB0, PB1, PB2, PB3, PB7 */
  /* NOTE: PB6 is left alone here, but killed inside the while(1) loop */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | \
                        GPIO_PIN_3 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* Port C Unused: PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void APP_ErrorHandler(void)
{
  while (1) { 
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      HAL_Delay(100);
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1) { }
}
#endif