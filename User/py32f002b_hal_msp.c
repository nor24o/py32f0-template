#include "py32f002b_hal.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};

  if (huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* UART TX (PB5) */
    GPIO_InitStruct.Pin       = GPIO_PIN_5;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART1; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* UART RX (PB4) */
    GPIO_InitStruct.Pin       = GPIO_PIN_4;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* 1. ENABLE LSI OSCILLATOR (CRITICAL FIX) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
     /* Trap if LSI fails to start */
     while(1); 
  }

  /* 2. Select LSI as LPTIM Clock Source */
  __HAL_RCC_LPTIM_CONFIG(RCC_LPTIMCLKSOURCE_LSI);

  /* 3. Enable LPTIM Peripheral Clock */
  __HAL_RCC_LPTIM_CLK_ENABLE();

  /* 4. Enable LPTIM Interrupt */
  HAL_NVIC_SetPriority(LPTIM1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
}