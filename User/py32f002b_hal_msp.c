#include "py32f002b_hal.h"

/**
  * @brief Initialize global MSP.
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize LPTIM MSP.
  */
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* 1. Enable LSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
     while(1); 
  }

  /* 2. Select LSI as LPTIM Clock Source */
  __HAL_RCC_LPTIM_CONFIG(RCC_LPTIMCLKSOURCE_LSI);

  /* 3. Enable LPTIM Clock */
  __HAL_RCC_LPTIM_CLK_ENABLE();

  /* 4. Enable LPTIM Interrupt */
  HAL_NVIC_SetPriority(LPTIM1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
}