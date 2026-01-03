#include "py32f002b_hal.h"
#include "py32f002b_it.h"

/* --- Handlers --- */
void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1); }
void SVC_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }

/* --- External Links --- */
extern void SX1262_DIO1_Callback(void);
extern LPTIM_HandleTypeDef hlptim;

/* --- Peripheral IRQs --- */

void LPTIM1_IRQHandler(void)
{
  HAL_LPTIM_IRQHandler(&hlptim);
}

/* PA4 falls into EXTI 4-15 range */
void EXTI4_15_IRQHandler(void)
{
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
    SX1262_DIO1_Callback();
  }
}