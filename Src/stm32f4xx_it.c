/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Обработчики исключений ядра Cortex-M4.
  ******************************************************************************
  */

#include "main.h"
#include "stm32f4xx_it.h"

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

/**
  * @brief  Системный таймер: инкремент счётчика миллисекунд HAL.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}
