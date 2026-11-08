/**
  ******************************************************************************
  * @file    clock.h
  * @brief   Настройка системы тактирования МК STM32F401RE (CMSIS).
  ******************************************************************************
  */

#ifndef __CLOCK_H
#define __CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

#ifndef HSI_VALUE
#define HSI_VALUE   16000000UL   /* внутренний генератор HSI */
#endif

/* Результирующие частоты при успешном запуске PLL */
#define SYSCLK_HZ   84000000UL   /* SYSCLK и AHB            */
#define PCLK1_HZ    42000000UL   /* APB1 (модули I2C1–I2C3) */
#define PCLK2_HZ    84000000UL   /* APB2                    */

/**
  * @brief Фактическая частота шины APB1, Гц. Устанавливается функцией
  *        RCC_HSI_init(): 42 МГц при работе от PLL либо 16 МГц, если
  *        PLL недоступен. Используется для расчёта делителей I2C.
  */
extern uint32_t PCLK1_Hz;

/**
  * @brief Тактирование от HSI 16 МГц через PLL: SYSCLK = 84 МГц,
  *        AHB = 84 МГц, APB1 = 42 МГц, APB2 = 84 МГц.
  *        Циклы ожидания ограничены таймаутом; при недоступном PLL
  *        тактирование остаётся от HSI 16 МГц.
  */
void RCC_HSI_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __CLOCK_H */
