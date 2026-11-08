/**
  ******************************************************************************
  * @file    delay.h
  * @brief   Программные задержки на холостом цикле (без периферии).
  ******************************************************************************
  */

#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief Сохранена для совместимости, настройка не требуется. */
void delay_init(void);

/** @brief Задержка в микросекундах. */
void delay_us(uint32_t us);

/** @brief Задержка в миллисекундах. */
void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */
