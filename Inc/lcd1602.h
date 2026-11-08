/**
  ******************************************************************************
  * @file    lcd1602.h
  * @brief   Драйвер знакосинтезирующего ЖК-индикатора LM016L (HD44780),
  *          4-проводный режим.
  *
  * Подключение (см. схему отчёта, рисунок 1):
  *   RS – PB2, E – PB3, D4...D7 – PB4...PB7, RW – общий провод.
  ******************************************************************************
  */

#ifndef __LCD1602_H
#define __LCD1602_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

/* --- распределение выводов -------------------------------------------- */
#define LCD_PORT        GPIOB
#define LCD_PORT_EN     RCC_AHB1ENR_GPIOBEN
#define LCD_RS_PIN      2U              /* PB2 – выбор регистра (0 – команда) */
#define LCD_E_PIN       3U              /* PB3 – строб                        */
#define LCD_D4_PIN      4U              /* PB4...PB7 – шина данных D4...D7     */

/* --- команды контроллера HD44780 --------------------------------------- */
#define LCD_CMD_CLEAR   0x01U           /* очистка экрана                      */
#define LCD_CMD_HOME    0x02U           /* курсор в начало                     */
#define LCD_CMD_ENTRY   0x06U           /* сдвиг курсора вправо, экран на месте */
#define LCD_CMD_ON      0x0CU           /* экран включён, курсор выключен      */
#define LCD_CMD_FUNC    0x28U           /* 4 бита, 2 строки, матрица 5x8       */

/** @brief Настройка выводов порта, к которому подключён индикатор. */
void LCD_port_init(void);

/** @brief Инициализация контроллера HD44780 в 4-проводном режиме. */
void LCD_init(void);

/** @brief Передача команды контроллеру индикатора. */
void LCD_send_cmd(uint8_t cmd);

/** @brief Передача одного символа в текущую позицию. */
void LCD_send_char(char c);

/** @brief Вывод строки, завершённой нулём. */
void LCD_send_string(const char *str);

/** @brief Установка курсора: row = 1...2, col = 1...16. */
void LCD_pos(uint8_t row, uint8_t col);

/** @brief Очистка экрана и возврат курсора в начало. */
void LCD_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* __LCD1602_H */
