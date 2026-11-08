/**
  ******************************************************************************
  * @file    lcd1602.c
  * @brief   Драйвер ЖК-индикатора LM016L (HD44780), 4-проводный режим.
  *
  * Вывод данных выполняется тетрадами: сначала старшая, затем младшая.
  * Каждая тетрада фиксируется индикатором по заднему фронту строба E.
  ******************************************************************************
  */

#include "lcd1602.h"
#include "delay.h"

/* --- низкоуровневые операции с выводами --------------------------------- */

static void LCD_rs(uint8_t state)
{
    if (state)
        LCD_PORT->BSRR = (1UL << LCD_RS_PIN);
    else
        LCD_PORT->BSRR = (1UL << (LCD_RS_PIN + 16U));
}

/** @brief Формирование строба E: данные защёлкиваются по спаду. */
static void LCD_strobe(void)
{
    LCD_PORT->BSRR = (1UL << LCD_E_PIN);
    delay_us(5U);
    LCD_PORT->BSRR = (1UL << (LCD_E_PIN + 16U));
    delay_us(60U);
}

/** @brief Выставление тетрады на линии D4...D7 и выдача строба. */
static void LCD_write_nibble(uint8_t nibble)
{
    uint32_t mask = 0x0FUL << LCD_D4_PIN;

    LCD_PORT->BSRR = (mask << 16U);                       /* сброс D4...D7   */
    LCD_PORT->BSRR = ((uint32_t)(nibble & 0x0FU)) << LCD_D4_PIN;
    LCD_strobe();
}

/** @brief Передача байта двумя тетрадами. */
static void LCD_write_byte(uint8_t data)
{
    LCD_write_nibble((uint8_t)(data >> 4));
    LCD_write_nibble((uint8_t)(data & 0x0FU));
}

/* --- открытый интерфейс -------------------------------------------------- */

void LCD_port_init(void)
{
    /* тактирование порта B */
    RCC->AHB1ENR |= LCD_PORT_EN;
    (void)RCC->AHB1ENR;

    /* PB2...PB7: выход, двухтактный, средняя скорость, без подтяжки */
    for (uint32_t pin = LCD_RS_PIN; pin <= (LCD_D4_PIN + 3U); pin++) {
        LCD_PORT->MODER   &= ~(3UL << (pin * 2U));
        LCD_PORT->MODER   |=  (1UL << (pin * 2U));   /* 01 – общего назначения */
        LCD_PORT->OTYPER  &= ~(1UL << pin);          /* 0  – двухтактный       */
        LCD_PORT->OSPEEDR &= ~(3UL << (pin * 2U));
        LCD_PORT->OSPEEDR |=  (1UL << (pin * 2U));   /* 01 – средняя скорость  */
        LCD_PORT->PUPDR   &= ~(3UL << (pin * 2U));   /* 00 – без подтяжки      */
    }

    /* исходное состояние линий – низкий уровень */
    LCD_PORT->BSRR = (0x3FUL << LCD_RS_PIN) << 16U;
}

void LCD_init(void)
{
    delay_ms(50U);          /* выдержка после подачи питания */

    LCD_rs(0);

    /* последовательность перевода контроллера в 4-проводный режим */
    LCD_write_nibble(0x03U);
    delay_ms(5U);
    LCD_write_nibble(0x03U);
    delay_us(150U);
    LCD_write_nibble(0x03U);
    delay_us(150U);
    LCD_write_nibble(0x02U);
    delay_us(150U);

    LCD_send_cmd(LCD_CMD_FUNC);     /* 4 бита, 2 строки, матрица 5x8 */
    LCD_send_cmd(LCD_CMD_ON);       /* включение экрана              */
    LCD_send_cmd(LCD_CMD_CLEAR);    /* очистка                       */
    delay_ms(2U);
    LCD_send_cmd(LCD_CMD_ENTRY);    /* автосдвиг курсора вправо      */
}

void LCD_send_cmd(uint8_t cmd)
{
    LCD_rs(0);
    LCD_write_byte(cmd);
    /* команды очистки и возврата курсора выполняются дольше остальных */
    if (cmd == LCD_CMD_CLEAR || cmd == LCD_CMD_HOME)
        delay_ms(2U);
}

void LCD_send_char(char c)
{
    LCD_rs(1);
    LCD_write_byte((uint8_t)c);
}

void LCD_send_string(const char *str)
{
    while (*str)
        LCD_send_char(*str++);
}

void LCD_pos(uint8_t row, uint8_t col)
{
    /* адреса памяти знакомест: строка 1 – 0x00, строка 2 – 0x40 */
    uint8_t addr = (row >= 2U) ? 0x40U : 0x00U;

    addr = (uint8_t)(addr + (col ? (col - 1U) : 0U));
    LCD_send_cmd((uint8_t)(0x80U | addr));
}

void LCD_clear(void)
{
    LCD_send_cmd(LCD_CMD_CLEAR);
}
