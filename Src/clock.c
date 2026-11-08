/**
  ******************************************************************************
  * @file    clock.c
  * @brief   Настройка системы тактирования МК STM32F401RE (CMSIS).
  *
  * Штатный режим: HSI = 16 МГц -> PLL (M = 16, N = 336, P = 4) ->
  * SYSCLK = 84 МГц, AHB = /1 (84 МГц), APB1 = /2 (42 МГц), APB2 = /1.
  *
  * Все циклы ожидания флагов ограничены таймаутом. Если PLL не выходит
  * в рабочий режим (такое поведение характерно для моделей STM32F4 в
  * системах моделирования, где блок PLL не реализован), тактирование
  * остаётся от HSI 16 МГц, а фактическая частота шины APB1 передаётся
  * через переменную PCLK1_Hz. Модуль I2C рассчитывает делитель CCR
  * именно по ней, поэтому заданная частота шины 200 кГц выдерживается
  * в обоих случаях, а программа не может остановиться на этапе
  * инициализации тактирования.
  ******************************************************************************
  */

#include "clock.h"

/* Фактическая частота шины APB1, Гц */
uint32_t PCLK1_Hz = HSI_VALUE;

/* Предельное число опросов флага */
#define CLK_TIMEOUT     100000UL

/**
  * @brief  Ожидание состояния бита с ограничением по времени.
  * @param  reg    адрес регистра
  * @param  mask   маска проверяемого бита
  * @param  state  1 – ждать установки, 0 – ждать сброса
  * @retval 1 – бит принял нужное состояние, 0 – истёк таймаут
  */
static uint8_t wait_bit(volatile uint32_t *reg, uint32_t mask,
                        uint8_t state)
{
    uint32_t timeout = CLK_TIMEOUT;

    while (timeout--) {
        uint8_t now = ((*reg & mask) != 0UL) ? 1U : 0U;
        if (now == state)
            return 1U;
    }
    return 0U;
}

void RCC_HSI_init(void)
{
    /* 1. Включение внутреннего генератора HSI */
    SET_BIT(RCC->CR, RCC_CR_HSION);
    wait_bit(&RCC->CR, RCC_CR_HSIRDY, 1U);

    /* 2. Переход на HSI и останов PLL для изменения его настроек */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
    wait_bit(&RCC->CR, RCC_CR_PLLRDY, 0U);

    /* 3. Безопасная конфигурация на время настройки: шины без деления,
          Flash без тактов ожидания */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE,  RCC_CFGR_HPRE_DIV1);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV1);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_0WS);

    SystemCoreClock = HSI_VALUE;    /* 16 МГц */
    PCLK1_Hz        = HSI_VALUE;    /* 16 МГц */

    /* 4. Питание: режим Scale 2 достаточен для частоты 84 МГц */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    MODIFY_REG(PWR->CR, PWR_CR_VOS, PWR_CR_VOS_1);

    /* 5. Настройка PLL: источник HSI, M = 16, N = 336, P = 4, Q = 7
          f_VCO_in  = 16 / 16  = 1 МГц
          f_VCO_out = 1 * 336  = 336 МГц
          f_PLL     = 336 / 4  = 84 МГц  */
    RCC->PLLCFGR = (16UL << RCC_PLLCFGR_PLLM_Pos) |
                   (336UL << RCC_PLLCFGR_PLLN_Pos) |
                   (1UL << RCC_PLLCFGR_PLLP_Pos) |  /* 01 = деление на 4 */
                   (7UL << RCC_PLLCFGR_PLLQ_Pos);
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);    /* источник PLL – HSI */

    /* 6. Запуск PLL. Если захвата частоты не произошло – работаем от HSI */
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    if (!wait_bit(&RCC->CR, RCC_CR_PLLRDY, 1U)) {
        CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
        return;
    }

    /* 7. До повышения частоты – задержка доступа к Flash 2 такта
          (84 МГц, питание 3,3 В), кэши команд и данных, предвыборка */
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN |
                 FLASH_ACR_LATENCY_2WS;

    /* 8. Шина APB1 не допускает частоты выше 42 МГц – включаем делитель */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);

    /* 9. Переключение SYSCLK на выход PLL */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
    if (!wait_bit(&RCC->CFGR, RCC_CFGR_SWS_PLL, 1U)) {
        /* возврат к исходной конфигурации при неудаче */
        MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);
        MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV1);
        MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_0WS);
        return;
    }

    /* 10. Фиксация достигнутых частот */
    SystemCoreClock = SYSCLK_HZ;    /* 84 МГц */
    PCLK1_Hz        = PCLK1_HZ;     /* 42 МГц */
}
