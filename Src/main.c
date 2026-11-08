// Лабораторная работа №5. Работа с датчиком давления
// BMP180 по интерфейсу I2C.
// МК STM32F401RE, вариант: 200 кГц, BMP180, PA8 – SCL, PC9 – SDA.
//
// Обмен по шине I2C реализован программно (bit-banging): линии
// SCL и SDA – выводы общего назначения в режиме «открытый сток»
// с подтяжкой к VDD. Протокол (START, Repeated START, STOP, ACK/NACK,
// передача старшим битом вперёд) формируется программой, поэтому
// обмен наблюдается в Proteus на выбранных по варианту выводах.
//
// Программа:
//  1) проверяет идентификатор BMP180 (регистр 0xD0 = 0x55);
//  2) читает 11 калибровочных коэффициентов (0xAA...0xBF);
//  3) циклически измеряет температуру и давление, выполняет
//     компенсацию по алгоритму из документации BMP180 и выводит
//     результат на ЖКИ LM016L.

// Библиотеки
#include "stm32f4xx.h"
#include "clock.h"
#include "delay.h"
#include "lcd1602.h"
#include <stdint.h>

//=============================================================
// ВЫВОДЫ ШИНЫ I2C (по варианту)
//=============================================================
#define SCL_PORT        GPIOA
#define SCL_PORT_EN     RCC_AHB1ENR_GPIOAEN
#define SCL_PIN         8U              // PA8 – SCL
#define SDA_PORT        GPIOC
#define SDA_PORT_EN     RCC_AHB1ENR_GPIOCEN
#define SDA_PIN         9U              // PC9 – SDA

// Открытый сток: запись 1 отпускает линию (её поднимает
// резистор), запись 0 прижимает линию к общему проводу.
// Регистр BSRR позволяет менять один вывод атомарно.
#define SCL_H()   (SCL_PORT->BSRR = (1UL << SCL_PIN))
#define SCL_L()   (SCL_PORT->BSRR = (1UL << (SCL_PIN + 16U)))
#define SDA_H()   (SDA_PORT->BSRR = (1UL << SDA_PIN))
#define SDA_L()   (SDA_PORT->BSRR = (1UL << (SDA_PIN + 16U)))
#define SCL_IN()  ((SCL_PORT->IDR >> SCL_PIN) & 1UL)
#define SDA_IN()  ((SDA_PORT->IDR >> SDA_PIN) & 1UL)

//=============================================================
// НАСТРОЙКИ
//=============================================================
// частота шины I2C, Гц – по варианту 200 кГц (Fast mode)
#define F_SCL             200000UL
// Такты ядра на одну итерацию цикла задержки (SUBS + BNE)
#define I2C_LOOP_CYCLES   4UL
// Такты ядра на обработку одного полупериода SCL вне цикла
// задержки (вызовы функций, обращения к GPIO). Уточняется по
// осциллограмме: увеличить, если частота SCL ниже 200 кГц.
#define I2C_OVERHEAD      60UL
// таймаут ожидания отпускания SCL ведомым, циклов
#define I2C_TIMEOUT       10000UL

// 7-битный адрес BMP180 (0xEE – запись, 0xEF – чтение)
#define BMP180_ADDR       0x77
// регистр идентификатора, всегда содержит 0x55
#define BMP180_CHIP_ID    0xD0
#define BMP180_ID_VALUE   0x55
// первый регистр калибровочных коэффициентов (0xAA...0xBF)
#define BMP180_CALIB      0xAA
// регистр управления измерением
#define BMP180_MEAS_CTRL  0xF4
// регистры результата: MSB, LSB, XLSB
#define BMP180_OUT_MSB    0xF6
#define BMP180_OUT_LSB    0xF7
#define BMP180_OUT_XLSB   0xF8
// команды запуска измерения
#define BMP180_CMD_TEMP   0x2E
#define BMP180_CMD_PRES   0x34
// режим передискретизации давления: 0...3
#define BMP180_OSS        0

// Результат операций на шине
#define I2C_OK            0x00
#define I2C_ERR           0xFF

//=============================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
//=============================================================
// число итераций цикла задержки на полупериод SCL
static uint32_t i2c_loops = 1;

// причина последней ошибки обмена (выводится на ЖКИ)
#define I2C_E_SCL         1   // ведомый держит SCL в нуле
#define I2C_E_SDA         2   // перед START линия SDA занята
#define I2C_E_NACK        3   // ведомый не подтвердил байт
static uint8_t i2c_err;
// номер переданного в транзакции байта: 1 – адрес, 2 – регистр...
static uint8_t i2c_byte;

// калибровочные коэффициенты BMP180
static struct {
    int16_t  AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t  B1, B2, MB, MC, MD;
} cal;

//=============================================================
// ПРОТОТИПЫ
//=============================================================
void    I2C_Init(void);
uint8_t I2C_ReadByte(uint8_t dev_addr, uint8_t reg, uint8_t *data);
uint8_t I2C_WriteByte(uint8_t dev_addr, uint8_t reg, uint8_t value);

uint8_t BMP180_ReadCalibration(void);
uint8_t BMP180_Measure(int32_t *t, int32_t *p);
void    BMP180_Compensate(int32_t ut, int32_t up,
                          int32_t *t, int32_t *p);

static void LCD_print_dec(int32_t value, uint8_t width);
static void LCD_print_hex(uint8_t value);
static void LCD_show_error(void);

//=============================================================
// ОСНОВНАЯ ПРОГРАММА
//=============================================================
int main(void)
{
    uint8_t id = 0;
    int32_t t;                     // температура, 0,1 °C
    int32_t p;                     // давление, Па

    RCC_HSI_init();                // тактирование 84 МГц от HSI
    LCD_port_init();               // инициализация порта ЖКИ
    LCD_init();                    // инициализация ЖКИ
    LCD_pos(1, 1);
    LCD_send_string("LCD OK! ");

    I2C_Init();                    // программный I2C: PA8, PC9
    LCD_send_string("I2C OK!");

    // 1. Проверка идентификатора датчика
    // (до 3 попыток: первая посылка после запуска может
    // совпасть с переходными процессами на линиях)
    LCD_pos(2, 1);
    for (uint8_t try = 0; ; try++) {
        if (I2C_ReadByte(BMP180_ADDR, BMP180_CHIP_ID, &id) == I2C_OK)
            break;
        if (try >= 2U) {
            LCD_show_error();
            while (1) {}
        }
        delay_ms(10);
    }
    if (id != BMP180_ID_VALUE) {
        LCD_send_string("BMP180 ID ERR ");
        LCD_print_hex(id);
        while (1) {}
    }
    LCD_send_string("BMP180 OK! ID=");
    LCD_print_hex(id);

    // 2. Чтение калибровочных коэффициентов
    if (BMP180_ReadCalibration() != I2C_OK) {
        LCD_pos(2, 1);
        LCD_send_string("Calib. error    ");
        while (1) {}
    }
    delay_ms(1000);
    LCD_clear();

    // 3. Циклическое измерение температуры и давления
    while (1) {
        if (BMP180_Measure(&t, &p) == I2C_OK) {
            // строка 1: температура, например "T= 25.0 C"
            LCD_pos(1, 1);
            LCD_send_string("T=");
            if (t < 0) { LCD_send_char('-'); t = -t; }
            else         LCD_send_char(' ');
            LCD_print_dec(t / 10, 3);
            LCD_send_char('.');
            LCD_print_dec(t % 10, 1);
            LCD_send_string(" C      ");

            // строка 2: давление, Па и мм рт. ст.
            // 1 мм рт. ст. = 133,322 Па
            LCD_pos(2, 1);
            LCD_send_string("P=");
            LCD_print_dec(p, 6);
            LCD_send_string("Pa ");
            LCD_print_dec((p * 10L + 666L) / 1333L, 3);
            LCD_send_string("mm");
        }
        else {
            LCD_show_error();
        }
        delay_ms(500);
    }
}

//=============================================================
// ПРОГРАММНАЯ РЕАЛИЗАЦИЯ ШИНЫ I2C
//=============================================================
// Задержка на n итераций. Цикл написан на ассемблере,
// чтобы его длительность не зависела от уровня оптимизации.
static void I2C_Wait(uint32_t n)
{
    if (n == 0U) return;
    __asm volatile ("1: subs %0, %0, #1 \n"
                    "   bne  1b         \n"
                    : "+r" (n) : : "cc");
}

// Полупериод SCL
static void I2C_Delay(void) { I2C_Wait(i2c_loops); }
// Низкий полупериод SCL делится на две части:
// удержание (hold) – SDA не меняется сразу после спада SCL,
// установка (setup) – SDA выставлена до подъёма SCL
static void I2C_Hold(void)  { I2C_Wait(i2c_loops / 4U); }
static void I2C_Setup(void) { I2C_Wait(i2c_loops - i2c_loops / 4U); }

// Прижатие SCL с ожиданием фактического спада линии и
// паузой удержания: SDA можно менять только после неё,
// иначе изменение SDA при ещё высокой SCL будет воспринято
// ведомым как ложное условие START или STOP
static void I2C_SCL_Low(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    SCL_L();
    while (SCL_IN() && --timeout) {};
    I2C_Hold();
}

// Отпускание SCL с ожиданием фактического подъёма линии:
// ведомый может удерживать SCL в нуле (clock stretching).
static uint8_t I2C_SCL_Release(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    SCL_H();
    while (!SCL_IN() && --timeout) {};
    if (timeout) return I2C_OK;
    i2c_err = I2C_E_SCL;
    return I2C_ERR;
}

// Отпускание SDA с ожиданием фактического подъёма линии.
// Единицу формирует только резистор подтяжки, и при большой
// ёмкости линии (подключённые приборы) фронт затягивается:
// если поднять SCL раньше, ведомый прочитает 0 вместо 1.
// Если SDA держит ведомый (ACK, данные), ожидание просто
// завершается по таймауту и ошибкой не считается.
static void I2C_SDA_Release(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    SDA_H();
    while (!SDA_IN() && --timeout) {};
}

// Настройка вывода: выход «открытый сток», высокая скорость,
// подтяжка к VDD; в выходной регистр заранее пишется 1,
// чтобы линия не прижималась при переключении режима
static void I2C_PinInit(GPIO_TypeDef *port, uint32_t pin)
{
    port->BSRR = (1UL << pin);                  // линия отпущена
    SET_BIT(port->OTYPER, (1UL << pin));        // открытый сток
    MODIFY_REG(port->OSPEEDR,                   // высокая скорость
               (3UL << (pin * 2U)),
               (3UL << (pin * 2U)));
    MODIFY_REG(port->PUPDR,                     // подтяжка к VDD
               (3UL << (pin * 2U)),
               (1UL << (pin * 2U)));
    MODIFY_REG(port->MODER,                     // режим выхода
               (3UL << (pin * 2U)),
               (1UL << (pin * 2U)));
}

// Условие START (и Repeated START): спад SDA при SCL = 1.
// Перед вызовом SCL = 0 (после байта) либо шина свободна
static uint8_t I2C_Start(void)
{
    I2C_SDA_Release();
    I2C_Setup();
    if (I2C_SCL_Release() != I2C_OK) return I2C_ERR;
    I2C_Delay();
    if (!SDA_IN()) {                // линию SDA держит другое устройство
        i2c_err = I2C_E_SDA;
        return I2C_ERR;
    }
    SDA_L();
    I2C_Delay();
    I2C_SCL_Low();
    return I2C_OK;
}

// Условие STOP: подъём SDA при SCL = 1.
// SDA меняется только при SCL = 0, иначе спад SDA при
// отпущенной SCL был бы воспринят как лишнее условие START
static void I2C_Stop(void)
{
    I2C_SCL_Low();
    SDA_L();
    I2C_Setup();
    I2C_SCL_Release();
    I2C_Delay();
    SDA_H();
    I2C_Delay();
}

// Передача байта старшим битом вперёд.
// Перед вызовом SCL = 0 и пауза удержания выдержана.
// Возвращает I2C_OK, если ведомый ответил ACK (SDA = 0)
static uint8_t I2C_TxByte(uint8_t byte)
{
    uint8_t ack;

    for (uint8_t i = 0; i < 8U; i++) {
        if (byte & 0x80U) I2C_SDA_Release();      // данные при SCL = 0
        else              SDA_L();
        byte <<= 1;
        I2C_Setup();
        if (I2C_SCL_Release() != I2C_OK) return I2C_ERR;
        I2C_Delay();                              // ведомый читает бит
        I2C_SCL_Low();
    }
    // девятый такт – бит подтверждения от ведомого
    SDA_H();
    I2C_Setup();
    if (I2C_SCL_Release() != I2C_OK) return I2C_ERR;
    I2C_Delay();
    ack = SDA_IN() ? I2C_ERR : I2C_OK;
    I2C_SCL_Low();
    i2c_byte++;
    if (ack != I2C_OK) i2c_err = I2C_E_NACK;
    return ack;
}

// Приём байта старшим битом вперёд; ack = 1 – ответить ACK
// (будут ещё байты), ack = 0 – ответить NACK (последний байт)
static uint8_t I2C_RxByte(uint8_t ack, uint8_t *byte)
{
    uint8_t data = 0;

    SDA_H();                                      // SDA отдаётся ведомому
    for (uint8_t i = 0; i < 8U; i++) {
        I2C_Setup();
        if (I2C_SCL_Release() != I2C_OK) return I2C_ERR;
        I2C_Delay();
        data = (uint8_t)((data << 1) | SDA_IN()); // чтение при SCL = 1
        I2C_SCL_Low();
    }
    // девятый такт – подтверждение от ведущего
    if (ack) SDA_L(); else I2C_SDA_Release();
    I2C_Setup();
    if (I2C_SCL_Release() != I2C_OK) return I2C_ERR;
    I2C_Delay();
    I2C_SCL_Low();
    SDA_H();
    *byte = data;
    return I2C_OK;
}

//=============================================================
// Инициализация программного I2C (200 кГц)
//=============================================================
void I2C_Init(void)
{
    uint32_t cycles;

    // 1. Тактирование портов GPIOA (SCL) и GPIOC (SDA)
    SET_BIT(RCC->AHB1ENR, SCL_PORT_EN | SDA_PORT_EN);

    // 2. Линии SCL и SDA: открытый сток, подтяжка к VDD
    I2C_PinInit(SCL_PORT, SCL_PIN);
    I2C_PinInit(SDA_PORT, SDA_PIN);

    // 3. Длительность полупериода SCL по текущей частоте ядра:
    //    84 МГц / (2 * 200 кГц) = 210 тактов на полупериод
    cycles = SystemCoreClock / (2UL * F_SCL);
    cycles = (cycles > I2C_OVERHEAD + I2C_LOOP_CYCLES)
             ? cycles - I2C_OVERHEAD : I2C_LOOP_CYCLES;
    i2c_loops = cycles / I2C_LOOP_CYCLES;

    // 4. Освобождение шины: если ведомый после сброса МК
    //    удерживает SDA, выдаём до 9 тактов SCL и STOP
    for (uint8_t i = 0; i < 9U && !SDA_IN(); i++) {
        SCL_L();
        I2C_Delay();
        I2C_SCL_Release();
        I2C_Delay();
    }
    I2C_Stop();
}

//=============================================================
// Чтение одного байта из регистра ведомого:
// S | ADDR+W | A | REG | A | Sr | ADDR+R | A | DATA | N | P
// Возвращает: I2C_OK либо I2C_ERR при ошибке
//=============================================================
uint8_t I2C_ReadByte(uint8_t dev_addr, uint8_t reg, uint8_t *data)
{
    i2c_byte = 0;
    // 1. START + адрес устройства (Write);
    //    0xFE = 11111110 – обнуление бита R/W
    if (I2C_Start() != I2C_OK)                     goto error;
    if (I2C_TxByte((dev_addr << 1) & 0xFE) != I2C_OK) goto error;
    // 2. Отправка номера регистра
    if (I2C_TxByte(reg) != I2C_OK)                 goto error;
    // 3. Repeated START + адрес устройства (Read)
    if (I2C_Start() != I2C_OK)                     goto error;
    if (I2C_TxByte((dev_addr << 1) | 0x01) != I2C_OK) goto error;
    // 4. Приём одного байта с ответом NACK и STOP
    if (I2C_RxByte(0, data) != I2C_OK)             goto error;
    I2C_Stop();
    return I2C_OK;

error:
    I2C_Stop();
    return I2C_ERR;
}

//=============================================================
// Запись одного байта в регистр ведомого:
// S | ADDR+W | A | REG | A | DATA | A | P
// Возвращает: I2C_OK либо I2C_ERR при ошибке
//=============================================================
uint8_t I2C_WriteByte(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    i2c_byte = 0;
    if (I2C_Start() != I2C_OK)                     goto error;
    if (I2C_TxByte((dev_addr << 1) & 0xFE) != I2C_OK) goto error;
    if (I2C_TxByte(reg) != I2C_OK)                 goto error;
    if (I2C_TxByte(value) != I2C_OK)               goto error;
    I2C_Stop();
    return I2C_OK;

error:
    I2C_Stop();
    return I2C_ERR;
}

//=============================================================
// ДАТЧИК BMP180
//=============================================================
// Чтение 16-битного значения (старший байт по адресу reg).
// Каждый байт читается отдельной транзакцией, поэтому
// автоинкремент адреса регистра у датчика не требуется.
static uint8_t BMP180_Read16(uint8_t reg, uint16_t *value)
{
    uint8_t msb, lsb;

    if (I2C_ReadByte(BMP180_ADDR, reg, &msb) != I2C_OK)     return I2C_ERR;
    if (I2C_ReadByte(BMP180_ADDR, reg + 1U, &lsb) != I2C_OK) return I2C_ERR;
    *value = (uint16_t)((msb << 8) | lsb);
    return I2C_OK;
}

// Чтение 11 калибровочных коэффициентов из EEPROM датчика.
// Значения 0x0000 и 0xFFFF означают ошибку обмена.
uint8_t BMP180_ReadCalibration(void)
{
    uint16_t raw[11];

    for (uint8_t i = 0; i < 11U; i++) {
        if (BMP180_Read16(BMP180_CALIB + 2U * i, &raw[i]) != I2C_OK)
            return I2C_ERR;
        if (raw[i] == 0x0000U || raw[i] == 0xFFFFU)
            return I2C_ERR;
    }
    cal.AC1 = (int16_t)raw[0];
    cal.AC2 = (int16_t)raw[1];
    cal.AC3 = (int16_t)raw[2];
    cal.AC4 = raw[3];
    cal.AC5 = raw[4];
    cal.AC6 = raw[5];
    cal.B1  = (int16_t)raw[6];
    cal.B2  = (int16_t)raw[7];
    cal.MB  = (int16_t)raw[8];
    cal.MC  = (int16_t)raw[9];
    cal.MD  = (int16_t)raw[10];
    return I2C_OK;
}

// Одно измерение: t – температура в 0,1 °C, p – давление в Па
uint8_t BMP180_Measure(int32_t *t, int32_t *p)
{
    // время преобразования давления для OSS = 0...3, мс
    static const uint8_t conv_ms[4] = {5, 8, 14, 26};
    uint16_t ut;
    uint8_t  msb, lsb, xlsb;
    int32_t  up;

    // 1. Некомпенсированная температура UT: запись 0x2E в 0xF4,
    //    ожидание 4,5 мс, чтение 0xF6 и 0xF7
    if (I2C_WriteByte(BMP180_ADDR, BMP180_MEAS_CTRL,
                      BMP180_CMD_TEMP) != I2C_OK) return I2C_ERR;
    delay_ms(5);
    if (BMP180_Read16(BMP180_OUT_MSB, &ut) != I2C_OK) return I2C_ERR;

    // 2. Некомпенсированное давление UP: запись 0x34 + (OSS << 6)
    //    в 0xF4, ожидание, чтение 0xF6, 0xF7, 0xF8
    if (I2C_WriteByte(BMP180_ADDR, BMP180_MEAS_CTRL,
                      BMP180_CMD_PRES + (BMP180_OSS << 6)) != I2C_OK)
        return I2C_ERR;
    delay_ms(conv_ms[BMP180_OSS]);
    if (I2C_ReadByte(BMP180_ADDR, BMP180_OUT_MSB,  &msb)  != I2C_OK ||
        I2C_ReadByte(BMP180_ADDR, BMP180_OUT_LSB,  &lsb)  != I2C_OK ||
        I2C_ReadByte(BMP180_ADDR, BMP180_OUT_XLSB, &xlsb) != I2C_OK)
        return I2C_ERR;
    up = (((int32_t)msb << 16) | ((int32_t)lsb << 8) | xlsb)
         >> (8 - BMP180_OSS);

    // 3. Компенсация по калибровочным коэффициентам
    BMP180_Compensate((int32_t)ut, up, t, p);
    return I2C_OK;
}

// Расчёт истинных температуры и давления
// (алгоритм из документации BMP180, раздел 3.5)
void BMP180_Compensate(int32_t ut, int32_t up, int32_t *t, int32_t *p)
{
    int32_t  x1, x2, x3, b3, b5, b6, pr;
    uint32_t b4, b7;

    // температура
    x1 = ((ut - (int32_t)cal.AC6) * (int32_t)cal.AC5) >> 15;
    x2 = ((int32_t)cal.MC << 11) / (x1 + cal.MD);
    b5 = x1 + x2;
    *t = (b5 + 8) >> 4;                         // 0,1 °C

    // давление
    b6 = b5 - 4000;
    x1 = (cal.B2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (cal.AC2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = ((((int32_t)cal.AC1 * 4 + x3) << BMP180_OSS) + 2) / 4;
    x1 = (cal.AC3 * b6) >> 13;
    x2 = (cal.B1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = ((uint32_t)cal.AC4 * (uint32_t)(x3 + 32768)) >> 15;
    b7 = ((uint32_t)up - (uint32_t)b3) * (50000UL >> BMP180_OSS);
    if (b7 < 0x80000000UL) pr = (int32_t)((b7 * 2UL) / b4);
    else                   pr = (int32_t)((b7 / b4) * 2UL);
    x1 = (pr >> 8) * (pr >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * pr) >> 16;
    *p = pr + ((x1 + x2 + 3791) >> 4);          // Па
}

//=============================================================
// ВЫВОД НА ЖКИ
//=============================================================
// Десятичное число, дополненное слева пробелами до width знаков
static void LCD_print_dec(int32_t value, uint8_t width)
{
    char buf[11];
    uint8_t n = 0;

    if (value < 0) value = 0;
    do {
        buf[n++] = (char)('0' + value % 10);
        value /= 10;
    } while (value && n < sizeof(buf));
    while (width > n) { LCD_send_char(' '); width--; }
    while (n) LCD_send_char(buf[--n]);
}

static void LCD_print_hex(uint8_t value)
{
    LCD_send_char("0123456789ABCDEF"[value >> 4]);
    LCD_send_char("0123456789ABCDEF"[value & 0x0F]);
}

// Строка 2: причина ошибки, например "I2C err NACK b1"
// (b1 – байт адреса, b2 – номер регистра, b3 – адрес чтения
// или данные записи), "I2C err SCL=0" или "I2C err SDA=0"
static void LCD_show_error(void)
{
    LCD_pos(2, 1);
    LCD_send_string("I2C err ");
    if (i2c_err == I2C_E_NACK) {
        LCD_send_string("NACK b");
        LCD_send_char((char)('0' + i2c_byte));
        LCD_send_char(' ');
    }
    else if (i2c_err == I2C_E_SCL) LCD_send_string("SCL=0   ");
    else if (i2c_err == I2C_E_SDA) LCD_send_string("SDA=0   ");
    else                           LCD_send_string("        ");
}
