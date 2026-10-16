#pragma once
#include "platform.h"
#include "motor_model.h"

// =====================================================================
//  Новый протокол Fardriver (по дампам и дизассемблеру ПО Fardriver)
//
//  Контроллер сам шлёт по кругу 55 пакетов по 16 байт:
//     AA | A1 | D0 … D11 | CRC_lo CRC_hi
//  A1 = 0x80 | idx, idx = 0…54 (0x80…0xB6). D0…D11 — шесть 16-битных
//  little-endian слов «памяти» контроллера с адреса FD_ADDR_MAP[idx]
//  (таблица из ПО: «fID → real fID»). Адреса 0xE2/0xE8/0xEE — оперативные
//  данные, повторяются 8 раз за цикл, остальное — параметры/служебное.
//  CRC — CRC-16/MODBUS (полином 0xA001) с начальным 0x7F3C по байтам 0…13.
//
//  До рукопожатия контроллер шлёт «AT+VERSION» (10 байт) каждые ~68 мс.
//  Хост отвечает 8-байтной командой  AA 13 EC 07 01 F1 A2 5D :
//     [0]=AA [1]=cmd [2]=~cmd [3..5]=параметры [6]=сумма[0..5] [7]=~[6]
// =====================================================================

#define FD_FRAME_LEN   16
#define FD_FRAME_COUNT 55
#define FD_HELLO_LEN   10
#define FD_CMD_LEN     8

// Адреса слов (real fID), которые эмулятор заполняет из модели
enum : uint8_t {
    FD_ADDR_ODO_LO = 0x69,   // пакет 0x94: D8-9  — младшее слово пробега (0.1 км)
    FD_ADDR_ODO_HI = 0x7C,   // пакет 0x98: D10-11 — старшее слово пробега
    FD_ADDR_B3     = 0xD6,   // пакет 0xB3: D2-3 бит11 (обрыв/КЗ фазы), D4-5 бит15 (пере-/недонапряжение), D10-11 T контроллера
    FD_ADDR_E2     = 0xE2,   // 0x80,87,8E,95,9C,A3,AA,B0: режим, холлы, ошибки, модуляция, RPM, Id, Iq
    FD_ADDR_E8     = 0xE8,   // 0x81,…: напряжение, 2-е напряжение, ток батареи, Iq*/Id*, газ
    FD_ADDR_EE     = 0xEE,   // 0x82,…: фазные токи (24-бит, √x·125/64)
    FD_ADDR_F4     = 0xF4,   // пакет 0xB5: D0-1 T мотора
    FD_ADDR_FA     = 0xFA,   // пакет 0xB6: нули датчиков тока
};

extern const uint8_t FD_HELLO[FD_HELLO_LEN] PROGMEM;   // "AT+VERSION"

uint8_t  fdAddrOf(uint8_t idx);
uint16_t fdCrc16(const uint8_t *p, uint8_t len);
// Собрать пакет idx (0…54) из «измерений» модели. Возвращает 16.
uint8_t  fdBuildFrame(uint8_t idx, const MotorModel &m, uint8_t *out);
// Проверка 8-байтной команды хоста. Возвращает cmd (байт 1) или -1.
int16_t  fdParseCommand(const uint8_t *p);
