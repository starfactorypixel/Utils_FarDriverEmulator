#pragma once
#include "platform.h"
#include "motor_model.h"
#include "fd_packet.h"
#include "modes.h"

#define EMU_TX_MAX (2 * FD_FRAME_LEN)   // самый длинный «залп» — два склеенных пакета

enum LinkState : uint8_t {
    LINK_BOOT = 0,     // «загружается», молчит
    LINK_HELLO,        // шлёт AT+VERSION, ждёт AA 13 …
    LINK_STREAM,       // шлёт пакеты 0x80…0xB6 по кругу
    LINK_OFF,          // молчит совсем (FAULT_NO_REPLY)
};

// Связывает модель, режимы и протокол. Не зависит от железа:
// время передаётся снаружи, байты — через onRxByte()/poll().
class Emulator {
public:
    void begin(uint32_t now_ms, uint32_t now_us);
    void setMode(uint8_t idx);                    // смена режима = «включение питания» контроллера
    void nextMode()    { setMode((_idx + 1) % MODE_COUNT); }
    void prevMode()    { setMode((_idx + MODE_COUNT - 1) % MODE_COUNT); }
    void restartMode() { setMode(_idx); }

    void    update(uint32_t now_ms);              // шаг физики (вызывать часто)
    void    onRxByte(uint8_t b, uint32_t now_us); // байт от опрашивающей платы
    // Если пора что-то передать — заполняет out (до EMU_TX_MAX байт) и возвращает длину,
    // иначе 0. Байты одного «залпа» передавать подряд, без пауз.
    uint8_t poll(uint32_t now_us, uint8_t *out);

    uint8_t         modeIndex() const { return _idx; }
    const ModeDesc &mode() const      { return _m; }
    float           modeTime() const  { return _t; }
    LinkState       link() const      { return _link; }
    const DriverInput &input() const  { return _in; }

    MotorModel model;
    uint32_t frames = 0, hellos = 0;          // отправлено пакетов данных / AT+VERSION
    uint32_t ka_ok = 0, ka_bad = 0, cmd_other = 0;   // принято команд хоста
    int16_t  last_cmd = -1;

private:
    void powerOn(uint32_t now_us);

    ModeDesc    _m;
    DriverInput _in;
    uint8_t  _idx = 0;
    float    _t = 0;                  // реальное время с начала режима, с
    uint32_t _last_ms = 0;
    uint32_t _now_us = 0;
    LinkState _link = LINK_BOOT;
    uint32_t _next_us = 0;            // момент следующей передачи
    uint32_t _stream_us = 0;          // начало потока (для FAULT_REBOOT)
    uint32_t _last_ka_us = 0;
    uint8_t  _frame = 0;              // следующий индекс пакета 0…54
    uint8_t  _rxw[FD_CMD_LEN];        // окно последних принятых байт
    uint8_t  _rxn = 0;
};
