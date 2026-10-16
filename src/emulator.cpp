#include "emulator.h"

static inline bool due(uint32_t now, uint32_t at) { return (int32_t)(now - at) >= 0; }

static const uint32_t PERIOD_US = CFG_PACKET_PERIOD_US;
static const uint32_t HELLO_US  = CFG_HELLO_PERIOD_MS * 1000UL;

void Emulator::begin(uint32_t now_ms, uint32_t now_us)
{
    _last_ms = now_ms;
    _now_us = now_us;
    model.begin();
    setMode(0);
}

void Emulator::powerOn(uint32_t now_us)
{
    _link = (_m.faults & FAULT_NO_REPLY) ? LINK_OFF : LINK_BOOT;
    _next_us = now_us + CFG_BOOT_DELAY_MS * 1000UL;
    _frame = 0;
    _rxn = 0;
}

void Emulator::setMode(uint8_t idx)
{
    if (idx >= MODE_COUNT) idx = 0;
    _idx = idx;
    memcpy_P(&_m, &MODES[idx], sizeof(ModeDesc));
    _t = 0;
    _in = DriverInput();
    model.reset(_m.start_soc >= 0 ? _m.start_soc / 100.0f : -1.0f,
                _m.ambient_c, _m.t_motor0, _m.t_ctrl0, _m.dev);
    powerOn(_now_us);
}

void Emulator::update(uint32_t now_ms)
{
    float dt = (now_ms - _last_ms) * 0.001f;
    if (dt < 0.02f) return;                 // шаг ~20 мс
    _last_ms = now_ms;
    if (dt > 0.2f) dt = 0.2f;               // после долгой передачи/паузы не «прыгаем»
    _in = DriverInput();
    _m.profile(_m, _t, model, _in);
    model.step(dt, dt * _m.speed, _in);
    _t += dt;
}

void Emulator::onRxByte(uint8_t b, uint32_t now_us)
{
    if (_rxn < FD_CMD_LEN) _rxw[_rxn++] = b;
    else { memmove(_rxw, _rxw + 1, FD_CMD_LEN - 1); _rxw[FD_CMD_LEN - 1] = b; }
    if (_rxn < FD_CMD_LEN || _rxw[0] != 0xAA) return;

    int16_t cmd = fdParseCommand(_rxw);
    if (cmd < 0) {
        if ((uint8_t)(_rxw[1] ^ _rxw[2]) == 0xFF) { ++ka_bad; _rxn = 0; }   // похоже на команду, но сумма не сошлась
        return;
    }
    _rxn = 0;
    last_cmd = cmd;
    if (cmd != 0x13) { ++cmd_other; return; }   // запись параметров и пр. — не эмулируется
    ++ka_ok;
    _last_ka_us = now_us;
    if (_link == LINK_HELLO && !(_m.faults & FAULT_NO_AUTH)) {
        _link = LINK_STREAM;
        _next_us = now_us + CFG_AUTH_REPLY_DELAY_MS * 1000UL;
        _stream_us = now_us;
        _frame = 0;
    }
}

uint8_t Emulator::poll(uint32_t now_us, uint8_t *out)
{
    _now_us = now_us;
    switch (_link) {
    case LINK_OFF:
        return 0;

    case LINK_BOOT:
        if (!due(now_us, _next_us)) return 0;
        _link = LINK_HELLO;
        _next_us = now_us;
        // fallthrough
    case LINK_HELLO:
        if (!due(now_us, _next_us)) return 0;
        _next_us += HELLO_US;
        if (due(now_us, _next_us)) _next_us = now_us + HELLO_US;
        memcpy_P(out, FD_HELLO, FD_HELLO_LEN);
        ++hellos;
        return FD_HELLO_LEN;

    case LINK_STREAM:
        break;
    }

    // ---- поток данных ----
#if CFG_KEEPALIVE_TIMEOUT_MS
    if ((uint32_t)(now_us - _last_ka_us) > CFG_KEEPALIVE_TIMEOUT_MS * 1000UL) {
        _link = LINK_HELLO;                 // хост пропал — снова ждём рукопожатия
        _next_us = now_us;
        return 0;
    }
#endif
    uint32_t since = now_us - _stream_us;
    if ((_m.faults & FAULT_REBOOT) && since >= 20000000UL) {
        _link = LINK_BOOT;                  // «перезагрузка»: 1.5 с тишины, потом AT+VERSION
        _next_us = now_us + 1500000UL;
        _frame = 0;
        return 0;
    }
    if (!due(now_us, _next_us)) return 0;
    _next_us += PERIOD_US;
    if (due(now_us, _next_us)) _next_us = now_us + PERIOD_US;   // отстали — не догоняем пачкой

    uint8_t idx = _frame;
    _frame = (uint8_t)((_frame + 1) % FD_FRAME_COUNT);

    if ((_m.faults & FAULT_DROPOUT) && (since / 1000000UL) % 15 >= 12) return 0;   // пропал на 3 с

    uint8_t len = fdBuildFrame(idx, model, out);
    ++frames;

    if ((_m.faults & FAULT_BAD_CRC) && frames % 3 == 0) out[15] ^= 0x5A;
    if (_m.faults & FAULT_GLUE) {
        if (frames % 13 == 0) {             // два пакета подряд без паузы
            len += fdBuildFrame(_frame, model, out + FD_FRAME_LEN);
            _frame = (uint8_t)((_frame + 1) % FD_FRAME_COUNT);
            _next_us += PERIOD_US;
            ++frames;
        } else if (frames % 17 == 0) {      // пакет оборвался
            len = 11;
        }
    }
    return len;
}
