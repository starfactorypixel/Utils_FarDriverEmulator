// =====================================================================
//  Эмулятор контроллера мотора Fardriver (новый протокол) на Arduino Nano
//
//  Порт контроллера: SoftwareSerial 19200 8N1, D6 = RX (от опрашивающей
//                    платы), D5 = TX. Контроллер шлёт AT+VERSION, после
//                    рукопожатия AA 13 EC 07 01 F1 A2 5D — поток пакетов
//                    0x80…0xB6 каждые ~28 мс.
//  Кнопка:    D3 -> GND. Короткое нажатие — следующий режим,
//                       долгое (>0.8 с) — перезапуск текущего режима.
//  Светодиод: D13 мигает номером режима (1..N).
//  USB Serial 115200: статус раз в секунду и команды (h — справка).
//
//  Все настройки — в config.h, режимы — в modes.cpp.
// =====================================================================
#include <SoftwareSerial.h>
#include "config.h"
#include "emulator.h"

static SoftwareSerial ctrlPort(PIN_CTRL_RX, PIN_CTRL_TX);
static Emulator emu;
static uint8_t  txb[EMU_TX_MAX];
static uint32_t ledT0 = 0;
static bool     hexDump = false;

// ------------------------------- лог ---------------------------------
#if CFG_DEBUG_BAUD
static const char *const LINK_NAMES[] = { "BOOT", "HELLO", "STREAM", "OFF" };

static void printMode()
{
    Serial.print(F("\r\n>>> Режим "));
    Serial.print(emu.modeIndex() + 1);
    Serial.print('/');
    Serial.print(MODE_COUNT);
    Serial.print(F(": "));
    Serial.println((const __FlashStringHelper *)emu.mode().name);
}

static void printHelp()
{
    Serial.println(F("Команды: n — след. режим, p — пред., r — перезапуск, x — hex-дамп передачи вкл/выкл,"));
    Serial.println(F("         <число>+Enter — выбрать режим, l — список режимов, h — справка"));
}

static void printList()
{
    for (uint8_t i = 0; i < MODE_COUNT; ++i) {
        ModeDesc m;
        memcpy_P(&m, &MODES[i], sizeof(m));
        Serial.print(i + 1);
        Serial.print(F(". "));
        Serial.println((const __FlashStringHelper *)m.name);
    }
}

static void printStatus()
{
    const MotorModel &m = emu.model;
    static const char GEARS[] = "NDRL";
    Serial.print('[');  Serial.print(emu.modeIndex() + 1);
    Serial.print(F("] t=")); Serial.print((uint32_t)emu.modeTime());
    Serial.print(F("s ")); Serial.print(LINK_NAMES[emu.link()]);
    Serial.print(F(" | ")); Serial.print(GEARS[m.gear]);
    Serial.print(F(" thr ")); Serial.print((int)(m.throttle() * 100));
    Serial.print(m.brake() ? F("% BRK ") : F("% "));
    Serial.print(m.speed_kmh(), 1); Serial.print(F("km/h "));
    Serial.print(m.rpmSigned(), 0); Serial.print(F("rpm"));
    if (m.weak) Serial.print(F(" WEAK"));
    Serial.print(F(" | ")); Serial.print(m.vbus(), 1);
    Serial.print(F("V ")); Serial.print(m.idc(), 1);
    Serial.print(F("A SOC ")); Serial.print(m.soc() * 100.0f, 0);
    Serial.print(F("% | Tc ")); Serial.print(m.t_ctrl);
    Serial.print(F(" Tm ")); Serial.print(m.t_motor);
    Serial.print(F(" | err 0x")); Serial.print(m.errors, HEX);
    Serial.print(F(" | tx ")); Serial.print(emu.frames);
    Serial.print(F(" hello ")); Serial.print(emu.hellos);
    Serial.print(F(" req ")); Serial.print(emu.ka_ok);
    Serial.print('/'); Serial.println(emu.ka_bad);
}

static void dump(const uint8_t *p, uint8_t n)
{
    for (uint8_t i = 0; i < n; ++i) {
        if (p[i] < 0x10) Serial.print('0');
        Serial.print(p[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}

static void pollConsole()
{
    static uint8_t num = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c >= '0' && c <= '9') { num = num * 10 + (c - '0'); continue; }
        if ((c == '\r' || c == '\n') && num) { emu.setMode(num - 1); num = 0; ledT0 = millis(); printMode(); continue; }
        num = 0;
        switch (c) {
            case 'n': emu.nextMode();    ledT0 = millis(); printMode(); break;
            case 'p': emu.prevMode();    ledT0 = millis(); printMode(); break;
            case 'r': emu.restartMode(); ledT0 = millis(); printMode(); break;
            case 'x': hexDump = !hexDump; break;
            case 'l': printList(); break;
            case 'h': printHelp(); break;
        }
    }
}
#else
static void printMode() {}
#endif

// ---------------------------- кнопка / LED ---------------------------
static void pollButton(uint32_t now)
{
    static bool raw = HIGH, stable = HIGH, longDone = false;
    static uint32_t tChange = 0, tDown = 0;
    bool r = digitalRead(PIN_BUTTON);
    if (r != raw) { raw = r; tChange = now; }
    if (now - tChange > 30 && raw != stable) {
        stable = raw;
        if (stable == LOW) { tDown = now; longDone = false; }
        else if (!longDone) { emu.nextMode(); ledT0 = now; printMode(); }
    }
    if (stable == LOW && !longDone && now - tDown >= CFG_LONG_PRESS_MS) {
        longDone = true;
        emu.restartMode(); ledT0 = now; printMode();
    }
}

static void ledTask(uint32_t now)
{
    uint32_t n = emu.modeIndex() + 1;
    uint32_t x = (now - ledT0) % (n * 400UL + 1500UL);
    digitalWrite(PIN_LED, (x < n * 400UL && (x % 400UL) < 150UL) ? HIGH : LOW);
}

// --------------------------------------------------------------------
void setup()
{
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
#if CFG_DEBUG_BAUD
    Serial.begin(CFG_DEBUG_BAUD);
    Serial.println(F("\r\nFardriver controller emulator (new protocol)"));
    Serial.print(F("UART ")); Serial.print(CFG_BAUD);
    Serial.print(F(", ")); Serial.print(CFG_BATT_CELLS); Serial.print(F("S "));
    Serial.print(CFG_BATT_AH, 0); Serial.print(F(" Ah, RPMx"));
    Serial.println(CFG_RPM_RAW_PER_RPM);
    printHelp();
    printList();
#endif
    ctrlPort.begin(CFG_BAUD);
    emu.begin(millis(), micros());
    ledT0 = millis();
    printMode();
}

void loop()
{
    uint32_t now = millis();

    while (ctrlPort.available()) emu.onRxByte((uint8_t)ctrlPort.read(), micros());

    uint8_t n = emu.poll(micros(), txb);
    if (n) {
        // Приём на время передачи выключаем. Иначе байт хоста, пришедший посреди пакета, запускает
        // прерывание SoftwareSerial::recv(), которое ~0.5 мс ждёт весь байт: передача встаёт между
        // байтами, принимающая сторона видит IDLE и получает пакет, разрезанный на два.
        ctrlPort.stopListening();
        ctrlPort.write(txb, n);     // ~8.3 мс на 16 байт, байты подряд без пауз
        ctrlPort.listen();
#if CFG_DEBUG_BAUD
        if (hexDump) dump(txb, n);
#endif
    }

    emu.update(now);
    pollButton(now);
    ledTask(now);

#if CFG_DEBUG_BAUD
    pollConsole();
    static uint32_t tStat = 0;
    if (now - tStat >= CFG_STATUS_PERIOD_MS) { tStat = now; printStatus(); }
#endif
}
