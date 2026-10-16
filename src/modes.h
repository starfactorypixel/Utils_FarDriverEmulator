#pragma once
#include "platform.h"
#include "motor_model.h"

// Неисправности канала связи (для проверки драйвера)
enum : uint8_t {
    FAULT_NONE     = 0,
    FAULT_NO_REPLY = 1 << 0,   // контроллер молчит полностью
    FAULT_NO_AUTH  = 1 << 1,   // шлёт AT+VERSION, но рукопожатие не принимает
    FAULT_BAD_CRC  = 1 << 2,   // каждый 3-й пакет с испорченной CRC
    FAULT_GLUE     = 1 << 3,   // иногда два пакета без паузы (32 байта) и обрезанный пакет
    FAULT_DROPOUT  = 1 << 4,   // каждые 15 с пропадает на 3 с, потом продолжает поток
    FAULT_REBOOT   = 1 << 5,   // каждые 20 с «перезагружается»: тишина, AT+VERSION, нужно новое рукопожатие
};

struct ModeDesc;
// Профиль режима — «водитель»: по времени t (реальные секунды с начала режима)
// и текущему состоянию модели выставляет газ, тормоз, передачу, уклон.
typedef void (*ProfileFn)(const ModeDesc &m, float t, const MotorModel &mm, DriverInput &in);

struct ModeDesc {
    const char *name;       // строка в PROGMEM
    ProfileFn   profile;
    float       p1, p2, p3; // параметры профиля (смысл — у каждой функции свой)
    int8_t      start_soc;  // SOC батареи при входе в режим, %; -1 — продолжить с текущего
    int8_t      ambient_c;  // температура воздуха
    int8_t      t_motor0;   // начальная температура мотора (-128 — как воздух)
    int8_t      t_ctrl0;    // начальная температура контроллера (-128 — как воздух)
    uint8_t     speed;      // ускорение модельного времени для тепла/батареи/пробега (1 = реальное)
    uint8_t     faults;     // FAULT_*
    uint8_t     dev;        // DEV_*
};

extern const ModeDesc MODES[] PROGMEM;
extern const uint8_t  MODE_COUNT;
