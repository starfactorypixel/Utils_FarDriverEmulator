// =====================================================================
//  Режимы эмуляции. Чтобы добавить режим:
//   1) (если нужно) написать функцию-профиль «водителя» ниже;
//   2) добавить имя N_xxx и строку в таблицу MODES[].
//  Порядок в таблице = порядок переключения кнопкой.
// =====================================================================
#include "modes.h"

static float hash01(uint32_t x)      // детерминированное «случайное» 0..1
{
    x ^= x >> 16; x *= 0x7FEB352DUL; x ^= x >> 15; x *= 0x846CA68BUL; x ^= x >> 16;
    return (float)(x & 0xFFFF) / 65535.0f;
}

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Неровности дороги: небольшие подъёмы/спуски, чтобы ток «дышал» даже на крейсере
static float road(float t)
{
    return 0.012f * sinf(t * 0.21f) + 0.006f * sinf(t * 0.77f + 1.3f);
}

// «Водитель» держит скорость target (км/ч) газом не больше thr_max
static float hold(const MotorModel &mm, float target, float thr_max, float grade)
{
    return clampf(0.12f * (target - mm.speed_kmh()) + 0.15f + grade * 4.0f, 0.0f, thr_max);
}

// ---------------------------- Профили --------------------------------

// Стоит на передаче p1 (0 N, 1 D, 2 R, 3 Low)
static void prof_idle(const ModeDesc &m, float, const MotorModel &, DriverInput &in)
{
    in.gear = (uint8_t)m.p1;
}

// Цикл 80 с: стоянка, разгон газом ≤ p2 до p1 км/ч, крейсер, торможение, стоянка. p3 — уклон
static void prof_cruise(const ModeDesc &m, float t, const MotorModel &mm, DriverInput &in)
{
    float x = fmodf(t, 80.0f);
    in.gear = GEAR_D;
    in.grade = m.p3 + road(t);
    if (x < 3.0f) return;
    if (x < 60.0f) { in.throttle = hold(mm, m.p1, m.p2, m.p3); return; }
    if (x < 70.0f) { in.brake = true; return; }
}

// Город, цикл 45 с: разгон газом p2 до ~p1 км/ч (каждый цикл по-разному), крейсер,
// накат, торможение с рекуперацией, стоянка. p3 — уклон.
static void prof_city(const ModeDesc &m, float t, const MotorModel &mm, DriverInput &in)
{
    uint32_t c = (uint32_t)(t / 45.0f);
    float x = fmodf(t, 45.0f);
    float target = m.p1 * (0.55f + 0.45f * hash01(c));
    in.gear = GEAR_D;
    in.grade = m.p3 + road(t);
    if (x < 2.0f)  return;                                              // стоим
    if (x < 25.0f) { in.throttle = (mm.speed_kmh() < target - 3.0f) ? m.p2 : hold(mm, target, m.p2, m.p3); return; }
    if (x < 28.0f) return;                                              // накат
    if (x < 36.0f) { in.brake = true; return; }                         // рекуперация + тормоз
    in.brake = (x < 38.0f);                                             // стоим
}

// Задний ход, цикл 14 с: N, R стоя, R с газом p1, тормоз, N
static void prof_reverse(const ModeDesc &m, float t, const MotorModel &, DriverInput &in)
{
    float x = fmodf(t, 14.0f);
    in.gear = (x < 1.0f || x >= 11.0f) ? GEAR_N : GEAR_R;
    if (x >= 2.0f && x < 8.0f) in.throttle = m.p1;
    if (x >= 8.0f && x < 11.0f) in.brake = true;
}

// Подъём: держит p1 км/ч на уклоне p2 на передаче p3 (если не тянет — полный газ)
static void prof_hill(const ModeDesc &m, float t, const MotorModel &mm, DriverInput &in)
{
    in.gear = (uint8_t)m.p3;
    in.grade = m.p2 + 0.5f * road(t);
    if (t < 2.0f || (mm.errors & 0x7FFF)) {          // ждёт «загрузки» контроллера; при ошибке — держит тормоз
        in.brake = true;
        return;
    }
    in.throttle = hold(mm, m.p1, 1.0f, m.p2);
}

// Толкают руками с силой p1 Н, цикл 24 с: в N вперёд, в N назад, в D назад (вращение против передачи)
static void prof_push(const ModeDesc &m, float t, const MotorModel &, DriverInput &in)
{
    float x = fmodf(t, 24.0f);
    in.gear = (x < 14.0f) ? GEAR_N : GEAR_D;
    if (x < 4.0f)                    in.push = m.p1;
    else if (x >= 8.0f && x < 12.0f) in.push = -m.p1;
    else if (x >= 15.0f && x < 19.0f) in.push = -m.p1;
}

// Полный газ на ровной дороге до максимума (ослабление поля), цикл 55 с
static void prof_topspeed(const ModeDesc &, float t, const MotorModel &, DriverInput &in)
{
    float x = fmodf(t, 55.0f);
    in.gear = GEAR_D;
    if (x < 3.0f) return;
    if (x < 40.0f) { in.throttle = 1.0f; return; }
    in.brake = x < 50.0f;
}

// Спуск с уклоном p1 (< 0): накат, затем торможение рекуперацией ×p2, цикл 30 с
static void prof_downhill(const ModeDesc &m, float t, const MotorModel &, DriverInput &in)
{
    float x = fmodf(t, 30.0f);
    in.gear = GEAR_D;
    in.grade = m.p1;
    in.regen = m.p2;
    in.brake = (x >= 12.0f && x < 26.0f);
}

// Газ p3 зажат при включении 4 с, затем отпущен; дальше — город (p1, p2 как у prof_city)
static void prof_pedal(const ModeDesc &m, float t, const MotorModel &mm, DriverInput &in)
{
    if (t < 4.0f) { in.gear = GEAR_D; in.throttle = m.p3; return; }
    ModeDesc c = m;
    c.p3 = 0;                                       // уклона нет
    if (t < 7.0f) { in.gear = GEAR_D; return; }
    prof_city(c, t - 7.0f, mm, in);
}

// Неисправен датчик Холла: водитель жмёт газ p1, потом толкает (цикл 12 с)
static void prof_tryhall(const ModeDesc &m, float t, const MotorModel &, DriverInput &in)
{
    float x = fmodf(t, 12.0f);
    in.gear = GEAR_D;
    if (x >= 2.0f && x < 5.0f) in.throttle = m.p1;
    else if (x < 9.0f) in.push = 200.0f;
}

// ---------------------------- Таблица режимов ------------------------
static const char N_IDLE[]    PROGMEM = "Стоянка, нейтраль";
static const char N_CRUISE[]  PROGMEM = "Разгон и крейсер 50 км/ч";
static const char N_CITY[]    PROGMEM = "Город: разгоны, рекуперация, стоянки";
static const char N_REVERSE[] PROGMEM = "Задний ход";
static const char N_LOW[]     PROGMEM = "Передача Low, подъём 6 %";
static const char N_PUSH[]    PROGMEM = "Толкают руками (N вперёд/назад, D назад)";
static const char N_TOP[]     PROGMEM = "Максимальная скорость, ослабление поля";
static const char N_CTRLHOT[] PROGMEM = "Затяжной подъём: перегрев контроллера";
static const char N_MOTHOT[]  PROGMEM = "Подъём с грузом: перегрев мотора";
static const char N_LOWBAT[]  PROGMEM = "Батарея разряжена: отсечка по напряжению";
static const char N_OVOLT[]   PROGMEM = "Полная батарея, спуск: перенапряжение";
static const char N_PEDAL[]   PROGMEM = "Газ зажат при включении";
static const char N_HALL[]    PROGMEM = "Обрыв датчика Холла";
static const char N_NOREPLY[] PROGMEM = "Сбой: нет ответа";
static const char N_NOAUTH[]  PROGMEM = "Сбой: нет рукопожатия (только AT+VERSION)";
static const char N_BADCRC[]  PROGMEM = "Сбой: битая CRC (каждый 3-й пакет)";
static const char N_GLUE[]    PROGMEM = "Сбой: склеенные и обрезанные пакеты";
static const char N_DROP[]    PROGMEM = "Сбой: пропадает на 3 с каждые 15 с";
static const char N_REBOOT[]  PROGMEM = "Сбой: перезагрузка каждые 20 с";

const ModeDesc MODES[] PROGMEM = {
//   name       profile         p1      p2      p3     soc amb  Tmot  Tctl speed faults           dev
    { N_IDLE,    prof_idle,     0,      0,      0,      80, 22, -128, -128, 1,  FAULT_NONE,     DEV_NONE },
    { N_CRUISE,  prof_cruise,   50.0f,  0.8f,   0,      85, 22, -128, -128, 1,  FAULT_NONE,     DEV_NONE },
    { N_CITY,    prof_city,     60.0f,  1.0f,   0,      75, 24, -128, -128, 1,  FAULT_NONE,     DEV_NONE },
    { N_REVERSE, prof_reverse,  0.6f,   0,      0,      70, 22, -128, -128, 1,  FAULT_NONE,     DEV_NONE },
    { N_LOW,     prof_hill,     60.0f,  0.06f,  GEAR_LOW, 70, 22, -128, -128, 1, FAULT_NONE,   DEV_NONE },
    { N_PUSH,    prof_push,     220.0f, 0,      0,      70, 22, -128, -128, 1,  FAULT_NONE,     DEV_NONE },
    { N_TOP,     prof_topspeed, 0,      0,      0,      95, 22, -128, -128, 1,  FAULT_NONE,     DEV_NONE },
    { N_CTRLHOT, prof_hill,     20.0f,  0.22f,  GEAR_D, 90, 30,   40,   75, 10, FAULT_NONE,     DEV_NONE },
    { N_MOTHOT,  prof_hill,     20.0f,  0.25f,  GEAR_D, 90, 15,  114,   15, 10, FAULT_NONE,     DEV_NONE },
    { N_LOWBAT,  prof_city,     70.0f,  1.0f,   0.02f,   1, 15, -128, -128, 1,  FAULT_NONE,     DEV_NONE },
    { N_OVOLT,   prof_downhill, -0.10f, 3.0f,   0,     100, 22, -128, -128, 1,  FAULT_NONE,     DEV_NONE },
    { N_PEDAL,   prof_pedal,    50.0f,  1.0f,   0.6f,      80, 22, -128, -128, 1,  FAULT_NONE,     DEV_NONE },
    { N_HALL,    prof_tryhall,  0.5f,   0,      0,      80, 22, -128, -128, 1,  FAULT_NONE,     DEV_HALL },
    { N_NOREPLY, prof_idle,     0,      0,      0,      -1, 22, -128, -128, 1,  FAULT_NO_REPLY, DEV_NONE },
    { N_NOAUTH,  prof_idle,     0,      0,      0,      -1, 22, -128, -128, 1,  FAULT_NO_AUTH,  DEV_NONE },
    { N_BADCRC,  prof_city,     50.0f,  0.7f,   0,      -1, 22, -128, -128, 1,  FAULT_BAD_CRC,  DEV_NONE },
    { N_GLUE,    prof_city,     50.0f,  0.7f,   0,      -1, 22, -128, -128, 1,  FAULT_GLUE,     DEV_NONE },
    { N_DROP,    prof_city,     50.0f,  0.7f,   0,      -1, 22, -128, -128, 1,  FAULT_DROPOUT,  DEV_NONE },
    { N_REBOOT,  prof_city,     50.0f,  0.7f,   0,      -1, 22, -128, -128, 1,  FAULT_REBOOT,   DEV_NONE },
};

const uint8_t MODE_COUNT = sizeof(MODES) / sizeof(MODES[0]);
