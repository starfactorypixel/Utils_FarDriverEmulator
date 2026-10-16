#pragma once
#include "platform.h"
#include "config.h"

// Передача (D0 биты 0-1 пакета 0x80, как в ПО Fardriver)
enum : uint8_t { GEAR_N = 0, GEAR_D = 1, GEAR_R = 2, GEAR_LOW = 3 };

// Биты ошибок (D2-3 пакета 0x80), названия — из ПО Fardriver
enum : uint16_t {
    ERR_HALL       = 1u << 0,    // 1.Motor Encoder Alarm
    ERR_PEDAL      = 1u << 1,    // 2.Acceleration pedal Alarm
    ERR_CUR_RESTART= 1u << 2,    // 3.Current Protect Restart  (не моделируется)
    ERR_VOLTAGE    = 1u << 4,    // 5.Over / 18.Under Voltage Alarm (уточняет бит15 D4-5 пакета 0xB3)
    ERR_MOTOR_OT   = 1u << 6,    // 7.Motor Over Temp
    ERR_CTRL_OT    = 1u << 7,    // 8.Controller Over Temp
    ERR_BRAKE      = 1u << 15,   // ПО исключает этот бит из «есть ошибка» (errors & 0x7FFF) — статус тормоза
};

// Неисправности самого «устройства» (не канала связи)
enum : uint8_t {
    DEV_NONE  = 0,
    DEV_HALL  = 1 << 0,          // оборван датчик Холла: холл = 111, ошибка бит 0, мотор не крутит
};

// Что «делает водитель» в данный момент (выдаёт профиль режима)
struct DriverInput {
    float   throttle = 0;   // 0..1
    bool    brake = false;  // ручка тормоза (рекуперация + механический тормоз)
    uint8_t gear = GEAR_N;  // GEAR_*
    float   grade = 0;      // уклон дороги, доля (0.08 = 8 % в гору)
    float   push = 0;       // внешняя сила (толкают руками), Н, + вперёд
    float   regen = 1;      // множитель тока рекуперации
};

// Модель: батарея + мотор-колесо + контроллер (FOC, ограничения, защиты) + транспорт.
// «Истинное» состояние (private) отделено от того, что контроллер «измеряет» и отдаёт.
class MotorModel {
public:
    void begin();
    // soc 0..1 (<0 — не менять); ambient, °C; t_motor0/t_ctrl0 — начальные температуры (-128 — = ambient)
    void reset(float soc, int8_t ambient, int8_t t_motor0, int8_t t_ctrl0, uint8_t dev_faults);
    // dt — реальное время шага, dts — модельное (ускоренное) для тепла/батареи/пробега
    void step(float dt, float dts, const DriverInput &in);

    // ---------- то, что «видит» и отдаёт контроллер ----------
    uint8_t  gear, hall;            // gear 0..3, hall 0..7
    bool     moving, against;       // D0 бит5 (движется), бит4 (вращение против выбранного направления)
    bool     weak;                  // D0 бит6 — ослабление поля
    uint16_t errors;                // D2-3
    bool     ov_flag;               // бит15 D4-5 пакета 0xB3: 1 — перенапряжение, 0 — недонапряжение
    uint16_t rpm;                   // |об/мин| мотора
    uint16_t mod_x128;              // D4-5 0x80: коэффициент модуляции ×128
    int16_t  id_x16, iq_x16;        // D8-9 / D10-11 0x80: токи по осям d/q ×16 (назначение — предположение)
    int16_t  idref_x16, iqref_x16;  // D6-7 / D8-9 0x81 (предположение: заданные токи)
    uint16_t v_x10, v2_x10;         // D0-1 / D2-3 0x81
    int16_t  idc_x4;                // D4-5 0x81: ток батареи ×4, < 0 — рекуперация
    int16_t  throttle_raw;          // D10-11 0x81
    float    iph_a, iph2_a;         // амплитуда фазного тока (пакет 0x82, D4-6/D7-9)
    int16_t  t_ctrl, t_motor;       // °C
    uint32_t odo_01km;              // пробег, 0.1 км
    uint16_t adc_ofs_a, adc_ofs_b;  // нули датчиков тока (D0-1/D2-3 пакета 0xB6, ~2040)
    uint16_t noise16;               // «шумное» служебное поле D8-9 пакета 0xB5

    // ---------- для консоли/тестов ----------
    float speed_kmh() const { return _v * 3.6f; }
    float rpmSigned() const { return _rpm; }
    float vbus() const { return _vbus; }
    float idc() const { return _idc; }
    float soc() const { return _soc; }
    float tCtrl() const { return _tc; }
    float tMotor() const { return _tm; }
    float throttle() const { return _thr; }
    bool  brake() const { return _brake; }

private:
    float rnd();                    // −1..1
    float ocv() const;
    void  alarms(float dt, const DriverInput &in);
    void  measure();

    float _v = 0, _rpm = 0, _theta = 0;
    float _iq = 0, _id = 0, _iq_cmd = 0, _id_cmd = 0;
    float _idc = 0, _vbus = 80, _soc = 0.8f;
    float _tc = 25, _tm = 25, _amb = 25;
    float _thr = 0, _odo_m = 0;
    bool  _brake = false;
    uint8_t _dev = 0;
    bool  _pedal_check = true;
    float _pedal_t = 0, _uv_t = 0, _uv_rec_t = 0, _ov_t = 0, _ov_rec_t = 0;
    uint32_t _rng = 1;
};
