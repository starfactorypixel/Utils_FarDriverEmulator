#include "motor_model.h"

// OCV ячейки NMC, шаг 10 % SOC
static const float OCV_TAB[11] PROGMEM = {
    3.000f, 3.450f, 3.550f, 3.620f, 3.670f, 3.730f, 3.810f, 3.900f, 3.980f, 4.070f, 4.180f };

static const float G        = 9.81f;
static const float R_WHEEL  = CFG_WHEEL_DIAMETER_MM * 0.0005f;           // м
static const float KT       = 60.0f / (2.0f * PI * CFG_MOTOR_KV);        // Н·м/А
static const float R_PH     = CFG_MOTOR_R_MOHM * 0.001f;
static const float R_BATT   = CFG_BATT_R_MOHM * 0.001f;
static const float P_IDLE_W = 5.0f;                                       // собственное потребление

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float sgn(float v) { return v > 0 ? 1.0f : (v < 0 ? -1.0f : 0.0f); }

float MotorModel::rnd()
{
    _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
    return (float)(_rng & 0xFFFF) / 32767.5f - 1.0f;
}

float MotorModel::ocv() const
{
    float tab[11];
    memcpy_P(tab, OCV_TAB, sizeof(tab));
    float x = clampf(_soc, 0.0f, 1.0f) * 10.0f;
    uint8_t k = (uint8_t)x;
    if (k >= 10) return tab[10] * CFG_BATT_CELLS;
    return (tab[k] + (tab[k + 1] - tab[k]) * (x - k)) * CFG_BATT_CELLS;
}

void MotorModel::begin()
{
    _rng = CFG_RANDOM_SEED | 1UL;
    odo_01km = CFG_ODOMETER_START;
    _odo_m = 0;
    reset(0.8f, 22, -128, -128, DEV_NONE);
}

void MotorModel::reset(float soc, int8_t ambient, int8_t t_motor0, int8_t t_ctrl0, uint8_t dev_faults)
{
    if (soc >= 0) _soc = soc;
    _amb = ambient;
    _tm = (t_motor0 == -128) ? (float)ambient : (float)t_motor0;
    _tc = (t_ctrl0 == -128) ? (float)ambient : (float)t_ctrl0;
    _dev = dev_faults;
    _v = _rpm = 0;
    _iq = _id = _iq_cmd = _id_cmd = 0;
    _idc = 0;
    _vbus = ocv();
    _thr = 0; _brake = false;
    _pedal_check = true;                     // как после включения питания
    _pedal_t = _uv_t = _uv_rec_t = _ov_t = _ov_rec_t = 0;
    errors = 0; ov_flag = false;
    gear = GEAR_N;
    measure();
}

void MotorModel::alarms(float dt, const DriverInput &in)
{
    uint16_t e = errors & ~(ERR_BRAKE | ERR_HALL);

    if (_dev & DEV_HALL) e |= ERR_HALL;

    // Газ нажат при включении — ошибка педали, пока газ не отпустят на 0.5 с
    if (_pedal_check) {
        if (in.throttle > 0.1f) { e |= ERR_PEDAL; _pedal_t = 0; }
        else if ((_pedal_t += dt) >= 0.5f) { _pedal_check = false; e &= ~ERR_PEDAL; }
    }

    // Недонапряжение: 1 с ниже порога, возврат через 2 с выше порога + 0.5 В
    if (!(e & ERR_VOLTAGE)) {
        _uv_t = (_vbus < CFG_UV_V) ? _uv_t + dt : 0;
        _ov_t = (_vbus > CFG_OV_V) ? _ov_t + dt : 0;
        if (_uv_t >= 1.0f)      { e |= ERR_VOLTAGE; ov_flag = false; _uv_rec_t = 0; }
        else if (_ov_t >= 0.2f) { e |= ERR_VOLTAGE; ov_flag = true;  _ov_rec_t = 0; }
    } else if (ov_flag) {
        _ov_rec_t = (_vbus < CFG_OV_V - 2.0f) ? _ov_rec_t + dt : 0;
        if (_ov_rec_t >= 1.0f) { e &= ~ERR_VOLTAGE; _ov_t = 0; }
    } else {
        _uv_rec_t = (_vbus > CFG_UV_V + 0.5f) ? _uv_rec_t + dt : 0;
        if (_uv_rec_t >= 2.0f) { e &= ~ERR_VOLTAGE; _uv_t = 0; }
    }

    // Перегревы с гистерезисом
    if (_tc >= CFG_CTRL_OT_C) e |= ERR_CTRL_OT;
    else if (_tc < CFG_CTRL_OT_C - 10) e &= ~ERR_CTRL_OT;
    if (_tm >= CFG_MOTOR_OT_C) e |= ERR_MOTOR_OT;
    else if (_tm < CFG_MOTOR_OT_C - 15) e &= ~ERR_MOTOR_OT;

    if (in.brake) e |= ERR_BRAKE;
    errors = e;
}

void MotorModel::step(float dt, float dts, const DriverInput &in)
{
    if (dt <= 0.0f) return;
    _thr = clampf(in.throttle, 0.0f, 1.0f);
    _brake = in.brake;
    gear = in.gear & 3;

    alarms(dt, in);

    // ---------------- 1. Задание тока (контроллер) ----------------
    float omega = _v / R_WHEEL * CFG_GEAR_RATIO;            // рад/с мотора
    _rpm = omega * (60.0f / (2.0f * PI));
    float emf = fabsf(_rpm) / CFG_MOTOR_KV;                 // В (эквивалент на шине)
    float dir = (gear == GEAR_R) ? -1.0f : (gear == GEAR_N ? 0.0f : 1.0f);

    bool blocked = errors & (ERR_HALL | ERR_PEDAL | ERR_CTRL_OT | ERR_MOTOR_OT);
    bool uv = (errors & ERR_VOLTAGE) && !ov_flag;
    bool ov = (errors & ERR_VOLTAGE) && ov_flag;
    float iq_cmd = 0;
    bool regen = false;

    if (!blocked) {
        if (in.brake) {
            if (fabsf(_rpm) > 30.0f && !ov) {               // E-ABS: рекуперация до ~30 об/мин
                iq_cmd = -sgn(_rpm) * CFG_REGEN_PHASE_A * in.regen * clampf(fabsf(_rpm) / 200.0f, 0.0f, 1.0f);
                regen = true;
            }
        } else if (dir != 0 && _thr > 0.02f && !uv) {
            iq_cmd = dir * _thr * CFG_PHASE_A_MAX;
            float pct = (gear == GEAR_LOW) ? CFG_LOW_SPEED_PCT : (gear == GEAR_R ? CFG_REV_SPEED_PCT : 200);
            float rpm_lim = CFG_MOTOR_KV * _vbus * pct * 0.01f;
            float over = dir * _rpm - rpm_lim;              // ограничитель оборотов Low/R
            if (over > 0) iq_cmd *= clampf(1.0f - over / (0.1f * rpm_lim + 1.0f), 0.0f, 1.0f);
            // предел по напряжению; с ослаблением поля — на ~25 % выше
            float vlim = _vbus * (CFG_FIELD_WEAKEN ? 1.25f : 1.0f);
            if (iq_cmd * _rpm >= 0) iq_cmd *= clampf((vlim - emf) / (0.12f * _vbus), 0.0f, 1.0f);
        }
        // температурное снижение тока: от 100 % на пороге DERATE до 60 % на пороге перегрева
        float fc = 1.0f - 0.4f * (_tc - CFG_CTRL_DERATE_C) / (float)(CFG_CTRL_OT_C - CFG_CTRL_DERATE_C);
        float fm = 1.0f - 0.4f * (_tm - CFG_MOTOR_DERATE_C) / (float)(CFG_MOTOR_OT_C - CFG_MOTOR_DERATE_C);
        iq_cmd *= clampf(fc < fm ? fc : fm, 0.6f, 1.0f);
    }

    // ослабление поля: отрицательный Id при противо-ЭДС около напряжения шины
    bool motoring = iq_cmd * _rpm > 0 && fabsf(iq_cmd) > 1.0f;
    float id_cmd = 0;
    if (CFG_FIELD_WEAKEN && motoring && emf > 0.92f * _vbus)
        id_cmd = -0.35f * CFG_PHASE_A_MAX * clampf((emf - 0.92f * _vbus) / (0.3f * _vbus), 0.0f, 1.0f);

    // ограничение тока батареи (прогноз по текущим оборотам)
    {
        float p = KT * iq_cmd * omega + 1.5f * R_PH * (iq_cmd * iq_cmd + id_cmd * id_cmd);
        float i_pred = p / _vbus;
        // при подходе к порогу недонапряжения контроллер плавно урезает ток батареи
        float lim_dsg = CFG_LINE_A_MAX * clampf((_vbus - CFG_UV_V) / 4.0f, 0.15f, 1.0f);
        float lim_chg = CFG_REGEN_LINE_A * in.regen;
        if (i_pred > lim_dsg)        iq_cmd *= lim_dsg / i_pred;
        else if (i_pred < -lim_chg)  iq_cmd *= lim_chg / -i_pred;
    }
    _iq_cmd = iq_cmd; _id_cmd = id_cmd;

    // ток следует за заданием с плавным нарастанием (рампа контроллера)
    float k = dt / ((regen ? 0.08f : 0.2f) + dt);
    _iq += (iq_cmd - _iq) * k;
    _id += (id_cmd - _id) * k;
    weak = _id < -2.0f;

    // ---------------- 2. Электрика ----------------
    float iph2 = _iq * _iq + _id * _id;
    float p_mech = KT * _iq * omega;
    float p_dc = p_mech + 1.5f * R_PH * iph2 + P_IDLE_W + 1.5f * 0.004f * iph2;   // + потери в ключах
    _idc = p_dc / _vbus;
    _vbus = ocv() - _idc * R_BATT;
    _soc = clampf(_soc - _idc * dts / (3600.0f * CFG_BATT_AH), 0.0f, 1.0f);

    // ---------------- 3. Движение ----------------
    float m = CFG_MASS_KG;
    float f_motor = KT * _iq * CFG_GEAR_RATIO / R_WHEEL;
    // толкают руками: сила спадает к скорости шага (~1.5 м/с)
    float push = in.push * clampf(1.0f - (in.push > 0 ? _v : -_v) / 1.5f, 0.0f, 1.0f);
    float f_ext = push - m * G * in.grade;
    float f_aero = 0.5f * 1.2f * CFG_CDA * _v * fabsf(_v);
    float f_fric = CFG_CRR * m * G + (in.brake ? 0.35f * m * G : 0.0f);  // качение + мех. тормоз
    float f_drive = f_motor + f_ext - f_aero;
    if (fabsf(_v) < 0.05f && fabsf(f_drive) <= f_fric) {
        _v = 0;                                          // стоит: трение покоя
    } else {
        float dir_v = (_v != 0) ? sgn(_v) : sgn(f_drive);
        float v_new = _v + (f_drive - dir_v * f_fric) / m * dt;
        if (_v != 0 && v_new * _v < 0 && fabsf(f_drive) <= f_fric) v_new = 0;
        _v = v_new;
    }
    _theta += omega * CFG_MOTOR_POLE_PAIRS * dt;          // электрический угол (для Холла)
    _theta = fmodf(_theta, 2.0f * PI);
    if (_theta < 0) _theta += 2.0f * PI;

    _odo_m += fabsf(_v) * dts;
    while (_odo_m >= 100.0f) { _odo_m -= 100.0f; ++odo_01km; }

    // ---------------- 4. Тепло (модельное время) ----------------
    float pc = 1.5f * 0.004f * iph2 + 0.01f * fabsf(p_dc) + 8.0f;   // проводимость + коммутация + своё
    float pm = 1.5f * R_PH * iph2 + 1.2e-4f * _rpm * _rpm;            // медь + железо
    _tc += (_amb + pc * 0.45f - _tc) * (dts / (300.0f + dts));        // 0.45 К/Вт, τ 5 мин
    _tm += (_amb + pm * 0.12f - _tm) * (dts / (900.0f + dts));        // 0.12 К/Вт, τ 15 мин

    measure();
}

void MotorModel::measure()
{
    static const uint8_t HALL_SEQ[6] = { 1, 3, 2, 6, 4, 5 };

    float arpm = fabsf(_rpm);
    rpm = (arpm < 0.5f) ? 0 : (uint16_t)lroundf(arpm + (arpm > 5 ? 0.6f * rnd() : 0.0f));
    moving = rpm > 0;
    float expected = (gear == GEAR_R) ? -1.0f : 1.0f;
    against = moving && sgn(_rpm) != expected;

    if (_dev & DEV_HALL) hall = 7;
    else hall = HALL_SEQ[(uint8_t)(_theta / (PI / 3.0f)) % 6];

    v_x10  = (uint16_t)lroundf(_vbus * 10.0f + 0.6f * rnd());
    v2_x10 = (uint16_t)lroundf(_vbus * 10.0f - 4.0f + 0.6f * rnd());
    idc_x4 = (int16_t)lroundf(_idc * 4.0f + (fabsf(_idc) > 0.5f ? 0.8f * rnd() : 0.0f));
    iq_x16 = (int16_t)lroundf(_iq * 16.0f - 2.0f + 2.0f * rnd());     // в дампе покоя: −1…−5
    id_x16 = (int16_t)lroundf(_id * 16.0f - 1.0f + 2.0f * rnd());
    iqref_x16 = (int16_t)lroundf(_iq_cmd * 16.0f);
    idref_x16 = (int16_t)lroundf(_id_cmd * 16.0f);

    float iph = sqrtf(_iq * _iq + _id * _id);
    iph_a  = iph;
    iph2_a = iph * (1.0f + 0.03f * rnd());
    if (moving || iph > 1.0f) {
        float emf = arpm / CFG_MOTOR_KV;
        mod_x128 = (uint16_t)lroundf(clampf((emf + iph * R_PH * 1.5f) / _vbus, 0.0f, 1.15f) * 128.0f);
    } else {
        mod_x128 = 0;
    }

    t_ctrl  = (int16_t)lroundf(_tc);
    t_motor = (int16_t)lroundf(_tm);
    throttle_raw = (int16_t)lroundf(CFG_THROTTLE_RAW_IDLE + _thr * (CFG_THROTTLE_RAW_FULL - CFG_THROTTLE_RAW_IDLE));
    adc_ofs_a = 2040 + (int16_t)lroundf(rnd());
    adc_ofs_b = 2039 + (int16_t)lroundf(rnd() * 0.6f);
    noise16   = 2600 + (int16_t)(rnd() * 180.0f);
}
