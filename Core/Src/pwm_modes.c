#include "pwm_modes.h"
#include <math.h>
#include <stdlib.h>

// Вспомогательная функция для получения текущего шага с учетом множителя
static float get_current_step(void) {
    extern pwm_config_t global_pwm_cfg;
    return (float)global_pwm_cfg.step_hz * global_pwm_cfg.step_multiplier;
}

// --- Универсальный линейный алгоритм (для Режимов 1, 3, 4) ---
static float linear_logic(float current_freq, bool *increasing) {
    extern pwm_config_t global_pwm_cfg;
    float step = get_current_step();

    if (*increasing) {
        float next = current_freq + step;
        if (next >= (float)global_pwm_cfg.freq_max) {
            *increasing = false;
            return (float)global_pwm_cfg.freq_max;
        }
        return next;
    } else {
        float next = current_freq - step;
        if (next <= (float)global_pwm_cfg.freq_min) {
            *increasing = true;
            return (float)global_pwm_cfg.freq_min;
        }
        return next;
    }
}

// --- Mode 1: Linear Sweep (Standard) ---
static void linear_init(void) {}
static float linear_update(float current_freq, bool *increasing) {
    return linear_logic(current_freq, increasing);
}

// --- Mode 2: Logarithmic Sweep ---
static void log_init(void) {}
static float log_update(float current_freq, bool *increasing) {
    extern pwm_config_t global_pwm_cfg;
    float base_k = 0.05f * global_pwm_cfg.step_multiplier;
    float k = 1.0f + base_k;

    if (*increasing) {
        float next = current_freq * k;
        if (next >= (float)global_pwm_cfg.freq_max) {
            *increasing = false;
            return (float)global_pwm_cfg.freq_max;
        }
        return (next <= current_freq) ? (current_freq + 1.0f) : next;
    } else {
        float next = current_freq / k;
        if (next <= (float)global_pwm_cfg.freq_min) {
            *increasing = true;
            return (float)global_pwm_cfg.freq_min;
        }
        return (next >= current_freq) ? (current_freq - 1.0f) : next;
    }
}

// --- Mode 3: Fast Linear (Interval-based) ---
static void mode3_init(void) {}
static float mode3_update(float current_freq, bool *increasing) {
    return linear_logic(current_freq, increasing);
}

// --- Mode 4: Tick-based Linear ---
static void mode4_init(void) {}
static float mode4_update(float current_freq, bool *increasing) {
    return linear_logic(current_freq, increasing);
}

const pwm_mode_handler_t pwm_handlers[PWM_MODE_COUNT] = {
    { linear_init, linear_update, "Linear" },
    { log_init,    log_update,    "Logarithmic" },
    { mode3_init,  mode3_update,  "Fast Linear (ms)" },
    { mode4_init,  mode4_update,  "Tick-based Linear" }
};
