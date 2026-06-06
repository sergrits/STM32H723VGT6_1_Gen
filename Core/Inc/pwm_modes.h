#ifndef INC_PWM_MODES_H_
#define INC_PWM_MODES_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PWM_MODE_LINEAR = 0,
    PWM_MODE_LOGARITHMIC,
    PWM_MODE_STEP_LADDER,
    PWM_MODE_JITTER,
    PWM_MODE_COUNT
} pwm_mode_id_t;

typedef struct {
    uint32_t freq_min;
    uint32_t freq_max;
    uint32_t step_hz;
    uint32_t interval_ms;
    float duty;
    float step_multiplier; // Коэффициент шага (от 0.3 до 1.0)
    uint32_t mode3_interval_ms; // Интервал для Mode 3 (1-100 мс)
    uint32_t mode4_ticks; // Интервал для Mode 4 (количество тиков/циклов process)
} pwm_config_t;

typedef struct {
    void (*init)(void);
    float (*update)(float current_freq, bool *increasing);
    const char* name;
} pwm_mode_handler_t;

extern const pwm_mode_handler_t pwm_handlers[PWM_MODE_COUNT];

#endif /* INC_PWM_MODES_H_ */
