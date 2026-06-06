/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : PWM фазировка с аппаратной синхронизацией (TIM1 -> TRGO -> slaves)
 *                   Портировано с STM32H562RGT6 на STM32H723VGT6
 *
 * Ключевые отличия H723 vs H562:
 * - TIMER_CLK_HZ = 120 000 000 (SYSCLK=240MHz, HCLK=120MHz, APB=60MHz, TIM=2x60)
 * - TIM5 тоже 32-битный (Period=0xFFFFFFFF), get_timer_arr_max учитывает TIM2 и TIM5
 * - FDCAN без поля ClockDivider; FDCAN clock = PLL1Q = 120 MHz
 *   500 kbps: Prescaler=15, TimeSeg1=12, TimeSeg2=3, SJW=4 (1+12+3=16 TQ)
 * - BreakAFMode убрано (нет в H7 HAL)
 * - ICACHE через SCB_EnableICache()
 * - FDCAN GPIO: PB8(RX), PB9(TX) согласно ioc-файлу
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "fdcan_comm.h"
#include "pwm_modes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * @brief Частота тактирования таймеров STM32H723
 * @details SYSCLK=240MHz, AHB_DIV=2->HCLK=120MHz, APB_DIV=2->PCLK=60MHz,
 *          TIM_CLK = 2xPCLK = 120 MHz (TIMPRE=0, APB prescaler > 1)
 */
#ifndef TIMER_CLK_HZ
#define TIMER_CLK_HZ 120000000U
#endif

#define PWM_FREQ_MIN            17000U
#define PWM_FREQ_MAX            2000000U
#define PWM_BASE_STEP_HZ        1U
#define PWM_DEFAULT_DUTY        0.5f
#define PWM_STEP_MULTIPLIER     0.5f
#define PWM_MODE1_INTERVAL_MS   100U
#define PWM_MODE3_INTERVAL_MS   10U
#define PWM_MODE4_TICKS         10U

/** 0=LINEAR, 1=LOGARITHMIC, 2=STEP_LADDER, 3=JITTER */
#define PWM_STARTUP_MODE        0U

#define N_CH 8

#define MODE_SWITCH_INTERVAL_MS 30000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */

/** @brief Массив указателей на таймеры (TIM1=master, остальные=slaves) */
TIM_HandleTypeDef *const timers[N_CH] = { &htim1, &htim2, &htim3, &htim4,
        &htim5, &htim8, &htim12, &htim15 };

/** @brief Каналы PWM (все TIM_CHANNEL_1) */
const uint32_t channels[N_CH] = {
    TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1,
    TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1
};

/** @brief Глобальная конфигурация PWM */
pwm_config_t global_pwm_cfg = {
    .freq_min          = PWM_FREQ_MIN,
    .freq_max          = PWM_FREQ_MAX,
    .step_hz           = PWM_BASE_STEP_HZ,
    .interval_ms       = PWM_MODE1_INTERVAL_MS,
    .duty              = PWM_DEFAULT_DUTY,
    .step_multiplier   = PWM_STEP_MULTIPLIER,
    .mode3_interval_ms = PWM_MODE3_INTERVAL_MS,
    .mode4_ticks       = PWM_MODE4_TICKS
};

typedef struct {
    pwm_mode_id_t current_mode;
    uint32_t last_mode_switch_tick;
    float current_freq_f;
    uint32_t current_freq;
    bool increasing;
    uint32_t last_step_tick;
    uint32_t mode4_tick_counter;
    uint32_t cycle_counter;
    uint32_t cycles_per_mode;
} pwm_manager_t;

static pwm_manager_t pwm_mgr;

typedef enum {
    PWM_STATE_NORMAL,
    PWM_STATE_PAUSED
} pwm_state_t;

/** НЕ static — нужен extern для fdcan_comm.c */
volatile pwm_state_t pwm_state = PWM_STATE_NORMAL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM8_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Максимальное значение ARR: TIM2 и TIM5 — 32-бит, остальные — 16-бит
 */
static uint32_t get_timer_arr_max(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2 || htim->Instance == TIM5) {
        return 0xFFFFFFFFU;
    }
    return 0xFFFFU;
}

/**
 * @brief Целочисленное вычисление PSC и ARR для нужной частоты
 */
static void compute_psc_arr_fast(uint32_t freq_hz, uint32_t arr_max, uint32_t *psc, uint32_t *arr) {
    if (freq_hz == 0) { *psc = 0; *arr = arr_max; return; }
    uint64_t timer_clk = (uint64_t)TIMER_CLK_HZ;
    uint64_t period_ticks = timer_clk / freq_hz;
    if (period_ticks <= ((uint64_t)arr_max + 1)) {
        *psc = 0;
        *arr = (uint32_t)(period_ticks - 1);
    } else {
        uint64_t p_calc = (period_ticks + (uint64_t)arr_max) / ((uint64_t)arr_max + 1);
        if (p_calc == 0) p_calc = 1;
        *psc = (uint32_t)(p_calc - 1);
        *arr = (uint32_t)(period_ticks / p_calc) - 1;
    }
}

/* Состояния верификации */
typedef enum {
    VERIFY_STATE_IDLE,
    VERIFY_STATE_STABILIZING,
    VERIFY_STATE_SWEEPING,
    VERIFY_STATE_FINISHING
} v_state_t;

static struct {
    v_state_t state;
    uint32_t base_freq;
    uint32_t current_freq_v;
    float fine_freq_v;
    float current_step;
    int step_idx;
    int mode_idx;
    int sweep_dir;
    uint32_t last_step_tick_v;
} v_sm;

static const float v_steps[] = { 1.0f, 0.9f, 0.8f, 0.7f, 0.6f, 0.5f };
static void verify_process(void);

/* Программирование параметров без UG */
static void set_timer_params_no_ug(TIM_HandleTypeDef *htim, uint32_t channel,
        uint32_t freq_hz, float duty, uint32_t *out_psc, uint32_t *out_arr) {
    uint32_t arr_max = get_timer_arr_max(htim);
    uint32_t psc = 0, arr = 0;
    compute_psc_arr_fast(freq_hz, arr_max, &psc, &arr);
    if (arr == 0) arr = 1;
    HAL_TIM_PWM_Stop(htim, channel);
    if (htim->Instance == TIM2) {
        uint32_t smcr_backup = htim->Instance->SMCR;
        htim->Instance->SMCR = 0;
        __HAL_TIM_SET_PRESCALER(htim, psc);
        __HAL_TIM_SET_AUTORELOAD(htim, arr);
        htim->Instance->SMCR = smcr_backup;
        htim->Instance->EGR = TIM_EGR_UG;
    } else {
        __HAL_TIM_SET_PRESCALER(htim, psc);
        __HAL_TIM_SET_AUTORELOAD(htim, arr);
    }
    uint32_t ccr = (uint32_t)(((uint64_t)(arr + 1) * (uint32_t)(duty * 1000 + 0.5f)) / 1000);
    if (ccr > arr) ccr = arr;
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
    if (out_psc) *out_psc = psc;
    if (out_arr) *out_arr = arr;
}

/* Синхронный старт всех таймеров */
static void preload_and_start_all_hw_sync(float duty) {
    for (uint32_t i = 0; i < N_CH; ++i) {
        uint32_t arr = timers[i]->Instance->ARR;
        uint32_t phase_ticks = (uint32_t)roundf(((float)i * (float)(arr + 1) / (float)N_CH));
        uint32_t preload_cnt = ((arr + 1) + ((arr + 1) - phase_ticks)) % (arr + 1);
        __HAL_TIM_SET_COUNTER(timers[i], preload_cnt);
        if (i > 0) { __HAL_TIM_CLEAR_FLAG(timers[i], TIM_FLAG_UPDATE); }
    }
    for (uint32_t i = 1; i < N_CH; ++i) { HAL_TIM_PWM_Start(timers[i], channels[i]); }
    HAL_TIM_PWM_Start(timers[0], channels[0]);
    timers[0]->Instance->EGR = TIM_EGR_UG;
}

static uint32_t prev_psc_cache[N_CH];
static uint32_t prev_arr_cache[N_CH];

void pwm_manager_init(void) {
    pwm_mgr.current_mode = (pwm_mode_id_t)PWM_STARTUP_MODE;
    pwm_mgr.current_freq_f = (float)global_pwm_cfg.freq_min;
    pwm_mgr.current_freq = global_pwm_cfg.freq_min;
    pwm_mgr.increasing = true;
    pwm_mgr.last_mode_switch_tick = HAL_GetTick();
    pwm_mgr.last_step_tick = HAL_GetTick();
    pwm_mgr.cycle_counter = 0;
    pwm_mgr.cycles_per_mode = 1;
    for (uint32_t i = 0; i < N_CH; ++i) {
        set_timer_params_no_ug(timers[i], channels[i], pwm_mgr.current_freq,
                global_pwm_cfg.duty, &prev_psc_cache[i], &prev_arr_cache[i]);
    }
    preload_and_start_all_hw_sync(global_pwm_cfg.duty);
    FDCAN_SendModeStatus((uint8_t)pwm_mgr.current_mode);
}

void pwm_manager_process(void) {
    uint32_t now = HAL_GetTick();
    verify_process();
    if (v_sm.state != VERIFY_STATE_IDLE) return;
    if (pwm_state == PWM_STATE_PAUSED) return;

    uint32_t current_interval = global_pwm_cfg.interval_ms;
    bool is_tick_based = false;
    if (pwm_mgr.current_mode == PWM_MODE_STEP_LADDER) {
        current_interval = global_pwm_cfg.mode3_interval_ms;
    } else if (pwm_mgr.current_mode == PWM_MODE_JITTER) {
        is_tick_based = true;
    }

    bool trigger_update = false;
    if (is_tick_based) {
        pwm_mgr.mode4_tick_counter++;
        if (pwm_mgr.mode4_tick_counter >= global_pwm_cfg.mode4_ticks) {
            pwm_mgr.mode4_tick_counter = 0;
            trigger_update = true;
        }
    } else {
        if (now - pwm_mgr.last_step_tick >= current_interval) {
            pwm_mgr.last_step_tick = now;
            trigger_update = true;
        }
    }

    if (trigger_update) {
        bool prev_increasing = pwm_mgr.increasing;
        float next_freq_f = pwm_handlers[pwm_mgr.current_mode].update(pwm_mgr.current_freq_f, &pwm_mgr.increasing);

        if (!prev_increasing && pwm_mgr.increasing) {
            pwm_mgr.cycle_counter++;
            if (pwm_mgr.cycle_counter >= pwm_mgr.cycles_per_mode) {
                pwm_mgr.cycle_counter = 0;
                pwm_mgr.current_mode = (pwm_mode_id_t)((pwm_mgr.current_mode + 1) % PWM_MODE_COUNT);
                if (pwm_handlers[pwm_mgr.current_mode].init) pwm_handlers[pwm_mgr.current_mode].init();
                FDCAN_SendModeStatus((uint8_t)pwm_mgr.current_mode);
            }
        }

        pwm_mgr.current_freq_f = next_freq_f;
        pwm_mgr.current_freq = (uint32_t)next_freq_f;

        bool need_restart = false;
        uint32_t tmp_psc[N_CH], tmp_arr[N_CH];
        for (uint32_t i = 0; i < N_CH; i++) {
            compute_psc_arr_fast(pwm_mgr.current_freq, get_timer_arr_max(timers[i]), &tmp_psc[i], &tmp_arr[i]);
            if (tmp_psc[i] != prev_psc_cache[i] || tmp_arr[i] != prev_arr_cache[i]) need_restart = true;
        }

        if (need_restart) {
            for (uint32_t i = 0; i < N_CH; i++) {
                uint32_t psc = tmp_psc[i];
                uint32_t arr = tmp_arr[i];
                if (arr == 0U) arr = 1U;
                HAL_TIM_PWM_Stop(timers[i], channels[i]);
                if (timers[i]->Instance == TIM2) {
                    uint32_t smcr_bk = timers[i]->Instance->SMCR;
                    timers[i]->Instance->SMCR = 0;
                    __HAL_TIM_SET_PRESCALER(timers[i], psc);
                    __HAL_TIM_SET_AUTORELOAD(timers[i], arr);
                    timers[i]->Instance->SMCR = smcr_bk;
                    timers[i]->Instance->EGR  = TIM_EGR_UG;
                } else {
                    __HAL_TIM_SET_PRESCALER(timers[i], psc);
                    __HAL_TIM_SET_AUTORELOAD(timers[i], arr);
                }
                uint32_t ccr = (uint32_t)(((uint64_t)(arr + 1U) *
                                (uint32_t)(global_pwm_cfg.duty * 1000.0f + 0.5f)) / 1000U);
                if (ccr > arr) ccr = arr;
                __HAL_TIM_SET_COMPARE(timers[i], channels[i], ccr);
                prev_psc_cache[i] = psc;
                prev_arr_cache[i] = arr;
            }
            preload_and_start_all_hw_sync(global_pwm_cfg.duty);
        } else {
            for (uint32_t i = 0; i < N_CH; i++) {
                uint32_t arr = prev_arr_cache[i];
                uint32_t ccr = (uint32_t)(((uint64_t)(arr + 1) * (uint32_t)(global_pwm_cfg.duty * 1000 + 0.5f)) / 1000);
                __HAL_TIM_SET_COMPARE(timers[i], channels[i], (ccr > arr ? arr : ccr));
            }
        }
        FDCAN_SendFrequency(pwm_mgr.current_freq);
    }

    /* Периодически повторяем статус режима (каждые 2 сек) */
    {
        static uint32_t s_last_mode_bcast = 0U;
        if ((now - s_last_mode_bcast) >= 2000U) {
            s_last_mode_bcast = now;
            FDCAN_SendModeStatus((uint8_t)pwm_mgr.current_mode);
        }
    }
}

static void verify_process(void) {
    uint32_t now = HAL_GetTick();
    switch (v_sm.state) {
    case VERIFY_STATE_IDLE: {
        uint32_t req = FDCAN_GetVerifyRequest();
        if (req != 0) {
            v_sm.base_freq = req;
            v_sm.mode_idx = 0; v_sm.step_idx = 0;
            v_sm.current_step = v_steps[0];
            v_sm.fine_freq_v = (float)v_sm.base_freq - 100.0f;
            v_sm.sweep_dir = 1;
            v_sm.state = VERIFY_STATE_STABILIZING;
            FDCAN_SendVerifyStatus(VERIFY_START);
            if (pwm_handlers[v_sm.mode_idx].init) pwm_handlers[v_sm.mode_idx].init();
            FDCAN_SendModeStatus((uint8_t)v_sm.mode_idx);
        }
        break;
    }
    case VERIFY_STATE_STABILIZING:
        FDCAN_SendVerifyStatus(VERIFY_BUSY);
        v_sm.state = VERIFY_STATE_SWEEPING;
        v_sm.last_step_tick_v = now;
        break;
    case VERIFY_STATE_SWEEPING:
        if (now - v_sm.last_step_tick_v >= 50) {
            v_sm.last_step_tick_v = now;
            v_sm.fine_freq_v += (v_sm.current_step * (float)v_sm.sweep_dir);
            uint32_t target_hz = (uint32_t)v_sm.fine_freq_v;
            for (uint32_t i = 0; i < N_CH; ++i) {
                uint32_t psc, arr;
                compute_psc_arr_fast(target_hz, get_timer_arr_max(timers[i]), &psc, &arr);
                if (psc != prev_psc_cache[i] || arr != prev_arr_cache[i]) {
                    set_timer_params_no_ug(timers[i], channels[i], target_hz, global_pwm_cfg.duty, &prev_psc_cache[i], &prev_arr_cache[i]);
                    preload_and_start_all_hw_sync(global_pwm_cfg.duty);
                } else {
                    uint32_t arr_v = prev_arr_cache[i];
                    uint32_t ccr = (uint32_t)(((uint64_t)(arr_v + 1) * (uint32_t)(global_pwm_cfg.duty * 1000 + 0.5f)) / 1000);
                    __HAL_TIM_SET_COMPARE(timers[i], channels[i], (ccr > arr_v ? arr_v : ccr));
                }
            }
            FDCAN_SendFrequency(target_hz);
            if (v_sm.sweep_dir == 1 && v_sm.fine_freq_v >= (float)v_sm.base_freq + 100.0f) {
                v_sm.sweep_dir = -1;
            } else if (v_sm.sweep_dir == -1 && v_sm.fine_freq_v <= (float)v_sm.base_freq - 100.0f) {
                v_sm.sweep_dir = 1;
                v_sm.step_idx++;
                if (v_sm.step_idx >= 6) {
                    v_sm.step_idx = 0; v_sm.mode_idx++;
                    if (v_sm.mode_idx >= 4) {
                        v_sm.state = VERIFY_STATE_FINISHING;
                    } else {
                        if (pwm_handlers[v_sm.mode_idx].init) pwm_handlers[v_sm.mode_idx].init();
                        FDCAN_SendModeStatus((uint8_t)v_sm.mode_idx);
                    }
                }
                v_sm.current_step = v_steps[v_sm.step_idx];
            }
        }
        break;
    case VERIFY_STATE_FINISHING:
        FDCAN_SendVerifyStatus(VERIFY_DONE);
        v_sm.state = VERIFY_STATE_IDLE;
        pwm_manager_init();
        break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Включаем Instruction Cache (L1) для STM32H7 */
  SCB_EnableICache();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM12_Init();
  MX_TIM15_Init();
  MX_TIM8_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize FDCAN communication */
  FDCAN_Comm_Init();

  /* Инициализация Independent Watchdog */
  IWDG1->KR  = 0x5555;       /* Unlock IWDG1 */
  IWDG1->PR  = IWDG_PR_PR_2;  /* Prescaler = 64 */
  IWDG1->RLR = 0xFFF;         /* Reload = 4095 (~8 sec) */
  IWDG1->KR  = 0xCCCC;       /* Start IWDG1 */

  /* Инициализация менеджера режимов PWM */
  pwm_manager_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Refresh Independent Watchdog */
    IWDG1->KR = 0xAAAA;

    /* Обработка команд из CAN ISR (PAUSE/RESUME/REBOOT) */
    FDCAN_ProcessPendingCommands();

    /* Мониторинг CAN ошибок (каждые 5 сек) */
    {
        static uint32_t last_can_diag_check = 0;
        uint32_t now_ms = HAL_GetTick();
        if ((now_ms - last_can_diag_check) >= 5000U) {
            last_can_diag_check = now_ms;
            fdcan_diag_t can_diag = FDCAN_GetDiagnostics();
            (void)can_diag;
        }
    }

    pwm_manager_process();
    __NOP();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 12;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 1;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim12, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim15, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* LED диагностики */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Не кормим IWDG -> через ~8 сек аппаратный сброс */
  while (1) {
    __NOP();
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
