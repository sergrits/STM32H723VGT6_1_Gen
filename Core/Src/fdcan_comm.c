/* fdcan_comm.c - FDCAN transmitter for STM32H723 */
#include "fdcan_comm.h"
#include "main.h"
#include <stdbool.h>

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "stm32h7xx_hal_fdcan.h"
#endif

/* NVIC для системного reset */
#include "stm32h7xx.h"

/* Provide a weak FDCAN handle in case MX code hasn't been generated yet. */
__weak FDCAN_HandleTypeDef hfdcan1;

typedef enum {
    FDCAN_LED_STATE_IDLE = 0,
    FDCAN_LED_STATE_TX_PULSE,
    FDCAN_LED_STATE_PASSIVE,
    FDCAN_LED_STATE_BUS_OFF
} fdcan_led_state_t;

typedef struct {
    fdcan_led_state_t state;
    fdcan_led_state_t reported_state;
    uint32_t tx_pulse_until;
    uint32_t blink_last;
    uint8_t tx_event_pending;
    bool led_on;
} fdcan_led_diag_t;

static fdcan_led_diag_t diag = { FDCAN_LED_STATE_IDLE, FDCAN_LED_STATE_IDLE, 0U, 0U, 0U, false };

#define FDCAN_FREQ_STD_ID      0x123U
#define FDCAN_VERIFY_REQ_ID    0x124U
#define FDCAN_VERIFY_STATUS_ID 0x125U
#define FDCAN_VERIFY_RES_ID    0x126U
#define FDCAN_MODE_STATUS_ID   0x127U

static volatile uint32_t s_verify_req_freq = 0;
static volatile uint8_t  s_verify_res_confirmed = 0;

/**
 * @brief Флаги отложенных команд — устанавливаются в ISR, обрабатываются
 *        в FDCAN_ProcessPendingCommands() из main loop.
 */
static volatile uint8_t s_pause_pending  = 0;
static volatile uint8_t s_resume_pending = 0;
static volatile uint8_t s_reboot_pending = 0;

/* Extern references to main.c globals for PAUSE/RESUME functionality */
extern volatile uint8_t pwm_state;
extern TIM_HandleTypeDef *const timers[];

static void diag_write_led(bool on)
{
    (void)on;
    /* LED indication disabled */
}

static fdcan_led_state_t diag_read_state(void)
{
    return FDCAN_LED_STATE_IDLE;
}

static void diag_begin_fault(fdcan_led_state_t fault_state)
{
    diag.state = fault_state;
    diag.blink_last = HAL_GetTick();
    diag_write_led(true);
}

/* Small integer -> string helper (unused but kept for potential debug) */
static void u32_to_str(char *buf, uint32_t v)
{
    char tmp[12];
    int p = 0;
    if (v == 0) tmp[p++] = '0';
    while (v != 0 && p < (int)sizeof(tmp)) { tmp[p++] = (char)('0' + (v % 10)); v /= 10; }
    int idx = 0;
    for (int i = p - 1; i >= 0; --i) buf[idx++] = tmp[i];
    buf[idx] = '\0';
}

void FDCAN_Comm_Init(void)
{
    if (hfdcan1.Instance == NULL) return;

    /* Аппаратный фильтр: принимаем все CAN ID в диапазоне 0x120–0x12F */
    FDCAN_FilterTypeDef filter = { 0 };
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x120U;
    filter.FilterID2 = 0x7F0U;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    /* Start FDCAN peripheral */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    /* Global filter: accept all to FIFO0 */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);

    /* Enable RX notification */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
        Error_Handler();
    }
    /* Bus-Off recovery interrupt */
    (void)HAL_FDCAN_ActivateNotification(&hfdcan1,
            FDCAN_IT_BUS_OFF | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR, 0U);

    diag.state = FDCAN_LED_STATE_IDLE;
    diag.reported_state = FDCAN_LED_STATE_IDLE;
    diag.tx_pulse_until = 0U;
    diag.blink_last = HAL_GetTick();
    diag.tx_event_pending = 0U;
    diag_write_led(false);
}

int FDCAN_SendFrequency(uint32_t freq_hz)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    txHeader.Identifier = FDCAN_FREQ_STD_ID;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_4;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0U;

    uint8_t txData[4];
    txData[0] = (uint8_t)(freq_hz & 0xFFU);
    txData[1] = (uint8_t)((freq_hz >> 8) & 0xFFU);
    txData[2] = (uint8_t)((freq_hz >> 16) & 0xFFU);
    txData[3] = (uint8_t)((freq_hz >> 24) & 0xFFU);

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
    return (st == HAL_OK) ? 0 : -1;
}

int FDCAN_SendVerifyStatus(verify_status_t status)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[1] = {(uint8_t)status};

    txHeader.Identifier = FDCAN_VERIFY_STATUS_ID;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_1;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
    return (st == HAL_OK) ? 0 : -1;
}

int FDCAN_SendModeStatus(uint8_t mode_id)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[1] = {mode_id};

    txHeader.Identifier = FDCAN_MODE_STATUS_ID;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_1;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
    return (st == HAL_OK) ? 0 : -1;
}

int FDCAN_SendRebootAck(void)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[1] = {0xBBU};

    txHeader.Identifier = FDCAN_REBOOT_ACK_ID;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_1;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
    return (st == HAL_OK) ? 0 : -1;
}

int FDCAN_SendPauseAck(void)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[1] = {0x50U};

    txHeader.Identifier = FDCAN_PAUSE_ACK_ID;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_1;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
    return (st == HAL_OK) ? 0 : -1;
}

uint32_t FDCAN_GetVerifyRequest(void)
{
    uint32_t f = s_verify_req_freq;
    s_verify_req_freq = 0;
    return f;
}

uint8_t FDCAN_GetVerifyResult(void)
{
    uint8_t r = s_verify_res_confirmed;
    s_verify_res_confirmed = 0;
    return r;
}

fdcan_diag_t FDCAN_GetDiagnostics(void)
{
    fdcan_diag_t result = {0};
    FDCAN_ErrorCountersTypeDef ec = {0};

    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec) == HAL_OK) {
        result.tx_error_count = ec.TxErrorCnt;
        result.rx_error_count = ec.RxErrorCnt;
    }

    return result;
}

void FDCAN_Comm_Task(void)
{
    /* Entire Task body disabled for performance/cleanness */
}

void FDCAN_ProcessPendingCommands(void)
{
    /* --- PAUSE: остановить все таймеры PWM --- */
    if (s_pause_pending) {
        s_pause_pending = 0;
        for (uint32_t i = 0; i < 8U; i++) {
            HAL_TIM_PWM_Stop(timers[i], TIM_CHANNEL_1);
        }
        pwm_state = 1U;  /* PWM_STATE_PAUSED */
        FDCAN_SendPauseAck();
    }

    /* --- RESUME: запустить все таймеры PWM --- */
    if (s_resume_pending) {
        s_resume_pending = 0;
        for (uint32_t i = 0; i < 8U; i++) {
            HAL_TIM_PWM_Start(timers[i], TIM_CHANNEL_1);
        }
        pwm_state = 0U;  /* PWM_STATE_NORMAL */
    }

    /* --- REBOOT: отправить ACK и перезагрузиться --- */
    if (s_reboot_pending) {
        s_reboot_pending = 0;
        FDCAN_SendRebootAck();

        uint32_t t0 = HAL_GetTick();
        while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 3U) {
            if ((HAL_GetTick() - t0) > 20U) break;
        }
        NVIC_SystemReset();
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == &hfdcan1) {
        while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
            FDCAN_RxHeaderTypeDef rxHeader;
            uint8_t rxData[8];
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
                if (rxHeader.IdType == FDCAN_STANDARD_ID) {
                    switch (rxHeader.Identifier) {

                    case FDCAN_VERIFY_REQ_ID:
                        s_verify_req_freq = ((uint32_t)rxData[0])
                                | ((uint32_t)rxData[1] << 8)
                                | ((uint32_t)rxData[2] << 16)
                                | ((uint32_t)rxData[3] << 24);
                        break;

                    case FDCAN_VERIFY_RES_ID:
                        s_verify_res_confirmed = rxData[0];
                        break;

                    case FDCAN_REMOTE_REBOOT_ID:
                        if (rxData[0] == 0xAAU) { s_reboot_pending = 1U; }
                        break;

                    case FDCAN_PAUSE_CMD_ID:
                        if (rxData[0] == 0x50U) { s_pause_pending = 1U; }
                        break;

                    case FDCAN_RESUME_CMD_ID:
                        if (rxData[0] == 0x52U) { s_resume_pending = 1U; }
                        break;

                    default:
                        break;
                    }
                }
            }
        }
    }
}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
    (void) hfdcan;
    (void) TxEventFifoITs;
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (hfdcan != &hfdcan1) return;

    /* Bus-Off recovery */
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
        (void)HAL_FDCAN_Stop(&hfdcan1);
        (void)HAL_FDCAN_Start(&hfdcan1);
        (void)HAL_FDCAN_ActivateNotification(&hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
            FDCAN_IT_BUS_OFF |
            FDCAN_IT_ARB_PROTOCOL_ERROR |
            FDCAN_IT_DATA_PROTOCOL_ERROR, 0U);
    }
}
