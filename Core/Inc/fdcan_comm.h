/* fdcan_comm.h - FDCAN transmit wrapper (H723) */
#ifndef FDCAN_COMM_H
#define FDCAN_COMM_H

#include <stdint.h>
#include <stdbool.h>

/* Тестовый режим отключён для production-сборки. Включайте явно через -DHARD_HW_TEST=1 */
#ifndef HARD_HW_TEST
#define HARD_HW_TEST 0
#endif

#ifndef HARD_HW_TEST_FORCE_TX
#define HARD_HW_TEST_FORCE_TX 0
#endif

void FDCAN_Comm_Init(void);
int  FDCAN_SendFrequency(uint32_t freq_hz);
void FDCAN_Comm_Task(void);

/* Verification protocol */
typedef enum {
    VERIFY_IDLE = 0,
    VERIFY_START,
    VERIFY_BUSY,
    VERIFY_DONE,
    VERIFY_FAIL
} verify_status_t;

int FDCAN_SendVerifyStatus(verify_status_t status);
int FDCAN_SendModeStatus(uint8_t mode_id);
uint32_t FDCAN_GetVerifyRequest(void); /* returns 0 if no request, or freq */
uint8_t FDCAN_GetVerifyResult(void);   /* returns 1 if confirmed */

/**
 * @brief CAN ID таблица
 *
 * H7 → H5/другой (команды):
 *   0x124 VERIFY_REQ    | 0x128 PAUSE_CMD   | 0x129 RESUME_CMD
 *   0x126 VERIFY_RES    | 0x12A REMOTE_REBOOT
 *
 * H5/другой → H7 (статусы):
 *   0x123 FREQ          | 0x125 VERIFY_STATUS | 0x127 MODE_STATUS
 *   0x12B REBOOT_ACK    | 0x12C PAUSE_ACK
 */
#define FDCAN_PAUSE_CMD_ID     0x128U  /**< H7 → slave: остановить PWM (magic 0x50) */
#define FDCAN_RESUME_CMD_ID    0x129U  /**< H7 → slave: запустить PWM  (magic 0x52) */
#define FDCAN_REMOTE_REBOOT_ID 0x12AU  /**< H7 → slave: перезапуск     (magic 0xAA) */
#define FDCAN_REBOOT_ACK_ID    0x12BU  /**< slave → H7: ACK перезапуска */
#define FDCAN_PAUSE_ACK_ID     0x12CU  /**< slave → H7: ACK паузы      */

int FDCAN_SendRebootAck(void);
int FDCAN_SendPauseAck(void);
void FDCAN_ProcessPendingCommands(void);

typedef struct {
    uint32_t tx_error_count;
    uint32_t rx_error_count;
} fdcan_diag_t;

fdcan_diag_t FDCAN_GetDiagnostics(void);

#endif /* FDCAN_COMM_H */
