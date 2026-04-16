#ifndef H7FOC_COMM_PROTOCOL_COMM_ECAT_IF_H
#define H7FOC_COMM_PROTOCOL_COMM_ECAT_IF_H

#include <stdint.h>

typedef enum
{
    COMM_ECAT_TRIGGER_TIM7 = 0,
    COMM_ECAT_TRIGGER_SYNC0 = 1,
} comm_ecat_trigger_source_t;

typedef struct
{
    uint8_t ready;
    uint8_t healthy;
    uint8_t failed;
    uint8_t trigger_source;
    uint8_t bad_health_samples;
    uint8_t reserved0;
    uint16_t reserved1;
    uint32_t init_attempts;
    uint32_t init_failures;
    uint32_t recovery_attempts;
    uint32_t recovery_successes;
    uint32_t recovery_failures;
    uint32_t mainloop_cycles;
    uint32_t axis_apply_cycles;
    uint32_t sync0_irq_count;
    uint32_t sync0_irq_handled_count;
    uint32_t rxpdo_update_count;
    uint16_t last_al_status;
    uint16_t reserved2;
} comm_ecat_diag_t;

void comm_ecat_if_init(void);
void comm_ecat_if_process(void);
uint8_t comm_ecat_if_is_ready(void);
uint8_t comm_ecat_if_is_healthy(void);
void comm_ecat_if_force_reinit(void);
void comm_ecat_if_set_trigger_source(comm_ecat_trigger_source_t source);
comm_ecat_trigger_source_t comm_ecat_if_get_trigger_source(void);
void comm_ecat_if_get_diag(comm_ecat_diag_t *diag);
void comm_ecat_if_on_sync0_irq(void);

void comm_ecat_if_fill_txpdo(uint16_t *status_word,
                             int32_t *actual_position,
                             int32_t *actual_velocity,
                             int16_t *mode_display,
                             int16_t *torque_actual);

#endif
