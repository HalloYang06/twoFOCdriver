#ifndef H7FOC_COMM_PROTOCOL_COMM_ECAT_IF_H
#define H7FOC_COMM_PROTOCOL_COMM_ECAT_IF_H

#include <stdint.h>

typedef enum
{
    COMM_ECAT_TRIGGER_TIM7 = 0,
    COMM_ECAT_TRIGGER_SYNC0 = 1,
} comm_ecat_trigger_source_t;

void comm_ecat_if_init(void);
void comm_ecat_if_process(void);
uint8_t comm_ecat_if_is_ready(void);
uint8_t comm_ecat_if_is_healthy(void);
void comm_ecat_if_set_trigger_source(comm_ecat_trigger_source_t source);
comm_ecat_trigger_source_t comm_ecat_if_get_trigger_source(void);
void comm_ecat_if_on_sync0_irq(void);

void comm_ecat_if_on_rxpdo(uint16_t control_word,
                           int32_t target_position,
                           int32_t target_velocity,
                           int16_t mode_of_operation);

void comm_ecat_if_fill_txpdo(uint16_t *status_word,
                             int32_t *actual_position,
                             int32_t *actual_velocity,
                             int16_t *mode_display,
                             int16_t *torque_actual);

#endif
