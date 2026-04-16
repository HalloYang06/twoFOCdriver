#ifndef H7FOC_COMM_PROTOCOL_COMM_OD_CORE_H
#define H7FOC_COMM_PROTOCOL_COMM_OD_CORE_H

#include <stdint.h>

#include "axis.h"

typedef struct
{
    uint16_t control_word;
    int32_t target_position;
    int32_t target_velocity;
    int16_t mode_of_operation;
    uint8_t valid;
} comm_od_ecat_rxpdo_t;

typedef struct
{
    uint16_t status_word;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t mode_display;
    int16_t torque_actual;
    uint8_t valid;
} comm_od_ecat_txpdo_t;

void comm_od_core_bind_axis0(axis_t *axis);
axis_t *comm_od_core_get_axis0(void);

void comm_od_core_write_ecat_rxpdo(uint16_t control_word,
                                   int32_t target_position,
                                   int32_t target_velocity,
                                   int16_t mode_of_operation);

uint8_t comm_od_core_read_ecat_rxpdo(comm_od_ecat_rxpdo_t *out);
uint32_t comm_od_core_get_ecat_rxpdo_update_count(void);
void comm_od_core_clear_ecat_rxpdo(void);

void comm_od_core_write_ecat_txpdo(uint16_t status_word,
                                   int32_t actual_position,
                                   int32_t actual_velocity,
                                   int16_t mode_display,
                                   int16_t torque_actual);

uint8_t comm_od_core_read_ecat_txpdo(comm_od_ecat_txpdo_t *out);
void comm_od_core_clear_ecat_txpdo(void);

#endif
