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

void comm_od_core_bind_axis0(axis_t *axis);
axis_t *comm_od_core_get_axis0(void);

void comm_od_core_write_ecat_rxpdo(uint16_t control_word,
                                   int32_t target_position,
                                   int32_t target_velocity,
                                   int16_t mode_of_operation);

uint8_t comm_od_core_read_ecat_rxpdo(comm_od_ecat_rxpdo_t *out);

#endif
