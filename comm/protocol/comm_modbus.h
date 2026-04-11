#ifndef H7FOC_COMM_PROTOCOL_COMM_MODBUS_H
#define H7FOC_COMM_PROTOCOL_COMM_MODBUS_H

#include <stdint.h>

#include "axis.h"

void comm_modbus_bind_axis0(axis_t *axis);
void comm_modbus_init(void);
void comm_modbus_process(void);

#endif
