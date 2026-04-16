#ifndef H7FOC_COMM_PROTOCOL_COMM_ECAT_IF_H
#define H7FOC_COMM_PROTOCOL_COMM_ECAT_IF_H

#include <stdint.h>

void comm_ecat_if_init(void);
void comm_ecat_if_process(void);
uint8_t comm_ecat_if_is_ready(void);
uint8_t comm_ecat_if_is_healthy(void);

#endif
