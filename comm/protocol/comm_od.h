#ifndef H7FOC_COMM_PROTOCOL_COMM_OD_H
#define H7FOC_COMM_PROTOCOL_COMM_OD_H

#include <stdint.h>

#include "axis.h"
#include "comm_types.h"

void comm_od_bind_axis0(axis_t *axis);

comm_status_t comm_od_read_holding_word(uint16_t reg_addr, uint16_t *out_word);
comm_status_t comm_od_read_input_word(uint16_t reg_addr, uint16_t *out_word);
comm_status_t comm_od_write_single(uint16_t reg_addr, uint16_t value);
comm_status_t comm_od_write_multi_words(uint16_t reg_addr, const uint16_t *words, uint16_t quantity);

#endif
