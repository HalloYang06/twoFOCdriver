#include "comm_od_core.h"

typedef struct
{
    axis_t *axis0;
    volatile uint32_t rxpdo_seq;
    volatile comm_od_ecat_rxpdo_t rxpdo_shadow;
} comm_od_core_ctx_t;

static comm_od_core_ctx_t g_comm_od_core;

void comm_od_core_bind_axis0(axis_t *axis)
{
    g_comm_od_core.axis0 = axis;
}

axis_t *comm_od_core_get_axis0(void)
{
    return g_comm_od_core.axis0;
}

void comm_od_core_write_ecat_rxpdo(uint16_t control_word,
                                   int32_t target_position,
                                   int32_t target_velocity,
                                   int16_t mode_of_operation)
{
    g_comm_od_core.rxpdo_seq++;
    g_comm_od_core.rxpdo_shadow.control_word = control_word;
    g_comm_od_core.rxpdo_shadow.target_position = target_position;
    g_comm_od_core.rxpdo_shadow.target_velocity = target_velocity;
    g_comm_od_core.rxpdo_shadow.mode_of_operation = mode_of_operation;
    g_comm_od_core.rxpdo_shadow.valid = 1U;
    g_comm_od_core.rxpdo_seq++;
}

uint8_t comm_od_core_read_ecat_rxpdo(comm_od_ecat_rxpdo_t *out)
{
    uint32_t seq_start;
    uint32_t seq_end;

    if (out == 0)
    {
        return 0U;
    }

    do
    {
        seq_start = g_comm_od_core.rxpdo_seq;
        if ((seq_start & 0x1U) != 0U)
        {
            continue;
        }

        *out = g_comm_od_core.rxpdo_shadow;
        seq_end = g_comm_od_core.rxpdo_seq;
    } while (seq_start != seq_end);

    return out->valid;
}

void comm_od_core_clear_ecat_rxpdo(void)
{
    g_comm_od_core.rxpdo_seq = 0U;
    g_comm_od_core.rxpdo_shadow.control_word = 0U;
    g_comm_od_core.rxpdo_shadow.target_position = 0;
    g_comm_od_core.rxpdo_shadow.target_velocity = 0;
    g_comm_od_core.rxpdo_shadow.mode_of_operation = 0;
    g_comm_od_core.rxpdo_shadow.valid = 0U;
}
