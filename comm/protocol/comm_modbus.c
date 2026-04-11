#include "comm_modbus.h"

#include <string.h>

#include "bsp_uart.h"
#include "comm_types.h"

#define COMM_MODBUS_SLAVE_ADDR              0x01U
#define COMM_MODBUS_FUNC_READ_HOLDING       0x03U
#define COMM_MODBUS_FUNC_READ_INPUT         0x04U
#define COMM_MODBUS_FUNC_WRITE_SINGLE       0x06U
#define COMM_MODBUS_FUNC_WRITE_MULTI        0x10U
#define COMM_MODBUS_FUNC_EXCEPTION_MASK     0x80U
#define COMM_MODBUS_MAX_FRAME_LEN           256U

typedef enum
{
    COMM_MODBUS_EXCP_ILLEGAL_FUNCTION = 0x01U,
    COMM_MODBUS_EXCP_ILLEGAL_ADDRESS = 0x02U,
    COMM_MODBUS_EXCP_ILLEGAL_VALUE = 0x03U,
} comm_modbus_exception_t;

static uint8_t g_modbus_rx_frame[COMM_MODBUS_MAX_FRAME_LEN];
static uint8_t g_modbus_tx_frame[COMM_MODBUS_MAX_FRAME_LEN];
static axis_t *g_modbus_axis0;

static void modbus_float_to_words(float value, uint16_t *hi, uint16_t *lo)
{
    uint32_t raw = 0U;

    memcpy(&raw, &value, sizeof(raw));
    *hi = (uint16_t)(raw >> 16);
    *lo = (uint16_t)(raw & 0xFFFFU);
}

static float modbus_words_to_float(const uint16_t *words)
{
    uint32_t raw = ((uint32_t)words[0] << 16) | (uint32_t)words[1];
    float value = 0.0f;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static uint16_t comm_modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit_index;

    for (index = 0U; index < len; ++index)
    {
        crc ^= data[index];

        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint16_t comm_modbus_append_crc(uint8_t *frame, uint16_t len)
{
    const uint16_t crc = comm_modbus_crc16(frame, len);

    frame[len++] = (uint8_t)(crc & 0xFFU);
    frame[len++] = (uint8_t)((crc >> 8U) & 0xFFU);
    return len;
}

static uint16_t comm_modbus_build_exception(uint8_t function_code, comm_modbus_exception_t exception_code)
{
    uint16_t tx_len = 0U;

    g_modbus_tx_frame[tx_len++] = COMM_MODBUS_SLAVE_ADDR;
    g_modbus_tx_frame[tx_len++] = (uint8_t)(function_code | COMM_MODBUS_FUNC_EXCEPTION_MASK);
    g_modbus_tx_frame[tx_len++] = (uint8_t)exception_code;
    return comm_modbus_append_crc(g_modbus_tx_frame, tx_len);
}

static void comm_modbus_reply_exception(uint8_t function_code, comm_modbus_exception_t exception_code)
{
    const uint16_t tx_len = comm_modbus_build_exception(function_code, exception_code);
    (void)bsp_uart_comm_send_frame(g_modbus_tx_frame, tx_len);
}

#define READ_FLOAT_REG(reg_base, field) \
    case (reg_base): \
    case (reg_base) + 1U: \
        modbus_float_to_words((field), &hi, &lo); \
        *out_word = (reg_addr == (reg_base)) ? hi : lo; \
        return COMM_OK

static comm_status_t modbus_read_holding_word(uint16_t reg_addr, uint16_t *out_word)
{
    uint16_t hi = 0U;
    uint16_t lo = 0U;
    axis_t *ax = g_modbus_axis0;

    if ((out_word == 0) || (ax == 0))
    {
        return COMM_ERROR;
    }

    switch (reg_addr)
    {
        case COMM_MODBUS_REG_STATUS_MOVE_DONE:
            *out_word = (uint16_t)ax->move_done;
            return COMM_OK;

        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_AXIS0_TARGET_POS,   ax->target_position);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_AXIS0_TARGET_SPEED, ax->foc.target_velocity);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_SPEED_KP,           ax->foc.pid_velocity.kp);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_SPEED_KI,           ax->foc.pid_velocity.ki);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_POS_KP,             ax->pid_position.kp);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_POS_KI,             ax->pid_position.ki);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_IQ_KP,              ax->foc.pid_iq.kp);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_IQ_KI,              ax->foc.pid_iq.ki);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_ID_KP,              ax->foc.pid_id.kp);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_ID_KI,              ax->foc.pid_id.ki);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_IQ_REF,             ax->foc.target_iq);

        case COMM_MODBUS_REG_HOLD_CTRL_MODE:
            *out_word = (uint16_t)ax->mode;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_MOVE_FLAG:
            *out_word = (uint16_t)ax->move_flag;
            return COMM_OK;

        default:
            return COMM_UNSUPPORTED;
    }
}

static comm_status_t modbus_read_input_word(uint16_t reg_addr, uint16_t *out_word)
{
    uint16_t hi = 0U;
    uint16_t lo = 0U;
    axis_t *ax = g_modbus_axis0;

    if ((out_word == 0) || (ax == 0))
    {
        return COMM_ERROR;
    }

    switch (reg_addr)
    {
        READ_FLOAT_REG(COMM_MODBUS_REG_INPUT_AXIS0_POS,     ax->position_mech_rad);
        READ_FLOAT_REG(COMM_MODBUS_REG_INPUT_AXIS0_SPEED,   ax->speed_mech_rad_s);
        READ_FLOAT_REG(COMM_MODBUS_REG_INPUT_AXIS0_CURRENT, ax->foc.i_dq.q);

        default:
            return COMM_UNSUPPORTED;
    }
}

static comm_status_t modbus_read_holding_words(uint16_t reg_addr, uint16_t quantity, uint16_t *out_words)
{
    uint16_t index;

    if ((out_words == 0) || (quantity == 0U))
    {
        return COMM_ERROR;
    }

    for (index = 0U; index < quantity; ++index)
    {
        comm_status_t status = modbus_read_holding_word((uint16_t)(reg_addr + index), &out_words[index]);
        if (status != COMM_OK)
        {
            return status;
        }
    }

    return COMM_OK;
}

static comm_status_t modbus_read_input_words(uint16_t reg_addr, uint16_t quantity, uint16_t *out_words)
{
    uint16_t index;

    if ((out_words == 0) || (quantity == 0U))
    {
        return COMM_ERROR;
    }

    for (index = 0U; index < quantity; ++index)
    {
        comm_status_t status = modbus_read_input_word((uint16_t)(reg_addr + index), &out_words[index]);
        if (status != COMM_OK)
        {
            return status;
        }
    }

    return COMM_OK;
}

static comm_status_t modbus_write_single(uint16_t reg_addr, uint16_t value)
{
    axis_t *ax = g_modbus_axis0;

    if (ax == 0)
    {
        return COMM_ERROR;
    }

    switch (reg_addr)
    {
        case COMM_MODBUS_REG_CMD_LED:
            return COMM_OK;

        case COMM_MODBUS_REG_CMD_AXIS0_ENABLE:
            if (value != 0U)
            {
                axis_set_position_ref(ax, ax->position_mech_rad);
                axis_stop_move(ax);
                axis_enable(ax);
            }
            else
            {
                axis_stop_move(ax);
                axis_disable(ax);
            }
            return COMM_OK;

        case COMM_MODBUS_REG_CMD_AXIS0_MOVE:
            if (value != 0U)
            {
                axis_request_move(ax);
            }
            else
            {
                axis_stop_move(ax);
            }
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_CTRL_MODE:
            axis_set_mode(ax, (axis_mode_t)value);
            return COMM_OK;

        default:
            return COMM_UNSUPPORTED;
    }
}

static comm_status_t modbus_write_multi_words(uint16_t reg_addr, const uint16_t *words, uint16_t quantity)
{
    axis_t *ax = g_modbus_axis0;
    float value;

    if ((words == 0) || (quantity == 0U) || (ax == 0))
    {
        return COMM_ERROR;
    }

    if (quantity == 1U)
    {
        return modbus_write_single(reg_addr, words[0]);
    }

    if (quantity != 2U)
    {
        return COMM_UNSUPPORTED;
    }

    value = modbus_words_to_float(words);

    switch (reg_addr)
    {
        case COMM_MODBUS_REG_HOLD_AXIS0_TARGET_POS:
            axis_set_position_ref(ax, value);
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_AXIS0_TARGET_SPEED:
            axis_set_velocity_ref(ax, value);
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_IQ_REF:
            axis_set_current_ref(ax, value);
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_SPEED_KP:
            ax->foc.pid_velocity.kp = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_SPEED_KI:
            ax->foc.pid_velocity.ki = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_POS_KP:
            ax->pid_position.kp = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_POS_KI:
            ax->pid_position.ki = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_IQ_KP:
            ax->foc.pid_iq.kp = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_IQ_KI:
            ax->foc.pid_iq.ki = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_ID_KP:
            ax->foc.pid_id.kp = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_ID_KI:
            ax->foc.pid_id.ki = value;
            return COMM_OK;

        default:
            return COMM_UNSUPPORTED;
    }
}

static uint16_t comm_modbus_build_read_response(uint8_t function_code,
                                                uint16_t start_reg,
                                                uint16_t quantity,
                                                comm_status_t (*read_words)(uint16_t reg_addr, uint16_t quantity, uint16_t *out_words))
{
    uint16_t tx_len = 0U;
    uint16_t words[125];
    uint16_t index;

    if ((quantity == 0U) || (quantity > 125U))
    {
        comm_modbus_reply_exception(function_code, COMM_MODBUS_EXCP_ILLEGAL_VALUE);
        return 0U;
    }

    if (read_words(start_reg, quantity, words) != COMM_OK)
    {
        comm_modbus_reply_exception(function_code, COMM_MODBUS_EXCP_ILLEGAL_ADDRESS);
        return 0U;
    }

    g_modbus_tx_frame[tx_len++] = COMM_MODBUS_SLAVE_ADDR;
    g_modbus_tx_frame[tx_len++] = function_code;
    g_modbus_tx_frame[tx_len++] = (uint8_t)(quantity * 2U);

    for (index = 0U; index < quantity; ++index)
    {
        g_modbus_tx_frame[tx_len++] = (uint8_t)((words[index] >> 8U) & 0xFFU);
        g_modbus_tx_frame[tx_len++] = (uint8_t)(words[index] & 0xFFU);
    }

    return comm_modbus_append_crc(g_modbus_tx_frame, tx_len);
}

static uint16_t comm_modbus_handle_read_holding(const uint8_t *frame, uint16_t frame_len)
{
    const uint16_t start_reg = (uint16_t)(((uint16_t)frame[2] << 8U) | frame[3]);
    const uint16_t quantity = (uint16_t)(((uint16_t)frame[4] << 8U) | frame[5]);

    (void)frame_len;
    return comm_modbus_build_read_response(COMM_MODBUS_FUNC_READ_HOLDING,
                                           start_reg,
                                           quantity,
                                           modbus_read_holding_words);
}

static uint16_t comm_modbus_handle_read_input(const uint8_t *frame, uint16_t frame_len)
{
    const uint16_t start_reg = (uint16_t)(((uint16_t)frame[2] << 8U) | frame[3]);
    const uint16_t quantity = (uint16_t)(((uint16_t)frame[4] << 8U) | frame[5]);

    (void)frame_len;
    return comm_modbus_build_read_response(COMM_MODBUS_FUNC_READ_INPUT,
                                           start_reg,
                                           quantity,
                                           modbus_read_input_words);
}

static uint16_t comm_modbus_handle_write_single(const uint8_t *frame, uint16_t frame_len)
{
    const uint16_t reg_addr = (uint16_t)(((uint16_t)frame[2] << 8U) | frame[3]);
    const uint16_t value = (uint16_t)(((uint16_t)frame[4] << 8U) | frame[5]);
    uint16_t tx_len = 0U;

    (void)frame_len;

    if (modbus_write_single(reg_addr, value) != COMM_OK)
    {
        comm_modbus_reply_exception(COMM_MODBUS_FUNC_WRITE_SINGLE, COMM_MODBUS_EXCP_ILLEGAL_ADDRESS);
        return 0U;
    }

    g_modbus_tx_frame[tx_len++] = COMM_MODBUS_SLAVE_ADDR;
    g_modbus_tx_frame[tx_len++] = COMM_MODBUS_FUNC_WRITE_SINGLE;
    g_modbus_tx_frame[tx_len++] = frame[2];
    g_modbus_tx_frame[tx_len++] = frame[3];
    g_modbus_tx_frame[tx_len++] = frame[4];
    g_modbus_tx_frame[tx_len++] = frame[5];
    return comm_modbus_append_crc(g_modbus_tx_frame, tx_len);
}

static uint16_t comm_modbus_handle_write_multi(const uint8_t *frame, uint16_t frame_len)
{
    const uint16_t reg_addr = (uint16_t)(((uint16_t)frame[2] << 8U) | frame[3]);
    const uint16_t quantity = (uint16_t)(((uint16_t)frame[4] << 8U) | frame[5]);
    const uint8_t byte_count = frame[6];
    uint16_t words[125];
    uint16_t tx_len = 0U;
    uint16_t index;

    if ((quantity == 0U) || (quantity > 125U) || (byte_count != (uint8_t)(quantity * 2U)))
    {
        comm_modbus_reply_exception(COMM_MODBUS_FUNC_WRITE_MULTI, COMM_MODBUS_EXCP_ILLEGAL_VALUE);
        return 0U;
    }

    if (frame_len != (uint16_t)(9U + byte_count))
    {
        comm_modbus_reply_exception(COMM_MODBUS_FUNC_WRITE_MULTI, COMM_MODBUS_EXCP_ILLEGAL_VALUE);
        return 0U;
    }

    for (index = 0U; index < quantity; ++index)
    {
        const uint16_t data_index = (uint16_t)(7U + index * 2U);
        words[index] = (uint16_t)(((uint16_t)frame[data_index] << 8U) | frame[data_index + 1U]);
    }

    if (modbus_write_multi_words(reg_addr, words, quantity) != COMM_OK)
    {
        comm_modbus_reply_exception(COMM_MODBUS_FUNC_WRITE_MULTI, COMM_MODBUS_EXCP_ILLEGAL_ADDRESS);
        return 0U;
    }

    g_modbus_tx_frame[tx_len++] = COMM_MODBUS_SLAVE_ADDR;
    g_modbus_tx_frame[tx_len++] = COMM_MODBUS_FUNC_WRITE_MULTI;
    g_modbus_tx_frame[tx_len++] = frame[2];
    g_modbus_tx_frame[tx_len++] = frame[3];
    g_modbus_tx_frame[tx_len++] = frame[4];
    g_modbus_tx_frame[tx_len++] = frame[5];
    return comm_modbus_append_crc(g_modbus_tx_frame, tx_len);
}

void comm_modbus_bind_axis0(axis_t *axis)
{
    g_modbus_axis0 = axis;
}

void comm_modbus_init(void)
{
    memset(g_modbus_rx_frame, 0, sizeof(g_modbus_rx_frame));
    memset(g_modbus_tx_frame, 0, sizeof(g_modbus_tx_frame));
}

void comm_modbus_process(void)
{
    uint16_t frame_len;
    uint16_t tx_len = 0U;
    const uint8_t *frame = g_modbus_rx_frame;
    uint16_t received_crc;
    uint16_t calculated_crc;

    frame_len = bsp_uart_comm_fetch_frame(g_modbus_rx_frame, COMM_MODBUS_MAX_FRAME_LEN);
    if (frame_len < 4U)
    {
        return;
    }

    received_crc = (uint16_t)((uint16_t)frame[frame_len - 2U] | ((uint16_t)frame[frame_len - 1U] << 8U));
    calculated_crc = comm_modbus_crc16(frame, (uint16_t)(frame_len - 2U));

    if (received_crc != calculated_crc)
    {
        return;
    }

    if (frame[0] != COMM_MODBUS_SLAVE_ADDR)
    {
        return;
    }

    switch (frame[1])
    {
        case COMM_MODBUS_FUNC_READ_HOLDING:
            if (frame_len != 8U)
            {
                comm_modbus_reply_exception(frame[1], COMM_MODBUS_EXCP_ILLEGAL_VALUE);
                return;
            }
            tx_len = comm_modbus_handle_read_holding(frame, frame_len);
            break;

        case COMM_MODBUS_FUNC_READ_INPUT:
            if (frame_len != 8U)
            {
                comm_modbus_reply_exception(frame[1], COMM_MODBUS_EXCP_ILLEGAL_VALUE);
                return;
            }
            tx_len = comm_modbus_handle_read_input(frame, frame_len);
            break;

        case COMM_MODBUS_FUNC_WRITE_SINGLE:
            if (frame_len != 8U)
            {
                comm_modbus_reply_exception(frame[1], COMM_MODBUS_EXCP_ILLEGAL_VALUE);
                return;
            }
            tx_len = comm_modbus_handle_write_single(frame, frame_len);
            break;

        case COMM_MODBUS_FUNC_WRITE_MULTI:
            tx_len = comm_modbus_handle_write_multi(frame, frame_len);
            break;

        default:
            comm_modbus_reply_exception(frame[1], COMM_MODBUS_EXCP_ILLEGAL_FUNCTION);
            return;
    }

    if (tx_len > 0U)
    {
        (void)bsp_uart_comm_send_frame(g_modbus_tx_frame, tx_len);
    }
}
