#include "comm_od.h"

#include <string.h>

#include "comm_ecat_if.h"
#include "comm_od_core.h"
#include "comm_modbus_regs.h"

#define READ_FLOAT_REG(reg_base, field) \
    case (reg_base): \
    case (reg_base) + 1U: \
        comm_od_float_to_words((field), &hi, &lo); \
        *out_word = (reg_addr == (reg_base)) ? hi : lo; \
        return COMM_OK

#define READ_U32_REG(reg_base, field) \
    case (reg_base): \
    case (reg_base) + 1U: \
        comm_od_u32_to_words((field), &hi, &lo); \
        *out_word = (reg_addr == (reg_base)) ? hi : lo; \
        return COMM_OK

static axis_t *g_axis0;
static float g_compat_target_current = 0.0f;
static float g_compat_accel = 0.0f;
static float g_compat_decel = 0.0f;
static float g_compat_runtime_ms = 0.0f;
static float g_compat_curr_kf = 0.0f;
static float g_compat_speed_kf = 0.0f;
static float g_compat_pos_vff = 0.0f;
static float g_compat_pos_aff = 0.0f;

static uint32_t comm_od_irq_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void comm_od_irq_unlock(uint32_t primask)
{
    if ((primask & 0x1U) == 0U)
    {
        __enable_irq();
    }
}

static void comm_od_float_to_words(float value, uint16_t *hi, uint16_t *lo)
{
    uint32_t raw = 0U;

    memcpy(&raw, &value, sizeof(raw));
    *hi = (uint16_t)(raw >> 16);
    *lo = (uint16_t)(raw & 0xFFFFU);
}

static float comm_od_words_to_float(const uint16_t *words)
{
    uint32_t raw = ((uint32_t)words[0] << 16) | (uint32_t)words[1];
    float value = 0.0f;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static void comm_od_u32_to_words(uint32_t value, uint16_t *hi, uint16_t *lo)
{
    *hi = (uint16_t)(value >> 16U);
    *lo = (uint16_t)(value & 0xFFFFU);
}

void comm_od_bind_axis0(axis_t *axis)
{
    g_axis0 = axis;
    comm_od_core_bind_axis0(axis);
}

comm_status_t comm_od_read_holding_word(uint16_t reg_addr, uint16_t *out_word)
{
    uint16_t hi = 0U;
    uint16_t lo = 0U;
    axis_t *ax = g_axis0;

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
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_AXIS0_TARGET_CURRENT, g_compat_target_current);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_AXIS0_ACCEL, g_compat_accel);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_AXIS0_DECEL, g_compat_decel);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_AXIS0_RUNTIME_MS, g_compat_runtime_ms);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_SPEED_KP,           ax->foc.pid_velocity.kp);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_SPEED_KI,           ax->foc.pid_velocity.ki);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_POS_KP,             ax->pid_position.kp);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_POS_KI,             ax->pid_position.ki);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_IQ_KP,              ax->foc.pid_iq.kp);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_IQ_KI,              ax->foc.pid_iq.ki);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_ID_KP,              ax->foc.pid_id.kp);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_ID_KI,              ax->foc.pid_id.ki);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_IQ_REF,             ax->foc.target_iq);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_CURR_KP,            ax->foc.pid_iq.kp);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_CURR_KI,            ax->foc.pid_iq.ki);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_CURR_KF,            g_compat_curr_kf);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_SPEED_LOOP_KP,      ax->foc.pid_velocity.kp);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_SPEED_LOOP_KI,      ax->foc.pid_velocity.ki);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_SPEED_KF,           g_compat_speed_kf);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_POS_LOOP_KP,        ax->pid_position.kp);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_POS_VFF,            g_compat_pos_vff);
        READ_FLOAT_REG(COMM_MODBUS_REG_HOLD_POS_AFF,            g_compat_pos_aff);

        case COMM_MODBUS_REG_HOLD_CTRL_MODE:
            *out_word = (uint16_t)ax->mode;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_MOVE_FLAG:
            *out_word = (uint16_t)ax->move_flag;
            return COMM_OK;

        case COMM_MODBUS_REG_CMD_AXIS0_ENABLE:
            *out_word = (uint16_t)((ax->state == AXIS_STATE_RUNNING) ? 1U : 0U);
            return COMM_OK;

        case COMM_MODBUS_REG_CMD_AXIS0_MOVE:
            *out_word = (uint16_t)ax->move_flag;
            return COMM_OK;

        default:
            return COMM_UNSUPPORTED;
    }
}

comm_status_t comm_od_read_input_word(uint16_t reg_addr, uint16_t *out_word)
{
    uint16_t hi = 0U;
    uint16_t lo = 0U;
    axis_t *ax = g_axis0;
    comm_ecat_diag_t ecat_diag;

    if ((out_word == 0) || (ax == 0))
    {
        return COMM_ERROR;
    }

    comm_ecat_if_get_diag(&ecat_diag);

    switch (reg_addr)
    {
        READ_FLOAT_REG(COMM_MODBUS_REG_INPUT_AXIS0_POS,     ax->position_mech_rad);
        READ_FLOAT_REG(COMM_MODBUS_REG_INPUT_AXIS0_SPEED,   ax->speed_mech_rad_s);
        READ_FLOAT_REG(COMM_MODBUS_REG_INPUT_AXIS0_CURRENT, ax->foc.i_dq.q);

        case COMM_MODBUS_REG_INPUT_ECAT_READY:
            *out_word = ecat_diag.ready;
            return COMM_OK;

        case COMM_MODBUS_REG_INPUT_ECAT_HEALTHY:
            *out_word = ecat_diag.healthy;
            return COMM_OK;

        case COMM_MODBUS_REG_INPUT_ECAT_FAILED:
            *out_word = ecat_diag.failed;
            return COMM_OK;

        case COMM_MODBUS_REG_INPUT_ECAT_TRIGGER_SOURCE:
            *out_word = ecat_diag.trigger_source;
            return COMM_OK;

        case COMM_MODBUS_REG_INPUT_ECAT_BAD_HEALTH_SAMPLES:
            *out_word = ecat_diag.bad_health_samples;
            return COMM_OK;

        case COMM_MODBUS_REG_INPUT_ECAT_LAST_AL_STATUS:
            *out_word = ecat_diag.last_al_status;
            return COMM_OK;

        READ_U32_REG(COMM_MODBUS_REG_INPUT_ECAT_INIT_ATTEMPTS, ecat_diag.init_attempts);
        READ_U32_REG(COMM_MODBUS_REG_INPUT_ECAT_INIT_FAILURES, ecat_diag.init_failures);
        READ_U32_REG(COMM_MODBUS_REG_INPUT_ECAT_RECOVERY_ATTEMPTS, ecat_diag.recovery_attempts);
        READ_U32_REG(COMM_MODBUS_REG_INPUT_ECAT_RECOVERY_SUCCESSES, ecat_diag.recovery_successes);
        READ_U32_REG(COMM_MODBUS_REG_INPUT_ECAT_RECOVERY_FAILURES, ecat_diag.recovery_failures);
        READ_U32_REG(COMM_MODBUS_REG_INPUT_ECAT_MAINLOOP_CYCLES, ecat_diag.mainloop_cycles);
        READ_U32_REG(COMM_MODBUS_REG_INPUT_ECAT_AXIS_APPLY_CYCLES, ecat_diag.axis_apply_cycles);
        READ_U32_REG(COMM_MODBUS_REG_INPUT_ECAT_SYNC0_IRQ_COUNT, ecat_diag.sync0_irq_count);
        READ_U32_REG(COMM_MODBUS_REG_INPUT_ECAT_SYNC0_IRQ_HANDLED, ecat_diag.sync0_irq_handled_count);
        READ_U32_REG(COMM_MODBUS_REG_INPUT_ECAT_RXPDO_UPDATE_COUNT, ecat_diag.rxpdo_update_count);

        default:
            return COMM_UNSUPPORTED;
    }
}

comm_status_t comm_od_write_single(uint16_t reg_addr, uint16_t value)
{
    axis_t *ax = g_axis0;
    uint32_t irq_state;

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
                irq_state = comm_od_irq_lock();
                axis_set_position_ref(ax, ax->position_mech_rad);
                axis_stop_move(ax);
                comm_od_irq_unlock(irq_state);
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
                irq_state = comm_od_irq_lock();
                axis_request_move(ax);
                comm_od_irq_unlock(irq_state);
            }
            else
            {
                irq_state = comm_od_irq_lock();
                axis_stop_move(ax);
                comm_od_irq_unlock(irq_state);
            }
            return COMM_OK;

        case COMM_MODBUS_REG_CMD_ECAT_REINIT:
            if (value != 0U)
            {
                comm_ecat_if_force_reinit();
            }
            return COMM_OK;

        case COMM_MODBUS_REG_CMD_SET_ZERO:
            /* Legacy host compatibility: keep command writable without changing control core. */
            return COMM_OK;

        case COMM_MODBUS_REG_CMD_CLEAR_FAULT:
            /* Legacy host compatibility: clear-fault command is accepted as no-op. */
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_CTRL_MODE:
            irq_state = comm_od_irq_lock();
            axis_set_mode(ax, (axis_mode_t)value);
            comm_od_irq_unlock(irq_state);
            return COMM_OK;

        default:
            return COMM_UNSUPPORTED;
    }
}

comm_status_t comm_od_write_multi_words(uint16_t reg_addr, const uint16_t *words, uint16_t quantity)
{
    axis_t *ax = g_axis0;
    float value;
    uint32_t irq_state;

    if ((words == 0) || (quantity == 0U) || (ax == 0))
    {
        return COMM_ERROR;
    }

    if (quantity == 1U)
    {
        return comm_od_write_single(reg_addr, words[0]);
    }

    if (quantity != 2U)
    {
        return COMM_UNSUPPORTED;
    }

    value = comm_od_words_to_float(words);

    switch (reg_addr)
    {
        case COMM_MODBUS_REG_HOLD_AXIS0_TARGET_POS:
            irq_state = comm_od_irq_lock();
            axis_set_position_ref(ax, value);
            comm_od_irq_unlock(irq_state);
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_AXIS0_TARGET_SPEED:
            axis_set_velocity_ref(ax, value);
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_AXIS0_TARGET_CURRENT:
            g_compat_target_current = value;
            axis_set_current_ref(ax, value);
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_AXIS0_ACCEL:
            g_compat_accel = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_AXIS0_DECEL:
            g_compat_decel = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_AXIS0_RUNTIME_MS:
            g_compat_runtime_ms = value;
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

        case COMM_MODBUS_REG_HOLD_CURR_KP:
            ax->foc.pid_iq.kp = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_CURR_KI:
            ax->foc.pid_iq.ki = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_CURR_KF:
            g_compat_curr_kf = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_SPEED_LOOP_KP:
            ax->foc.pid_velocity.kp = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_SPEED_LOOP_KI:
            ax->foc.pid_velocity.ki = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_SPEED_KF:
            g_compat_speed_kf = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_POS_LOOP_KP:
            ax->pid_position.kp = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_POS_VFF:
            g_compat_pos_vff = value;
            return COMM_OK;

        case COMM_MODBUS_REG_HOLD_POS_AFF:
            g_compat_pos_aff = value;
            return COMM_OK;

        default:
            return COMM_UNSUPPORTED;
    }
}
