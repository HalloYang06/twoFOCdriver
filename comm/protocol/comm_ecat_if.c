#include "comm_ecat_if.h"

#include <string.h>

#include "9252_HW.h"
#include "applInterface.h"
#include "app_axis.h"
#include "bsp_lan9252.h"
#include "cia402appl.h"
#include "comm_od_core.h"
#include "ecatslv.h"
#include "esc.h"

extern UINT16 CiA402_Init(void);
extern void CiA402_DeallocateAxis(void);
extern TCiA402Axis LocalAxes[MAX_AXES];

#ifndef COMM_ECAT_POS_SCALE
#define COMM_ECAT_POS_SCALE 1.0f
#endif

#ifndef COMM_ECAT_VEL_SCALE
#define COMM_ECAT_VEL_SCALE 1.0f
#endif

#define COMM_ECAT_MODE_CSP 8
#define COMM_ECAT_MODE_CSV 9
#define COMM_ECAT_MODE_CST 10

static uint8_t g_ecat_ready = 0U;
static uint8_t g_ecat_failed = 0U;
static uint8_t g_ecat_health_ok = 0U;
static uint16_t g_last_control_word = 0U;
static int32_t g_last_target_position = 0;
static uint32_t g_health_divider = 0U;
static uint8_t g_bad_health_samples = 0U;
static uint32_t g_rxpdo_update_count = 0U;
static volatile uint32_t g_sync0_irq_seq = 0U;
static uint32_t g_sync0_irq_seq_handled = 0U;
static comm_ecat_trigger_source_t g_trigger_source = COMM_ECAT_TRIGGER_TIM7;
static uint32_t g_init_attempts = 0U;
static uint32_t g_init_failures = 0U;
static uint32_t g_recovery_attempts = 0U;
static uint32_t g_recovery_successes = 0U;
static uint32_t g_recovery_failures = 0U;
static uint32_t g_mainloop_cycles = 0U;
static uint32_t g_axis_apply_cycles = 0U;
static uint16_t g_last_al_status = 0U;
static uint32_t g_reinit_divider = 0U;

static uint8_t comm_ecat_stack_bootstrap(void)
{
    UINT16 init_err;
    UINT16 cia402_err;
    UINT16 mapping_err;
    uint8_t cia402_initialized = 0U;

    if (HW_Init() != 0U)
    {
        return 0U;
    }

    init_err = MainInit();
    if (init_err != 0U)
    {
        return 0U;
    }

    cia402_err = CiA402_Init();
    if (cia402_err != 0U)
    {
        return 0U;
    }
    cia402_initialized = 1U;

    mapping_err = APPL_GenerateMapping(&nPdInputSize, &nPdOutputSize);
    if (mapping_err != 0U)
    {
        if (cia402_initialized != 0U)
        {
            CiA402_DeallocateAxis();
        }
        return 0U;
    }

    bRunApplication = TRUE;
    return 1U;
}

static uint16_t comm_ecat_build_status_word(void)
{
    uint16_t status = 0U;

    status |= (1U << 0); /* Ready to switch on */

    if (g_axis0.state == AXIS_STATE_RUNNING)
    {
        status |= (1U << 1); /* Switched on */
        status |= (1U << 2); /* Operation enabled */
    }

    if (g_axis0.state != AXIS_STATE_FAULT)
    {
        status |= (1U << 5); /* Quick stop */
    }

    if (g_axis0.move_done != 0U)
    {
        status |= (1U << 10); /* Target reached */
    }

    return status;
}

static void comm_ecat_update_feedback_to_cia402(void)
{
    uint16_t status_word = 0U;
    int32_t actual_position = 0;
    int32_t actual_velocity = 0;
    int16_t mode_display = 0;
    int16_t torque_actual = 0;

    comm_ecat_if_fill_txpdo(&status_word, &actual_position, &actual_velocity, &mode_display, &torque_actual);

    LocalAxes[0].Objects.objStatusWord = (INT16)status_word;
    LocalAxes[0].Objects.objPositionActualValue = actual_position;
    LocalAxes[0].Objects.objVelocityActualValue = actual_velocity;
    LocalAxes[0].Objects.objTorqueActualValue = torque_actual;
    LocalAxes[0].Objects.objModesOfOperationDisplay = mode_display;
}

static axis_mode_t comm_ecat_axis_mode_from_cia402(INT16 cia402_mode)
{
    switch (cia402_mode)
    {
        case COMM_ECAT_MODE_CSP:
            return AXIS_MODE_POSITION;
        case COMM_ECAT_MODE_CSV:
            return AXIS_MODE_VELOCITY;
        case COMM_ECAT_MODE_CST:
            return AXIS_MODE_CURRENT;
        default:
            return g_axis0.mode;
    }
}

static void comm_ecat_apply_cia402_commands(void)
{
    comm_od_ecat_rxpdo_t rxpdo_cmd = {0};
    uint16_t control_word = (uint16_t)LocalAxes[0].Objects.objControlWord;
    int16_t mode_of_operation = LocalAxes[0].Objects.objModesOfOperation;
    int32_t target_pos_raw = LocalAxes[0].Objects.objTargetPosition;
    int32_t target_vel_raw = LocalAxes[0].Objects.objTargetVelocity;
    axis_mode_t requested_mode;

    if (comm_od_core_read_ecat_rxpdo(&rxpdo_cmd) != 0U)
    {
        control_word = rxpdo_cmd.control_word;
        mode_of_operation = rxpdo_cmd.mode_of_operation;
        target_pos_raw = rxpdo_cmd.target_position;
        target_vel_raw = rxpdo_cmd.target_velocity;
    }

    requested_mode = comm_ecat_axis_mode_from_cia402(mode_of_operation);
    const uint8_t enable_request = (((control_word & 0x000FU) == 0x000FU) ? 1U : 0U);

    if (requested_mode != g_axis0.mode)
    {
        axis_set_mode(&g_axis0, requested_mode);
    }

    if (enable_request != 0U)
    {
        if (g_axis0.state != AXIS_STATE_RUNNING)
        {
            axis_set_position_ref(&g_axis0, g_axis0.position_mech_rad);
            axis_stop_move(&g_axis0);
            axis_enable(&g_axis0);
        }
    }
    else if (g_axis0.state == AXIS_STATE_RUNNING)
    {
        axis_stop_move(&g_axis0);
        axis_disable(&g_axis0);
    }

    if (requested_mode == AXIS_MODE_POSITION)
    {
        const float target_pos = ((float)target_pos_raw) / COMM_ECAT_POS_SCALE;

        axis_set_position_ref(&g_axis0, target_pos);
        if ((target_pos_raw != g_last_target_position) || ((g_last_control_word & 0x0010U) == 0U && (control_word & 0x0010U) != 0U))
        {
            axis_request_move(&g_axis0);
        }
    }
    else if (requested_mode == AXIS_MODE_VELOCITY)
    {
        const float target_vel = ((float)target_vel_raw) / COMM_ECAT_VEL_SCALE;
        axis_set_velocity_ref(&g_axis0, target_vel);
    }

    g_last_target_position = target_pos_raw;
    g_last_control_word = control_word;
}

static void comm_ecat_health_poll(void)
{
    uint16_t al_status = 0U;

    g_health_divider++;
    if (g_health_divider < 1000U)
    {
        return;
    }

    g_health_divider = 0U;
    HW_EscReadWord(al_status, ESC_AL_STATUS_OFFSET);
    g_last_al_status = al_status;

    if ((al_status == 0x0000U) || (al_status == 0xFFFFU))
    {
        if (g_bad_health_samples < 255U)
        {
            g_bad_health_samples++;
        }
    }
    else
    {
        g_bad_health_samples = 0U;
    }

    if (g_bad_health_samples < 3U)
    {
        g_ecat_health_ok = 1U;
        return;
    }

    g_recovery_attempts++;
    bRunApplication = FALSE;
    CiA402_DeallocateAxis();

    if (bsp_lan9252_is_initialized() == 0U)
    {
        bsp_lan9252_init_default();
    }

    if ((bsp_lan9252_is_initialized() != 0U) && (comm_ecat_stack_bootstrap() != 0U))
    {
        g_bad_health_samples = 0U;
        g_ecat_health_ok = 1U;
        g_recovery_successes++;
        return;
    }

    g_recovery_failures++;
    g_ecat_health_ok = 0U;
}

void comm_ecat_if_init(void)
{
    if (g_ecat_ready)
    {
        return;
    }

    g_init_attempts++;

    if (bsp_lan9252_is_initialized() == 0U)
    {
        bsp_lan9252_init_default();
    }

    if (!bsp_lan9252_is_initialized())
    {
        g_ecat_health_ok = 0U;
        g_ecat_failed = 1U;
        g_init_failures++;
        return;
    }

    if (comm_ecat_stack_bootstrap() == 0U)
    {
        g_ecat_health_ok = 0U;
        g_ecat_failed = 1U;
        g_init_failures++;
        return;
    }

    g_ecat_ready = 1U;
    g_ecat_failed = 0U;
    g_ecat_health_ok = 1U;
    g_last_control_word = 0U;
    g_last_target_position = 0;
    g_health_divider = 0U;
    g_bad_health_samples = 0U;
    g_rxpdo_update_count = 0U;
    comm_od_core_clear_ecat_rxpdo();
    g_sync0_irq_seq = 0U;
    g_sync0_irq_seq_handled = 0U;
    g_trigger_source = COMM_ECAT_TRIGGER_TIM7;
    g_reinit_divider = 0U;
}

void comm_ecat_if_process(void)
{
    uint8_t apply_axis_cmd = 0U;

    if (!g_ecat_ready)
    {
        g_reinit_divider++;
        if (g_reinit_divider >= 1000U)
        {
            g_reinit_divider = 0U;
            comm_ecat_if_init();
        }
        return;
    }

    if (g_trigger_source == COMM_ECAT_TRIGGER_TIM7)
    {
        apply_axis_cmd = 1U;
    }
    else if (g_sync0_irq_seq_handled != g_sync0_irq_seq)
    {
        apply_axis_cmd = 1U;
        g_sync0_irq_seq_handled = g_sync0_irq_seq;
    }

    g_mainloop_cycles++;
    comm_ecat_update_feedback_to_cia402();
    MainLoop();
    if (apply_axis_cmd != 0U)
    {
        g_axis_apply_cycles++;
        comm_ecat_apply_cia402_commands();
    }
    comm_ecat_health_poll();
}

uint8_t comm_ecat_if_is_ready(void)
{
    return g_ecat_ready;
}

uint8_t comm_ecat_if_is_healthy(void)
{
    return g_ecat_health_ok;
}

void comm_ecat_if_force_reinit(void)
{
    uint32_t irq_state = bsp_lan9252_irq_lock();

    bRunApplication = FALSE;
    g_ecat_ready = 0U;
    g_ecat_failed = 0U;
    g_ecat_health_ok = 0U;
    g_last_control_word = 0U;
    g_last_target_position = 0;
    g_health_divider = 0U;
    g_bad_health_samples = 0U;
    g_rxpdo_update_count = 0U;
    comm_od_core_clear_ecat_rxpdo();
    g_sync0_irq_seq = 0U;
    g_sync0_irq_seq_handled = 0U;
    g_last_al_status = 0U;

    bsp_lan9252_irq_unlock(irq_state);
}

void comm_ecat_if_set_trigger_source(comm_ecat_trigger_source_t source)
{
    g_trigger_source = source;
}

comm_ecat_trigger_source_t comm_ecat_if_get_trigger_source(void)
{
    return g_trigger_source;
}

void comm_ecat_if_get_diag(comm_ecat_diag_t *diag)
{
    uint32_t irq_state;

    if (diag == 0)
    {
        return;
    }

    irq_state = bsp_lan9252_irq_lock();

    memset(diag, 0, sizeof(*diag));
    diag->ready = g_ecat_ready;
    diag->healthy = g_ecat_health_ok;
    diag->failed = g_ecat_failed;
    diag->trigger_source = (uint8_t)g_trigger_source;
    diag->bad_health_samples = g_bad_health_samples;
    diag->init_attempts = g_init_attempts;
    diag->init_failures = g_init_failures;
    diag->recovery_attempts = g_recovery_attempts;
    diag->recovery_successes = g_recovery_successes;
    diag->recovery_failures = g_recovery_failures;
    diag->mainloop_cycles = g_mainloop_cycles;
    diag->axis_apply_cycles = g_axis_apply_cycles;
    diag->sync0_irq_count = g_sync0_irq_seq;
    diag->sync0_irq_handled_count = g_sync0_irq_seq_handled;
    diag->rxpdo_update_count = g_rxpdo_update_count;
    diag->last_al_status = g_last_al_status;

    bsp_lan9252_irq_unlock(irq_state);
}

void comm_ecat_if_on_sync0_irq(void)
{
    g_sync0_irq_seq++;
}

void comm_ecat_if_on_rxpdo(uint16_t control_word,
                           int32_t target_position,
                           int32_t target_velocity,
                           int16_t mode_of_operation)
{
    comm_od_core_write_ecat_rxpdo(control_word, target_position, target_velocity, mode_of_operation);
    g_rxpdo_update_count++;
}

void comm_ecat_if_fill_txpdo(uint16_t *status_word,
                             int32_t *actual_position,
                             int32_t *actual_velocity,
                             int16_t *mode_display,
                             int16_t *torque_actual)
{
    if (status_word != 0)
    {
        *status_word = comm_ecat_build_status_word();
    }

    if (actual_position != 0)
    {
        *actual_position = (int32_t)(g_axis0.position_mech_rad * COMM_ECAT_POS_SCALE);
    }

    if (actual_velocity != 0)
    {
        *actual_velocity = (int32_t)(g_axis0.speed_mech_rad_s * COMM_ECAT_VEL_SCALE);
    }

    if (mode_display != 0)
    {
        *mode_display = LocalAxes[0].Objects.objModesOfOperation;
    }

    if (torque_actual != 0)
    {
        *torque_actual = (INT16)g_axis0.foc.i_dq.q;
    }
}
