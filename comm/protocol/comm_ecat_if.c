#include "comm_ecat_if.h"

#include "9252_HW.h"
#include "applInterface.h"
#include "app_axis.h"
#include "bsp_lan9252.h"
#include "cia402appl.h"
#include "ecatslv.h"
#include "esc.h"

extern UINT16 CiA402_Init(void);
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
static uint8_t g_ecat_health_ok = 1U;
static uint16_t g_last_control_word = 0U;
static int32_t g_last_target_position = 0;
static uint32_t g_health_divider = 0U;
static uint8_t g_bad_health_samples = 0U;

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
    LocalAxes[0].Objects.objStatusWord = (INT16)comm_ecat_build_status_word();
    LocalAxes[0].Objects.objPositionActualValue = (INT32)(g_axis0.position_mech_rad * COMM_ECAT_POS_SCALE);
    LocalAxes[0].Objects.objVelocityActualValue = (INT32)(g_axis0.speed_mech_rad_s * COMM_ECAT_VEL_SCALE);
    LocalAxes[0].Objects.objTorqueActualValue = (INT16)g_axis0.foc.i_dq.q;
    LocalAxes[0].Objects.objModesOfOperationDisplay = LocalAxes[0].Objects.objModesOfOperation;
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
    const uint16_t control_word = (uint16_t)LocalAxes[0].Objects.objControlWord;
    const axis_mode_t requested_mode = comm_ecat_axis_mode_from_cia402(LocalAxes[0].Objects.objModesOfOperation);
    const int32_t target_pos_raw = LocalAxes[0].Objects.objTargetPosition;
    const int32_t target_vel_raw = LocalAxes[0].Objects.objTargetVelocity;
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

    g_ecat_health_ok = (g_bad_health_samples < 3U) ? 1U : 0U;
}

void comm_ecat_if_init(void)
{
    UINT16 init_err;
    UINT16 cia402_err;
    UINT16 mapping_err;

    if (g_ecat_ready || g_ecat_failed)
    {
        return;
    }

    bsp_lan9252_init_default();
    if (!bsp_lan9252_is_initialized())
    {
        g_ecat_failed = 1U;
        return;
    }

    if (HW_Init() != 0U)
    {
        g_ecat_failed = 1U;
        return;
    }

    init_err = MainInit();
    if (init_err != 0U)
    {
        g_ecat_failed = 1U;
        return;
    }

    cia402_err = CiA402_Init();
    if (cia402_err != 0U)
    {
        g_ecat_failed = 1U;
        return;
    }

    mapping_err = APPL_GenerateMapping(&nPdInputSize, &nPdOutputSize);
    if (mapping_err != 0U)
    {
        g_ecat_failed = 1U;
        return;
    }

    bRunApplication = TRUE;
    g_ecat_ready = 1U;
    g_ecat_health_ok = 1U;
    g_last_control_word = 0U;
    g_last_target_position = 0;
    g_health_divider = 0U;
    g_bad_health_samples = 0U;
}

void comm_ecat_if_process(void)
{
    if (!g_ecat_ready)
    {
        return;
    }

    comm_ecat_update_feedback_to_cia402();
    MainLoop();
    comm_ecat_apply_cia402_commands();
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
