#include "app_comm.h"

#include "app_axis.h"
#include "bsp_uart.h"
#include "comm_modbus.h"
#include "comm_ecat_if.h"
#include "comm_vofa.h"
#include "usart.h"

static uint16_t g_vofa_divider = 0U;

void App_CommInit(void)
{
    bsp_uart_comm_init();
    bsp_uart_comm_bind(&huart3);
    comm_modbus_bind_axis0(&g_axis0);
    comm_modbus_init();
    comm_ecat_if_init();
    comm_vofa_init(&huart4);
}

void App_CommTick(void)
{
    g_vofa_divider++;

    if (g_vofa_divider >= 2U)
    {
        g_vofa_divider = 0U;
        comm_vofa_send_float(
            g_axis0.foc.theta_elec,
            g_axis0.position_mech_rad,
            g_axis0.foc.i_abc.ia,
            g_axis0.foc.i_abc.ib,
            g_axis0.foc.i_abc.ic,
            g_axis0.speed_mech_rad_s,
            g_axis0.foc.i_dq.q,
            g_axis0.foc.target_iq
        );
    }

    comm_modbus_process();
    comm_ecat_if_process();
}

void App_CommUartTxCplt(UART_HandleTypeDef *huart)
{
    comm_vofa_uart_tx_cplt_callback(huart);
    bsp_uart_comm_tx_cplt_callback(huart);
}

void App_CommUartRxCplt(UART_HandleTypeDef *huart)
{
    bsp_uart_comm_rx_cplt_callback(huart);
}

void App_CommUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    bsp_uart_comm_rx_event_callback(huart, size);
}
