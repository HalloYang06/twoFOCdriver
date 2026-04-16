#include "app_axis.h"

#include "adc.h"
#include "bsp_current_sense.h"
#include "bsp_encoder.h"
#include "bsp_tamagawa_uart.h"
#include "tim.h"
#include "usart.h"

/* 业务实例迁到 app 层，Core 只保留 CubeMX 初始化和中断转发。 */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
__attribute__((section(".ARM.__at_0x30020000"))) bsp_current_sense_t g_axis0_current_sense;
#elif defined(__GNUC__)
__attribute__((section(".RAM_D2"))) bsp_current_sense_t g_axis0_current_sense;
#endif

/* 多摩川继续沿用原有时序逻辑，只调整归属位置。 */
Tamagawa_TypeDef tamagawa_M0;

axis_t g_axis0;
static volatile app_axis_slow_loop_source_t g_slow_loop_source = APP_AXIS_SLOW_LOOP_SRC_TIM7;

void App_AxisInit(void)
{
    axis_cfg_t axis0_cfg;

    /* 先复用已经跑通的串口 2 多摩川链路，后续再扩到第二轴。 */
    bsp_tamagawa_uart_init(&tamagawa_M0, &huart2, MOTOR_POLE_PAIRS);

    axis0_cfg.id = AXIS_ID_0;
    axis0_cfg.pole_pairs = MOTOR_POLE_PAIRS;
    axis0_cfg.voltage_supply = 24.0f;
    axis0_cfg.encoder_ops = &g_bsp_encoder_tamagawa_ops;
    axis0_cfg.encoder_ctx = &tamagawa_M0;
    axis0_cfg.encoder_kind = ENCODER_KIND_TAMAGAWA;
    axis0_cfg.current_sense = &g_axis0_current_sense;
    axis0_cfg.hadc = &hadc2;
    axis0_cfg.pwm_htim = &htim1;
    axis0_cfg.pwm_channel_u = TIM_CHANNEL_1;
    axis0_cfg.pwm_channel_v = TIM_CHANNEL_2;
    axis0_cfg.pwm_channel_w = TIM_CHANNEL_3;
    axis0_cfg.pwm_period = 2999U;

    axis_init(&g_axis0, &axis0_cfg);
    axis_calibrate_current_sense(&g_axis0, 2000U);
    bsp_current_sense_start(g_axis0.current_sense);
    axis_set_mode(&g_axis0, AXIS_MODE_CURRENT);
    axis_set_current_ref(&g_axis0, 0.05f);
    g_slow_loop_source = APP_AXIS_SLOW_LOOP_SRC_TIM7;
}

void App_AxisEnable(void)
{
    axis_enable(&g_axis0);
}

void App_AxisDisable(void)
{
    axis_disable(&g_axis0);
}

void App_AxisSetSlowLoopSource(app_axis_slow_loop_source_t source)
{
    g_slow_loop_source = source;
}

app_axis_slow_loop_source_t App_AxisGetSlowLoopSource(void)
{
    return g_slow_loop_source;
}

void App_AxisSlowLoopTick(void)
{
    if (g_slow_loop_source != APP_AXIS_SLOW_LOOP_SRC_TIM7)
    {
        return;
    }

    axis_slow_loop_handler(&g_axis0, 0.001f);
}

void App_AxisSync0Tick(void)
{
    if (g_slow_loop_source != APP_AXIS_SLOW_LOOP_SRC_SYNC0)
    {
        return;
    }

    axis_slow_loop_handler(&g_axis0, 0.001f);
}

void App_AxisCurrentLoopIrqHandler(uint16_t sample_ch15, uint16_t sample_ch3, uint16_t sample_ch8)
{
    axis_current_loop_irq_handler(&g_axis0, sample_ch15, sample_ch3, sample_ch8);
}

void App_AxisUartTxCplt(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        bsp_tamagawa_uart_tx_cplt(&tamagawa_M0);
    }
}

void App_AxisUartRxCplt(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        bsp_tamagawa_uart_rx_cplt(&tamagawa_M0);
    }
}

void App_AxisUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart2)
    {
        bsp_tamagawa_uart_rx_event(&tamagawa_M0, size);
    }
}
