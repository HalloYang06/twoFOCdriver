#include "adc.h"

#include "app_axis.h"
#include "app_comm.h"
#include "bsp_lan9252.h"

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc2)
    {
        uint16_t sample_ch3 = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        uint16_t sample_ch15 = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        uint16_t sample_ch8 = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
        App_AxisCurrentLoopIrqHandler(sample_ch15, sample_ch3, sample_ch8);
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    bsp_lan9252_handle_exti(gpio_pin);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    App_CommUartTxCplt(huart);
    App_AxisUartTxCplt(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    App_CommUartRxCplt(huart);
    App_AxisUartRxCplt(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    App_CommUartRxEvent(huart, size);
    App_AxisUartRxEvent(huart, size);
}
