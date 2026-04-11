#ifndef H7FOC_BSP_BSP_TAMAGAWA_UART_H
#define H7FOC_BSP_BSP_TAMAGAWA_UART_H

#include "main.h"

/* ==================== 多摩川协议数据 ID ==================== */
enum TAMAGAWA_DATA_ID
{
    TAMA_DATA_ID_0,
    TAMA_DATA_ID_1,
    TAMA_DATA_ID_2,
    TAMA_DATA_ID_3,
    TAMA_DATA_ID_6,
    TAMA_DATA_ID_7,
    TAMA_DATA_ID_8,
    TAMA_DATA_ID_C,
    TAMA_DATA_ID_D,
    TAMA_DATA_ID_NUM
};

/* ==================== 多摩川命令字节 ==================== */
#define TAMA_CMD_ID0    0x02U
#define TAMA_CMD_ID1    0x8AU
#define TAMA_CMD_ID2    0x92U
#define TAMA_CMD_ID3    0x1AU
#define TAMA_CMD_ID6    0x32U
#define TAMA_CMD_ID7    0xBAU
#define TAMA_CMD_ID8    0xC2U
#define TAMA_CMD_IDC    0x62U
#define TAMA_CMD_IDD    0xEAU

/* ==================== 编码器与电机参数 ==================== */
#define TAMA_ENCODER_RESOLUTION     131072
#define TAMA_ENCODER_RESOLUTION_F   131072.0f
#define TAMA_POSITION_MASK          (TAMA_ENCODER_RESOLUTION - 1U)
#define TAMA_RX_TIMEOUT_TICKS       5U
#define MOTOR_POLE_PAIRS            5U

/* ==================== RS485 方向控制 ==================== */
#define TAMA_RS485_TX()  HAL_GPIO_WritePin(DMC1_RE_GPIO_Port, DMC1_RE_Pin, GPIO_PIN_SET)
#define TAMA_RS485_RX()  HAL_GPIO_WritePin(DMC1_RE_GPIO_Port, DMC1_RE_Pin, GPIO_PIN_RESET)

typedef struct
{
    uint8_t adf;
    uint8_t edf;
} Tamagawa_Tx_TypeDef;

typedef struct
{
    uint32_t abs;
    int16_t abm;
    uint8_t cf;
    uint8_t sf;
    uint8_t enid;
    uint8_t almc;
    uint8_t adf;
    uint8_t edf;
    uint8_t crc;
} Tamagawa_Rx_TypeDef;

typedef struct
{
    UART_HandleTypeDef *huart;

    uint8_t data_id;
    Tamagawa_Tx_TypeDef tx;
    Tamagawa_Rx_TypeDef rx;

    uint8_t rx_size;
    uint8_t tx_size;
    volatile uint8_t rx_flag;
    uint8_t rx_count;

    int32_t position;
    int16_t turns;

    float angle_mech_rad;
    float angle_elec_rad;
    float elec_zero_offset;
    uint8_t pole_pairs;
    uint32_t angle_update_seq;

    int32_t position_last;
    int32_t position_total;
    int32_t position_total_last;
    int32_t position_raw_last;
    uint8_t position_raw_valid;
    float speed_rps;
    float speed_rpm;
} Tamagawa_TypeDef;

#define TAMA_RX_BUF_SIZE 11U
#define TAMA_TX_BUF_SIZE 4U

extern uint8_t tama_tx_buf[TAMA_TX_BUF_SIZE];
extern uint8_t tama_rx_buf[TAMA_RX_BUF_SIZE];

/* ==================== 底层多摩川能力 ==================== */
void Tamagawa_Init(Tamagawa_TypeDef *tama, UART_HandleTypeDef *huart, uint8_t pole_pairs);
void Tamagawa_RequestData(Tamagawa_TypeDef *tama, uint8_t data_id);
void Tamagawa_RxParse(Tamagawa_TypeDef *tama, uint8_t *buf);
void Tamagawa_Update(Tamagawa_TypeDef *tama);
void Tamagawa_UART_RxCpltCallback(Tamagawa_TypeDef *tama);
void Tamagawa_UART_TxCpltCallback_Handler(Tamagawa_TypeDef *tama);
void Tamagawa_UART_RxEventCallback(Tamagawa_TypeDef *tama, uint16_t size);
void Tamagawa_AlignElectricZero(Tamagawa_TypeDef *tama);
float Tamagawa_GetAngle_Elec_Rad(Tamagawa_TypeDef *tama);
float Tamagawa_GetAngle_Mech_Rad(Tamagawa_TypeDef *tama);
float Tamagawa_GetSpeed_RPS(Tamagawa_TypeDef *tama);
void Tamagawa_UpdateSpeed(Tamagawa_TypeDef *tama, float dt);
uint8_t Tamagawa_CRC(uint8_t *data, uint8_t len);

/* ==================== BSP 对外入口 ==================== */
void bsp_tamagawa_uart_init(Tamagawa_TypeDef *tama, UART_HandleTypeDef *huart, uint8_t pole_pairs);
void bsp_tamagawa_uart_tx_cplt(Tamagawa_TypeDef *tama);
void bsp_tamagawa_uart_rx_cplt(Tamagawa_TypeDef *tama);
void bsp_tamagawa_uart_rx_event(Tamagawa_TypeDef *tama, uint16_t size);

#endif
