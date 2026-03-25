#ifndef TAMAGAWA_H
#define TAMAGAWA_H

#include "main.h"

/* ==================== 多摩川协议数据ID ==================== */
enum TAMAGAWA_DATA_ID
{
    TAMA_DATA_ID_0,  /* 读取单圈数据 */
    TAMA_DATA_ID_1,  /* 读取多圈数据 */
    TAMA_DATA_ID_2,  /* 读取编码器ID */
    TAMA_DATA_ID_3,  /* 读取单圈+多圈+ID+报警 */
    TAMA_DATA_ID_6,  /* 写EEPROM */
    TAMA_DATA_ID_7,  /* 重置错误 */
    TAMA_DATA_ID_8,  /* 重置圈数 */
    TAMA_DATA_ID_C,  /* 重置圈数与错误 */
    TAMA_DATA_ID_D,  /* 读EEPROM */
    TAMA_DATA_ID_NUM
};

/* 多摩川命令字节 */
#define TAMA_CMD_ID0    0x02    /* 读取单圈 */
#define TAMA_CMD_ID1    0x8A    /* 读取圈数 */
#define TAMA_CMD_ID2    0x92    /* 读取编码器编号 */
#define TAMA_CMD_ID3    0x1A    /* 读取单圈+圈数 */
#define TAMA_CMD_ID6    0x32    /* 写EEPROM */
#define TAMA_CMD_ID7    0xBA    /* 重置错误 */
#define TAMA_CMD_ID8    0xC2    /* 重置圈数 */
#define TAMA_CMD_IDC    0x62    /* 重置圈数与错误 */
#define TAMA_CMD_IDD    0xEA    /* 读EEPROM */

/* 编码器分辨率 (17bit = 131072) */
#define TAMA_ENCODER_RESOLUTION     131072
#define TAMA_ENCODER_RESOLUTION_F   131072.0f
#define TAMA_POSITION_MASK          (TAMA_ENCODER_RESOLUTION - 1U)
#define TAMA_RX_TIMEOUT_TICKS       5U

/* 电机极对数 */
#define MOTOR_POLE_PAIRS            5

/* ==================== RS485方向控制 ==================== */
/* USART2 DE引脚由硬件自动控制(RS485模式)，但保留手动控制宏备用 */
/* PD4 = DMC1_RE_Pin，用于控制RS485收发方向 */
#define TAMA_RS485_TX()     HAL_GPIO_WritePin(DMC1_RE_GPIO_Port, DMC1_RE_Pin, GPIO_PIN_SET)
#define TAMA_RS485_RX()     HAL_GPIO_WritePin(DMC1_RE_GPIO_Port, DMC1_RE_Pin, GPIO_PIN_RESET)

/* ==================== 数据结构 ==================== */

/* 多摩川发送数据 */
typedef struct {
    uint8_t adf;    /* EEPROM地址 */
    uint8_t edf;    /* EEPROM数据 */
} Tamagawa_Tx_TypeDef;

/* 多摩川接收数据 */
typedef struct {
    uint32_t abs;   /* 单圈绝对位置 */
    int16_t  abm;   /* 多圈数据 */
    uint8_t  cf;    /* 控制帧 */
    uint8_t  sf;    /* 状态帧 */
    uint8_t  enid;  /* 编码器ID */
    uint8_t  almc;  /* 编码器报警 */
    uint8_t  adf;   /* EEPROM地址 */
    uint8_t  edf;   /* EEPROM数据 */
    uint8_t  crc;   /* CRC校验 */
} Tamagawa_Rx_TypeDef;

/* 多摩川编码器接口 */
typedef struct {
    UART_HandleTypeDef *huart;      /* UART句柄 */

    uint8_t  data_id;               /* 当前请求的数据ID */
    Tamagawa_Tx_TypeDef tx;         /* 发送数据 */
    Tamagawa_Rx_TypeDef rx;         /* 接收数据 */

    uint8_t  rx_size;               /* 期望接收字节数 */
    uint8_t  tx_size;               /* 发送字节数 */
    volatile uint8_t rx_flag;       /* 接收状态: 0=空闲可发送, 1=接收完成待解析, 2=等待接收 */
    uint8_t  rx_count;              /* 已接收字节计数 */

    /* 解析后的位置数据 */
    int32_t  position;              /* 单圈绝对位置 (0 ~ RESOLUTION-1) */
    int16_t  turns;                 /* 多圈圈数 */

    /* FOC需要的角度数据 */
    float    angle_mech_rad;        /* 机械角度 (rad), [0, 2π) */
    float    angle_elec_rad;        /* 电角度 (rad), [0, 2π) */
    float    elec_zero_offset;      /* 电角度零点偏移 (rad) */
    uint8_t  pole_pairs;            /* 极对数 */

    /* 速度计算 */
    int32_t  position_last;         /* 上次位置 (用于速度计算) */
    int32_t  position_total;        /* 累计位置 (含多圈) */
    int32_t  position_total_last;   /* 上次累计位置 */
    float    speed_rps;             /* 转速 (转/秒) */
    float    speed_rpm;             /* 转速 (RPM) */
} Tamagawa_TypeDef;

/* DMA收发缓冲区 (放在D2 SRAM避免DMA访问问题) */
#define TAMA_RX_BUF_SIZE    11
#define TAMA_TX_BUF_SIZE    4

extern uint8_t tama_tx_buf[TAMA_TX_BUF_SIZE];
extern uint8_t tama_rx_buf[TAMA_RX_BUF_SIZE];

/* ==================== 函数声明 ==================== */
void Tamagawa_Init(Tamagawa_TypeDef *tama, UART_HandleTypeDef *huart, uint8_t pole_pairs);
void Tamagawa_RequestData(Tamagawa_TypeDef *tama, uint8_t data_id);
void Tamagawa_RxParse(Tamagawa_TypeDef *tama, uint8_t *buf);
void Tamagawa_Update(Tamagawa_TypeDef *tama);
void Tamagawa_UART_RxCpltCallback(Tamagawa_TypeDef *tama);
void Tamagawa_UART_TxCpltCallback_Handler(Tamagawa_TypeDef *tama);
void Tamagawa_UART_RxEventCallback(Tamagawa_TypeDef *tama, uint16_t Size);
void Tamagawa_AlignElectricZero(Tamagawa_TypeDef *tama);
float Tamagawa_GetAngle_Elec_Rad(Tamagawa_TypeDef *tama);
float Tamagawa_GetAngle_Mech_Rad(Tamagawa_TypeDef *tama);
float Tamagawa_GetSpeed_RPS(Tamagawa_TypeDef *tama);
void Tamagawa_UpdateSpeed(Tamagawa_TypeDef *tama, float dt);
uint8_t Tamagawa_CRC(uint8_t *data, uint8_t len);

#endif /* TAMAGAWA_H */
