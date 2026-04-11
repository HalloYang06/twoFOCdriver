#ifndef H7FOC_MC_MC_TYPES_H
#define H7FOC_MC_MC_TYPES_H

#include <stdint.h>

typedef struct
{
    float ia;
    float ib;
    float ic;
} phase_currents_t;

typedef struct
{
    float alpha;
    float beta;
} alpha_beta_t;

typedef struct
{
    float d;
    float q;
} dq_frame_t;

typedef enum
{
    AXIS_MODE_DISABLED = 0,
    AXIS_MODE_CURRENT,
    AXIS_MODE_VELOCITY,
    AXIS_MODE_POSITION,
    AXIS_MODE_OPEN_LOOP,
} axis_mode_t;

typedef enum
{
    AXIS_STATE_IDLE = 0,
    AXIS_STATE_CALIBRATING,
    AXIS_STATE_READY,
    AXIS_STATE_RUNNING,
    AXIS_STATE_FAULT,
} axis_state_t;

typedef enum
{
    AXIS_ID_0 = 0,
    AXIS_ID_1 = 1,
} axis_id_t;

#endif
