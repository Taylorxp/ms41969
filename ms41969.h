/***************************************************************************************************
* FILE: ms41969.h
*
* DESCRIPTION: --
*
***************************************************************************************************/
#ifndef __MS41969_H__
#define __MS41969_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define MS41969_MOTOR1          0
#define MS41969_MOTOR2          1

/* 电机驱动操作接口 */
struct ms41969_ops
{
    int32_t (*write_reg)(uint8_t addr, uint16_t val);
    void (*m1_start_vd)(void);
    void (*m1_stop_vd)(void);
    void (*m2_start_vd)(void);
    void (*m2_stop_vd)(void);
};

/* 初始化 */
const char* MS41969_GetVersion(void);
int32_t MS41969_Init(struct ms41969_ops *ops);

/* 基本控制 */
int32_t MS41969_MotorEnable(int32_t motor, int32_t new_state);
int32_t MS41969_SetStopPower(int32_t motor, uint8_t per);
int32_t MS41969_SetRunPower(int32_t motor, uint8_t per);
int32_t MS41969_SetStopCutOff(int32_t motor, uint8_t onoff);
int32_t MS41969_SetDirInv(int32_t motor, uint8_t inv);
int32_t MS41969_SetFrequency(int32_t motor, uint8_t freq, uint32_t osc);

/* 状态 */
int32_t MS41969_IsMotorRunning(int32_t motor);
int32_t MS41969_GetMotorCurSpeed(int32_t motor);
int32_t MS41969_GetMotorPosition(int32_t motor);
int32_t MS41969_SetMotorPosition(int32_t motor, int32_t new_pos);

/* 运行控制 */
int32_t MS41969_MotorBrake(int32_t motor, int8_t dec);
int32_t MS41969_MotorGoto(int32_t motor, int32_t tar_pos, uint8_t speed, int8_t acc, int8_t dec);

/* 脉冲驱动，必须调用 */
void MS41969_M1_VdPulseHandler(void);
void MS41969_M2_VdPulseHandler(void);

#ifdef __cplusplus
}
#endif

#endif
/****************************************** END OF FILE *******************************************/
