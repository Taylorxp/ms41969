/***************************************************************************************************
* FILE: ms41969.c
*
* DESCRIPTION:  --
*
***************************************************************************************************/
#include "ms41969.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>



/* 版本信息 */
#define VER_MAJOR   1
#define VER_MINOR1  0
#define VER_MINOR2  01
#define DATE_YEAR   2020
#define DATE_MONTH  07
#define DATE_DAY    25

#define STR(s)     #s
#define VERSION(a, b, c)    STR(a) "." STR(b) "." STR(c)
#define DATE(y, m, d)       STR(y) "." STR(m) "." STR(d)

//static const char version[] = VERSION(VER_MAJOR, VER_MINOR1, VER_MINOR2);
static const char version_full[] = VERSION(VER_MAJOR, VER_MINOR1, VER_MINOR2) " " DATE(DATE_YEAR, DATE_MONTH, DATE_DAY);


#define DEF_M1_VD_FREQ          50
#define DEF_M2_VD_FREQ          50

#define DEF_OSC_FREQ			(16 * 1000000u)

#define DEF_M1_STOP_POWER       60
#define DEF_M1_RUN_POWER        100
#define DEF_M2_STOP_POWER       60
#define DEF_M2_RUN_POWER        100

/* 反向运行时刹车速度 */
#define INV_BRAKE_SPEED             10







/* 电机运转方向 */
#define DIR_POS                 0
#define DIR_NEG                 1

/* 电机刹车，一般不使用 */
#define BRAKE_DISABLE           0
#define BRAKE_ENABLE            1

/* 细分模式 */
#define DIV_64                  3
#define DIV_128                 2
#define DIV_256                 0



///////////////////////////////////////// 寄存器参数 ///////////////////////////////////////////////

/* VD FZ极性*/
#define VD_RAISING                  0
#define VD_FALLING                  1
#define VD_POLARITY                 VD_RAISING


/* DELAY1，单位：303.4us */
#define DELAY1                      15

/* PWM频率=(OSCIN / ((mode * 2^3) * 2^res)) 
默认mode=8，res=1，此时pwm频率为210.9k */
/* PWM模式 */
#define PWM_MODE                    8

/* PWM分辨率 */
#define PWM_RES                     2


/* 测试模式 */
#define TEST_MODE_



/* MOTOR1 DELAY，单位：303.4us */
#define M1_DELAY                    2

/* MOTOR1 相位矫正 */
#define M1_PHASE                    0

/* MOTOR1 峰值脉冲 */
#define M1_MAX_PULSE                100

/* MOTOR1 细分数 */
#define M1_DIV                      DIV_256


/* MOTOR2 DELAY，单位：303.4us */
#define M2_DELAY                    2

/* MOTOR2 相位矫正 */
#define M2_PHASE                    0

/* MOTOR2 峰值脉冲 */
#define M2_MAX_PULSE                100

/* MOTOR2 细分数 */
#define M2_DIV                      DIV_256



/* 判断电机是否在运行 */
#define IS_RUNNING(m)               (!((m)->cur_pos == (m)->tar_pos))

/* 计算运转方向 */
#define RUN_DIR(cur, tar)           (((tar)-(cur)) > 0 ? 1:0)

/* 计算距离差值 */
#define POS_DIFF(a, b)              (m_abs(b-a))


/* 电机参数 */
struct motor
{
    /* 操作接口 */
    void (*start_vd)(void);
    void (*stop_vd)(void);
    
    /* VD */
    uint8_t vd_freq;
    uint16_t period;
    
    /* 模式参数 */
    uint8_t dir_reverse;
    uint8_t stop_power;
    uint8_t run_power;
    uint8_t disable_when_stop;

    /* 寄存器参数 */
    uint8_t enable;         //电机使能
    uint8_t brake;          //刹车
    uint8_t max_power;      //峰值脉冲，0~100
    uint8_t phase;          //相位矫正
    
    uint8_t dir;            //运转方向
    uint8_t steps;          //一个VD周期运行的步数
    uint16_t step_period;   //微步周期
    
    int32_t cur_pos;        //当前位置
    int32_t tar_pos;        //目标位置
    //    int32_t sta_pos;        //初始位置
    int32_t acc_pos;        //加速结束位置
    int32_t dec_pos;        //开始减速位置
    
    int32_t cur_speed;       //当前速度
    //    int32_t sta_speed;       //初始速度
    int32_t tar_speed;       //目标速度
    int32_t dec_off;        //减速偏差
    
    int32_t acc;             //加速度1
    int32_t dec;             //加速度2
    
};

/* ms41969驱动参数 */
struct ms41969
{
    struct motor m1;
    struct motor m2;
};

static struct ms41969 ms41969_status;
static struct ms41969_ops ms41969_ops_s;

int32_t MS41969_SetPwmPulse(struct motor *pmotor, uint8_t per);

/***************************************************************************************************
* Description:  计算绝对值
***************************************************************************************************/
static inline int32_t m_abs(int32_t n)
{
    return abs(n);
}

/***************************************************************************************************
* Description:  计算从v0到v1所需的位移
***************************************************************************************************/
static inline int32_t m_acc_sum(int32_t v0, int32_t v1, int32_t acc)
{
    int32_t n, sum;
    
    if(v0 > v1) 
    {
        if((v0-v1) < acc) return (v0-v1);
        
        n = (v0-v1) / acc;
        sum = (((v0-acc) * n) - (n * (n-1)) * acc / 2);
    }
    else if(v0 < v1)
    {
        n = (v1 - v0) / acc;
        sum = (((v0+acc) * n) + (n * (n-1)) * acc / 2);
        if((v0 + n * acc) < v1)
            sum += v1;
    }
    else return 0;
    return sum;
}

/***************************************************************************************************
* Description: 计算减速到0所需位移
***************************************************************************************************/
static inline int32_t m_dec_sum(int32_t v, int32_t dec)
{
    int32_t n, sum;
    
    if(v <= dec)
        return v;
    
    n = v / dec;
    sum = (((v-dec) * n) - (n * (n-1)) * dec / 2);
    return sum;
}

/***************************************************************************************************
* Description:  设置电机峰值脉冲宽度
***************************************************************************************************/
int32_t MS41969_SetPwmPulse(struct motor *pmotor, uint8_t per)
{
    uint16_t temp;
    uint8_t pulse;
    
    if(per > 100) per = 100;
    
    /* 峰值脉冲 = x / (PWM_MODE * 8) */
    pulse = (PWM_MODE << 3) * per / 100;
    
    temp = (pulse << 8) | pulse;
    
    /* 电机2 */
    if(pmotor == &ms41969_status.m2)
    {
        if(ms41969_ops_s.write_reg(0x28, temp) != 0) return 1;
    }
    /* 电机1 */
    else
    {
        if(ms41969_ops_s.write_reg(0x23, temp) != 0) return 1;
    }
    return 0;
}

/***************************************************************************************************
* Description:  写入运行参数
***************************************************************************************************/
static inline void MS41969_WriteM1RunData(void)
{
    uint16_t temp;
    
    /* 细分，LED，使能，刹车，方向，步数 */
    temp = (M1_DIV << 12) | (ms41969_status.m1.enable << 10) | \
        ((ms41969_status.m1.dir ^ ms41969_status.m1.dir_reverse) << 8) | ms41969_status.m1.steps;
    ms41969_ops_s.write_reg(0x24, temp);
    
    /* 微步周期 */
    ms41969_ops_s.write_reg(0x25, ms41969_status.m1.step_period);
}
static inline void MS41969_WriteM2RunData(void)
{
    uint16_t temp;
    /* 细分，LED，使能，刹车，方向，步数 */
    temp = (M2_DIV << 12) | (ms41969_status.m2.enable << 10) | \
        ((ms41969_status.m2.dir ^ ms41969_status.m2.dir_reverse) << 8) | ms41969_status.m2.steps;
    ms41969_ops_s.write_reg(0x29, temp);
    
    /* 微步周期 */
    ms41969_ops_s.write_reg(0x2A, ms41969_status.m2.step_period);
}

/***************************************************************************************************
* Description:  计算运行参数
***************************************************************************************************/
static inline void MS41969_CalcuMotorPara(struct motor *pmotor, int32_t speed)
{
    pmotor->steps = speed;
    pmotor->step_period = pmotor->period / speed;
}

static inline int32_t MS41969_CalcuRunPara(struct motor *pmotor)
{
    if(IS_RUNNING(pmotor))
    {
        /********** 正转 **********/
        if(pmotor->dir)
        {
            /********** 加速 **********/
            if(pmotor->cur_pos < pmotor->acc_pos)
            {
                pmotor->cur_speed += pmotor->acc;
                if(pmotor->acc > 0)
                {
                    if(pmotor->cur_speed > pmotor->tar_speed) pmotor->cur_speed = pmotor->tar_speed;
                }
                else 
                {
                    if(pmotor->cur_speed < pmotor->tar_speed) pmotor->cur_speed = pmotor->tar_speed;
                }
                
                pmotor->cur_pos += pmotor->cur_speed;
                
                MS41969_CalcuMotorPara(pmotor, pmotor->cur_speed);
            }
            
            /********** 匀速 **********/
            else if(pmotor->cur_pos < pmotor->dec_pos)
            {
                pmotor->cur_pos += pmotor->cur_speed;
                return 0;
            }
            
            /********** 减速 **********/
            else if(pmotor->cur_pos < pmotor->tar_pos)
            {
                /* 插值 */
                if(pmotor->cur_speed <= pmotor->dec_off)
                {
                    if((pmotor->cur_pos + pmotor->dec_off) > pmotor->tar_pos)
                    {
                        pmotor->dec_off = pmotor->tar_pos - pmotor->cur_pos;
                        pmotor->cur_pos = pmotor->tar_pos;
                    }
                    else
                    {
                        pmotor->cur_pos += pmotor->dec_off;
                    }
                    MS41969_CalcuMotorPara(pmotor, pmotor->dec_off);
                    pmotor->dec_off = -1;
                }
                else
                {
                    pmotor->cur_speed -= pmotor->dec;
                    
                    if(pmotor->cur_speed == 0)
                    {
                        if(pmotor->tar_pos == (pmotor->cur_pos + pmotor->dec))
                        {
                            pmotor->cur_pos = pmotor->tar_pos;
                            MS41969_CalcuMotorPara(pmotor, pmotor->dec);
                            return 0;
                        }
                    }
                    
                    if(pmotor->cur_speed < 0) pmotor->cur_speed = 1;
                    
                    if((pmotor->cur_pos + pmotor->cur_speed) > pmotor->tar_pos)
                    {
                        pmotor->cur_speed = pmotor->tar_pos - pmotor->cur_pos;
                        pmotor->cur_pos = pmotor->tar_pos;
                    }
                    else
                    {
                        pmotor->cur_pos += pmotor->cur_speed;
                    }
                    MS41969_CalcuMotorPara(pmotor, pmotor->cur_speed);
                }
            }
            else
            {
                pmotor->cur_speed = 0;
                MS41969_CalcuMotorPara(pmotor, pmotor->cur_speed);
            }
        }
        
        
        /********** 反转 **********/
        else
        {
            /********** 加速 **********/
            if(pmotor->cur_pos > pmotor->acc_pos)
            {
                pmotor->cur_speed += pmotor->acc;
                if(pmotor->acc > 0)
                {
                    if(pmotor->cur_speed > pmotor->tar_speed) pmotor->cur_speed = pmotor->tar_speed;
                }
                else 
                {
                    if(pmotor->cur_speed < pmotor->tar_speed) pmotor->cur_speed = pmotor->tar_speed;
                }
                
                pmotor->cur_pos -= pmotor->cur_speed;
                
                MS41969_CalcuMotorPara(pmotor, pmotor->cur_speed);
            }
            
            /********** 匀速 **********/
            else if(pmotor->cur_pos > pmotor->dec_pos)
            {
                pmotor->cur_pos -= pmotor->cur_speed;
                return 0;
            }
            
            /********** 减速 **********/
            else if(pmotor->cur_pos > pmotor->tar_pos)
            {
                /* 插值 */
                if(pmotor->cur_speed <= pmotor->dec_off)
                {
                    if((pmotor->cur_pos - pmotor->dec_off) < pmotor->tar_pos)
                    {
                        pmotor->dec_off = pmotor->cur_pos - pmotor->tar_pos;
                        pmotor->cur_pos = pmotor->tar_pos;
                    }
                    else
                    {
                        pmotor->cur_pos -= pmotor->dec_off;
                    }
                    MS41969_CalcuMotorPara(pmotor, pmotor->dec_off);
                    pmotor->dec_off = -1;
                }
                else
                {
                    pmotor->cur_speed -= pmotor->dec;
                    
                    if(pmotor->cur_speed == 0)
                    {
                        if(pmotor->tar_pos == (pmotor->cur_pos - pmotor->dec))
                        {
                            pmotor->cur_pos = pmotor->tar_pos;
                            MS41969_CalcuMotorPara(pmotor, pmotor->dec);
                            return 0;
                        }
                    }
                    
                    if(pmotor->cur_speed < 0) pmotor->cur_speed = 1;
                    
                    if((pmotor->cur_pos - pmotor->cur_speed) < pmotor->tar_pos)
                    {
                        pmotor->cur_speed = pmotor->cur_pos - pmotor->tar_pos;
                        pmotor->cur_pos = pmotor->tar_pos;
                    }
                    else
                    {
                        pmotor->cur_pos -= pmotor->cur_speed;
                    }
                    
                    MS41969_CalcuMotorPara(pmotor, pmotor->cur_speed);
                }
            }
            else
            {
                pmotor->cur_speed = 0;
                MS41969_CalcuMotorPara(pmotor, pmotor->cur_speed);
            }
        }
        return 1;
    }
    
    
    /********** 停车 **********/
    else
    {
        if(pmotor->brake == 0)
        {
            pmotor->brake = 1;
            pmotor->cur_speed = 0;
            MS41969_CalcuMotorPara(pmotor, 0);
            
            if(pmotor->disable_when_stop)
                pmotor->enable = 0;
            pmotor->max_power = pmotor->stop_power;
            MS41969_SetPwmPulse(pmotor, pmotor->max_power);
            
            return 1;
        }
        else if(pmotor->brake == 1)
        {
            pmotor->brake = 2;
            
            if((!IS_RUNNING(&ms41969_status.m1)) && (!IS_RUNNING(&ms41969_status.m2)))
            {
                pmotor->stop_vd();
            }
        }
    }
    
    return 0;
}

/***************************************************************************************************
* Description:  VD脉冲处理
***************************************************************************************************/
void MS41969_M1_VdPulseHandler(void)
{
    if(MS41969_CalcuRunPara(&ms41969_status.m1))
    {
        MS41969_WriteM1RunData();
    }
}
void MS41969_M2_VdPulseHandler(void)
{
    if(MS41969_CalcuRunPara(&ms41969_status.m2))
    {
        MS41969_WriteM2RunData();
    }
}

/***************************************************************************************************
* Description:  获取版本号
***************************************************************************************************/
const char* MS41969_GetVersion(void)
{
    return version_full;
}

/***************************************************************************************************
* Description:  设置电机运转方向
***************************************************************************************************/
int32_t MS41969_SetDirInv(int32_t motor, uint8_t inv)
{
    if(motor)
    {
        if(inv) ms41969_status.m2.dir_reverse = 1;
        else ms41969_status.m2.dir_reverse = 0;
    }
    else
    {
        if(inv) ms41969_status.m1.dir_reverse = 1;
        else ms41969_status.m1.dir_reverse = 0;
    }
    return 0;
}

/***************************************************************************************************
* Description:  设置运行和停止电压比
***************************************************************************************************/
int32_t MS41969_SetStopPower(int32_t motor, uint8_t per)
{
    if(per > 100) per = 100;
    
    if(motor) ms41969_status.m2.stop_power = per;
    else ms41969_status.m1.stop_power = per;
    
    return 0;
}
int32_t MS41969_SetRunPower(int32_t motor, uint8_t per)
{
    if(per > 100) per = 100;
    
    if(motor) ms41969_status.m2.run_power = per;
    else ms41969_status.m1.run_power = per;
    
    return 0;
}

/***************************************************************************************************
* Description:  停止时关断电机
***************************************************************************************************/
int32_t MS41969_SetStopCutOff(int32_t motor, uint8_t onoff)
{
    if(onoff) onoff = 1;
    
    if(motor) ms41969_status.m2.disable_when_stop = onoff;
    else ms41969_status.m1.disable_when_stop = onoff;
    
    return 0;
}

/***************************************************************************************************
* Description:  ms41969全局初始化
***************************************************************************************************/
int32_t MS41969_Init(struct ms41969_ops *ops)
{
    uint16_t temp;
    
    if(ops == NULL)
    {
        return 1;
    }
    /* 注册操作接口 */
    memcpy(&ms41969_ops_s, ops, sizeof(struct ms41969_ops));
    
    /* 初始化状态 */
    memset(&ms41969_status, 0, sizeof(struct ms41969));

    ms41969_status.m1.start_vd = ms41969_ops_s.m1_start_vd;
    ms41969_status.m1.stop_vd = ms41969_ops_s.m1_stop_vd;
    ms41969_status.m2.start_vd = ms41969_ops_s.m2_start_vd;
    ms41969_status.m2.stop_vd = ms41969_ops_s.m2_stop_vd;
    
    /* 设置默认VD频率 */
    MS41969_SetFrequency(MS41969_MOTOR1, DEF_M1_VD_FREQ, DEF_OSC_FREQ);
    MS41969_SetFrequency(MS41969_MOTOR2, DEF_M2_VD_FREQ, DEF_OSC_FREQ);
    
    /* 设置默认电压 */
    ms41969_status.m1.stop_power = DEF_M1_STOP_POWER;
    ms41969_status.m1.run_power = DEF_M1_RUN_POWER;
    ms41969_status.m2.stop_power = DEF_M2_STOP_POWER;
    ms41969_status.m2.run_power = DEF_M2_RUN_POWER;

    /************************ 全局参数 ************************/
    /* VD 极性 */
    temp = (VD_POLARITY << 8) | (1<<7);
    if(ms41969_ops_s.write_reg(0x0B, temp) != 0) goto error;
    
    /* pwm分辨率，pwm模式，delay1 */
    temp = (PWM_RES << 13) | (PWM_MODE << 8) | DELAY1;
    if(ms41969_ops_s.write_reg(0x20, temp) != 0) goto error;
    
    /* 测试模式 */
    if(ms41969_ops_s.write_reg(0x21, 0x87) != 0) goto error;
    
    
    /************************ MOTOR1 ************************/
    /* 相位差，motor1_delay2 */
    temp = (M1_PHASE << 8) | M1_DELAY;
    if(ms41969_ops_s.write_reg(0x22, temp) != 0) goto error;
    
    /* 步数，方向，刹车，使能，LED，细分 */
    if(ms41969_ops_s.write_reg(0x24, 0) != 0) goto error;
    
    /* 微步周期 */
    if(ms41969_ops_s.write_reg(0x25, 0) != 0) goto error;
    
    
    /************************ MOTOR2 ************************/
    /* 相位差，motor2_delay2 */
    temp = (M2_PHASE << 8) | M2_DELAY;
    if(ms41969_ops_s.write_reg(0x27, temp) != 0) goto error;
    
    /* 步数，方向，刹车，使能，LED，细分 */
    if(ms41969_ops_s.write_reg(0x29, 0) != 0) goto error;
    
    /* 微步周期 */
    if(ms41969_ops_s.write_reg(0x2A, 0) != 0) goto error;
    
    return 0;
    
    error:
    return 1;
}

/***************************************************************************************************
* Description:  电机刹车
***************************************************************************************************/
int32_t MS41969_MotorBrake(int32_t motor, int8_t dec)
{
    struct motor *pmotor;
    int32_t dec_dist;
    
    if(motor) pmotor = &ms41969_status.m2;
    else pmotor = &ms41969_status.m1;
    
    if(!IS_RUNNING(pmotor))
        return 0;
    
    pmotor->stop_vd();
    
    dec_dist = m_dec_sum(pmotor->cur_speed, dec);
    
    pmotor->dec = dec;
    pmotor->tar_speed = 0;
    pmotor->acc_pos = pmotor->cur_pos;
    pmotor->dec_pos = pmotor->cur_pos;
    pmotor->dec_off = -1;
    
    if(pmotor->dir)
    {
        pmotor->tar_pos = pmotor->cur_pos + dec_dist;
    }
    else
    {
        pmotor->tar_pos = pmotor->cur_pos - dec_dist;
    }
    pmotor->brake = 0;
    
    pmotor->start_vd();
    
    return dec_dist;
}

/***************************************************************************************************
* Description:  电机运行
***************************************************************************************************/
int32_t MS41969_MotorGoto(int32_t motor, int32_t tar_pos, uint8_t speed, int8_t acc, int8_t dec)
{
    uint8_t flag = 0;
    struct motor *pmotor;
    static int32_t acc_dist, dec_dist, pos_off;
    
    if(speed == 0 || acc == 0 || dec == 0)
        return 1;
    
    if(motor) pmotor = &ms41969_status.m2;
    else pmotor = &ms41969_status.m1;
    
    if(tar_pos == pmotor->cur_pos)
        return 1;
    
    if((tar_pos == pmotor->tar_pos) && (speed == pmotor->tar_speed))
        return 1;
    
    pmotor->stop_vd();
    
    /* 正在运行 */
    if(IS_RUNNING(pmotor))
    {
        flag = 1;
        /* 反向 */
        if(pmotor->dir != RUN_DIR(pmotor->cur_pos, tar_pos))
        {
            MS41969_MotorBrake(motor, INV_BRAKE_SPEED);
            while(IS_RUNNING(pmotor))
            {
                
            }
            MS41969_MotorGoto(motor, tar_pos, speed, acc, dec);
        }
        /* 同向 */
        else
        {
            pos_off = POS_DIFF(tar_pos, pmotor->cur_pos);
            acc_dist = m_acc_sum(pmotor->cur_speed, speed, acc);
            dec_dist = m_dec_sum(speed, dec);
            
            /* 距离不足 */
            if(pos_off < (acc_dist + dec_dist))
            {
                /* 距离不足以直接减速 */
                if(pos_off < m_dec_sum(pmotor->cur_speed, dec))
                {
                    /* 距离足以直接刹车，计算合适的减速度立即减速运行到目标位置 */
                    if(pos_off > m_dec_sum(pmotor->cur_speed, INV_BRAKE_SPEED))
                    {
                        uint8_t new_dec = dec;
                        do {
                            dec_dist = m_dec_sum(pmotor->cur_speed, new_dec);
                        } while((pos_off < dec_dist) && (++new_dec));
                        
                        if((pos_off-dec_dist) % speed)
                        {
                            pmotor->dec_off = (pos_off-dec_dist) % speed;
                            dec_dist += pmotor->dec_off;
                        }
                        acc_dist = 0;
                        pmotor->acc = 0;
                        pmotor->dec = new_dec;
                        pmotor->tar_speed = 0;
                        pmotor->acc_pos = pmotor->cur_pos;
                        pmotor->dec_pos = pmotor->cur_pos;
                        goto END;
                    }
                    /* 距离不足以直接刹车，先刹停再运行到目标位置 */
                    else
                    {
                        MS41969_MotorBrake(motor, INV_BRAKE_SPEED);
                        while(IS_RUNNING(pmotor))
                        {
                            
                        }
                        MS41969_MotorGoto(motor, tar_pos, speed, acc, dec);
                    }
                }
                /* 距离足以直接减速 */
                else
                {
                    /* 当前速度大于期望速度，加大加速度以期望的速度和减速度运行 */
                    if(pmotor->cur_speed > speed)
                    {
                        uint8_t new_acc = acc;
						if(pos_off <= dec_dist)
						{
							new_acc = new_acc;
						}
						else
						{
							while(pos_off < (m_acc_sum(pmotor->cur_speed, speed, new_acc) + dec_dist))
							{
								new_acc++;
							}
						}
                        acc = new_acc;
                        goto R;
                    }
                    /* 当前速度小于等于期望速度，降低期望速度以期望的加速度和减速度运行 */
                    else
                    {
                        goto R;
                    }
                }
            }
            /* 距离足以加减速 */
            else
            {
                goto R;
            }
        }
    }
    /* 静止状态 */
    else
    {
        R:
        if(flag == 0)
        {
            pmotor->cur_speed = 0;
        }
        pos_off = POS_DIFF(tar_pos, pmotor->cur_pos);
        acc_dist = m_acc_sum(pmotor->cur_speed, speed, acc);
        dec_dist = m_dec_sum(speed, dec);
        
        /* 距离不足 */
        if(pos_off < (acc_dist + dec_dist))
        {
            uint8_t new_speed = speed;
            do
            {
                if(new_speed > 1)
                    new_speed--;
                else
                    new_speed = 1;
                
                acc_dist = m_acc_sum(pmotor->cur_speed, new_speed, acc);
                dec_dist = m_dec_sum(new_speed, dec);
            } while(((acc_dist + dec_dist) > pos_off) && (new_speed > 1));
            speed = new_speed;
            if(speed < 1)
            {
                speed = 1;
                acc_dist = m_acc_sum(0, new_speed, acc);
                dec_dist = m_dec_sum(new_speed, dec);
            }
        }
        
        /* 消除偏差 */
        if((pos_off-acc_dist-dec_dist) % speed)
        {
            pmotor->dec_off = (pos_off-acc_dist-dec_dist) % speed;
            dec_dist += pmotor->dec_off;
            if(dec == 0)
                dec = 1;
        }
        
        if(pmotor->cur_speed < speed) pmotor->acc = acc;
        else if(pmotor->cur_speed > speed)pmotor->acc = acc * -1;
        else pmotor->acc = 0;
        pmotor->dec = dec;
        pmotor->tar_speed = speed;
        pmotor->tar_pos = tar_pos;
    }
    
    END:
    if(RUN_DIR(pmotor->cur_pos, pmotor->tar_pos))
    {
        pmotor->acc_pos = pmotor->cur_pos + acc_dist;
        pmotor->dec_pos = tar_pos - dec_dist;
        pmotor->dir = 1;
    }
    else
    {
        pmotor->acc_pos = pmotor->cur_pos - acc_dist;
        pmotor->dec_pos = tar_pos + dec_dist;
        pmotor->dir = 0;
    }
    
    pmotor->max_power = pmotor->run_power;
    MS41969_SetPwmPulse(pmotor, pmotor->max_power);
    pmotor->brake = 0;
    pmotor->enable = 1;
    /* 恢复脉冲 */
    pmotor->start_vd();
    
    return 0;
}

/***************************************************************************************************
* Description:  使能电机
***************************************************************************************************/
int32_t MS41969_MotorEnable(int32_t motor, int32_t new_state)
{
    if(motor)
    {
        if(new_state)
        {
            ms41969_status.m2.enable = 1;
        }
        else
        {
            if(IS_RUNNING(&ms41969_status.m2))
                return 1;
            ms41969_status.m2.enable = 0;
        }
        MS41969_WriteM2RunData();
    }
    else
    {
        if(new_state)
        {
            ms41969_status.m1.enable = 1;
        }
        else
        {
            if(IS_RUNNING(&ms41969_status.m1))
                return 1;
            ms41969_status.m1.enable = 0;
        }
        MS41969_WriteM1RunData();
    }
    return 0;
}

/***************************************************************************************************
* Description:  检查电机状态
***************************************************************************************************/
int32_t MS41969_IsMotorRunning(int32_t motor)
{
    if(motor)
    {
        return IS_RUNNING(&ms41969_status.m2);
    }
    else
    {
        return IS_RUNNING(&ms41969_status.m1);
    }
}

/***************************************************************************************************
* Description:  获取电机当前的速度
***************************************************************************************************/
int32_t MS41969_GetMotorCurSpeed(int32_t motor)
{
    if(motor)
    {
        return ms41969_status.m2.cur_speed;
    }
    else
    {
        return ms41969_status.m1.cur_speed;
    }
}

/***************************************************************************************************
* Description:  获取电机当前位置
***************************************************************************************************/
int32_t MS41969_GetMotorPosition(int32_t motor)
{
    if(motor)
    {
        return ms41969_status.m2.cur_pos;
    }
    else
    {
        return ms41969_status.m1.cur_pos;
    }
}

/***************************************************************************************************
* Description:  修改电机当前位置
***************************************************************************************************/
int32_t MS41969_SetMotorPosition(int32_t motor, int32_t new_pos)
{
    if(motor)
    {
        ms41969_status.m2.cur_pos = new_pos;
    }
    else
    {
        ms41969_status.m1.cur_pos = new_pos;
    }
    return 0;
}

/***************************************************************************************************
* Description:  设置脉冲周期
***************************************************************************************************/
int32_t MS41969_SetFrequency(int32_t motor, uint8_t freq, uint32_t osc)
{
    uint16_t period;
     
    if(freq == 0)
        return 1;
    
    period = osc / 24 / freq;
    
    if(motor)
    {
        ms41969_status.m2.vd_freq = freq;
        ms41969_status.m2.period = period;
    }
    else
    {
        ms41969_status.m1.vd_freq = freq;
        ms41969_status.m1.period = period;
    }

    return 0;
}








/****************************************** END OF FILE *******************************************/
