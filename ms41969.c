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



/* �汾��Ϣ */
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

/* ��������ʱɲ���ٶ� */
#define INV_BRAKE_SPEED             10







/* �����ת���� */
#define DIR_POS                 0
#define DIR_NEG                 1

/* ���ɲ����һ�㲻ʹ�� */
#define BRAKE_DISABLE           0
#define BRAKE_ENABLE            1

/* ϸ��ģʽ */
#define DIV_64                  3
#define DIV_128                 2
#define DIV_256                 0



///////////////////////////////////////// �Ĵ������� ///////////////////////////////////////////////

/* VD FZ����*/
#define VD_RAISING                  0
#define VD_FALLING                  1
#define VD_POLARITY                 VD_RAISING


/* DELAY1����λ��303.4us */
#define DELAY1                      15

/* PWMƵ��=(OSCIN / ((mode * 2^3) * 2^res)) 
Ĭ��mode=8��res=1����ʱpwmƵ��Ϊ210.9k */
/* PWMģʽ */
#define PWM_MODE                    8

/* PWM�ֱ��� */
#define PWM_RES                     2


/* ����ģʽ */
#define TEST_MODE_



/* MOTOR1 DELAY����λ��303.4us */
#define M1_DELAY                    2

/* MOTOR1 ��λ���� */
#define M1_PHASE                    0

/* MOTOR1 ��ֵ���� */
#define M1_MAX_PULSE                100

/* MOTOR1 ϸ���� */
#define M1_DIV                      DIV_256


/* MOTOR2 DELAY����λ��303.4us */
#define M2_DELAY                    2

/* MOTOR2 ��λ���� */
#define M2_PHASE                    0

/* MOTOR2 ��ֵ���� */
#define M2_MAX_PULSE                100

/* MOTOR2 ϸ���� */
#define M2_DIV                      DIV_256



/* �жϵ���Ƿ������� */
#define IS_RUNNING(m)               (!((m)->cur_pos == (m)->tar_pos))

/* ������ת���� */
#define RUN_DIR(cur, tar)           (((tar)-(cur)) > 0 ? 1:0)

/* ��������ֵ */
#define POS_DIFF(a, b)              (m_abs(b-a))


/* ������� */
struct motor
{
    /* �����ӿ� */
    void (*start_vd)(void);
    void (*stop_vd)(void);
    
    /* VD */
    uint8_t vd_freq;
    uint16_t period;
    
    /* ģʽ���� */
    uint8_t dir_reverse;
    uint8_t stop_power;
    uint8_t run_power;
    uint8_t disable_when_stop;

    /* �Ĵ������� */
    uint8_t enable;         //���ʹ��
    uint8_t brake;          //ɲ��
    uint8_t max_power;      //��ֵ���壬0~100
    uint8_t phase;          //��λ����
    
    uint8_t dir;            //��ת����
    uint8_t steps;          //һ��VD�������еĲ���
    uint16_t step_period;   //΢������
    
    int32_t cur_pos;        //��ǰλ��
    int32_t tar_pos;        //Ŀ��λ��
    //    int32_t sta_pos;        //��ʼλ��
    int32_t acc_pos;        //���ٽ���λ��
    int32_t dec_pos;        //��ʼ����λ��
    
    int32_t cur_speed;       //��ǰ�ٶ�
    //    int32_t sta_speed;       //��ʼ�ٶ�
    int32_t tar_speed;       //Ŀ���ٶ�
    int32_t dec_off;        //����ƫ��
    
    int32_t acc;             //���ٶ�1
    int32_t dec;             //���ٶ�2
    
};

/* ms41969�������� */
struct ms41969
{
    struct motor m1;
    struct motor m2;
};

static struct ms41969 ms41969_status;
static struct ms41969_ops ms41969_ops_s;

int32_t MS41969_SetPwmPulse(struct motor *pmotor, uint8_t per);

/***************************************************************************************************
* Description:  �������ֵ
***************************************************************************************************/
static inline int32_t m_abs(int32_t n)
{
    return abs(n);
}

/***************************************************************************************************
* Description:  �����v0��v1�����λ��
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
* Description: ������ٵ�0����λ��
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
* Description:  ���õ����ֵ������
***************************************************************************************************/
int32_t MS41969_SetPwmPulse(struct motor *pmotor, uint8_t per)
{
    uint16_t temp;
    uint8_t pulse;
    
    if(per > 100) per = 100;
    
    /* ��ֵ���� = x / (PWM_MODE * 8) */
    pulse = (PWM_MODE << 3) * per / 100;
    
    temp = (pulse << 8) | pulse;
    
    /* ���2 */
    if(pmotor == &ms41969_status.m2)
    {
        if(ms41969_ops_s.write_reg(0x28, temp) != 0) return 1;
    }
    /* ���1 */
    else
    {
        if(ms41969_ops_s.write_reg(0x23, temp) != 0) return 1;
    }
    return 0;
}

/***************************************************************************************************
* Description:  д�����в���
***************************************************************************************************/
static inline void MS41969_WriteM1RunData(void)
{
    uint16_t temp;
    
    /* ϸ�֣�LED��ʹ�ܣ�ɲ�������򣬲��� */
    temp = (M1_DIV << 12) | (ms41969_status.m1.enable << 10) | \
        ((ms41969_status.m1.dir ^ ms41969_status.m1.dir_reverse) << 8) | ms41969_status.m1.steps;
    ms41969_ops_s.write_reg(0x24, temp);
    
    /* ΢������ */
    ms41969_ops_s.write_reg(0x25, ms41969_status.m1.step_period);
}
static inline void MS41969_WriteM2RunData(void)
{
    uint16_t temp;
    /* ϸ�֣�LED��ʹ�ܣ�ɲ�������򣬲��� */
    temp = (M2_DIV << 12) | (ms41969_status.m2.enable << 10) | \
        ((ms41969_status.m2.dir ^ ms41969_status.m2.dir_reverse) << 8) | ms41969_status.m2.steps;
    ms41969_ops_s.write_reg(0x29, temp);
    
    /* ΢������ */
    ms41969_ops_s.write_reg(0x2A, ms41969_status.m2.step_period);
}

/***************************************************************************************************
* Description:  �������в���
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
        /********** ��ת **********/
        if(pmotor->dir)
        {
            /********** ���� **********/
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
            
            /********** ���� **********/
            else if(pmotor->cur_pos < pmotor->dec_pos)
            {
                pmotor->cur_pos += pmotor->cur_speed;
                return 0;
            }
            
            /********** ���� **********/
            else if(pmotor->cur_pos < pmotor->tar_pos)
            {
                /* ��ֵ */
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
        
        
        /********** ��ת **********/
        else
        {
            /********** ���� **********/
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
            
            /********** ���� **********/
            else if(pmotor->cur_pos > pmotor->dec_pos)
            {
                pmotor->cur_pos -= pmotor->cur_speed;
                return 0;
            }
            
            /********** ���� **********/
            else if(pmotor->cur_pos > pmotor->tar_pos)
            {
                /* ��ֵ */
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
    
    
    /********** ͣ�� **********/
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
* Description:  VD���崦��
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
* Description:  ��ȡ�汾��
***************************************************************************************************/
const char* MS41969_GetVersion(void)
{
    return version_full;
}

/***************************************************************************************************
* Description:  ���õ����ת����
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
* Description:  �������к�ֹͣ��ѹ��
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
* Description:  ֹͣʱ�ضϵ��
***************************************************************************************************/
int32_t MS41969_SetStopCutOff(int32_t motor, uint8_t onoff)
{
    if(onoff) onoff = 1;
    
    if(motor) ms41969_status.m2.disable_when_stop = onoff;
    else ms41969_status.m1.disable_when_stop = onoff;
    
    return 0;
}

/***************************************************************************************************
* Description:  ms41969ȫ�ֳ�ʼ��
***************************************************************************************************/
int32_t MS41969_Init(struct ms41969_ops *ops)
{
    uint16_t temp;
    
    if(ops == NULL)
    {
        return 1;
    }
    /* ע������ӿ� */
    memcpy(&ms41969_ops_s, ops, sizeof(struct ms41969_ops));
    
    /* ��ʼ��״̬ */
    memset(&ms41969_status, 0, sizeof(struct ms41969));

    ms41969_status.m1.start_vd = ms41969_ops_s.m1_start_vd;
    ms41969_status.m1.stop_vd = ms41969_ops_s.m1_stop_vd;
    ms41969_status.m2.start_vd = ms41969_ops_s.m2_start_vd;
    ms41969_status.m2.stop_vd = ms41969_ops_s.m2_stop_vd;
    
    /* ����Ĭ��VDƵ�� */
    MS41969_SetFrequency(MS41969_MOTOR1, DEF_M1_VD_FREQ, DEF_OSC_FREQ);
    MS41969_SetFrequency(MS41969_MOTOR2, DEF_M2_VD_FREQ, DEF_OSC_FREQ);
    
    /* ����Ĭ�ϵ�ѹ */
    ms41969_status.m1.stop_power = DEF_M1_STOP_POWER;
    ms41969_status.m1.run_power = DEF_M1_RUN_POWER;
    ms41969_status.m2.stop_power = DEF_M2_STOP_POWER;
    ms41969_status.m2.run_power = DEF_M2_RUN_POWER;

    /************************ ȫ�ֲ��� ************************/
    /* VD ���� */
    temp = (VD_POLARITY << 8) | (1<<7);
    if(ms41969_ops_s.write_reg(0x0B, temp) != 0) goto error;
    
    /* pwm�ֱ��ʣ�pwmģʽ��delay1 */
    temp = (PWM_RES << 13) | (PWM_MODE << 8) | DELAY1;
    if(ms41969_ops_s.write_reg(0x20, temp) != 0) goto error;
    
    /* ����ģʽ */
    if(ms41969_ops_s.write_reg(0x21, 0x87) != 0) goto error;
    
    
    /************************ MOTOR1 ************************/
    /* ��λ�motor1_delay2 */
    temp = (M1_PHASE << 8) | M1_DELAY;
    if(ms41969_ops_s.write_reg(0x22, temp) != 0) goto error;
    
    /* ����������ɲ����ʹ�ܣ�LED��ϸ�� */
    if(ms41969_ops_s.write_reg(0x24, 0) != 0) goto error;
    
    /* ΢������ */
    if(ms41969_ops_s.write_reg(0x25, 0) != 0) goto error;
    
    
    /************************ MOTOR2 ************************/
    /* ��λ�motor2_delay2 */
    temp = (M2_PHASE << 8) | M2_DELAY;
    if(ms41969_ops_s.write_reg(0x27, temp) != 0) goto error;
    
    /* ����������ɲ����ʹ�ܣ�LED��ϸ�� */
    if(ms41969_ops_s.write_reg(0x29, 0) != 0) goto error;
    
    /* ΢������ */
    if(ms41969_ops_s.write_reg(0x2A, 0) != 0) goto error;
    
    return 0;
    
    error:
    return 1;
}

/***************************************************************************************************
* Description:  ���ɲ��
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
* Description:  �������
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
    
    /* �������� */
    if(IS_RUNNING(pmotor))
    {
        flag = 1;
        /* ���� */
        if(pmotor->dir != RUN_DIR(pmotor->cur_pos, tar_pos))
        {
            MS41969_MotorBrake(motor, INV_BRAKE_SPEED);
            while(IS_RUNNING(pmotor))
            {
                
            }
            MS41969_MotorGoto(motor, tar_pos, speed, acc, dec);
        }
        /* ͬ�� */
        else
        {
            pos_off = POS_DIFF(tar_pos, pmotor->cur_pos);
            acc_dist = m_acc_sum(pmotor->cur_speed, speed, acc);
            dec_dist = m_dec_sum(speed, dec);
            
            /* ���벻�� */
            if(pos_off < (acc_dist + dec_dist))
            {
                /* ���벻����ֱ�Ӽ��� */
                if(pos_off < m_dec_sum(pmotor->cur_speed, dec))
                {
                    /* ��������ֱ��ɲ����������ʵļ��ٶ������������е�Ŀ��λ�� */
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
                    /* ���벻����ֱ��ɲ������ɲͣ�����е�Ŀ��λ�� */
                    else
                    {
                        MS41969_MotorBrake(motor, INV_BRAKE_SPEED);
                        while(IS_RUNNING(pmotor))
                        {
                            
                        }
                        MS41969_MotorGoto(motor, tar_pos, speed, acc, dec);
                    }
                }
                /* ��������ֱ�Ӽ��� */
                else
                {
                    /* ��ǰ�ٶȴ��������ٶȣ��Ӵ���ٶ����������ٶȺͼ��ٶ����� */
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
                    /* ��ǰ�ٶ�С�ڵ��������ٶȣ����������ٶ��������ļ��ٶȺͼ��ٶ����� */
                    else
                    {
                        goto R;
                    }
                }
            }
            /* �������ԼӼ��� */
            else
            {
                goto R;
            }
        }
    }
    /* ��ֹ״̬ */
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
        
        /* ���벻�� */
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
        
        /* ����ƫ�� */
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
    /* �ָ����� */
    pmotor->start_vd();
    
    return 0;
}

/***************************************************************************************************
* Description:  ʹ�ܵ��
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
* Description:  �����״̬
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
* Description:  ��ȡ�����ǰ���ٶ�
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
* Description:  ��ȡ�����ǰλ��
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
* Description:  �޸ĵ����ǰλ��
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
* Description:  ������������
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
