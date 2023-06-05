/**
 * @file stepmotor.h
 * @author advancer (2782654660@qq.com)
 * @brief 步进电机驱动头文件
 * @version 0.1
 * @date 2023-05-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float step_angle;        // 步距角，结合细分系数填写
    int32_t steps_per_round; // 步进电机转一圈的步数，结合细分系数填写

    bool is_range_limited;  // 是否旋转范围受限
    int32_t max_step_value; // 范围限制
    int32_t min_step_value;

    bool is_step_speed_limited; // 是否步进速度受限
    int32_t max_step_speed;     // 速度受限
    int32_t min_step_speed;
} step_motor_desc_t;

typedef struct
{
    int32_t step_value;   // 距离上电开始走过的位置
    int32_t rotate_speed; // 旋转速度，步每秒,逆时针为正
    bool en;
    bool busy_flag;
    int32_t target_step_value; // 目标位置
    bool pulse_level;          // 脉冲引脚电平
    bool dir;                  // 方向,true为顺时针，false为逆时针
    // 摆动模式
} step_motor_state_t;

struct step_motor_driver;
typedef struct step_motor_driver step_motor_driver_t;
typedef struct
{
    step_motor_driver_t *driver;
    step_motor_state_t state;
} step_motor_handle_t;
typedef step_motor_handle_t *StepMotorHandle_t;

struct step_motor_driver
{
    step_motor_desc_t *desc;                   // 步进电机的描述
    void (*Msp_Init)(void);                    // MCU相关初始化
    void (*EN_State)(bool state);              // 打开或关闭使能，使能打开时，步进电机输出力矩，手无法转动
    void (*Set_Dir)(bool closeWise);           // 设置旋转方向，closeWise:是否顺时钟旋转，方向信号控制步进电机的旋转方向
    void (*PulsePin_Setlevel)(bool highLevel); // 设置脉冲引脚的电平
    void (*Timer_SetPeroid)(int32_t us);       // 设置定时器的溢出周期，单位us
};

typedef enum
{
    step_motor_ok = 0,
    step_motor_busy,                // 步进电机正忙
    step_motor_state_enable,        // 步进电机已使能
    step_motor_state_disable,       // 步进电机未使能
    step_motor_speed_zero,          // 步进电机未设置速度，速度为0
    step_motor_direction_error,     // 步进电机方向错误
    step_motor_pos_rel_range_error, // 步进电机相对位置范围错误
                                    // ，正确范围0~steps_per_round，或0度~360度
    step_motor_pos_abs_range_error, // 步进电机绝对位置错误
    step_motor_speed_range_error,   // 步进电机速度范围错误
} step_motor_ret_t;

StepMotorHandle_t StepMotorSetup(const step_motor_desc_t *MotorDesc, const step_motor_driver_t *MotorDriver);
step_motor_ret_t StepMotorResetState(StepMotorHandle_t Motor);
int32_t GetStepMotorPosAbs(StepMotorHandle_t Motor);
int32_t GetStepMotorPosRel(StepMotorHandle_t Motor);
float GetStepMotorAngleAbs(StepMotorHandle_t Motor);
float GetStepMotorAngleRel(StepMotorHandle_t Motor);
int32_t GetStepMotorRotSpeed(StepMotorHandle_t Motor);
float GetStepMotorRotAngSpeed(StepMotorHandle_t Motor);
step_motor_ret_t SetStepMotorPosAbs(StepMotorHandle_t Motor, int32_t Pos);
step_motor_ret_t SetStepMotorPosRel(StepMotorHandle_t Motor, int32_t Pos);
step_motor_ret_t SetStepMotorPosInc(StepMotorHandle_t Motor, int32_t PosInc, int32_t AbsSpeed);
step_motor_ret_t SetStepMotorAngleAbs(StepMotorHandle_t Motor, float Ang);
step_motor_ret_t SetStepMotorAngleRel(StepMotorHandle_t Motor, float Ang);
step_motor_ret_t SetStepMotorRotSpeed(StepMotorHandle_t Motor, int32_t Speed);
step_motor_ret_t SetStepMotorRotAngSpeed(StepMotorHandle_t Motor, float Speed);
step_motor_ret_t StepMotorCmd(StepMotorHandle_t Motor, bool State);
step_motor_ret_t StepAbort(StepMotorHandle_t Motor);
bool StepMotorIsBusy(StepMotorHandle_t Motor);
float StepMotorStep2Angle(StepMotorHandle_t Motor, int32_t Steps);
int32_t StepMotorAngle2Step(StepMotorHandle_t Motor, float Angle);
void StepMotorInit(void);
void StepMotorTimerCallback(StepMotorHandle_t Motor);