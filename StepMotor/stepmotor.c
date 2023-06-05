/**
 * @file stepMotor.c
 * @author advancer (2782654660@qq.com)
 * @brief 步进电机驱动源文件
 * @version 0.1
 * @date 2023-05-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "./stepmotor.h"
#include <stdlib.h>

/**
 * @brief 初始化一个步进电机
 *
 * @param MotorDesc 步进电机描述结构体
 * @param MotorDriver 步进电机驱动结构体
 * @return StepMotorHandle_t 步进电机句柄
 */
StepMotorHandle_t StepMotorSetup(const step_motor_desc_t *MotorDesc, const step_motor_driver_t *MotorDriver)
{
    if (!MotorDesc || !MotorDriver)
    {
        return NULL;
    }
    StepMotorHandle_t handle = (StepMotorHandle_t)malloc(sizeof(step_motor_handle_t));
    if (handle)
    {
        handle->driver = MotorDriver;
        handle->driver->desc = MotorDesc;
        StepMotorResetState(handle);
        handle->driver->Msp_Init();
        return handle;
    }
    return NULL;
}

/**
 * @brief 重置步进电机状态值
 *
 * @param Motor 步进电机句柄
 */
step_motor_ret_t StepMotorResetState(StepMotorHandle_t Motor)
{
    Motor->state.step_value = 0;
    Motor->state.rotate_speed = 0;
    Motor->state.en = false;
    Motor->state.busy_flag = false;
    Motor->state.pulse_level = false;
    return step_motor_ok;
}

/**
 * @brief Get the Step Motor Pos Abs object
 *
 * @param Motor 步进电机句柄
 * @return int32_t 步进电机的绝对位置,单位:步,逆时针
 */
int32_t GetStepMotorPosAbs(StepMotorHandle_t Motor)
{
    return Motor->state.step_value;
}

/**
 * @brief Get the Step Motor Pos Rel object
 *
 * @param Motor 步进电机句柄
 * @return int32_t 步进电机的相对位置,单位:步
 */
int32_t GetStepMotorPosRel(StepMotorHandle_t Motor)
{
    return Motor->state.step_value % Motor->driver->desc->steps_per_round;
}

/**
 * @brief Get the Step Motor Angle Abs object
 *
 * @param Motor 步进电机句柄
 * @return float 步进电机的绝对角度,单位:度
 */
float GetStepMotorAngleAbs(StepMotorHandle_t Motor)
{
    return (float)Motor->state.step_value / Motor->driver->desc->steps_per_round * 360.0f;
}

/**
 * @brief Get the Step Motor Angle Rel object
 *
 * @param Motor 步进电机句柄
 * @return float 步进电机的相对角度
 */
float GetStepMotorAngleRel(StepMotorHandle_t Motor)
{
    return (float)(Motor->state.step_value % Motor->driver->desc->steps_per_round) /
           Motor->driver->desc->steps_per_round * 360.0f;
}

/**
 * @brief Get the Step Motor Rot Speed object
 *
 * @param Motor 步进电机句柄
 * @return int32_t 步进速度，步每秒,逆时针为正，顺时针为负
 */
int32_t GetStepMotorRotSpeed(StepMotorHandle_t Motor) // 步进速度
{
    return Motor->state.rotate_speed;
}

/**
 * @brief Get the Step Motor Rot Ang Speed object
 *
 * @param Motor 步进电机句柄
 * @return float 角速度，度每秒，逆时针为正，顺时针为负
 */
float GetStepMotorRotAngSpeed(StepMotorHandle_t Motor) // 角速度
{
    return (float)Motor->state.rotate_speed * Motor->driver->desc->step_angle;
}

/**
 * @brief Set the Step Motor Pos Abs object，
 *
 * @param Motor 步进电机句柄
 * @param Pos 相对于上电时的绝对位置，范围：min_step_value~max_step_value
 * @return step_motor_ret_t 错误码
 */
step_motor_ret_t SetStepMotorPosAbs(StepMotorHandle_t Motor, int32_t Pos) // 绝对位置
{
    if (Motor->state.busy_flag)
        return step_motor_busy; // 步进电机正忙
    int32_t PosInc = Pos - Motor->state.step_value;
    if (PosInc * Motor->state.rotate_speed < 0)
        return step_motor_direction_error; // 步进电机方向错误
    if (Motor->driver->desc->is_range_limited)
    {
        if (Pos > Motor->driver->desc->max_step_value || Pos < Motor->driver->desc->min_step_value)
            return step_motor_pos_abs_range_error;
    }
    Motor->state.target_step_value = Pos;
    if (!Motor->state.en)
    {
        StepMotorCmd(Motor, true); // 打开使能
    }
    Motor->state.busy_flag = true;
    return step_motor_ok;
}

/**
 * @brief Set the Step Motor Pos Rel object
 *
 * @param Motor 步进电机句柄
 * @param Pos 相对于上电位置的相对位置,范围：0~steps_per_round
 * @return step_motor_ret_t 错误码
 */
step_motor_ret_t SetStepMotorPosRel(StepMotorHandle_t Motor, int32_t Pos) // 相对位置
{
    if (Pos < 0 || Pos > Motor->driver->desc->steps_per_round)
        return step_motor_pos_rel_range_error; // 范围错误
    int32_t PosInc = Pos - GetStepMotorPosRel(Motor);
    int PosAbs = 0;
    if (Motor->state.dir)
        PosAbs = Motor->state.step_value - (Motor->driver->desc->steps_per_round - PosInc) % Motor->driver->desc->steps_per_round;
    else
        PosAbs = Motor->state.step_value + (Motor->driver->desc->steps_per_round + PosInc) % Motor->driver->desc->steps_per_round;
    return SetStepMotorPosAbs(Motor, PosAbs);
}

/**
 * @brief Set the Step Motor Pos Inc object,从当前位置以一定速度旋转一定增量，增量方向和速度方向需要相同
 *
 * @param Motor  步进电机句柄
 * @param PosInc 旋转增量
 * @param AbsSpeed 旋转速度
 * @return step_motor_ret_t 错误码
 */
step_motor_ret_t SetStepMotorPosInc(StepMotorHandle_t Motor, int32_t PosInc, int32_t AbsSpeed) // 以AbsSpeed从当前位置转动PosInc
{
    if (Motor->state.busy_flag)
        return step_motor_busy; // 步进电机正忙
    if (PosInc * Motor->state.rotate_speed < 0)
        return step_motor_direction_error; // 步进电机方向错误
    step_motor_ret_t ret = step_motor_ok;
    if (ret = SetStepMotorRotSpeed(Motor, AbsSpeed))
    {
        return ret;
    }
    return SetStepMotorPosAbs(Motor, PosInc + Motor->state.step_value);
}

/**
 * @brief Set the Step Motor Angle Abs object
 *
 * @param Motor  步进电机句柄
 * @param Ang 相对于上电时的绝对角度
 * @return step_motor_ret_t  错误码
 */
step_motor_ret_t SetStepMotorAngleAbs(StepMotorHandle_t Motor, float Ang) // 绝对角度
{
    int32_t PosAbs = StepMotorAngle2Step(Motor, Ang);
    return SetStepMotorPosAbs(Motor, PosAbs);
}

/**
 * @brief Set the Step Motor Angle Rel object
 *
 * @param Motor  步进电机句柄
 * @param Ang 相对于上电位置的相对角度，范围0~360
 * @return step_motor_ret_t  错误码
 */
step_motor_ret_t SetStepMotorAngleRel(StepMotorHandle_t Motor, float Ang) // 相对角度
{
    int32_t PosRel = StepMotorAngle2Step(Motor, Ang);
    return SetStepMotorPosRel(Motor, PosRel);
}

/**
 * @brief Set the Step Motor Rot Speed object
 *
 * @param Motor  步进电机句柄
 * @param Speed 电机转动速度，步每秒
 * @return step_motor_ret_t  错误码
 */
step_motor_ret_t SetStepMotorRotSpeed(StepMotorHandle_t Motor, int32_t Speed) // 步进速度
{
    if (Motor->driver->desc->is_step_speed_limited)
    {
        if (Speed > Motor->driver->desc->max_step_speed || Speed < Motor->driver->desc->min_step_speed)
            return step_motor_speed_range_error;
    }
    int32_t interrupt_us = 1000000 / (Speed > 0 ? Speed : -Speed) / 2;
    Motor->driver->Timer_SetPeroid(interrupt_us);
    Motor->driver->Set_Dir(Speed < 0);
    Motor->state.dir = Speed < 0;
    Motor->state.rotate_speed = Speed;
    return step_motor_ok;
}

/**
 * @brief Set the Step Motor Rot Ang Speed object
 *
 * @param Motor  步进电机句柄
 * @param Speed 电机转动角速度，度每秒
 * @return step_motor_ret_t  错误码
 */
step_motor_ret_t SetStepMotorRotAngSpeed(StepMotorHandle_t Motor, float Speed) // 角速度
{
    int32_t speed = StepMotorAngle2Step(Motor, Speed);
    return SetStepMotorRotSpeed(Motor, speed);
}

/**
 * @brief 步进电机停止转动，与StepAbort()不同的是，该函数不会改变目标位置,重新打开后，电机会转到目标位置
 *
 * @param Motor  步进电机句柄
 * @param State true:打开步进电机，false:关闭步进电机
 * @return step_motor_ret_t  错误码
 */
step_motor_ret_t StepMotorCmd(StepMotorHandle_t Motor, bool State) // 打开或关闭步进电机
{
    Motor->driver->EN_State(State);
    Motor->state.en = State;
    return step_motor_ok;
}

/**
 * @brief 终止当前的转动任务，与StepMotorCmd()不同的是，该函数会改变目标位置
 *
 * @param Motor  步进电机句柄
 * @return step_motor_ret_t  错误码
 */
step_motor_ret_t StepAbort(StepMotorHandle_t Motor) // 终止转动
{
    if (!Motor->state.en)
    {
        return step_motor_state_disable; // 未打开使能
    }
    if (StepMotorIsBusy(Motor))
    {
        if (Motor->state.pulse_level)
        {
            Motor->state.target_step_value = Motor->state.dir ? Motor->state.step_value - 1 : Motor->state.step_value + 1;
        }
        else
        {
            Motor->state.target_step_value = Motor->state.step_value;
        }
    }
    while (StepMotorIsBusy(Motor))
        ;
    return step_motor_ok;
}

/**
 * @brief 查询电机是否正忙
 *
 * @param Motor  步进电机句柄
 * @return true 电机正忙，无法执行设定目标位置等操作
 * @return false 电机闲，允许执行任何操作
 */
bool StepMotorIsBusy(StepMotorHandle_t Motor)
{
    return Motor->state.busy_flag;
}

/**
 * @brief 将步进电机的步数转换为角度
 *
 * @param Motor  步进电机句柄
 * @param Steps 需要转换的步进值
 * @return float 对应的角度值
 */
float StepMotorStep2Angle(StepMotorHandle_t Motor, int32_t Steps)
{
    return (float)Steps * Motor->driver->desc->step_angle;
}

/**
 * @brief 将步进电机的角度转化为步数
 *
 * @param Motor  步进电机句柄
 * @param Angle 需要转换的角度值
 * @return int32_t 对应的步进值
 */
int32_t StepMotorAngle2Step(StepMotorHandle_t Motor, float Angle)
{
    return (int32_t)(Angle / Motor->driver->desc->step_angle);
}

/**
 * @brief 步进电机定时器回调函数
 *
 * @param Motor  步进电机句柄
 */
void StepMotorTimerCallback(StepMotorHandle_t Motor)
{
    if (!Motor || !Motor->state.en)
    {
        return; // 未打开使能或句柄未初始化
    }
    if (Motor->state.pulse_level)
    {
        Motor->state.dir ? Motor->state.step_value-- : Motor->state.step_value++;
        Motor->driver->PulsePin_Setlevel(false); // 脉冲引脚输出低电平
        Motor->state.pulse_level = false;
    }
    else
    {
        if (Motor->state.step_value != Motor->state.target_step_value)
        {
            Motor->driver->PulsePin_Setlevel(true); // 脉冲引脚输出高电平
            Motor->state.pulse_level = true;
            Motor->state.busy_flag = true;
        }
        else
        {
            Motor->state.busy_flag = false;
        }
    }
}
