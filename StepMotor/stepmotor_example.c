/**
 * @file stepmotor_example.c
 * @author advancer (2782654660@qq.com)
 * @brief 这是步进电机的示例文件
 * @version 0.1
 * @date 2023-06-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "stepmotor.h"
extern StepMotorHandle_t step_motor0_handle;
int app_main(void)
{
    StepMotorInit(); // 步进电机初始化

    // 除能测试
    SetStepMotorRotSpeed(step_motor0_handle, 400); // 旋转速度，每秒400步
    SetStepMotorPosAbs(step_motor0_handle, 400);   // 设置位置，400
    rt_thread_mdelay(5000);                        // 延时，等待电机转到指定位置
    StepMotorCmd(step_motor0_handle, false);       // 除能步进电机，此时步进电机可以用手转动

    // 绝对位置
    while (1)
    {
        SetStepMotorRotSpeed(step_motor0_handle, 800);//正转至4圈耗时1秒
        SetStepMotorPosAbs(step_motor0_handle, 800);
        rt_thread_mdelay(5000);
        SetStepMotorRotSpeed(step_motor0_handle, -800);//反转至0耗时1秒
        SetStepMotorPosAbs(step_motor0_handle, 0);
        rt_thread_mdelay(5000);
    }
    // 相对位置
    //    while(1)
    //    {
    //        SetStepMotorRotSpeed(step_motor0_handle,200);
    //        SetStepMotorPosRel(step_motor0_handle,50);
    //        rt_thread_mdelay(2000);
    ////        SetStepMotorRotSpeed(step_motor0_handle,-200);
    //        SetStepMotorPosRel(step_motor0_handle,150);
    //        rt_thread_mdelay(2000);
    //    }

    // 增量位置
    //        while(1)
    //        {
    //            SetStepMotorPosInc(step_motor0_handle, 200, 100);
    //            rt_thread_mdelay(5000);
    //            SetStepMotorPosInc(step_motor0_handle, -200, -100);
    //            rt_thread_mdelay(5000);
    //        }

    // 角度位置
    //        while(1)
    //        {
    //            SetStepMotorRotSpeed(step_motor0_handle,200);
    //            SetStepMotorAngleAbs(step_motor0_handle,360.0f);
    //            rt_thread_mdelay(5000);
    //            SetStepMotorRotSpeed(step_motor0_handle,-200);
    //            SetStepMotorAngleAbs(step_motor0_handle,-360.0f);
    //            rt_thread_mdelay(5000);
    //        }

    // 角速度
    //    while(1)
    //    {
    //        SetStepMotorRotAngSpeed(step_motor0_handle,360.0f);//逆时针一周每秒
    //        SetStepMotorAngleAbs(step_motor0_handle,360.0f);
    //        rt_thread_mdelay(5000);
    //        SetStepMotorRotAngSpeed(step_motor0_handle,-360.0f);
    //        SetStepMotorAngleAbs(step_motor0_handle,-360.0f);
    //        rt_thread_mdelay(5000);
    //    }
    return 0;
}
