/**
 * @file stepMotor_driver.c
 * @author advancer (2782654660@qq.com)
 * @brief 步进电机驱动移植示例文件，以CH32V307为例
 * @version 0.1
 * @date 2023-05-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "./stepmotor.h"
#include "ch32v30x_conf.h"

// 引脚定义
#define ENPORT GPIOB
#define ENPIN GPIO_Pin_4
#define PULSEPORT GPIOA
#define PULSEPIN GPIO_Pin_15
#define DIRPORT GPIOA
#define DIRPIN GPIO_Pin_3

// 步进电机句柄
StepMotorHandle_t step_motor0_handle = NULL;

// 步进电机描述
static step_motor_desc_t step_motor_desc0 = {
    .step_angle = 1.8f,
    .steps_per_round = (int32_t)360.0f / 1.8f,
    .is_range_limited = false,
    .max_step_value = 0,
    .min_step_value = 0,
    .is_step_speed_limited = false,
    .max_step_speed = 0,
    .min_step_speed = 0,
};

// 步进电机MCU级初始化
static void step_motor0_Msp_Init(void);
// 步进电机打开或关闭使能
static void step_motor0_EN_State(bool state);
// 步进电机设置方向引脚电平
static void step_motor0_Set_Dir(bool closeWise);
// 步进电机设置脉冲引脚电平
static void step_motor0_PulsePin_Setlevel(bool highLevel);
// 步进电机设置定时器溢出时间
static void step_motor0_Timer_SetPeroid(int32_t us);

// 步进电机驱动
static step_motor_driver_t step_motor_drvier0 = {
    .desc = NULL,
    .Msp_Init = step_motor0_Msp_Init,
    .EN_State = step_motor0_EN_State,
    .Set_Dir = step_motor0_Set_Dir,
    .PulsePin_Setlevel = step_motor0_PulsePin_Setlevel,
    .Timer_SetPeroid = step_motor0_Timer_SetPeroid,
};
static void step_motor0_Msp_Init(void)
{
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    // 初始化引脚
    GPIO_InitTypeDef gpio_InitStructure = {
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_50MHz,
    };
    gpio_InitStructure.GPIO_Pin = ENPIN;
    GPIO_Init(ENPORT, &gpio_InitStructure);
    gpio_InitStructure.GPIO_Pin = PULSEPIN;
    GPIO_Init(PULSEPORT, &gpio_InitStructure);
    gpio_InitStructure.GPIO_Pin = DIRPIN;
    GPIO_Init(DIRPORT, &gpio_InitStructure);
    // 初始化定时器
    TIM_TimeBaseInitTypeDef tim_Init = {
        .TIM_Prescaler = 143, // 1MHz
        .TIM_CounterMode = TIM_CounterMode_Up,
        .TIM_Period = 1250, // 2.5ms一个脉冲
        .TIM_ClockDivision = TIM_CKD_DIV1,
        .TIM_RepetitionCounter = TIM_OPMode_Repetitive,
    };

    TIM_ARRPreloadConfig(TIM2, ENABLE); // 使能影子寄存器
    TIM_TimeBaseInit(TIM2, &tim_Init);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // 使能溢出中断
    TIM_Cmd(TIM2, ENABLE);
    //    TIM2->CTLR1 |= TIM_CEN;
    TIM2->CNT = 0;
    // 使能NVIC中断
    NVIC_InitTypeDef NVIC_InitStructure = {
        .NVIC_IRQChannel =
            TIM2_IRQn,
        .NVIC_IRQChannelPreemptionPriority = 5,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE,
    };
    NVIC_Init(&NVIC_InitStructure);
}

// 定时器中断
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void)
{
    rt_interrupt_enter();
    if (SET == TIM_GetFlagStatus(TIM2, TIM_FLAG_Update))
    {
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);       // 清除中断标志
        StepMotorTimerCallback(step_motor0_handle); // 调用步进电机回调函数，并传入步进电机句柄
    }
    rt_interrupt_leave();
}

static void step_motor0_EN_State(bool state)
{
    state ? (ENPORT->BSHR = ENPIN) : (ENPORT->BCR = ENPIN);
}
static void step_motor0_Set_Dir(bool closeWise)
{
    closeWise ? (DIRPORT->BSHR = DIRPIN) : (DIRPORT->BCR = DIRPIN);
}
static void step_motor0_PulsePin_Setlevel(bool highLevel)
{
    highLevel ? (PULSEPORT->BSHR = PULSEPIN) : (PULSEPORT->BCR = PULSEPIN);
}
static void step_motor0_Timer_SetPeroid(int32_t us)
{
    TIM2->ATRLR = us;
}

/**
 * @brief 步进电机初始化，需要在移植文件中实现
 * 
 */
void StepMotorInit(void)
{
    //初始化句柄，传入步进电机的描述和驱动
    step_motor0_handle = StepMotorSetup(&step_motor_desc0, &step_motor_drvier0);
}
