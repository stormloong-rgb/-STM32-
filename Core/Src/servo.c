#include "servo.h"

/* 舵机当前状态 */
static Servo_StateTypeDef servo_state = SERVO_STATE_CLOSED;

/* 舵机打开时的时间戳（用于超时自动关闭） */
static uint32_t servo_open_tick = 0;

/**
 * @brief  舵机初始化
 * @note   启动PWM输出，初始化为0度（关闭状态）
 */
void Servo_Init(void)
{
    if (htim3.Instance == NULL) return;
    
    /* 启动TIM3通道1的PWM输出 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    
    /* 初始化为0度位置（关闭状态） */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_ANGLE_0);
    
    servo_state = SERVO_STATE_CLOSED;
}

/**
 * @brief  设置舵机角度
 * @param  angle: 目标角度（0-180度）
 * @note   将角度转换为PWM脉宽：0度对应500us，180度对应2500us
 */
void Servo_SetAngle(uint16_t angle)
{
    if (angle > 180) angle = 180;
    
    /* 线性映射：角度 -> PWM脉宽 */
    uint16_t pulse = SERVO_ANGLE_0 + (angle * (SERVO_ANGLE_180 - SERVO_ANGLE_0) / 180);
    
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}

/**
 * @brief  打开舵机（抬杆）
 * @note   设置角度为90度，记录打开时间
 */
void Servo_Open(void)
{
    Servo_SetAngle(90);
    servo_state = SERVO_STATE_OPEN;
    servo_open_tick = HAL_GetTick();
}

/**
 * @brief  关闭舵机（落杆）
 * @note   设置角度为0度
 */
void Servo_Close(void)
{
    Servo_SetAngle(0);
    servo_state = SERVO_STATE_CLOSED;
}

/**
 * @brief  延时打开后自动关闭
 * @param  delay_ms: 保持打开状态的延时（毫秒）
 * @note   常用于车辆通行后自动落杆
 */
void Servo_OpenDelay(uint32_t delay_ms)
{
    Servo_Open();
    HAL_Delay(delay_ms);
    Servo_Close();
}

/**
 * @brief  获取舵机当前状态
 * @retval 返回舵机状态枚举值
 * @note   若打开超过3秒则自动关闭
 */
Servo_StateTypeDef Servo_GetState(void)
{
    /* 检查是否处于打开状态且超时 */
    if (servo_state == SERVO_STATE_OPEN)
    {
        if (HAL_GetTick() - servo_open_tick > 3000)
        {
            /* 超过3秒自动关闭 */
            Servo_Close();
        }
    }
    
    return servo_state;
}