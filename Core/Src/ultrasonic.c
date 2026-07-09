#include "ultrasonic.h"
#include "main.h"
#include "tim.h"

/* 车位状态标志位：0表示空闲，1表示占用 */
volatile uint8_t parking1_status = 0;
volatile uint8_t parking2_status = 0;

/* 车位编号 */
volatile uint8_t parking1_number = 1;
volatile uint8_t parking2_number = 2;

/* 超声波传感器配置结构体 */
/* trig_port/trig_pin: 触发引脚（输出） */
/* echo_port/echo_pin: 回响引脚（输入） */
static Ultrasonic_ConfigTypeDef ultrasonic_config[2] = {
    {GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1},
    {GPIOA, GPIO_PIN_8, GPIOA, GPIO_PIN_9}
};

/* 超声波等待回波超时（微秒） */
#define ULTRASONIC_ECHO_TIMEOUT_US 30000U
/* 单超声波模式：1=仅使用PA0/PA1通道，0=双通道（整体流程联调建议 0） */
#define ULTRASONIC_SINGLE_CH1_ONLY 0

/**
 * @brief  超声波传感器初始化
 * @note   配置Trig引脚为输出模式，Echo引脚为输入模式
 */
void Ultrasonic_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* 配置Trig引脚为推挽输出 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    /* 车位1的Trig引脚：PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* 车位2的Trig引脚：PA8 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* 配置Echo引脚为输入模式 */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    
    /* 车位1的Echo引脚：PA1 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* 车位2的Echo引脚：PA9 */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 启用DWT周期计数器，用于微秒级计时 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

/**
 * @brief  微秒级延时函数
 * @param  us: 延时微秒数
 */
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { }
}

/**
 * @brief  获取当前微秒计数
 * @retval 当前时间（微秒）
 */
static uint32_t micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

/**
 * @brief  获取超声波传感器距离
 * @param  sensor_num: 传感器编号（0或1）
 * @retval 返回测量距离（单位：厘米）
 */
float Ultrasonic_GetDistance(uint8_t sensor_num)
{
    if (sensor_num > 1) return ULTRASONIC_MAX_DISTANCE;
    
    Ultrasonic_ConfigTypeDef *config = &ultrasonic_config[sensor_num];
    
    /* 发送10us的高电平触发脉冲 */
    HAL_GPIO_WritePin(config->trig_port, config->trig_pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(config->trig_port, config->trig_pin, GPIO_PIN_RESET);
    
    /* 等待Echo上升沿 */
    uint32_t t0 = micros();
    while (HAL_GPIO_ReadPin(config->echo_port, config->echo_pin) == GPIO_PIN_RESET)
    {
        if ((micros() - t0) > ULTRASONIC_ECHO_TIMEOUT_US) return ULTRASONIC_MAX_DISTANCE;
    }

    /* 记录高电平开始时间 */
    uint32_t pulse_start = micros();

    /* 等待Echo下降沿 */
    while (HAL_GPIO_ReadPin(config->echo_port, config->echo_pin) == GPIO_PIN_SET)
    {
        if ((micros() - pulse_start) > ULTRASONIC_ECHO_TIMEOUT_US) return ULTRASONIC_MAX_DISTANCE;
    }

    /* 高电平宽度（微秒） */
    uint32_t pulse_us = micros() - pulse_start;

    /* 计算距离：us * 0.0343 / 2 (cm) */
    float distance = pulse_us * 0.0343f / 2.0f;
    
    /* 距离超出范围则返回最大值 */
    if (distance > ULTRASONIC_MAX_DISTANCE || distance < 0)
    {
        distance = ULTRASONIC_MAX_DISTANCE;
    }
    
    return distance;
}

/**
 * @brief  检测车位是否被占用
 * @param  sensor_num: 传感器编号（0或1）
 * @retval 1表示占用，0表示空闲
 */
uint8_t Ultrasonic_IsOccupied(uint8_t sensor_num)
{
    float distance = Ultrasonic_GetDistance(sensor_num);
    
    /* 距离小于阈值且大于0认为有车停放 */
    if (distance < PARKING_OCCUPIED_DISTANCE && distance > 0)
    {
        return 1;
    }
    
    return 0;
}

/**
 * @brief  更新车位状态
 * @note   每500ms检测一次，更新全局车位状态变量
 */
void Ultrasonic_UpdateParkingStatus(void)
{
    static uint32_t last_update = 0;
    
    /* 每500ms更新一次，避免频繁检测 */
    if (HAL_GetTick() - last_update > 500)
    {
        parking1_status = Ultrasonic_IsOccupied(0);
        #if ULTRASONIC_SINGLE_CH1_ONLY
        /* 单通道联调阶段，第二车位固定为空闲 */
        parking2_status = 0;
        #else
        parking2_status = Ultrasonic_IsOccupied(1);
        #endif
        
        last_update = HAL_GetTick();
    }
}
