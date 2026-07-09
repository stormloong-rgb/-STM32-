/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
#include "esp01s.h"          // ESP-01s WiFi模块驱动
#include "openmv.h"          // OpenMV车牌识别模块驱动
#include "st7735.h"         // ST7735 LCD显示屏驱动
#include "ultrasonic.h"     // 超声波传感器驱动
#include "servo.h"          // 舵机驱动
#include "path_planner.h"   // 路径规划模块

#include <string.h>
#include <stdio.h>
#include <ctype.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief 系统状态枚举
 * @details 定义智能停车场系统的各个工作状态
 */
typedef enum {
    SYSTEM_STATE_INIT,              // 系统初始化状态
    SYSTEM_STATE_IDLE,              // 系统空闲状态，等待车辆进入
    SYSTEM_STATE_LICENSE_DETECTED,  // 检测到车牌号码
    SYSTEM_STATE_GATE_OPENING,      // 道闸正在开启
    SYSTEM_STATE_GATE_OPEN,         // 道闸已开启
    SYSTEM_STATE_PARKING,           // 车辆正在停车
    SYSTEM_STATE_BILLING,           // 计费状态
    SYSTEM_STATE_GATE_CLOSING       // 道闸正在关闭
} System_StateTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * @brief 费率定义
 * @details 每分钟停车费用（单位：元）
 */
#define BILLING_RATE_PER_MINUTE  5

/**
 * @brief 道闸开启延迟时间（毫秒）
 */
#define GATE_OPEN_DELAY_MS       3000

/**
 * @brief 入场后最短停车时长（毫秒）
 * @note 防止相机连续识别同一车牌，刚进场就被判定为出场计费
 */
#define EXIT_RECOGNIZE_MIN_MS    10000
/**
 * @brief 自动出库超时时间（毫秒）
 * @note  在场车辆超过该时长未人工触发出库时，自动进入计费出库流程
 */
#define AUTO_CHECKOUT_TIMEOUT_MS 30000
/* 云端周期上报间隔（ms） */
#define CLOUD_PARKING_REPORT_INTERVAL_MS 10000U

/**
 * @brief 路径最大长度
 */
#define MAX_PATH_LENGTH          50

/**
 * @brief LCD连续刷新测试开关
 * @note  1: 仅测试LCD显示；0: 正常运行智能停车场业务
 */
#define LCD_CONTINUOUS_TEST_MODE 0

/**
 * @brief 舵机单测开关
 * @note  1: 仅测试Servo_Open + 自动关闭；0: 正常运行
 */
#define SERVO_SINGLE_TEST_MODE   0

/**
 * @brief 超声波双通道单测开关
 * @note  1: 仅测试两个超声波占用检测；0: 正常运行
 */
#define ULTRASONIC_DUAL_TEST_MODE 0

/**
 * @brief OpenMV识别结果LCD显示测试开关
 * @note  1: 仅测试OpenMV串口接收并显示车牌；0: 正常运行
 */
#define OPENMV_LCD_TEST_MODE      0

/**
 * @brief 主流程冒烟时忽略超声波开关
 * @note  1: 强制车位空闲，先验证OpenMV触发主链路；0: 使用真实超声波状态
 */
#define SMOKE_IGNORE_ULTRASONIC   0

/**
 * @brief 主流程冒烟时禁用ESP处理开关
 * @note  1: 跳过ESP阻塞流程，聚焦OpenMV->状态机->舵机链路
 */
#define SMOKE_DISABLE_ESP_PROCESS 0

/**
 * @brief 主流程调试信息显示开关
 * @note  1: 在LCD底部显示状态机状态与OpenMV串口接收计数
 */
#define SMOKE_DEBUG_OVERLAY       1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/**
 * @brief 系统当前状态
 */
System_StateTypeDef system_state = SYSTEM_STATE_INIT;

/**
 * @brief 当前识别的车牌号码
 */
char current_license_plate[16] = {0};

/**
 * @brief 当前在场车辆车牌（规范化：大写、无空格、仅字母数字）
 */
char active_license_plate[16] = {0};

/**
 * @brief 停车开始时间（毫秒）
 */
uint32_t parking_start_time = 0;

/**
 * @brief 停车费用（元）
 */
uint32_t billing_fee = 0;

/**
 * @brief 目标停车位编号（1或2）
 */
uint8_t target_parking = 0;

/**
 * @brief 当前是否存在在场车辆记录
 */
uint8_t has_active_parking = 0;

/**
 * @brief 在场车辆对应车位编号
 */
uint8_t active_parking_spot = 0;
/**
 * @brief 云端车牌轮询槽位（0->license_plate1，1->license_plate2）
 */
uint8_t cloud_plate_slot_toggle = 0;
/**
 * @brief 云端停车位状态上报时间戳
 */
uint32_t cloud_last_parking_report_ms = 0;


/**
 * @brief 上一次显示状态（用于判断是否需要刷新显示）
 */
uint8_t last_display_state = 0;

/**
 * @brief ESP-01s WiFi模块配置参数
 * @details 连接到阿里云IoT平台的配置信息
 */
ESP01S_ConfigTypeDef esp01s_config = {
    .ssid = "Ciallo",                                          // WiFi热点名称
    .password = "0d000721",                                    // WiFi密码
    .mqtt_server = "iot-06z00i4rmqe2hzc.mqtt.iothub.aliyuncs.com",  // 阿里云MQTT服务器地址
    .mqtt_port = 1883,                                         // MQTT端口
    /* 与阿里云控制台/串口实测一致：timestamp 与 password_mqtt 必须同批生成 */
    .client_id = "k1uoeMNdaXi.parking|securemode=2,signmethod=hmacsha256,timestamp=1776835489362|",
    .username = "parking&k1uoeMNdaXi",                         // 用户名
    .password_mqtt = "06c905c82d5c2900ed7af65dfa445e119c56c29aa79c5f754ce9ca2773734ac9",  // MQTT密码（与串口实测一致）
    .publish_topic = "/sys/k1uoeMNdaXi/parking/thing/event/property/post",       // 发布主题
    .subscribe_topic = "/sys/k1uoeMNdaXi/parking/thing/service/property/set"      // 订阅主题
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);      // 系统时钟配置
void System_Init(void);              // 系统初始化
void System_Process(void);           // 系统主处理函数
void LCD_ContinuousRefreshTest(void);// LCD连续刷新测试
void Servo_SingleTest(void);         // 舵机单测
void Ultrasonic_DualTest(void);      // 超声波双通道单测
void OpenMV_LcdTest(void);            // OpenMV识别LCD显示测试
void Smoke_DebugOverlay(void);        // 冒烟调试覆盖显示
void ESP01S_LCD_DebugPump(void);      // ESP 阻塞时强制刷新 LCD 调试区
void USART2_RxCpltCallback(void);   // USART2接收完成回调
void USART3_RxCpltCallback(void);   // USART3接收完成回调

/* USER CODE BEGIN PFP */

static void License_Normalize(const char *src, char *out, size_t out_sz);
static uint8_t License_SameVehicle(const char *raw_new, const char *stored_norm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 车牌规范化：去空白、转大写、只保留字母数字（缓解 OpenMV 空格/抖动）
 */
static void License_Normalize(const char *src, char *out, size_t out_sz)
{
    size_t j = 0;

    if (out_sz == 0U) {
        return;
    }

    for (size_t i = 0; src[i] != '\0' && j < out_sz - 1U; i++) {
        unsigned char c = (unsigned char)src[i];
        if (isspace(c) != 0) {
            continue;
        }
        if (isalnum(c) == 0) {
            continue;
        }
        out[j++] = (char)toupper(c);
    }
    out[j] = '\0';
}

/**
 * @brief 判断两次识别是否为同一辆车（精确匹配规范化串，或允许少量字符误差）
 */
static uint8_t License_SameVehicle(const char *raw_new, const char *stored_norm)
{
    char norm_new[16];

    License_Normalize(raw_new, norm_new, sizeof(norm_new));

    if (norm_new[0] == '\0' || stored_norm[0] == '\0') {
        return 0;
    }

    if (strcmp(norm_new, stored_norm) == 0) {
        return 1;
    }

    /* 长度差 1：常见为末尾少识别一位 */
    size_t ln = strlen(norm_new);
    size_t ls = strlen(stored_norm);
    if (ln + 1U == ls && strncmp(stored_norm, norm_new, ln) == 0) {
        return 1;
    }
    if (ls + 1U == ln && strncmp(norm_new, stored_norm, ls) == 0) {
        return 1;
    }

    /* 等长且足够长：允许至多 2 位字符不同（识别抖动） */
    if (ln == ls && ln >= 4U) {
        int diff = 0;
        for (size_t i = 0; i < ln; i++) {
            if (norm_new[i] != stored_norm[i]) {
                diff++;
            }
        }
        if (diff <= 2) {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief UART接收完成回调函数
 * @param huart UART句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // USART2用于ESP-01s WiFi模块通信
    if (huart->Instance == USART2)
    {
        USART2_RxCpltCallback();
    }
    // USART3用于OpenMV车牌识别模块通信
    else if (huart->Instance == USART3)
    {
        USART3_RxCpltCallback();
    }
}

/**
 * @brief USART2接收完成回调函数
 * @details 处理来自ESP-01s WiFi模块的数据
 */
void USART2_RxCpltCallback(void)
{
    ESP01S_UART_RxCpltHandler();
}

/**
 * @brief USART3接收完成回调函数
 * @details 处理来自OpenMV车牌识别模块的数据
 */
void USART3_RxCpltCallback(void)
{
    OpenMV_UART_RxCpltCallback();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();  // 初始化HAL库

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();  // 配置系统时钟

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();        // 初始化GPIO
  MX_SPI2_Init();        // 初始化SPI2（用于LCD）
  MX_TIM3_Init();        // 初始化TIM3（用于舵机PWM）
  MX_USART2_UART_Init(); // 初始化USART2（用于ESP-01s）
  MX_USART3_UART_Init(); // 初始化USART3（用于OpenMV）
  
  /* USER CODE BEGIN 2 */

  System_Init();  // 初始化系统

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    #if LCD_CONTINUOUS_TEST_MODE
    LCD_ContinuousRefreshTest();  // 仅执行LCD连续刷新测试
    #elif SERVO_SINGLE_TEST_MODE
    Servo_SingleTest();           // 仅执行舵机单测
    #elif ULTRASONIC_DUAL_TEST_MODE
    Ultrasonic_DualTest();        // 仅执行超声波双通道单测
    #elif OPENMV_LCD_TEST_MODE
    OpenMV_LcdTest();             // 仅执行OpenMV识别LCD显示测试
    #else
    System_Process();             // 处理系统任务
    #endif
    
    HAL_Delay(10);     // 延时10ms
  }
  /* USER CODE END 3 */
}

/**
 * @brief 系统初始化函数
 * @details 初始化所有外设和模块
 */
void System_Init(void)
{
    // 初始化LCD显示屏
    ST7735_Init();
//    // 填充红色测试
//    ST7735_FillColor(ST7735_COLOR_RED);
//    HAL_Delay(1000);
//    // 填充绿色测试
//    ST7735_FillColor(ST7735_COLOR_GREEN);
//    HAL_Delay(1000);
//    // 填充蓝色测试
//    ST7735_FillColor(ST7735_COLOR_BLUE);
//    HAL_Delay(1000);
//    // 填充黑色
//    ST7735_FillColor(ST7735_COLOR_BLACK);
    // 开启显示
    ST7735_DisplayOn();
    // 显示欢迎界面
    ST7735_DisplayWelcome();
    HAL_Delay(2000);  // 延时2秒
    
    #if SERVO_SINGLE_TEST_MODE
    /* 舵机单测：最小初始化，避免其他外设干扰供电与时序 */
    Servo_Init();
    #else
    // 初始化超声波传感器
    Ultrasonic_Init();

    // 初始化舵机
    Servo_Init();
    
    // 初始化ESP-01s WiFi模块
    ESP01S_Init(&esp01s_config);
    
    // 初始化OpenMV车牌识别模块
    OpenMV_Init();
    
    // 初始化路径规划模块
    PathPlanner_Init();
    #endif
    
    // 设置系统状态为空闲
    system_state = SYSTEM_STATE_IDLE;
    
    // 显示初始停车位状态
    ST7735_DisplayParkingStatus(parking1_status, parking2_status, 
                                 parking1_number, parking2_number);
}

/**
 * @brief 系统主处理函数
 * @details 根据系统状态执行相应的操作（状态机）
 */
void System_Process(void)
{
    #if SMOKE_IGNORE_ULTRASONIC
    /* 冒烟阶段先屏蔽超声波不稳定影响，确保主流程可验证 */
    parking1_status = 0;
    parking2_status = 0;
    #else
    // 更新超声波传感器检测的停车位状态
    Ultrasonic_UpdateParkingStatus();
    #endif

    // 更新路径规划模块的停车位状态
    PathPlanner_SetParkingStatus(parking1_status, parking2_status);

	// 获取舵机状态
    Servo_GetState();
    
    // 状态机处理
    switch (system_state)
    {
        case SYSTEM_STATE_INIT:
            // 初始化状态，跳转到空闲状态
            system_state = SYSTEM_STATE_IDLE;
            break;
            
        case SYSTEM_STATE_IDLE:
            /*
             * 自动出库：在场车辆超时后，直接触发计费出库
             * 使用在场车牌与在场车位，避免依赖再次识别。
             */
            if (has_active_parking != 0U &&
                (HAL_GetTick() - parking_start_time) >= AUTO_CHECKOUT_TIMEOUT_MS)
            {
                strncpy(current_license_plate, active_license_plate, sizeof(current_license_plate) - 1U);
                current_license_plate[sizeof(current_license_plate) - 1U] = '\0';
                target_parking = active_parking_spot;
                system_state = SYSTEM_STATE_BILLING;
                break;
            }

            // 空闲状态：检查是否有车牌识别结果
            if (OpenMV_CheckAvailable())
            {
                // 获取车牌号码
                OpenMV_GetLicensePlate(current_license_plate);
                // 清除识别标志
                OpenMV_Clear();
                
                // 如果识别到车牌
                if (strlen(current_license_plate) > 0)
                {
                    /*
                     * 二次识别到同一车牌：视为离场，直接进入计费
                     * 其他情况：按入场流程处理
                     */
                    if (has_active_parking != 0U &&
                        License_SameVehicle(current_license_plate, active_license_plate) != 0U)
                    {
                        /*
                         * 同牌二次识别：
                         * - 到达最短停车时长后 -> 进入计费（出库）
                         * - 未到时长 -> 忽略本次识别，避免被当成“新入场”重置停车计时
                         */
                        if ((HAL_GetTick() - parking_start_time) >= EXIT_RECOGNIZE_MIN_MS)
                        {
                            target_parking = active_parking_spot;
                            system_state = SYSTEM_STATE_BILLING;
                        }
                        else
                        {
                            system_state = SYSTEM_STATE_IDLE;
                        }
                    }
                    else
                    {
                        const char *plate_slot_key = (cloud_plate_slot_toggle == 0U) ? "license_plate1" : "license_plate2";
                        ESP01S_SendStringProperty(plate_slot_key, current_license_plate);
                        cloud_plate_slot_toggle ^= 1U;
                        system_state = SYSTEM_STATE_LICENSE_DETECTED;
                    }
                }
            }
            
            // 如果显示状态改变，刷新显示
            if (last_display_state != 0)
            {
                ST7735_DisplayParkingStatus(parking1_status, parking2_status,
                                             parking1_number, parking2_number);
                last_display_state = 0;
            }
            break;
            
        case SYSTEM_STATE_LICENSE_DETECTED:
            // 使用Dijkstra算法选择最优停车位
            target_parking = PathPlanner_FindOptimalSpot();
            
            if (target_parking > 0)
            {
                // 生成从入口到停车位的路径
                Point_t start = {0, 0};  // 入口坐标
                Point_t goal;
                
                // 根据目标停车位设置终点坐标
                if (target_parking == 1) {
                    goal.x = 2;
                    goal.y = 2;
                } else {
                    goal.x = 4;
                    goal.y = 4;
                }
                
                // 生成路径
                Point_t path[MAX_PATH_LENGTH];
                uint8_t path_len = PathPlanner_GeneratePath(start, goal, path, MAX_PATH_LENGTH);
                
                // 如果路径生成成功
                if (path_len > 0) {
                    // LCD显示导航信息（传入目标车位坐标）
                    ST7735_DisplayNavigation(goal.x, goal.y, current_license_plate);
                    last_display_state = 1;
                    
                    // 开启道闸
                    Servo_Open();
                    // 记录停车开始时间
                    parking_start_time = HAL_GetTick();
                    // 跳转到道闸开启状态
                    system_state = SYSTEM_STATE_GATE_OPENING;
                } else {
                    // 路径生成失败
                    ST7735_DisplayNavigation(0, 0, current_license_plate);
                    last_display_state = 1;
                    HAL_Delay(3000);
                    system_state = SYSTEM_STATE_IDLE;
                }
            }
            else
            {
                // 没有可用停车位
                ST7735_DisplayNavigation(0, 0, current_license_plate);
                last_display_state = 1;
                HAL_Delay(3000);
                system_state = SYSTEM_STATE_IDLE;
            }
            break;
            
        case SYSTEM_STATE_GATE_OPENING:
            // 道闸开启状态
            if (HAL_GetTick() - parking_start_time > 500)
            {
                // 道闸开启完成，跳转到停车状态
                system_state = SYSTEM_STATE_PARKING;
            }
            break;
            
        case SYSTEM_STATE_PARKING:
            // 停车状态：更新停车位状态
            if (target_parking == 1)
            {
                parking1_status = 1;  // 车位1已被占用
            }
            else if (target_parking == 2)
            {
                parking2_status = 1;  // 车位2已被占用
            }
            
            // 更新路径规划模块的停车位状态
            PathPlanner_SetParkingStatus(parking1_status, parking2_status);

            // 记录在场车辆（规范化后保存，便于二次识别容忍空格/少量误识别）
            License_Normalize(current_license_plate, active_license_plate, sizeof(active_license_plate));
            has_active_parking = 1;
            active_parking_spot = target_parking;

            /* 出库计费仅由“同车牌二次识别”触发，不再由满位条件触发 */
            system_state = SYSTEM_STATE_IDLE;
            break;
            
        case SYSTEM_STATE_BILLING:
            {
                // 计算停车时长（秒）
                uint32_t parking_duration = (HAL_GetTick() - parking_start_time) / 1000;
                // 计算停车费用（元）
                billing_fee = (parking_duration / 60 + 1) * BILLING_RATE_PER_MINUTE;
                /* 出库属性上报：license_plate + fee（非阻塞，不等待模组回包） */
                ESP01S_SendCheckout(current_license_plate, billing_fee);

                /* 出库计费阶段主动开闸，便于车辆离场 */
                Servo_Open();
                
                // LCD显示费用信息
                ST7735_DisplayBilling(current_license_plate, billing_fee);
                last_display_state = 2;
                
                HAL_Delay(5000);  // 显示5秒

                /* 计费展示结束后主动关闸，避免阻塞期间无法依赖状态机自动回收 */
                Servo_Close();
                
                // 释放停车位
                if (target_parking == 1)
                {
                    parking1_status = 0;  // 车位1空闲
                }
                else if (target_parking == 2)
                {
                    parking2_status = 0;  // 车位2空闲
                }
                
                // 更新路径规划模块的停车位状态
                PathPlanner_SetParkingStatus(parking1_status, parking2_status);
                
                // 清除车牌号码和费用
                memset(current_license_plate, 0, sizeof(current_license_plate));
                memset(active_license_plate, 0, sizeof(active_license_plate));
                billing_fee = 0;
                target_parking = 0;
                has_active_parking = 0;
                active_parking_spot = 0;
                
                // 返回空闲状态
                system_state = SYSTEM_STATE_IDLE;
            }
            break;
            
        default:
            // 未知状态，跳转到空闲状态
            system_state = SYSTEM_STATE_IDLE;
            break;
    }

    if ((HAL_GetTick() - cloud_last_parking_report_ms) >= CLOUD_PARKING_REPORT_INTERVAL_MS) {
        cloud_last_parking_report_ms = HAL_GetTick();
        ESP01S_SendParkingStatus(parking1_status, parking2_status);
    }

    #if SMOKE_DISABLE_ESP_PROCESS
    /* 冒烟阶段禁用ESP阻塞流程 */
    #else
    /*
     * ESP AT 流程是阻塞式的，若在状态机前执行会卡住“导航->开闸->停车”主链路。
     * 仅在空闲态推进云连接，避免导航页被长时间停住。
     */
    if (system_state == SYSTEM_STATE_IDLE) {
        ESP01S_Process();
    }
    #endif

    #if SMOKE_DEBUG_OVERLAY
    Smoke_DebugOverlay();
    #endif
}

/**
 * @brief LCD连续刷新测试
 * @details 轮流显示停车状态、导航、计费和纯色背景，用于观察花屏/闪屏
 */
void LCD_ContinuousRefreshTest(void)
{
    static uint8_t page = 0;

    switch (page)
    {
        case 0:
            ST7735_DisplayParkingStatus(0, 1, 1, 2);
            break;

        case 1:
            ST7735_DisplayNavigation(2, 2, "DEMO12");
            break;

        case 2:
            ST7735_DisplayBilling("A12345", 15);
            break;

        case 3:
            ST7735_FillColor(ST7735_COLOR_BLACK);
            ST7735_DrawString(8, 70, "LCD REFRESH TEST", ST7735_COLOR_CYAN, ST7735_COLOR_BLACK);
            break;

        default:
            page = 0;
            return;
    }

    page = (page + 1) % 4;
    HAL_Delay(800);
}

/**
 * @brief 舵机单测
 * @details 每6秒触发一次开闸，依赖Servo_GetState在3秒后自动落杆
 */
void Servo_SingleTest(void)
{
    static uint32_t last_open_tick = 0;
    uint32_t now = HAL_GetTick();

    /* 维持舵机状态机运行（含超时自动关闭） */
    Servo_GetState();

    /* 周期触发开闸，观察抬杆->自动落杆循环 */
    if ((now - last_open_tick) >= 6000U)
    {
        Servo_Open();
        last_open_tick = now;

        ST7735_FillColor(ST7735_COLOR_BLACK);
        ST7735_DrawString(8, 60, "SERVO OPEN", ST7735_COLOR_GREEN, ST7735_COLOR_BLACK);
    }
    else if ((now - last_open_tick) >= 3500U)
    {
        ST7735_FillColor(ST7735_COLOR_BLACK);
        ST7735_DrawString(8, 60, "SERVO CLOSED", ST7735_COLOR_YELLOW, ST7735_COLOR_BLACK);
    }

    HAL_Delay(10);
}

/**
 * @brief 超声波双通道单测
 * @details 定时采样两个通道的距离和占用状态，并在LCD显示
 */
void Ultrasonic_DualTest(void)
{
    static uint32_t last_refresh = 0;
    char line1[24];
    char line2[24];
    float d1;
    float d2;

    if ((HAL_GetTick() - last_refresh) < 300U)
    {
        HAL_Delay(10);
        return;
    }
    last_refresh = HAL_GetTick();

    d1 = Ultrasonic_GetDistance(0);
    d2 = Ultrasonic_GetDistance(1);
    parking1_status = (d1 < PARKING_OCCUPIED_DISTANCE) ? 1 : 0;
    parking2_status = (d2 < PARKING_OCCUPIED_DISTANCE) ? 1 : 0;

    ST7735_FillColor(ST7735_COLOR_BLACK);
    ST7735_DrawString(6, 10, "ULTRASONIC TEST", ST7735_COLOR_CYAN, ST7735_COLOR_BLACK);

    snprintf(line1, sizeof(line1), "P1:%3dcm %s", (int)d1, parking1_status ? "OCC" : "FREE");
    snprintf(line2, sizeof(line2), "P2:%3dcm %s", (int)d2, parking2_status ? "OCC" : "FREE");
    ST7735_DrawString(6, 50, line1, ST7735_COLOR_GREEN, ST7735_COLOR_BLACK);
    ST7735_DrawString(6, 75, line2, ST7735_COLOR_YELLOW, ST7735_COLOR_BLACK);
}

/**
 * @brief OpenMV识别LCD显示测试
 * @details 接收到帧后将车牌号显示在LCD上，便于现场联调
 */
void OpenMV_LcdTest(void)
{
    static uint8_t initialized = 0;
    char plate[16] = {0};
    char dbg1[24];
    char dbg2[24];
    static uint32_t last_dbg_refresh = 0;

    if (!initialized)
    {
        ST7735_FillColor(ST7735_COLOR_BLACK);
        ST7735_DrawString(6, 10, "OPENMV LCD TEST", ST7735_COLOR_CYAN, ST7735_COLOR_BLACK);
        ST7735_DrawString(6, 35, "Waiting frame...", ST7735_COLOR_WHITE, ST7735_COLOR_BLACK);
        initialized = 1;
    }

    if (OpenMV_CheckAvailable())
    {
        OpenMV_GetLicensePlate(plate);

        ST7735_FillArea(0, 55, ST7735_WIDTH, 80, ST7735_COLOR_BLACK);
        ST7735_DrawString(6, 60, "Plate:", ST7735_COLOR_GREEN, ST7735_COLOR_BLACK);
        ST7735_DrawString(6, 85, plate, ST7735_COLOR_YELLOW, ST7735_COLOR_BLACK);

        OpenMV_Clear();
    }

    if ((HAL_GetTick() - last_dbg_refresh) > 300U)
    {
        last_dbg_refresh = HAL_GetTick();
        snprintf(dbg1, sizeof(dbg1), "RX:%lu", (unsigned long)openmv_rx_debug_count);
        snprintf(dbg2, sizeof(dbg2), "Last:0x%02X", openmv_rx_debug_last_byte);
        ST7735_FillArea(0, 130, ST7735_WIDTH, 30, ST7735_COLOR_BLACK);
        ST7735_DrawString(6, 130, dbg1, ST7735_COLOR_WHITE, ST7735_COLOR_BLACK);
        ST7735_DrawString(70, 130, dbg2, ST7735_COLOR_WHITE, ST7735_COLOR_BLACK);
    }
}

#if SMOKE_DEBUG_OVERLAY
/**
 * @brief 立即绘制 LCD 调试条（无节流；供 ESP AT 阻塞等待时调用）
 */
static void Smoke_DebugOverlay_Draw(void)
{
    char line_wifi[20];
    char line_cloud[20];
    char line1[32];
    char line2[32];
    uint16_t col_wifi = ST7735_COLOR_LIGHT_GRAY;
    uint16_t col_cloud = ST7735_COLOR_LIGHT_GRAY;
    uint32_t pwm = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    Servo_StateTypeDef ss = Servo_GetState();

    if (last_display_state == 1U) {
        return;
    }

#if SMOKE_DISABLE_ESP_PROCESS
    snprintf(line_wifi, sizeof(line_wifi), "WiFi: (off)");
    snprintf(line_cloud, sizeof(line_cloud), "Set DIS_ESP=0");
    col_wifi = ST7735_COLOR_GRAY;
    col_cloud = ST7735_COLOR_GRAY;
#else
    switch (esp01s_state)
    {
        case ESP01S_STATE_IDLE:
            snprintf(line_wifi, sizeof(line_wifi), "WiFi: standby");
            snprintf(line_cloud, sizeof(line_cloud), "Cloud: --");
            col_wifi = ST7735_COLOR_LIGHT_GRAY;
            col_cloud = ST7735_COLOR_LIGHT_GRAY;
            break;
        case ESP01S_STATE_WIFI_CONNECTING:
            if (esp01s_wifi_last_fail_step != 0U) {
                snprintf(line_wifi, sizeof(line_wifi), "WiFi: retry..");
                switch (esp01s_wifi_last_fail_step) {
                    case 1U:
                        snprintf(line_cloud, sizeof(line_cloud), "E1: no AT OK");
                        break;
                    case 2U:
                        snprintf(line_cloud, sizeof(line_cloud), "E2: ATE0 fail");
                        break;
                    case 3U:
                        snprintf(line_cloud, sizeof(line_cloud), "E3: CWMODE");
                        break;
                    case 4U:
                        snprintf(line_cloud, sizeof(line_cloud), "E4: CWJAP");
                        break;
                    default:
                        snprintf(line_cloud, sizeof(line_cloud), "E?: WiFi");
                        break;
                }
            } else {
                snprintf(line_wifi, sizeof(line_wifi), "WiFi: linking..");
                snprintf(line_cloud, sizeof(line_cloud), "Cloud: --");
            }
            col_wifi = ST7735_COLOR_YELLOW;
            col_cloud = ST7735_COLOR_GRAY;
            break;
        case ESP01S_STATE_MQTT_CONNECTING:
            snprintf(line_wifi, sizeof(line_wifi), "WiFi: OK");
            if (esp01s_mqtt_last_fail_step != 0U) {
                switch (esp01s_mqtt_last_fail_step) {
                    case 1U:
                        snprintf(line_cloud, sizeof(line_cloud), "M1: USERCFG");
                        break;
                    case 2U:
                        snprintf(line_cloud, sizeof(line_cloud), "M2: MQTTCONN");
                        break;
                    case 3U:
                        snprintf(line_cloud, sizeof(line_cloud), "M3: SUBSCRIBE");
                        break;
                    case 5U:
                        snprintf(line_cloud, sizeof(line_cloud), "M5: LONG CID");
                        break;
                    case 6U:
                        snprintf(line_cloud, sizeof(line_cloud), "M6: USERCFG FB");
                        break;
                    case 7U:
                        snprintf(line_cloud, sizeof(line_cloud), "M7: MQTT USER");
                        break;
                    case 8U:
                        snprintf(line_cloud, sizeof(line_cloud), "M8: MQTT PWD");
                        break;
                    case 9U:
                        snprintf(line_cloud, sizeof(line_cloud), "M9: MQTT CID");
                        break;
                    case 11U:
                        snprintf(line_cloud, sizeof(line_cloud), "M11: NO MQTT");
                        break;
                    default:
                        snprintf(line_cloud, sizeof(line_cloud), "M?: MQTT");
                        break;
                }
            } else {
                snprintf(line_cloud, sizeof(line_cloud), "Cloud: link..");
            }
            col_wifi = ST7735_COLOR_GREEN;
            col_cloud = ST7735_COLOR_YELLOW;
            break;
        case ESP01S_STATE_CONNECTED:
            snprintf(line_wifi, sizeof(line_wifi), "WiFi: OK");
            snprintf(line_cloud, sizeof(line_cloud), "Cloud: OK");
            col_wifi = ST7735_COLOR_GREEN;
            col_cloud = ST7735_COLOR_GREEN;
            break;
        case ESP01S_STATE_ERROR:
            snprintf(line_wifi, sizeof(line_wifi), "WiFi: ERR");
            snprintf(line_cloud, sizeof(line_cloud), "Cloud: ERR");
            col_wifi = ST7735_COLOR_RED;
            col_cloud = ST7735_COLOR_RED;
            break;
        default:
            snprintf(line_wifi, sizeof(line_wifi), "WiFi: ?");
            snprintf(line_cloud, sizeof(line_cloud), "Cloud: ?");
            col_wifi = ST7735_COLOR_ORANGE;
            col_cloud = ST7735_COLOR_ORANGE;
            break;
    }
#endif

    snprintf(line1, sizeof(line1), "S:%d", (int)system_state);
    snprintf(line2, sizeof(line2), "SV:%d PWM:%lu", (int)ss, (unsigned long)pwm);

    /* 按调试刷新节奏局部刷新停车页上方车位区（不重绘整屏、不额外新增显示项） */
    if (last_display_state == 0U) {
        ST7735_RefreshParkingStatusArea(parking1_status, parking2_status);
    }

    ST7735_FillArea(0, 96, ST7735_WIDTH, 64, ST7735_COLOR_BLACK);
    ST7735_DrawString(2, 96, line_wifi, col_wifi, ST7735_COLOR_BLACK);
    ST7735_DrawString(2, 112, line_cloud, col_cloud, ST7735_COLOR_BLACK);
    ST7735_DrawString(2, 128, line1, ST7735_COLOR_WHITE, ST7735_COLOR_BLACK);
    ST7735_DrawString(2, 144, line2, ST7735_COLOR_WHITE, ST7735_COLOR_BLACK);
}

/**
 * @brief 冒烟调试覆盖显示（约 300ms 刷新一次）
 */
void Smoke_DebugOverlay(void)
{
    static uint32_t last_refresh = 0;

    /* 导航全屏指路+车牌时不再画底部调试条，避免盖住 y=96 以下区域 */
    if (last_display_state == 1U) {
        return;
    }
    if ((HAL_GetTick() - last_refresh) < 300U)
    {
        return;
    }
    last_refresh = HAL_GetTick();
    Smoke_DebugOverlay_Draw();
}

/**
 * @brief ESP AT 等待期间强制刷新 LCD（定义于 main，供 esp01s.c 调用）
 */
void ESP01S_LCD_DebugPump(void)
{
    Smoke_DebugOverlay_Draw();
}
#else
void Smoke_DebugOverlay(void) { }
void ESP01S_LCD_DebugPump(void) { }
#endif

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;  // 使用HSI内部高速时钟
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                     // 开启HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;  // 默认校准值
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;               // 不使用PLL
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;       // 系统时钟源为HSI
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;           // AHB时钟不分频
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;            // APB1时钟不分频
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;            // APB2时钟不分频

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  // 用户可以在此添加自己的错误处理代码
  // 禁用所有中断
  __disable_irq();
  // 死循环
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  // 用户可以在这里添加代码来报告错误的文件名和行号
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
