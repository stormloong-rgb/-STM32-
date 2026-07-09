#include "openmv.h"

/* Keil ARMCC 无 strnlen：带上限的字符串长度 */
static size_t bounded_strlen(const char *s, size_t max)
{
    size_t i;
    for (i = 0U; i < max && s[i] != '\0'; i++) {
    }
    return i;
}

/* OpenMV状态：空闲/接收中/已接收/错误 */
OpenMV_StateTypeDef openmv_state = OPENMV_STATE_IDLE;

/* 当前识别的车牌信息结构体 */
OpenMV_LicensePlateTypeDef current_plate;
volatile uint32_t openmv_rx_debug_count = 0;
volatile uint8_t openmv_rx_debug_last_byte = 0;

/* 串口接收缓冲区 */
static uint8_t openmv_rx_buffer[OPENMV_RX_BUF_SIZE];
static uint8_t openmv_rx_byte = 0;
/* 收到一帧完整车牌后，暂停接收窗口（毫秒） */
#define OPENMV_RX_COOLDOWN_MS 10000U
static uint8_t openmv_rx_cooldown_active = 0U;
static uint32_t openmv_rx_cooldown_start_ms = 0U;

/**
 * @brief  OpenMV模块初始化
 * @note   初始化状态、缓冲区和UART接收中断
 */
void OpenMV_Init(void)
{
    /* 初始化状态为空闲 */
    openmv_state = OPENMV_STATE_IDLE;
    
    /* 初始化车牌为无效 */
    current_plate.valid = 0;
    memset(current_plate.license_plate, 0, sizeof(current_plate.license_plate));
    memset(openmv_rx_buffer, 0, OPENMV_RX_BUF_SIZE);
    
    /* 启动UART逐字节中断接收 */
    HAL_UART_Receive_IT(&huart3, &openmv_rx_byte, 1);
}

/**
 * @brief  OpenMV串口接收完成回调
 * @note   每收到1字节后喂给协议解析器，并立即续接收
 */
void OpenMV_UART_RxCpltCallback(void)
{
    openmv_rx_debug_last_byte = openmv_rx_byte;
    openmv_rx_debug_count++;
    OpenMV_ProcessRxData(openmv_rx_byte);
    HAL_UART_Receive_IT(&huart3, &openmv_rx_byte, 1);
}

/**
 * @brief  处理OpenMV串口接收数据
 * @param  byte: 接收到的单个字节
 * @note   按帧格式解析车牌数据：0xFF + 数据 + 0xFE
 */
void OpenMV_ProcessRxData(uint8_t byte)
{
    static uint16_t data_index = 0;
    uint32_t now = HAL_GetTick();
    uint8_t cooldown_blocked = 0U;
    /* 冷却期判定：按“整帧”过滤，不在字节级直接丢弃，避免错过帧边界 */
    if (openmv_rx_cooldown_active != 0U) {
        if ((now - openmv_rx_cooldown_start_ms) < OPENMV_RX_COOLDOWN_MS) {
            cooldown_blocked = 1U;
        } else {
            openmv_rx_cooldown_active = 0U;
        }
    }
    
    /* 空闲状态：检测帧头0xFF */
    if (openmv_state == OPENMV_STATE_IDLE)
    {
        if (byte == OPENMV_FRAME_START)
        {
            /* 进入接收状态 */
            openmv_state = _OPENMV_STATE_RECEIVING;
            data_index = 0;
        }
    }
    /* 接收状态：解析数据 */
    else if (openmv_state == _OPENMV_STATE_RECEIVING)
    {
        /* 检测到帧尾0xFE，数据接收完成 */
        if (byte == OPENMV_FRAME_END)
        {
            /* 添加字符串结束符 */
            openmv_rx_buffer[data_index] = '\0';
            
            /* 有效数据检查 */
            if (data_index > 0 && data_index < sizeof(current_plate.license_plate))
            {
                if (cooldown_blocked == 0U) {
                    /* 复制车牌数据到结构体（先清空，避免短字符串残留旧尾巴） */
                    memset(current_plate.license_plate, 0, sizeof(current_plate.license_plate));
                    memcpy(current_plate.license_plate, openmv_rx_buffer, data_index);
                    current_plate.license_plate[data_index] = '\0';
                    current_plate.valid = 1;
                    current_plate.confidence = 80;

                    /* 标记为已接收完成，并开启冷却窗口 */
                    openmv_state = OPENMV_STATE_RECEIVED;
                    openmv_rx_cooldown_active = 1U;
                    openmv_rx_cooldown_start_ms = now;
                } else {
                    /* 冷却期内整帧丢弃，继续保持空闲等待后续帧 */
                    openmv_state = OPENMV_STATE_IDLE;
                }
            } else {
                openmv_state = OPENMV_STATE_IDLE;
            }

            data_index = 0;
        }
        /* 继续接收数据 */
        else if (data_index < OPENMV_RX_BUF_SIZE - 1)
        {
            openmv_rx_buffer[data_index++] = byte;
        }
    }
}

/**
 * @brief  检查是否有新车牌数据
 * @retval 1表示有新数据，0表示无
 */
uint8_t OpenMV_CheckAvailable(void)
{
    if (openmv_state == OPENMV_STATE_RECEIVED)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief  获取当前车牌数据
 * @param  plate: 存放车牌的字符数组指针
 */
void OpenMV_GetLicensePlate(char *plate)
{
    if (plate == NULL) {
        return;
    }

    if (current_plate.valid)
    {
        size_t n = bounded_strlen(current_plate.license_plate, sizeof(current_plate.license_plate));
        if (n > 0U) {
            /* 保证输出以 '\0' 结尾，避免残留字符影响显示/业务逻辑 */
            if (n > 15U) {
                n = 15U;
            }
            memcpy(plate, current_plate.license_plate, n);
            plate[n] = '\0';
        }
        else {
            plate[0] = '\0';
        }
    }
    else {
        plate[0] = '\0';
    }
}

/**
 * @brief  清除车牌数据，恢复空闲状态
 * @note   用于处理完车牌后重置状态
 */
void OpenMV_Clear(void)
{
    openmv_state = OPENMV_STATE_IDLE;
    current_plate.valid = 0;
    memset(current_plate.license_plate, 0, sizeof(current_plate.license_plate));
}
