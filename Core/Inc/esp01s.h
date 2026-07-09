#ifndef __ESP01S_H
#define __ESP01S_H

#include "main.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#define ESP01S_RX_BUF_SIZE  512
#define ESP01S_TIMEOUT      3000

typedef enum {
    ESP01S_STATE_IDLE,
    ESP01S_STATE_WIFI_CONNECTING,
    ESP01S_STATE_MQTT_CONNECTING,
    ESP01S_STATE_CONNECTED,
    ESP01S_STATE_ERROR
} ESP01S_StateTypeDef;

typedef struct {
    char ssid[32];
    char password[64];
    char mqtt_server[64];
    uint16_t mqtt_port;
    char client_id[144];
    char username[64];
    /** 阿里云密码常为 64 位 hex，需 ≥65 字节含 '\0'；原 [64] 会截断/无终止符导致 M2 */
    char password_mqtt[80];
    char publish_topic[128];
    char subscribe_topic[128];
} ESP01S_ConfigTypeDef;

extern ESP01S_StateTypeDef esp01s_state;
extern char esp01s_rx_buffer[ESP01S_RX_BUF_SIZE];
extern uint16_t esp01s_rx_index;
/** WiFi 联调：0=无 1=AT无应答 2=关回显失败 3=设STA失败 4=CWJAP失败 */
extern volatile uint8_t esp01s_wifi_last_fail_step;
/** MQTT 联调：1=基础USERCFG 7=USERNAME 8=PASSWORD 9=CLIENTID 2=CONN 3=SUB 5/6=旧回退 11=无MQTT AT */
extern volatile uint8_t esp01s_mqtt_last_fail_step;

void ESP01S_Init(ESP01S_ConfigTypeDef *config);
/** 在 main.c 实现：ESP AT 阻塞等待时刷新 LCD 调试区 */
void ESP01S_LCD_DebugPump(void);
void ESP01S_UART_RxCpltHandler(void);
void ESP01S_Reset(void);
uint8_t ESP01S_SendCommand(char *cmd, char *response, uint32_t timeout);
uint8_t ESP01S_ConnectWiFi(char *ssid, char *password);
uint8_t ESP01S_ConnectMQTT(void);
void ESP01S_Publish(char *topic, char *payload);
uint8_t ESP01S_Subscribe(char *topic);
void ESP01S_Process(void);
/**
 * @brief 上报停车位状态（parking1/parking2）
 */
void ESP01S_SendParkingStatus(uint8_t parking1_occupied, uint8_t parking2_occupied);
/**
 * @brief 上报单个字符串属性（如 license_plate1/license_plate2）
 */
void ESP01S_SendStringProperty(const char *property_key, const char *value);
/**
 * @brief 上报单个整型属性（如 fee）
 */
void ESP01S_SendIntProperty(const char *property_key, int32_t value);
/**
 * @brief 出库上报（license_plate + fee）
 */
void ESP01S_SendCheckout(const char *license_plate, uint32_t fee);

#endif
