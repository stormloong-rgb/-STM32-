#include "esp01s.h"

/**
 * @brief 为 AT 双引号字段转义 \\ " ,
 * @note ESP8266 AT 文档：MQTT 参数中含特殊字符需转义
 */
static void ESP01S_EscapeAtQuotedField(const char *src, char *dst, size_t dst_sz)
{
    size_t j = 0;

    if (dst_sz == 0U) {
        return;
    }

    for (size_t i = 0; src[i] != '\0' && j + 2U < dst_sz; i++) {
        if (src[i] == '\\') {
            dst[j++] = '\\';
            dst[j++] = '\\';
        } else if (src[i] == '"') {
            dst[j++] = '\\';
            dst[j++] = '"';
        } else if (src[i] == ',') {
            dst[j++] = '\\';
            dst[j++] = ',';
        } else {
            dst[j++] = (char)src[i];
        }
    }
    dst[j] = '\0';
}

/* 仅用于 MQTTPUB 的 AT 引号字段转义：保留逗号，避免 JSON 内 ',' 被改写为 '\,' */
static void ESP01S_EscapeAtPubField(const char *src, char *dst, size_t dst_sz)
{
    size_t j = 0;
    if (dst_sz == 0U) {
        return;
    }
    for (size_t i = 0; src[i] != '\0' && j + 2U < dst_sz; i++) {
        if (src[i] == '\\') {
            dst[j++] = '\\';
            dst[j++] = '\\';
        } else if (src[i] == '"') {
            dst[j++] = '\\';
            dst[j++] = '"';
        } else {
            dst[j++] = (char)src[i];
        }
    }
    dst[j] = '\0';
}

/**
 * @brief JSON 字符串转义（用于 Alink params.license_plate）
 * @note 仅处理常见转义：\\, ", 控制字符 -> \\u00XX
 */
static void ESP01S_EscapeJsonString(const char *src, char *dst, size_t dst_sz)
{
    size_t j = 0;
    if (dst_sz == 0U) {
        return;
    }
    for (size_t i = 0; src[i] != '\0' && j + 1U < dst_sz; i++) {
        unsigned char c = (unsigned char)src[i];
        if (c == '\\' || c == '"') {
            if (j + 2U >= dst_sz) break;
            dst[j++] = '\\';
            dst[j++] = (char)c;
        } else if (c <= 0x1FU) {
            /* \u00XX */
            if (j + 6U >= dst_sz) break;
            (void)snprintf(&dst[j], dst_sz - j, "\\u%04X", (unsigned int)c);
            j += 6U;
        } else {
            dst[j++] = (char)c;
        }
    }
    dst[j] = '\0';
}

ESP01S_StateTypeDef esp01s_state = ESP01S_STATE_IDLE;
char esp01s_rx_buffer[ESP01S_RX_BUF_SIZE];
uint16_t esp01s_rx_index = 0;
volatile uint8_t esp01s_wifi_last_fail_step = 0;
volatile uint8_t esp01s_mqtt_last_fail_step = 0;

static ESP01S_ConfigTypeDef *esp01s_config = NULL;
static uint8_t esp01s_rx_byte = 0;

void ESP01S_Init(ESP01S_ConfigTypeDef *config)
{
    esp01s_config = config;
    esp01s_state = ESP01S_STATE_IDLE;
    esp01s_rx_index = 0;
    memset(esp01s_rx_buffer, 0, ESP01S_RX_BUF_SIZE);
    
    HAL_UART_Receive_IT(&huart2, &esp01s_rx_byte, 1);
    /* 上电/EN 拉高后 ESP 需一段时间才响应 AT */
    HAL_Delay(1500);
}

void ESP01S_UART_RxCpltHandler(void)
{
    /* 快满时丢弃最旧数据，避免 WiFi/MQTT 日志淹没缓冲区导致永远等不到 OK */
    if (esp01s_rx_index >= ESP01S_RX_BUF_SIZE - 4U) {
        size_t drop = (ESP01S_RX_BUF_SIZE * 3U) / 4U;
        if (esp01s_rx_index > drop) {
            size_t keep = esp01s_rx_index - drop;
            memmove(esp01s_rx_buffer, esp01s_rx_buffer + drop, keep);
            esp01s_rx_index = keep;
            esp01s_rx_buffer[esp01s_rx_index] = '\0';
        } else {
            esp01s_rx_index = 0U;
            esp01s_rx_buffer[0] = '\0';
        }
    }
    if (esp01s_rx_index < ESP01S_RX_BUF_SIZE - 1U) {
        esp01s_rx_buffer[esp01s_rx_index++] = (char)esp01s_rx_byte;
        esp01s_rx_buffer[esp01s_rx_index] = '\0';
    }
    HAL_UART_Receive_IT(&huart2, &esp01s_rx_byte, 1);
}

void ESP01S_Reset(void)
{
    /*
     * ESP-01S 的 RST 引脚未连接到MCU时，无法执行硬件复位。
     * 保留短暂延时用于上电稳定，避免占用其他GPIO。
     */
    HAL_Delay(3000);
}

uint8_t ESP01S_SendCommand(char *cmd, char *response, uint32_t timeout)
{
    uint8_t ret = 0;
    uint32_t start_tick = HAL_GetTick();
    uint32_t pump_tick = start_tick;
    
    __disable_irq();
    memset(esp01s_rx_buffer, 0, ESP01S_RX_BUF_SIZE);
    esp01s_rx_index = 0;
    __enable_irq();
    
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), timeout);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, timeout);
    
    while ((HAL_GetTick() - start_tick) < timeout)
    {
        if (strstr(esp01s_rx_buffer, response) != NULL)
        {
            ret = 1;
            break;
        }
        if (strstr(esp01s_rx_buffer, "FAIL") != NULL)
        {
            ret = 0;
            break;
        }
        if (strstr(esp01s_rx_buffer, "ERROR") != NULL)
        {
            ret = 0;
            break;
        }
        if (strstr(esp01s_rx_buffer, "busy") != NULL)
        {
            ret = 0;
            break;
        }
        /* 主循环被阻塞时仍能刷新 LCD（WiFi/MQTT 等待可达数十秒） */
        if ((HAL_GetTick() - pump_tick) >= 250U) {
            pump_tick = HAL_GetTick();
            ESP01S_LCD_DebugPump();
        }
    }
    
    return ret;
}

/**
 * @brief 发送 AT+MQTTCONN，成功条件：收到 OK 或 +MQTTCONNECTED:0（避免日志冲掉末尾 OK 误判失败）
 */
static uint8_t ESP01S_MqttConnAt(const char *host, uint16_t port, uint8_t reconnect, uint32_t timeout)
{
    char cmd[192];
    uint8_t ret = 0;
    uint32_t start_tick = HAL_GetTick();
    uint32_t pump_tick = start_tick;

    snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%u,%u", host, (unsigned)port, (unsigned)reconnect);

    __disable_irq();
    memset(esp01s_rx_buffer, 0, ESP01S_RX_BUF_SIZE);
    esp01s_rx_index = 0;
    __enable_irq();

    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), timeout);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, timeout);

    while ((HAL_GetTick() - start_tick) < timeout) {
        if (strstr(esp01s_rx_buffer, "+MQTTCONNECTED:0") != NULL) {
            ret = 1U;
            break;
        }
        if (strstr(esp01s_rx_buffer, "OK") != NULL) {
            ret = 1U;
            break;
        }
        if (strstr(esp01s_rx_buffer, "FAIL") != NULL) {
            break;
        }
        if (strstr(esp01s_rx_buffer, "ERROR") != NULL) {
            break;
        }
        if (strstr(esp01s_rx_buffer, "busy") != NULL) {
            break;
        }
        if ((HAL_GetTick() - pump_tick) >= 250U) {
            pump_tick = HAL_GetTick();
            ESP01S_LCD_DebugPump();
        }
    }
    return ret;
}

uint8_t ESP01S_ConnectWiFi(char *ssid, char *password)
{
    char cmd[128];
    uint8_t ret = 0;
    int attempt;

    esp01s_wifi_last_fail_step = 0;

    /* 多次尝试 AT，避免上电瞬间无应答 */
    for (attempt = 0; attempt < 8; attempt++) {
        if (ESP01S_SendCommand("AT", "OK", 2000U)) {
            break;
        }
        HAL_Delay(400);
    }
    if (attempt >= 8) {
        esp01s_wifi_last_fail_step = 1U;
        return 0U;
    }

    HAL_Delay(200);
    if (!ESP01S_SendCommand("ATE0", "OK", 3000U)) {
        esp01s_wifi_last_fail_step = 2U;
        return 0U;
    }

    HAL_Delay(200);
    /* 必须先为 STA 模式，否则部分固件 CWJAP 失败或行为异常 */
    if (!ESP01S_SendCommand("AT+CWMODE=1", "OK", 5000U)) {
        if (!ESP01S_SendCommand("AT+CWMODE_CUR=1", "OK", 5000U)) {
            esp01s_wifi_last_fail_step = 3U;
            return 0U;
        }
    }

    HAL_Delay(800);
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    /* 成功后固件会打印 WIFI GOT IP，最终以 OK 结尾；单次发送避免 busy/重入 */
    ret = ESP01S_SendCommand(cmd, "OK", 45000U);
    if (!ret) {
        esp01s_wifi_last_fail_step = 4U;
        return 0U;
    }

    ESP01S_SendCommand("AT+CIFSR", "OK", 5000U);
    esp01s_wifi_last_fail_step = 0U;
    return 1U;
}

uint8_t ESP01S_ConnectMQTT(void)
{
    char cmd[384];
    char esc[280];
    char esc_client[200];
    uint8_t ret = 0;
    uint8_t client_in_usercfg = 0U;
    const char *host = esp01s_config->mqtt_server;
    uint16_t port = esp01s_config->mqtt_port;

    esp01s_mqtt_last_fail_step = 0U;

    /* 清上次半连接，避免紧连 MQTTCONN 失败；无会话时仍可能 OK */
    (void)ESP01S_SendCommand("AT+MQTTCLEAN=0", "OK", 2500U);
    HAL_Delay(250);

    /*
     * 优先一行写入 client_id + 用户名 + 密码（均做 AT 引号内转义），避免部分固件上
     * AT+MQTTCLIENTID 失败后再走 USERCFG 回退触发 M6。
     * 你方固件：AT+MQTTUSERCFG? 会 ERROR，但 SET 可用；AT+CWLAP 不是联网命令。
     */
    {
        char esc_user[96];
        char esc_pass[200];
        char esc_cli[200];
        ESP01S_EscapeAtQuotedField(esp01s_config->client_id, esc_cli, sizeof(esc_cli));
        ESP01S_EscapeAtQuotedField(esp01s_config->username, esc_user, sizeof(esc_user));
        ESP01S_EscapeAtQuotedField(esp01s_config->password_mqtt, esc_pass, sizeof(esc_pass));
        snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"",
                 esc_cli, esc_user, esc_pass);
        if (strlen(cmd) < 250U) {
            ret = ESP01S_SendCommand(cmd, "OK", 15000U);
            if (ret) {
                client_in_usercfg = 1U;
            }
        }
    }

    if (!ret) {
        char esc_user[96];
        char esc_pass[200];
        ESP01S_EscapeAtQuotedField(esp01s_config->username, esc_user, sizeof(esc_user));
        ESP01S_EscapeAtQuotedField(esp01s_config->password_mqtt, esc_pass, sizeof(esc_pass));
        snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"NULL\",\"%s\",\"%s\",0,0,\"\"",
                 esc_user, esc_pass);
    }

    if (!client_in_usercfg && strlen(cmd) < 250U) {
        ret = ESP01S_SendCommand(cmd, "OK", 15000U);
    } else if (!client_in_usercfg) {
        ret = 0U;
    }

    if (!ret) {
        /* 回退：短占位 + 分条写用户名/密码（总长超 256 或一行式被拒时） */
        {
            int attempt;
            for (attempt = 0; attempt < 4; attempt++) {
                ret = ESP01S_SendCommand("AT+MQTTUSERCFG=0,1,\"NULL\",\"a\",\"a\",0,0,\"\"", "OK", 8000U);
                if (ret) {
                    break;
                }
                HAL_Delay(300U + (uint32_t)attempt * 200U);
            }
        }
        if (!ret) {
            esp01s_mqtt_last_fail_step = 1U;
            return 0U;
        }
        HAL_Delay(150);

        ESP01S_EscapeAtQuotedField(esp01s_config->username, esc, sizeof(esc));
        snprintf(cmd, sizeof(cmd), "AT+MQTTUSERNAME=\"%s\"", esc);
        if (strlen(cmd) >= 250U) {
            esp01s_mqtt_last_fail_step = 7U;
            return 0U;
        }
        if (!ESP01S_SendCommand(cmd, "OK", 15000U)) {
            esp01s_mqtt_last_fail_step = 7U;
            return 0U;
        }
        HAL_Delay(100);

        ESP01S_EscapeAtQuotedField(esp01s_config->password_mqtt, esc, sizeof(esc));
        snprintf(cmd, sizeof(cmd), "AT+MQTTPASSWORD=\"%s\"", esc);
        if (strlen(cmd) >= 250U) {
            esp01s_mqtt_last_fail_step = 8U;
            return 0U;
        }
        if (!ESP01S_SendCommand(cmd, "OK", 15000U)) {
            esp01s_mqtt_last_fail_step = 8U;
            return 0U;
        }
        HAL_Delay(100);
    } else {
        HAL_Delay(150);
    }

    if (!client_in_usercfg) {
        ESP01S_EscapeAtQuotedField(esp01s_config->client_id, esc, sizeof(esc));
        /* 与串口实测一致：必须带 LinkID，例 AT+MQTTCLIENTID=0,"..." */
        snprintf(cmd, sizeof(cmd), "AT+MQTTCLIENTID=0,\"%s\"", esc);
        if (strlen(cmd) >= 250U) {
            esp01s_mqtt_last_fail_step = 9U;
            return 0U;
        }
        if (!ESP01S_SendCommand(cmd, "OK", 15000U)) {
            esp01s_mqtt_last_fail_step = 9U;
            /* 回退：单条 USERCFG，三字段均做引号内转义（旧代码仅转义 client 且 user/pass 未转义易 ERROR/M6） */
            (void)ESP01S_SendCommand("AT+MQTTCLEAN=0", "OK", 3000U);
            HAL_Delay(400);
            esp01s_mqtt_last_fail_step = 5U;
            {
                char esc_u[96];
                char esc_p[200];
                ESP01S_EscapeAtQuotedField(esp01s_config->client_id, esc_client, sizeof(esc_client));
                ESP01S_EscapeAtQuotedField(esp01s_config->username, esc_u, sizeof(esc_u));
                ESP01S_EscapeAtQuotedField(esp01s_config->password_mqtt, esc_p, sizeof(esc_p));
                snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"",
                         esc_client, esc_u, esc_p);
            }
            if (strlen(cmd) >= 250U) {
                esp01s_mqtt_last_fail_step = 6U;
                return 0U;
            }
            ret = ESP01S_SendCommand(cmd, "OK", 15000U);
            if (!ret) {
                esp01s_mqtt_last_fail_step = 6U;
                return 0U;
            }
            esp01s_mqtt_last_fail_step = 0U;
        }
    }

    HAL_Delay(300);

    /*
     * 关闭模块侧自动重连（reconnect=0）：
     * 避免模块内部重连与主状态机重连叠加，触发云端“同设备会话互踢(427)”。
     */
    ret = ESP01S_MqttConnAt(host, port, 0U, 30000U);
    if (!ret) {
        esp01s_mqtt_last_fail_step = 2U;
        return 0U;
    }

    esp01s_mqtt_last_fail_step = 0U;
    return 1U;
}

void ESP01S_Publish(char *topic, char *payload)
{
    char cmd[320];
    char esc_topic[256];
    uint16_t payload_len;
    uint32_t start_ms;

    /*
     * 改回串口联调一致的 RAW 发送方式：
     * 1) AT+MQTTPUBRAW=...
     * 2) 收到 '>' 后发送原始 JSON
     * 不等待最终 +MQTTPUB:OK，避免阻塞主流程。
     */
    ESP01S_EscapeAtPubField(topic, esc_topic, sizeof(esc_topic));
    payload_len = (uint16_t)strlen(payload);

    if (esp01s_state != ESP01S_STATE_CONNECTED) {
        return;
    }

    snprintf(cmd, sizeof(cmd), "AT+MQTTPUBRAW=0,\"%s\",%u,0,0",
             esc_topic, (unsigned int)payload_len);

    __disable_irq();
    memset(esp01s_rx_buffer, 0, ESP01S_RX_BUF_SIZE);
    esp01s_rx_index = 0;
    __enable_irq();

    (void)HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), 120U);
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 20U);

    /*
     * 仅短等待 '>' 提示符（最多 200ms），保证 RAW 数据被模组接收。
     * 不等待 +MQTTPUB:OK，避免长阻塞。
     */
    start_ms = HAL_GetTick();
    while ((HAL_GetTick() - start_ms) < 200U) {
        if (strstr(esp01s_rx_buffer, ">") != NULL) {
            break;
        }
    }

    (void)HAL_UART_Transmit(&huart2, (uint8_t *)payload, payload_len, 180U);
}

uint8_t ESP01S_Subscribe(char *topic)
{
    char cmd[256];
    
    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",0", topic);
    return ESP01S_SendCommand(cmd, "OK", 5000U);
}

void ESP01S_Process(void)
{
    if (esp01s_config == NULL) return;
    static uint8_t mqtt_connecting_seen = 0U;
    static uint32_t mqtt_connecting_start_ms = 0U;
    
    if (esp01s_state == ESP01S_STATE_IDLE)
    {
        mqtt_connecting_seen = 0U;
        esp01s_state = ESP01S_STATE_WIFI_CONNECTING;
    }
    else if (esp01s_state == ESP01S_STATE_WIFI_CONNECTING)
    {
        mqtt_connecting_seen = 0U;
        if (ESP01S_ConnectWiFi(esp01s_config->ssid, esp01s_config->password))
        {
            esp01s_state = ESP01S_STATE_MQTT_CONNECTING;
        }
    }
    else if (esp01s_state == ESP01S_STATE_MQTT_CONNECTING)
    {
        static uint32_t mqtt_next_try_ms = 0U;
        uint32_t now = HAL_GetTick();
        if (mqtt_connecting_seen == 0U) {
            mqtt_connecting_seen = 1U;
            mqtt_connecting_start_ms = now;
        } else if ((now - mqtt_connecting_start_ms) > 20000U) {
            /* 长时间卡在 link.. 时，主动回到 WiFi 重建链路 */
            (void)ESP01S_SendCommand("AT+MQTTCLEAN=0", "OK", 2000U);
            HAL_Delay(200U);
            mqtt_next_try_ms = now + 3000U;
            mqtt_connecting_seen = 0U;
            esp01s_state = ESP01S_STATE_WIFI_CONNECTING;
            return;
        }
        /* 失败后立即再连易 busy/丢应答；退避减轻并给模组恢复时间 */
        if (mqtt_next_try_ms != 0U && now < mqtt_next_try_ms) {
            return;
        }
        if (ESP01S_ConnectMQTT()) {
            mqtt_next_try_ms = 0U;
            mqtt_connecting_seen = 0U;
            esp01s_state = ESP01S_STATE_CONNECTED;
            if (!ESP01S_Subscribe(esp01s_config->subscribe_topic)) {
                esp01s_mqtt_last_fail_step = 3U;
            }
        } else {
            /* 云端出现 427 时，避免短周期重复登录导致持续互踢 */
            mqtt_next_try_ms = now + 60000U;
        }
    }
}

void ESP01S_SendParkingStatus(uint8_t parking1_occupied, uint8_t parking2_occupied)
{
    char payload[256];
    uint32_t msg_id = HAL_GetTick();
    snprintf(payload, sizeof(payload),
             "{\"id\":\"%lu\",\"version\":\"1.0\",\"params\":{\"parking1\":%u,\"parking2\":%u},\"method\":\"thing.event.property.post\"}",
             (unsigned long)msg_id,
             (unsigned int)parking1_occupied,
             (unsigned int)parking2_occupied);
    ESP01S_Publish(esp01s_config->publish_topic, payload);
}

void ESP01S_SendStringProperty(const char *property_key, const char *value)
{
    char escaped_value[96];
    char payload[320];
    uint32_t msg_id = HAL_GetTick();

    if (property_key == NULL || value == NULL || property_key[0] == '\0') {
        return;
    }

    ESP01S_EscapeJsonString(value, escaped_value, sizeof(escaped_value));
    snprintf(payload, sizeof(payload),
             "{\"id\":\"%lu\",\"version\":\"1.0\",\"params\":{\"%s\":\"%s\"},\"method\":\"thing.event.property.post\"}",
             (unsigned long)msg_id, property_key, escaped_value);
    ESP01S_Publish(esp01s_config->publish_topic, payload);
}

void ESP01S_SendIntProperty(const char *property_key, int32_t value)
{
    char payload[256];
    uint32_t msg_id = HAL_GetTick();

    if (property_key == NULL || property_key[0] == '\0') {
        return;
    }

    snprintf(payload, sizeof(payload),
             "{\"id\":\"%lu\",\"version\":\"1.0\",\"params\":{\"%s\":%ld},\"method\":\"thing.event.property.post\"}",
             (unsigned long)msg_id, property_key, (long)value);
    ESP01S_Publish(esp01s_config->publish_topic, payload);
}

void ESP01S_SendCheckout(const char *license_plate, uint32_t fee)
{
    char escaped_plate[96];
    char payload[384];
    uint32_t msg_id = HAL_GetTick();
    const char *plate = (license_plate != NULL) ? license_plate : "";

    ESP01S_EscapeJsonString(plate, escaped_plate, sizeof(escaped_plate));
    snprintf(payload, sizeof(payload),
             "{\"id\":\"%lu\",\"version\":\"1.0\",\"params\":{\"license_plate\":\"%s\",\"fee\":%lu},\"method\":\"thing.event.property.post\"}",
             (unsigned long)msg_id, escaped_plate, (unsigned long)fee);
    ESP01S_Publish(esp01s_config->publish_topic, payload);
}
