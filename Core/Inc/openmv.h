#ifndef __OPENMV_H
#define __OPENMV_H

#include "main.h"
#include "usart.h"
#include <string.h>

#define OPENMV_RX_BUF_SIZE  64
#define OPENMV_FRAME_START  0xFF
#define OPENMV_FRAME_END    0xFE

typedef enum {
    OPENMV_STATE_IDLE,
   _OPENMV_STATE_RECEIVING,
    OPENMV_STATE_RECEIVED,
    OPENMV_STATE_ERROR
} OpenMV_StateTypeDef;

typedef struct {
    char license_plate[16];
    uint8_t valid;
    uint8_t confidence;
} OpenMV_LicensePlateTypeDef;

extern OpenMV_StateTypeDef openmv_state;
extern OpenMV_LicensePlateTypeDef current_plate;
extern volatile uint32_t openmv_rx_debug_count;
extern volatile uint8_t openmv_rx_debug_last_byte;

void OpenMV_Init(void);
void OpenMV_UART_RxCpltCallback(void);
void OpenMV_ProcessRxData(uint8_t byte);
uint8_t OpenMV_CheckAvailable(void);
void OpenMV_GetLicensePlate(char *plate);
void OpenMV_Clear(void);

#endif
