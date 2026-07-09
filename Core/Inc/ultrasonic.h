#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"
#include <stdint.h>

#define ULTRASONIC_MAX_DISTANCE  400
#define PARKING_OCCUPIED_DISTANCE  50

typedef struct {
    GPIO_TypeDef* trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef* echo_port;
    uint16_t echo_pin;
} Ultrasonic_ConfigTypeDef;

void Ultrasonic_Init(void);
float Ultrasonic_GetDistance(uint8_t sensor_num);
uint8_t Ultrasonic_IsOccupied(uint8_t sensor_num);
void Ultrasonic_UpdateParkingStatus(void);

extern volatile uint8_t parking1_status;
extern volatile uint8_t parking2_status;
extern volatile uint8_t parking1_number;
extern volatile uint8_t parking2_number;

#endif
