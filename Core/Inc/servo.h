#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

#define SERVO_ANGLE_0    500
#define SERVO_ANGLE_90   1500
#define SERVO_ANGLE_180  2500

typedef enum {
    SERVO_STATE_CLOSED,
    SERVO_STATE_OPENING,
    SERVO_STATE_OPEN,
    SERVO_STATE_CLOSING
} Servo_StateTypeDef;

void Servo_Init(void);
void Servo_SetAngle(uint16_t angle);
void Servo_Open(void);
void Servo_Close(void);
void Servo_OpenDelay(uint32_t delay_ms);
Servo_StateTypeDef Servo_GetState(void);

#endif
