/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : path_planner.h
  * @brief          : Path planner for parking lot navigation
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
/* USER CODE END Header */

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <stdint.h>

#define PARKING_LOT_ROWS 11
#define PARKING_LOT_COLS 11

typedef struct {
    int8_t x;
    int8_t y;
} Point_t;

typedef enum {
    PATH_TYPE_ENTRY_TO_SPOT,
    PATH_TYPE_SPOT_TO_EXIT
} PathType_t;

void PathPlanner_Init(void);
void PathPlanner_SetParkingStatus(uint8_t spot1, uint8_t spot2);
uint8_t PathPlanner_FindOptimalSpot(void);
uint8_t PathPlanner_GeneratePath(Point_t start, Point_t goal, Point_t *path, uint8_t max_path_len);
uint8_t PathPlanner_GetPathLength(void);
void PathPlanner_DisplayPath(void);

#endif /* PATH_PLANNER_H */
