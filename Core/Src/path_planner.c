/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : path_planner.c
  * @brief          : Path planner implementation for parking lot navigation
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

#include "path_planner.h"
#include "st7735.h"
#include <string.h>

/* 路径最大长度 */
#define MAX_PATH_LENGTH 50
/* 无穷大（表示不可达） */
#define INF 255

/* 停车场地图：0=空车位, 1=已占用, -1=道路 */
static uint8_t parking_lot[PARKING_LOT_ROWS][PARKING_LOT_COLS];
/* 路径点数组 */
static Point_t path[MAX_PATH_LENGTH];
/* 当前路径长度 */
static uint8_t path_length = 0;
/* 车位1占用状态 */
static uint8_t parking1_status = 0;
/* 车位2占用状态 */
static uint8_t parking2_status = 0;

/**
 * @brief  判断指定位置是否为道路
 * @param  i: 行坐标
 * @param  j: 列坐标
 * @retval 1=道路, 0=停车位
 * @note   奇数行全为道路，列0、5、10为道路
 */
static uint8_t isRoad(int8_t i, int8_t j) {
    if (i < 0 || i >= PARKING_LOT_ROWS || j < 0 || j >= PARKING_LOT_COLS) {
        return 0;
    }
    if (i % 2 == 1) return 1;
    if (j == 0 || j == 5 || j == 10) return 1;
    return 0;
}

/**
 * @brief  计算两点之间的曼哈顿距离（近似）
 * @param  x1, y1: 起点坐标
 * @param  x2, y2: 终点坐标
 * @retval 移动代价（水平2米，垂直3米，对角4米）
 */
static uint8_t getDistance(int8_t x1, int8_t y1, int8_t x2, int8_t y2) {
    if (x1 == x2) return 2;  // 水平移动 2米
    if (y1 == y2) return 3;  // 垂直移动 3米
    return 4;  // 对角线移动，取近似值
}

/**
 * @brief  计算停车难度惩罚值
 * @param  i, j: 车位坐标
 * @retval 惩罚值（左右都有车时+1）
 */
static uint8_t getParkingPenalty(int8_t i, int8_t j) {
    if (i < 0 || i >= PARKING_LOT_ROWS || j < 0 || j >= PARKING_LOT_COLS) {
        return 0;
    }
    if (isRoad(i, j)) return 0;
    if (j > 0 && j < PARKING_LOT_COLS - 1) {
        if (parking_lot[i][j-1] == 1 && parking_lot[i][j+1] == 1) {
            return 1;  // 左右都有车，停车难度+1
        }
    }
    return 0;
}

/**
 * @brief  获取指定位置的路径权重
 * @param  i, j: 位置坐标
 * @retval 权重值（越小越优），INF表示不可通行
 * @note   权重由位置、方向、停车难度等因素决定
 */
static uint8_t getWeight(int8_t i, int8_t j) {
    if (i < 0 || i >= PARKING_LOT_ROWS || j < 0 || j >= PARKING_LOT_COLS) {
        return INF;
    }
    if (parking_lot[i][j] == 1) return INF;
    if (isRoad(i, j)) return 0;
    
    uint8_t weight = 0;
    if (j > 0 && j < 5) weight += 2;
    else if (j >= 5 && j < 10) weight += 2;
    
    if (i > 0) weight += 3;
    
    weight += getParkingPenalty(i, j);
    
    return weight;
}

/**
 * @brief  获取指定点的相邻可达点
 * @param  p: 当前点坐标
 * @param  neighbors: 存储相邻点的数组
 * @param  count: 相邻点数量输出
 * @note   只考虑上下左右四个方向
 */
static void getNeighbors(Point_t p, Point_t neighbors[4], uint8_t *count) {
    *count = 0;
    int8_t directions[4][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
    
    for (uint8_t d = 0; d < 4; d++) {
        int8_t ni = p.x + directions[d][0];
        int8_t nj = p.y + directions[d][1];
        
        if (ni >= 0 && ni < PARKING_LOT_ROWS && nj >= 0 && nj < PARKING_LOT_COLS) {
            if (parking_lot[ni][nj] != 1) {
                neighbors[*count].x = ni;
                neighbors[*count].y = nj;
                (*count)++;
            }
        }
    }
}

/**
 * @brief  初始化停车场地图
 * @note   将道路设为-1，停车位设为0
 */
void PathPlanner_Init(void) {
    for (int8_t i = 0; i < PARKING_LOT_ROWS; i++) {
        for (int8_t j = 0; j < PARKING_LOT_COLS; j++) {
            if (isRoad(i, j)) {
                parking_lot[i][j] = -1;
            } else {
                parking_lot[i][j] = 0;
            }
        }
    }
}

/**
 * @brief  设置车位占用状态
 * @param  spot1: 车位1是否占用
 * @param  spot2: 车位2是否占用
 */
void PathPlanner_SetParkingStatus(uint8_t spot1, uint8_t spot2) {
    parking1_status = spot1;
    parking2_status = spot2;
    
    // 更新停车位状态
    if (spot1) parking_lot[2][2] = 1;
    else parking_lot[2][2] = 0;
    
    if (spot2) parking_lot[4][4] = 1;
    else parking_lot[4][4] = 0;
}

/**
 * @brief  查找最优车位
 * @retval 车位编号（1或2），0表示无空闲车位
 * @note   基于权重算法选择最优车位
 */
uint8_t PathPlanner_FindOptimalSpot(void) {
    const int8_t spot1_x = 2, spot1_y = 2;
    const int8_t spot2_x = 4, spot2_y = 4;
    uint8_t spot1_free = (parking_lot[spot1_x][spot1_y] == 0);
    uint8_t spot2_free = (parking_lot[spot2_x][spot2_y] == 0);

    if (!spot1_free && !spot2_free) {
        return 0;
    }
    if (spot1_free && !spot2_free) {
        return 1;
    }
    if (!spot1_free && spot2_free) {
        return 2;
    }

    /* 两个车位都空闲时，选权重更小的车位 */
    if (getWeight(spot1_x, spot1_y) <= getWeight(spot2_x, spot2_y)) {
        return 1;
    }
    return 2;
}

/**
 * @brief  生成从起点到终点的最优路径
 * @param  start: 起点坐标
 * @param  goal: 终点坐标
 * @param  path: 输出路径数组
 * @param  max_path_len: 路径数组最大长度
 * @retval 路径长度，0表示无路径
 * @note   使用Dijkstra最短路径算法
 */
uint8_t PathPlanner_GeneratePath(Point_t start, Point_t goal, Point_t *path, uint8_t max_path_len) {
    uint8_t dist[PARKING_LOT_ROWS][PARKING_LOT_COLS];
    Point_t prev[PARKING_LOT_ROWS][PARKING_LOT_COLS];
    uint8_t visited[PARKING_LOT_ROWS][PARKING_LOT_COLS];
    
    // 初始化距离和访问数组
    for (int8_t i = 0; i < PARKING_LOT_ROWS; i++) {
        for (int8_t j = 0; j < PARKING_LOT_COLS; j++) {
            dist[i][j] = INF;
            prev[i][j].x = -1;
            prev[i][j].y = -1;
            visited[i][j] = 0;
        }
    }
    
    dist[start.x][start.y] = 0;
    
    // Dijkstra算法主循环
    for (uint8_t iter = 0; iter < PARKING_LOT_ROWS * PARKING_LOT_COLS; iter++) {
        // 找到距离最小的未访问节点
        uint8_t min_dist = INF;
        Point_t u = { -1, -1 };
        
        for (int8_t i = 0; i < PARKING_LOT_ROWS; i++) {
            for (int8_t j = 0; j < PARKING_LOT_COLS; j++) {
                if (!visited[i][j] && dist[i][j] < min_dist) {
                    min_dist = dist[i][j];
                    u.x = i;
                    u.y = j;
                }
            }
        }
        
        if (u.x == -1) break;
        
        visited[u.x][u.y] = 1;
        
        // 如果到达目标，结束
        if (u.x == goal.x && u.y == goal.y) break;
        
        // 处理邻居
        Point_t neighbors[4];
        uint8_t neighbor_count = 0;
        getNeighbors(u, neighbors, &neighbor_count);
        
        for (uint8_t n = 0; n < neighbor_count; n++) {
            Point_t v = neighbors[n];
            if (!visited[v.x][v.y]) {
                uint8_t cost = getDistance(u.x, u.y, v.x, v.y);
                cost += getParkingPenalty(v.x, v.y);
                
                if (dist[v.x][v.y] > dist[u.x][u.y] + cost) {
                    dist[v.x][v.y] = dist[u.x][u.y] + cost;
                    prev[v.x][v.y] = u;
                }
            }
        }
    }
    
    // 重建路径（从终点回溯到起点）
    if (dist[goal.x][goal.y] == INF) {
        return 0;
    }
    
    uint8_t path_idx = 0;
    Point_t current = goal;
    
    while (current.x != -1 && current.y != -1) {
        if (path_idx < max_path_len) {
            path[path_idx] = current;
            path_idx++;
        }
        current = prev[current.x][current.y];
    }
    
    // 反转路径（从起点到终点）
    for (uint8_t i = 0; i < path_idx / 2; i++) {
        Point_t temp = path[i];
        path[i] = path[path_idx - i - 1];
        path[path_idx - i - 1] = temp;
    }
    
    path_length = path_idx;
    return path_idx;
}

/**
 * @brief  获取当前路径长度
 * @retval 路径点数量
 */
uint8_t PathPlanner_GetPathLength(void) {
    return path_length;
}

/**
 * @brief  在LCD上显示导航路径
 * @note   显示目标车位坐标导航，若无路径则显示已满
 */
void PathPlanner_DisplayPath(void) {
    if (path_length == 0) {
        ST7735_DisplayNavigation(0, 0, NULL);
        return;
    }
    
    ST7735_DisplayNavigation(path[path_length-1].x, path[path_length-1].y, NULL);
}
