

#ifndef __ASTAR_CONFIG_H
#define __ASTAR_CONFIG_H

#include "Type.h"
#include "Micromouse.h"
#include "Mouse_Config.h"

/* 路径节点结构 */
typedef struct {
    char  cX;                                                               /*  X坐标                       */
    char  cY;                                                               /*  Y坐标                       */
    uchar ucDir;                                                            /*  到达该点的方向              */
} PATH_NODE;

/* 路径结构 */
typedef struct {
    PATH_NODE nodes[MAZETYPE * MAZETYPE];                                   /*  路径节点数组                */
    uchar     ucLength;                                                     /*  路径长度                    */
} ASTAR_PATH;

/* A* 算法参数结构 */
typedef struct {
    float fHeuristicWeight;                                                 /*  启发函数权重 (0.8-2.0)      */
    uchar ucMoveCost;                                                       /*  直线移动代价                */
    uchar ucBaseTurnCost;                                                   /*  基础转弯代价                */
    uchar ucSpeedFactor;                                                    /*  速度影响因子 (0-100)        */
    uchar ucTurnPenalty;                                                    /*  转弯惩罚权重                */
} ASTAR_PARAMS;

#endif
