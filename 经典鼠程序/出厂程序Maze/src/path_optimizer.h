

#ifndef __PATH_OPTIMIZER_H
#define __PATH_OPTIMIZER_H

#include "astar_config.h"

/* 自适应模式枚举 */
typedef enum {
    ADAPTIVE_MODE_SPEED = 0,                                                /*  速度优先模式                */
    ADAPTIVE_MODE_SMOOTH = 1,                                               /*  平滑优先模式                */
    ADAPTIVE_MODE_BALANCED = 2                                              /*  平衡模式                    */
} ADAPTIVE_MODE;

/* 迷宫复杂度结构 */
typedef struct {
    uchar ucBranchCount;                                                    /*  分支点数量                  */
    uchar ucDeadEndCount;                                                   /*  死胡同数量                  */
    uchar ucComplexity;                                                     /*  复杂度评分 (0-100)          */
} MAZE_COMPLEXITY;

/* 外部函数声明 */
void  pathOptimizerInit(void);
void  pathSmooth(ASTAR_PATH *path);
uchar pathGetTurnCount(ASTAR_PATH *path);
void  adaptiveSetMode(ADAPTIVE_MODE mode);
void  adaptiveAnalyzeMaze(MAZE_COMPLEXITY *complexity);
void  adaptiveAdjustParams(MAZE_COMPLEXITY *complexity, ASTAR_PARAMS *params);

#endif
