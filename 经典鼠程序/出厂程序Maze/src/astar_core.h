

#ifndef __ASTAR_CORE_H
#define __ASTAR_CORE_H

#include "astar_config.h"

/* 外部函数声明 */
void  astarInit(void);
uchar astarFindPath(char cXStart, char cYStart, char cXGoal, char cYGoal,
                    uchar ucStartDir, uint uiCurrentSpeed, ASTAR_PATH *path);
void  astarSetParams(float fHWeight, uchar ucMCost, uchar ucTCost,
                     uchar ucSpeedFactor, uchar ucTurnPenalty);
void  astarGetParams(ASTAR_PARAMS *params);

#endif
