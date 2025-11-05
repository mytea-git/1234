

#ifndef __MOTION_PARAMS_H
#define __MOTION_PARAMS_H

#include "Type.h"

/* 机器鼠运动参数结构 */
typedef struct {
    uint  uiMaxSpeed;                                                       /*  最大速度                    */
    uint  uiMinSpeed;                                                       /*  最小速度                    */
    uint  uiSearchSpeed;                                                    /*  搜索速度                    */
    uchar ucTurnAngle;                                                      /*  转弯角度 (90度的倍数)       */
} MOTION_PARAMS;

/* 外部函数声明 */
void  motionParamsInit(void);
void  motionSetSpeed(uint uiMax, uint uiMin, uint uiSearch);
void  motionGetSpeed(uint *puiMax, uint *puiMin, uint *puiSearch);
void  motionSetTurnAngle(uchar ucAngle);
uchar motionGetTurnAngle(void);
uint  motionGetCurrentMaxSpeed(void);
uint  motionGetCurrentSearchSpeed(void);

#endif
