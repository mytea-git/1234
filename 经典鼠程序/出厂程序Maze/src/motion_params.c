
/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "motion_params.h"
#include "Mouse_Config.h"


/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/
static MOTION_PARAMS GmotionParams;


/*********************************************************************************************************
** Function name:       motionParamsInit
** Descriptions:        初始化运动参数
*********************************************************************************************************/
void motionParamsInit(void)
{
    GmotionParams.uiMaxSpeed = MAXSPEED;
    GmotionParams.uiMinSpeed = 10;
    GmotionParams.uiSearchSpeed = SEARCHSPEED;
    GmotionParams.ucTurnAngle = 90;
}

/*********************************************************************************************************
** Function name:       motionSetSpeed
** Descriptions:        设置速度参数
*********************************************************************************************************/
void motionSetSpeed(uint uiMax, uint uiMin, uint uiSearch)
{
    if (uiMax > 0) GmotionParams.uiMaxSpeed = uiMax;
    if (uiMin > 0 && uiMin < GmotionParams.uiMaxSpeed) GmotionParams.uiMinSpeed = uiMin;
    if (uiSearch > 0 && uiSearch <= GmotionParams.uiMaxSpeed) GmotionParams.uiSearchSpeed = uiSearch;
}

/*********************************************************************************************************
** Function name:       motionGetSpeed
** Descriptions:        获取速度参数
*********************************************************************************************************/
void motionGetSpeed(uint *puiMax, uint *puiMin, uint *puiSearch)
{
    *puiMax = GmotionParams.uiMaxSpeed;
    *puiMin = GmotionParams.uiMinSpeed;
    *puiSearch = GmotionParams.uiSearchSpeed;
}

/*********************************************************************************************************
** Function name:       motionSetTurnAngle
** Descriptions:        设置转弯角度
*********************************************************************************************************/
void motionSetTurnAngle(uchar ucAngle)
{
    if (ucAngle % 45 == 0 && ucAngle <= 180) {
        GmotionParams.ucTurnAngle = ucAngle;
    }
}

/*********************************************************************************************************
** Function name:       motionGetTurnAngle
** Descriptions:        获取转弯角度
*********************************************************************************************************/
uchar motionGetTurnAngle(void)
{
    return GmotionParams.ucTurnAngle;
}

/*********************************************************************************************************
** Function name:       motionGetCurrentMaxSpeed
** Descriptions:        获取当前最大速度
*********************************************************************************************************/
uint motionGetCurrentMaxSpeed(void)
{
    return GmotionParams.uiMaxSpeed;
}

/*********************************************************************************************************
** Function name:       motionGetCurrentSearchSpeed
** Descriptions:        获取当前搜索速度
*********************************************************************************************************/
uint motionGetCurrentSearchSpeed(void)
{
    return GmotionParams.uiSearchSpeed;
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
