

#ifndef __Maze_h
#define __Maze_h


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "drive_7289.h"
#include "Type.h"
#include "Micromouse.h"
#include "Mouse_Config.h"


/*********************************************************************************************************
  常量宏定义 -- 定义电脑鼠的四种状态
*********************************************************************************************************/
#define  WAIT           0                                               /*  等待状态                    */
#define  START          1                                               /*  启动状态                    */
#define  MAZESEARCH     2                                               /*  搜寻状态                    */
#define  SPURT          3                                               /*  冲刺状态                    */


/*********************************************************************************************************
  申明需要使用的外部函数
*********************************************************************************************************/
extern void  mouseInit(void);                                           /*  底层驱动程序初始化          */
extern void  mouseGoahead(char  cNBlock);                               /*  前进N格                     */
extern void  mazeSearch(void);                                          /*  迷宫搜索                    */
extern void  mouseTurnleft(void);                                       /*  向左转90度                  */
extern void  mouseTurnright(void);                                      /*  向右转90度                  */
extern void  mouseTurnback(void);                                       /*  向后转                      */
extern uchar keyCheck(void);                                            /*  检测按键                    */
extern void  sensorDebug(void);                                         /*  传感器调试                  */
extern void  voltageDetect(void);                                       /*  电压监测                    */

/*********************************************************************************************************
  申明需要使用的外部变量
*********************************************************************************************************/
extern MAZECOOR GmcMouse;                                               /*  GmcMouse.x :电脑鼠横坐标    */
                                                                        /*  GmcMouse.y :电脑鼠纵坐标    */
                                                                        
extern uchar    GucMouseDir;                                            /*  电脑鼠的前进方向            */
extern uchar    GucMapBlock[MAZETYPE][MAZETYPE];                        /*  GucMapBlock[x][y]           */
                                                                        /*  x,横坐标;y,纵坐标;          */
                                                                        /*  bit3~bit0分别代表左下右上   */
                                                                        /*  0:该方向无路，1:该方向有路  */

static void  mapStepEdit(char  cX, char  cY);
static void  mouseSpurt(void);
static void  objectGoTo(char  cXdst, char  cYdst);
static uchar mazeBlockDataGet(uchar  ucDirTemp);
static void  rightMethod(void);
static void  leftMethod(void);
static void  frontRightMethod(void);
static void  frontLeftMethod(void);
static void  centralMethod(void);
static void spurTrackChoice(void);


#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
