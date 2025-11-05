

#ifndef __Maze_h
#define __Maze_h


/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "drive_7289.h"
#include "Type.h"
#include "Micromouse.h"
#include "Mouse_Config.h"


/*********************************************************************************************************
  �����궨�� -- ��������������״̬
*********************************************************************************************************/
#define  WAIT           0                                               /*  �ȴ�״̬                    */
#define  START          1                                               /*  ����״̬                    */
#define  MAZESEARCH     2                                               /*  ��Ѱ״̬                    */
#define  SPURT          3                                               /*  ���״̬                    */


/*********************************************************************************************************
  ������Ҫʹ�õ��ⲿ����
*********************************************************************************************************/
extern void  mouseInit(void);                                           /*  �ײ����������ʼ��          */
extern void  mouseGoahead(char  cNBlock);                               /*  ǰ��N��                     */
extern void  mazeSearch(void);                                          /*  �Թ�����                    */
extern void  mouseTurnleft(void);                                       /*  ����ת90��                  */
extern void  mouseTurnright(void);                                      /*  ����ת90��                  */
extern void  mouseTurnback(void);                                       /*  ���ת                      */
extern uchar keyCheck(void);                                            /*  ��ⰴ��                    */
extern void  sensorDebug(void);                                         /*  ����������                  */
extern void  voltageDetect(void);                                       /*  ��ѹ���                    */

/*********************************************************************************************************
  ������Ҫʹ�õ��ⲿ����
*********************************************************************************************************/
extern MAZECOOR GmcMouse;                                               /*  GmcMouse.x :�����������    */
                                                                        /*  GmcMouse.y :������������    */
                                                                        
extern uchar    GucMouseDir;                                            /*  �������ǰ������            */
extern uchar    GucMapBlock[MAZETYPE][MAZETYPE];                        /*  GucMapBlock[x][y]           */
                                                                        /*  x,������;y,������;          */
                                                                        /*  bit3~bit0�ֱ������������   */
                                                                        /*  0:�÷�����·��1:�÷�����·  */

static void  mapStepEdit(char  cX, char  cY);
static void  mouseSpurt(void);
static void  objectGoTo(char  cXdst, char  cYdst);
static void  objectGoToAstar(char  cXdst, char  cYdst);
static void  mouseSpurtAstar(void);
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
