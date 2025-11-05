
/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "Maze.h"
#include "astar_core.h"
#include "path_optimizer.h"
#include "motion_params.h"


/*********************************************************************************************************
  �㷨ѡ������
*********************************************************************************************************/
#define USE_ASTAR_ALGORITHM  1                                              /*  1=ʹ��A*, 0=ʹ�ú�ˮ�㷨    */


/*********************************************************************************************************
  ȫ�ֱ�������
*********************************************************************************************************/
static uchar    GucXStart                           = 0;                /*  ��������                  */
static uchar    GucYStart                           = 0;                /*  ���������                  */

static uchar    GucXGoal0                           = XDST0;            /*  �յ�X���꣬������ֵ         */
static uchar    GucXGoal1                           = XDST1;
static uchar    GucYGoal0                           = YDST0;            /*  �յ�Y���꣬������ֵ         */
static uchar    GucYGoal1                           = YDST1;

static uchar    GucMouseTask                        = WAIT;             /*  ״̬������ʼ״̬Ϊ�ȴ�      */

#if !USE_ASTAR_ALGORITHM
static uchar    GucMapStep[MAZETYPE][MAZETYPE]      = {0xff};           /*  ���������ĵȸ�ֵ          */

static MAZECOOR GmcStack[MAZETYPE * MAZETYPE]       = {0};              /*  ��mapStepEdit()������ջʹ�� */
#endif
static MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE]    = {0};              /*  Main()���ݴ�δ�߹�֧·����  */
/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        ��ʱ����
** input parameters:    uiD :��ʱ������ֵԽ����ʱԽ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void delay (uint uiD)
{
    for (; uiD; uiD--);
}

#if !USE_ASTAR_ALGORITHM
/*********************************************************************************************************
** Function name:       mapStepEdit
** Descriptions:        ������Ŀ���Ϊ���ĵȸ�ͼ
** input parameters:    uiX:    Ŀ�ĵغ�����
**                      uiY:    Ŀ�ĵ�������
** output parameters:   GucMapStep[][]:  �������ϵĵȸ�ֵ
** Returned value:      ��
*********************************************************************************************************/
void mapStepEdit (char  cX, char  cY)
{
    uchar n         = 0;                                                /*  GmcStack[]�±�              */
    uchar ucStep    = 1;                                                /*  �ȸ�ֵ                      */
    uchar ucStat    = 0;                                                /*  ͳ�ƿ�ǰ���ķ�����          */
    uchar i,j;
    
    GmcStack[n].cX  = cX;                                               /*  ���Xֵ��ջ                 */
    GmcStack[n].cY  = cY;                                               /*  ���Yֵ��ջ                 */
    n++;
    /*
     *  ��ʼ��������ȸ�ֵ
     */
    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            GucMapStep[i][j] = 0xff;
        }
    }
    /*
     *  �����ȸ�ͼ��ֱ����ջ���������ݴ������
     */
    while (n) {
        GucMapStep[cX][cY] = ucStep++;                                  /*  ����ȸ�ֵ                  */

        /*
         *  �Ե�ǰ��������ǰ���ķ���ͳ��
         */
        ucStat = 0;
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  ǰ����·                    */
            (GucMapStep[cX][cY + 1] > (ucStep))) {                      /*  ǰ���ȸ�ֵ���ڼƻ��趨ֵ    */
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep[cX + 1][cY] > (ucStep))) {                      /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&
            (GucMapStep[cX][cY - 1] > (ucStep))) {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&
            (GucMapStep[cX - 1][cY] > (ucStep))) {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        /*
         *  û�п�ǰ���ķ�������ת���������ķ�֧��
         *  ������ѡһ��ǰ������ǰ��
         */
        if (ucStat == 0) {
            n--;
            cX = GmcStack[n].cX;
            cY = GmcStack[n].cY;
            ucStep = GucMapStep[cX][cY];
        } else {
            if (ucStat > 1) {                                           /*  �ж����ǰ�����򣬱�������  */
                GmcStack[n].cX = cX;                                    /*  ������Xֵ��ջ               */
                GmcStack[n].cY = cY;                                    /*  ������Yֵ��ջ               */
                n++;
            }
            /*
             *  ����ѡ��һ����ǰ���ķ���ǰ��
             */
            if ((GucMapBlock[cX][cY] & 0x01) &&                         /*  �Ϸ���·                    */
                (GucMapStep[cX][cY + 1] > (ucStep))) {                  /*  �Ϸ��ȸ�ֵ���ڼƻ��趨ֵ    */
                cY++;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x02) &&                         /*  �ҷ���·                    */
                (GucMapStep[cX + 1][cY] > (ucStep))) {                  /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
                cX++;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x04) &&                         /*  �·���·                    */
                (GucMapStep[cX][cY - 1] > (ucStep))) {                  /*  �·��ȸ�ֵ���ڼƻ��趨ֵ    */
                cY--;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x08) &&                         /*  ����·                    */
                (GucMapStep[cX - 1][cY] > (ucStep))) {                  /*  �󷽵ȸ�ֵ���ڼƻ��趨ֵ    */
                cX--;                                                   /*  �޸�����                    */
                continue;
            }
        }
    }
}
#endif

#if !USE_ASTAR_ALGORITHM
/*********************************************************************************************************
** Function name:       mouseSpurt
** Descriptions:        ���������������·�������յ�
** input parameters:    ��
 ** output parameters:  ��
** Returned value:      ��
*********************************************************************************************************/
void mouseSpurt (void)
{
    uchar ucTemp = 0xff;
    char cXdst = 0,cYdst = 0;
    /*
     *  ���յ���ĸ�����ֱ������ȸ�ͼ
     *  ȡ����������һ������ΪĿ���
     */
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    objectGoTo(cXdst,cYdst);                                            /*  ���е�ָ��Ŀ���            */
}
#endif

#if USE_ASTAR_ALGORITHM
/*********************************************************************************************************
** Function name:       mouseSpurtAstar
** Descriptions:        使用A*算法选择最优路径并冲刺到终点
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseSpurtAstar(void)
{
    ASTAR_PATH path;
    uchar ucMinLen = 0xFF;
    char cXdst = 0, cYdst = 0;
    uint uiSpeed;
    char goals[4][2] = {
        {GucXGoal0, GucYGoal0},
        {GucXGoal0, GucYGoal1},
        {GucXGoal1, GucYGoal0},
        {GucXGoal1, GucYGoal1}
    };
    uchar i;

    uiSpeed = motionGetCurrentMaxSpeed();

    for (i = 0; i < 4; i++) {
        if (astarFindPath(GucXStart, GucYStart, goals[i][0], goals[i][1],
                         GucMouseDir, uiSpeed, &path)) {
            if (path.ucLength < ucMinLen) {
                ucMinLen = path.ucLength;
                cXdst = goals[i][0];
                cYdst = goals[i][1];
            }
        }
    }

    objectGoToAstar(cXdst, cYdst);
}
#endif

#if !USE_ASTAR_ALGORITHM
/*********************************************************************************************************
** Function name:       objectGoTo
** Descriptions:        ʹ�������˶���ָ������
** input parameters:    cXdst: Ŀ�ĵصĺ�����
**                      cYdst: Ŀ�ĵص�������
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void objectGoTo (char  cXdst, char  cYdst)
{
    uchar ucStep = 1;
    char  cNBlock = 0, cDirTemp;
    char cX,cY;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdit(cXdst,cYdst);                                           /*  �����ȸ�ͼ                  */
    /*
     *  ���ݵȸ�ֵ��Ŀ����˶���ֱ���ﵽĿ�ĵ�
     */
    while ((cX != cXdst) || (cY != cYdst)) {
        ucStep = GucMapStep[cX][cY];
        /*
         *  ��ѡһ���ȸ�ֵ�ȵ�ǰ�����ȸ�ֵС�ķ���ǰ��
         */
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  �Ϸ���·                    */
            (GucMapStep[cX][cY + 1] < ucStep)) {                        /*  �Ϸ��ȸ�ֵ��С              */
            cDirTemp = UP;                                              /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep[cX + 1][cY] < ucStep)) {                        /*  �ҷ��ȸ�ֵ��С              */
            cDirTemp = RIGHT;                                           /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  �·���·                    */
            (GucMapStep[cX][cY - 1] < ucStep)) {                        /*  �·��ȸ�ֵ��С              */
            cDirTemp = DOWN;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY--;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  ����·                    */
            (GucMapStep[cX - 1][cY] < ucStep)) {                        /*  �󷽵ȸ�ֵ��С              */
            cDirTemp = LEFT;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX--;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;                      /*  ���㷽��ƫ����              */

        if (cNBlock) {
            mouseGoahead(cNBlock);                                      /*  ǰ��cNBlock��               */
        }
        cNBlock = 0;                                                    /*  ��������                    */

        /*
         *  ���Ƶ�����ת��
         */
        switch (cDirTemp) {

        case 1:
            mouseTurnright();
            break;

        case 2:
            mouseTurnback();
            break;

        case 3:
            mouseTurnleft();
            break;

        default:
            break;
        }
    }
    /*
     *  �ж������Ƿ���ɣ��������ǰ��
     */
    if (cNBlock) {
        mouseGoahead(cNBlock);
    }
}
#endif

#if USE_ASTAR_ALGORITHM
/*********************************************************************************************************
** Function name:       objectGoToAstar
** Descriptions:        使用A*算法移动到指定坐标
** input parameters:    cXdst: 目的地的横坐标
**                      cYdst: 目的地的纵坐标
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void objectGoToAstar(char cXdst, char cYdst)
{
    ASTAR_PATH path;
    uint uiSpeed;
    char cNBlock = 0;
    uchar i;

    uiSpeed = motionGetCurrentSearchSpeed();

    if (!astarFindPath(GmcMouse.cX, GmcMouse.cY, cXdst, cYdst,
                       GucMouseDir, uiSpeed, &path)) {
        return;
    }

    pathSmooth(&path);

    for (i = 0; i < path.ucLength; i++) {
        if (path.nodes[i].ucDir == GucMouseDir) {
            cNBlock++;
        } else {
            if (cNBlock > 0) {
                mouseGoahead(cNBlock);
                cNBlock = 0;
            }

            char cDirDiff = (path.nodes[i].ucDir + 4 - GucMouseDir) % 4;
            switch (cDirDiff) {
            case 1:
                mouseTurnright();
                break;
            case 2:
                mouseTurnback();
                break;
            case 3:
                mouseTurnleft();
                break;
            default:
                break;
            }
            cNBlock = 1;
        }

        GmcMouse.cX = path.nodes[i].cX;
        GmcMouse.cY = path.nodes[i].cY;
    }

    if (cNBlock > 0) {
        mouseGoahead(cNBlock);
    }
}
#endif

/*********************************************************************************************************
** Function name:       mazeBlockDataGet
** Descriptions:        ���ݵ��������Է���ȡ���÷������Թ����ǽ������
** input parameters:    ucDir: ���������Է���
** output parameters:   ��
** Returned value:      GucMapBlock[cX][cY] : ǽ������
*********************************************************************************************************/
uchar mazeBlockDataGet (uchar  ucDirTemp)
{
    char cX = 0,cY = 0;
    
    /*
     *  �ѵ��������Է���ת��Ϊ���Է���
     */
    switch (ucDirTemp) {

    case MOUSEFRONT:
        ucDirTemp = GucMouseDir;
        break;

    case MOUSELEFT:
        ucDirTemp = (GucMouseDir + 3) % 4;
        break;

    case MOUSERIGHT:
        ucDirTemp = (GucMouseDir + 1) % 4;
        break;

    default:
        break;
    }
    
    /*
     *  ���ݾ��Է������÷��������ڸ������
     */
    switch (ucDirTemp) {

    case 0:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY + 1;
        break;
        
    case 1:
        cX = GmcMouse.cX + 1;
        cY = GmcMouse.cY;
        break;
        
    case 2:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY - 1;
        break;
        
    case 3:
        cX = GmcMouse.cX - 1;
        cY = GmcMouse.cY;
        break;
        
    default:
        break;
    }
    
    return(GucMapBlock[cX][cY]);                                        /*  �����Թ����ϵ�����          */
}
/*********************************************************************************************************
** Function name:       rightMethod
** Descriptions:        ���ַ�����������ǰ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void rightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       leftMethod
** Descriptions:        ���ַ������������˶�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void leftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontRightMethod
** Descriptions:        ���ҷ���������ǰ���У��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void frontRightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontLeftMethod
** Descriptions:        ������������ǰ���У��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void frontLeftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       centralMethod
** Descriptions:        ���ķ��򣬸��ݵ�����Ŀǰ���Թ���������λ�þ���ʹ�ú�����������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void centralMethod (void)
{
    if (GmcMouse.cX & 0x08) {
        if (GmcMouse.cY & 0x08) {

            /*
             *  ��ʱ���������Թ������Ͻ�
             */ 
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  ��ʱ���������Թ������½�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            default:
                break;
            }
        }
    } else {
        if (GmcMouse.cY & 0x08) {

            /*
             *  ��ʱ���������Թ������Ͻ�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  ��ʱ���������Թ������½�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            default:
                break;
            }
        }
    }
}
/*********************************************************************************************************
** Function name:       crosswayCheck
** Descriptions:        ͳ��ĳ������ڻ�δ�߹���֧·��
** input parameters:    ucX����Ҫ����ĺ�����
**                      ucY����Ҫ�����������
** output parameters:   ��
** Returned value:      ucCt��δ�߹���֧·��
*********************************************************************************************************/
uchar crosswayCheck (char  cX, char  cY)
{
    uchar ucCt = 0;
    if ((GucMapBlock[cX][cY] & 0x01) &&                                 /*  ���Է����Թ��Ϸ���·      */
        (GucMapBlock[cX][cY + 1]) == 0x00) {                            /*  ���Է����Թ��Ϸ�δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x02) &&                                 /*  ���Է����Թ��ҷ���·      */
        (GucMapBlock[cX + 1][cY]) == 0x00) {                            /*  ���Է����Թ��ҷ�û���߹�  */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x04) &&                                 /*  ���Է����Թ��·���·      */
        (GucMapBlock[cX][cY - 1]) == 0x00) {                            /*  ���Է����Թ��·�δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x08) &&                                 /*  ���Է����Թ�����·      */
        (GucMapBlock[cX - 1][cY]) == 0x00) {                            /*  ���Է����Թ���δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    return ucCt;
}
/*********************************************************************************************************
** Function name:       crosswayChoice
** Descriptions:        ѡ��һ��֧·��Ϊǰ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void crosswayChoice (void)
{
    switch (SEARCHMETHOD) {
        
    case RIGHTMETHOD:
        rightMethod();
        break;
    
    case LEFTMETHOD:
        leftMethod();
        break;
    
    case CENTRALMETHOD:
        centralMethod();
        break;

    case FRONTRIGHTMETHOD:
        frontRightMethod();
        break;

    case FRONTLEFTMETHOD:
        frontLeftMethod();
        break;

    default:
        break;
    }
}
/*********************************************************************************************************
** Function name:       main
** Descriptions:        ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
int main (void)
{
    uchar n          = 0;                                               /*  GmcCrossway[]�±�           */
    uchar ucRoadStat = 0;                                               /*  ͳ��ĳһ�����ǰ����֧·��  */
    uchar ucTemp     = 0;                                               /*  ����START״̬������ת��     */

    mouseInit();                                                        /*  �ײ������ĳ�ʼ��            */
    Init_7289();                                                      /*  ��ʾģ���ʼ��              */

    while (1) {
        switch (GucMouseTask) {                                         /*  ״̬������                  */
            
        case WAIT:
            sensorDebug();
            voltageDetect();
            delay(100000);
            if (keyCheck() == true) {                                   /*  ��ⰴ���ȴ�����            */
                Reset_7289();                                         /*  ��λZLG7289                 */
                GucMouseTask = START;
            }
            break;
            
        case START:                                                     /*  �жϵ��������ĺ�����      */
            mazeSearch();                                               /*  ��ǰ����                    */
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) {         /*  �жϵ���������Ƿ���ڳ���  */
                if (MAZETYPE == 8) {                                    /*  �޸��ķ�֮һ�Թ����յ�����  */
                    GucXGoal0 = 1;
                    GucXGoal1 = 0;
                }
                GucXStart   = MAZETYPE - 1;                             /*  �޸ĵ��������ĺ�����      */
                GmcMouse.cX = MAZETYPE - 1;                             /*  �޸ĵ�����ǰλ�õĺ�����  */    
                /*
                 *  ����Ĭ�ϵ����Ϊ(0,0)��������Ҫ���Ѽ�¼��ǽ������ת������
                 */
                ucTemp = GmcMouse.cY;
                do {
                    GucMapBlock[MAZETYPE - 1][ucTemp] = GucMapBlock[0][ucTemp];
                    GucMapBlock[0 ][ucTemp] = 0;
                }while (ucTemp--);
                /*
                 *  ��OFFSHOOT[0]�б����������
                 */
                GmcCrossway[n].cX = MAZETYPE - 1;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  ״̬ת��Ϊ��Ѱ״̬          */
            }
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02) {         /*  �жϵ������ұ��Ƿ���ڳ���  */
                /*
                 *  ��OFFSHOOT[0]�б����������
                 */
                GmcCrossway[n].cX = 0;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  ״̬ת��Ϊ��Ѱ״̬          */
            }
            break;
            
        case MAZESEARCH:
          if (((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal0))||((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal1))
           ||((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal0))||((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal1)))
          {    
             mouseTurnback();
#if USE_ASTAR_ALGORITHM
             objectGoToAstar(GucXStart,GucYStart);
#else
             objectGoTo(GucXStart,GucYStart);
#endif
             mouseTurnback();  
             GucMouseTask = SPURT;
             break;
          }          
          else{
            ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);        /*  ͳ�ƿ�ǰ����֧·��          */
            if (ucRoadStat) 
            {                                                           /*  �п�ǰ������                */
                if (ucRoadStat > 1) {                                   /*  �ж�����ǰ�����򣬱�������  */
                    GmcCrossway[n].cX = GmcMouse.cX;
                    GmcCrossway[n].cY = GmcMouse.cY;
                    n++;
                }
                crosswayChoice();                                       /*  �����ַ�������ѡ��ǰ������  */
                mazeSearch();                                           /*  ǰ��һ��                    */
            } 
               else if(ucRoadStat==1)
              {
                  crosswayChoice();                                       /*  �����ַ�������ѡ��ǰ������  */
                  mazeSearch();
              }
              else 
             {                                                    /*  û�п�ǰ�����򣬻ص����֧·*/
                mouseTurnback();
                n=n-1;
#if USE_ASTAR_ALGORITHM
                objectGoToAstar(GmcCrossway[n].cX,GmcCrossway[n].cY);
#else
                objectGoTo(GmcCrossway[n].cX,GmcCrossway[n].cY);
#endif
                
                ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);
                if (ucRoadStat > 1) {
                    GmcCrossway[n].cX = GmcMouse.cX;
                    GmcCrossway[n].cY = GmcMouse.cY;
                    n++;     
                } 
                crosswayChoice();
                mazeSearch();                            
            }
          }
            break;

        case SPURT:
#if USE_ASTAR_ALGORITHM
            mouseSpurtAstar();                                          /*  ʹ��A*�㷨������·�������յ�*/
            objectGoToAstar(GucXStart,GucYStart);                       /*  �ص����                    */
#else
            mouseSpurt();                                               /*  ������·�������յ�          */
            objectGoTo(GucXStart,GucYStart);                            /*  �ص����                    */
#endif
            mouseTurnback();                                            /*  ���ת���ָ���������        */
            while (1) {
                if (keyCheck() == true) {
                    break;
                }
                sensorDebug();
                delay(20000);
            }
            break;

        default:
            break;
        }
    }
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
