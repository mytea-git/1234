
/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "Maze.h"
#include <limits.h> // 为了使用 UINT_MAX
#include <stdlib.h> // 为了 abs 函数 (虽然在提供的代码中没直接用abs，但计算曼哈顿距离时可能需要，或者将来扩展时用到)

/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/
static uchar    GucXStart                           = 0;                /*  起点横坐标                  */
static uchar    GucYStart                           = 0;                /*  起点纵坐标                  */

static uchar    GucXGoal0                           = XDST0;            /*  终点X坐标，有两个值         */
static uchar    GucXGoal1                           = XDST1;
static uchar    GucYGoal0                           = YDST0;            /*  终点Y坐标，有两个值         */
static uchar    GucYGoal1                           = YDST1;

static uchar    GucMouseTask                        = WAIT;             /*  状态机，初始状态为等待      */

static uchar    GucMapStep[MAZETYPE][MAZETYPE]      = {0xff};           /*  保存各坐标的等高值          */

static MAZECOOR GmcStack[MAZETYPE * MAZETYPE]       = {0};              /*  在mapStepEdit()中作堆栈使用 */
static MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE]    = {0};              /*  Main()中暂存未走过支路坐标  */
// A* 节点结构
typedef struct {
    MAZECOOR parent;
    uint gCost;
    uint hCost;
    uint fCost;
    uchar onOpenList;
    uchar onClosedList;
} AStarNode;

// 全局 A* 节点信息数组
static AStarNode aStarNodes[MAZETYPE][MAZETYPE];

// 开放列表
#define OPEN_LIST_MAX_SIZE (MAZETYPE * MAZETYPE)
static MAZECOOR openList[OPEN_LIST_MAX_SIZE];
static int openListSize = 0;
/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void delay (uint uiD)
{
    for (; uiD; uiD--);
}
// 计算曼哈顿距离作为启发式成本 h(n)
uint calculateHeuristic(char cX1, char cY1, char cX2, char cY2) {
    uint dx = (cX1 > cX2) ? (cX1 - cX2) : (cX2 - cX1);
    uint dy = (cY1 > cY2) ? (cY1 - cY2) : (cY2 - cY1);
    // 假设移动一格的成本是 1
    return dx + dy;
} 
#include <stdlib.h> // 为了 abs 函数

// --- 开放列表的辅助函数 (简化版，非优先队列) ---
void addToOpenList(char cX, char cY) {
    if (openListSize < OPEN_LIST_MAX_SIZE) {
        openList[openListSize].cX = cX;
        openList[openListSize].cY = cY;
        openListSize++;
        aStarNodes[cX][cY].onOpenList = TRUE;
    }
    // 实际应用中需要检查重复添加和处理边界情况
}

// 从开放列表中找到 fCost 最低的节点 (简化版，效率较低)
MAZECOOR getLowestFCostNode() {
    uint minFCost = UINT_MAX;
    int minIndex = -1;
    MAZECOOR node = {-1, -1}; // 无效坐标表示未找到

    for (int i = 0; i < openListSize; ++i) {
        char cx = openList[i].cX;
        char cy = openList[i].cY;
        if (aStarNodes[cx][cy].fCost < minFCost) {
            minFCost = aStarNodes[cx][cy].fCost;
            minIndex = i;
        }
    }

    if (minIndex != -1) {
        node = openList[minIndex];
        // 从列表中移除 (将最后一个元素移到当前位置)
        openList[minIndex] = openList[openListSize - 1];
        openListSize--;
        aStarNodes[node.cX][node.cY].onOpenList = FALSE; // 从开放列表移除标记
    }
    return node;
}

void removeFromOpenList(char cX, char cY) {
     for (int i = 0; i < openListSize; ++i) {
        if (openList[i].cX == cX && openList[i].cY == cY) {
            openList[i] = openList[openListSize - 1];
            openListSize--;
            aStarNodes[cX][cY].onOpenList = FALSE;
            break;
        }
    }
}


// A* 搜索主函数
// 返回 TRUE 如果找到路径，FALSE 如果找不到
// path 参数用于存储回溯后的路径 (需要调用者分配足够空间)
// pathLen 返回路径长度
uchar findPathAStar(MAZECOOR start, MAZECOOR goal, MAZECOOR path[], int* pathLen) {
    int i, j;
    char currentX, currentY;
    MAZECOOR currentNode;

    // 1. 初始化
    openListSize = 0;
    for (i = 0; i < MAZETYPE; ++i) {
        for (j = 0; j < MAZETYPE; ++j) {
            aStarNodes[i][j].parent.cX = -1; // 无效父节点
            aStarNodes[i][j].parent.cY = -1;
            aStarNodes[i][j].gCost = UINT_MAX;
            aStarNodes[i][j].hCost = UINT_MAX;
            aStarNodes[i][j].fCost = UINT_MAX;
            aStarNodes[i][j].onOpenList = FALSE;
            aStarNodes[i][j].onClosedList = FALSE;
        }
    }

    // 2. 将起点加入开放列表
    aStarNodes[start.cX][start.cY].gCost = 0;
    aStarNodes[start.cX][start.cY].hCost = calculateHeuristic(start.cX, start.cY, goal.cX, goal.cY);
    aStarNodes[start.cX][start.cY].fCost = aStarNodes[start.cX][start.cY].hCost;
    addToOpenList(start.cX, start.cY);

    // 3. 主循环
    while (openListSize > 0) {
        // 3.1 从开放列表取出 fCost 最低的节点
        currentNode = getLowestFCostNode();
        currentX = currentNode.cX;
        currentY = currentNode.cY;

        // 3.2 如果是目标节点，则找到路径
        if (currentX == goal.cX && currentY == goal.cY) {
            // 回溯路径
            *pathLen = 0;
            MAZECOOR temp = goal;
            while (temp.cX != -1 && temp.cY != -1) {
                 if (*pathLen < OPEN_LIST_MAX_SIZE) { // 防止数组越界
                    path[*pathLen] = temp;
                    (*pathLen)++;
                    temp = aStarNodes[temp.cX][temp.cY].parent;
                } else {
                    // 路径太长，处理错误
                    return FALSE;
                }
            }
             // 翻转路径数组 (因为是从终点回溯的)
            for (i = 0; i < (*pathLen) / 2; ++i) {
                MAZECOOR swap = path[i];
                path[i] = path[*pathLen - 1 - i];
                path[*pathLen - 1 - i] = swap;
            }
            return TRUE; // 找到路径
        }

        // 3.3 将当前节点移到封闭列表
        aStarNodes[currentX][currentY].onClosedList = TRUE;

        // 3.4 检查所有邻居
        int dx[] = {0, 1, 0, -1}; // 上, 右, 下, 左
        int dy[] = {1, 0, -1, 0};
        uchar walls[] = {0x01, 0x02, 0x04, 0x08}; // 对应方向的墙壁掩码 (上右下左)

        for (i = 0; i < 4; ++i) {
            // 检查是否有墙壁阻挡
            if (GucMapBlock[currentX][currentY] & walls[i]) {
                char neighborX = currentX + dx[i];
                char neighborY = currentY + dy[i];

                // 检查邻居是否在迷宫范围内且不是障碍物(这里简化，假设能走的就是通路)
                if (neighborX >= 0 && neighborX < MAZETYPE && neighborY >= 0 && neighborY < MAZETYPE) {
                    // 如果邻居已在封闭列表，跳过
                    if (aStarNodes[neighborX][neighborY].onClosedList) {
                        continue;
                    }

                    // 计算通过当前节点到达邻居的 gCost
                    // 假设移动成本为 1
                    uint tentativeGCost = aStarNodes[currentX][currentY].gCost + 1;

                    // 如果这条路径更好，或者邻居不在开放列表
                    if (tentativeGCost < aStarNodes[neighborX][neighborY].gCost || !aStarNodes[neighborX][neighborY].onOpenList) {
                        // 更新邻居信息
                        aStarNodes[neighborX][neighborY].parent = currentNode;
                        aStarNodes[neighborX][neighborY].gCost = tentativeGCost;
                        aStarNodes[neighborX][neighborY].hCost = calculateHeuristic(neighborX, neighborY, goal.cX, goal.cY);
                        aStarNodes[neighborX][neighborY].fCost = aStarNodes[neighborX][neighborY].gCost + aStarNodes[neighborX][neighborY].hCost;

                        // 如果邻居不在开放列表，则加入
                        if (!aStarNodes[neighborX][neighborY].onOpenList) {
                            addToOpenList(neighborX, neighborY);
                        }
                        // 注意：如果使用真正的优先队列，这里可能需要更新节点在队列中的位置
                    }
                }
            }
        }
    }

    return FALSE; // 未找到路径
}
/*********************************************************************************************************
** Function name:       mapStepEdit
** Descriptions:        制作以目标点为起点的等高图
** input parameters:    uiX:    目的地横坐标
**                      uiY:    目的地纵坐标
** output parameters:   GucMapStep[][]:  各坐标上的等高值
** Returned value:      无
*********************************************************************************************************/
void mapStepEdit (char  cX, char  cY)
{
    uchar n         = 0;                                                /*  GmcStack[]下标              */
    uchar ucStep    = 1;                                                /*  等高值                      */
    uchar ucStat    = 0;                                                /*  统计可前进的方向数          */
    uchar i,j;
    
    GmcStack[n].cX  = cX;                                               /*  起点X值入栈                 */
    GmcStack[n].cY  = cY;                                               /*  起点Y值入栈                 */
    n++;
    /*
     *  初始化各坐标等高值
     */
    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            GucMapStep[i][j] = 0xff;
        }
    }
    /*
     *  制作等高图，直到堆栈中所有数据处理完毕
     */
    while (n) {
        GucMapStep[cX][cY] = ucStep++;                                  /*  填入等高值                  */

        /*
         *  对当前坐标格里可前进的方向统计
         */
        ucStat = 0;
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  前方有路                    */
            (GucMapStep[cX][cY + 1] > (ucStep))) {                      /*  前方等高值大于计划设定值    */
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  右方有路                    */
            (GucMapStep[cX + 1][cY] > (ucStep))) {                      /*  右方等高值大于计划设定值    */
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&
            (GucMapStep[cX][cY - 1] > (ucStep))) {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&
            (GucMapStep[cX - 1][cY] > (ucStep))) {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        /*
         *  没有可前进的方向，则跳转到最近保存的分支点
         *  否则任选一可前进方向前进
         */
        if (ucStat == 0) {
            n--;
            cX = GmcStack[n].cX;
            cY = GmcStack[n].cY;
            ucStep = GucMapStep[cX][cY];
        } else {
            if (ucStat > 1) {                                           /*  有多个可前进方向，保存坐标  */
                GmcStack[n].cX = cX;                                    /*  横坐标X值入栈               */
                GmcStack[n].cY = cY;                                    /*  纵坐标Y值入栈               */
                n++;
            }
            /*
             *  任意选择一条可前进的方向前进
             */
            if ((GucMapBlock[cX][cY] & 0x01) &&                         /*  上方有路                    */
                (GucMapStep[cX][cY + 1] > (ucStep))) {                  /*  上方等高值大于计划设定值    */
                cY++;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x02) &&                         /*  右方有路                    */
                (GucMapStep[cX + 1][cY] > (ucStep))) {                  /*  右方等高值大于计划设定值    */
                cX++;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x04) &&                         /*  下方有路                    */
                (GucMapStep[cX][cY - 1] > (ucStep))) {                  /*  下方等高值大于计划设定值    */
                cY--;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x08) &&                         /*  左方有路                    */
                (GucMapStep[cX - 1][cY] > (ucStep))) {                  /*  左方等高值大于计划设定值    */
                cX--;                                                   /*  修改坐标                    */
                continue;
            }
        }
    }
}
/*********************************************************************************************************
** Function name:       mouseSpurt
** Descriptions:        电脑鼠从起点以最短路径跑向终点
** input parameters:    无
 ** output parameters:  无
** Returned value:      无
*********************************************************************************************************/
// 新的函数：根据A*生成的路径移动老鼠
void followAStarPath(MAZECOOR path[], int pathLen) {
    if (pathLen <= 1) return; // 路径无效或已在终点

    char cNBlock = 0; // 连续前进的格数
    for (int i = 0; i < pathLen - 1; ++i) {
        MAZECOOR current = path[i];
        MAZECOOR next = path[i + 1];

        // 计算需要移动的方向 (绝对方向)
        char targetDir;
        if (next.cY > current.cY) targetDir = UP;
        else if (next.cX > current.cX) targetDir = RIGHT;
        else if (next.cY < current.cY) targetDir = DOWN;
        else if (next.cX < current.cX) targetDir = LEFT;
        else continue; // 坐标相同，理论上不应发生

        // 如果方向与当前方向不同，先执行之前的连续前进，再转弯
        if (targetDir != GucMouseDir) {
            if (cNBlock > 0) {
                mouseGoahead(cNBlock); // 执行连续前进
                cNBlock = 0;          // 重置计数
            }
            // 计算相对方向并转弯
            char relativeDir = (targetDir + 4 - GucMouseDir) % 4;
            switch (relativeDir) {
                case 1: mouseTurnright(); break;
                case 2: mouseTurnback(); break; // 理论上A*路径不会需要180度转弯
                case 3: mouseTurnleft(); break;
            }
        }
        // 方向相同，增加连续前进格数
        cNBlock++;
    }

    // 执行最后一段连续前进
    if (cNBlock > 0) {
        mouseGoahead(cNBlock);
    }
     // 更新老鼠当前坐标（虽然 mouseGoahead 内部会更新，但确保最终位置正确）
     if (pathLen > 0) {
        GmcMouse = path[pathLen - 1];
     }
}

// 修改 mouseSpurt 函数以使用 A*
void mouseSpurt (void)
{
    uchar ucTemp = 0xff;
    char cXdst = -1, cYdst = -1;
    MAZECOOR start = {GucXStart, GucYStart};
    MAZECOOR goalCandidates[4];
    int numGoalCandidates = 0;
    MAZECOOR bestGoal = {-1, -1};
    uint minCost = UINT_MAX;
    MAZECOOR finalPath[OPEN_LIST_MAX_SIZE]; // 存储最终路径
    int finalPathLen = 0;


    // 确定所有可能的终点目标格
    if (GucMapBlock[GucXGoal0][GucYGoal0] != 0) { goalCandidates[numGoalCandidates++] = (MAZECOOR){GucXGoal0, GucYGoal0}; }
    if (GucMapBlock[GucXGoal0][GucYGoal1] != 0) { goalCandidates[numGoalCandidates++] = (MAZECOOR){GucXGoal0, GucYGoal1}; }
    if (GucMapBlock[GucXGoal1][GucYGoal0] != 0) { goalCandidates[numGoalCandidates++] = (MAZECOOR){GucXGoal1, GucYGoal0}; }
    if (GucMapBlock[GucXGoal1][GucYGoal1] != 0) { goalCandidates[numGoalCandidates++] = (MAZECOOR){GucXGoal1, GucYGoal1}; }

    // 对每个可能的终点运行 A*，找到成本最低的路径
    for (int i = 0; i < numGoalCandidates; ++i) {
        MAZECOOR currentGoal = goalCandidates[i];
        MAZECOOR tempPath[OPEN_LIST_MAX_SIZE];
        int tempPathLen = 0;

        if (findPathAStar(start, currentGoal, tempPath, &tempPathLen)) {
             // A* 找到路径后，gCost 就是实际成本
            uint currentCost = aStarNodes[currentGoal.cX][currentGoal.cY].gCost;
            if (currentCost < minCost) {
                minCost = currentCost;
                bestGoal = currentGoal;
                // 复制路径
                finalPathLen = tempPathLen;
                for(int j=0; j < finalPathLen; ++j) {
                    finalPath[j] = tempPath[j];
                }
            }
        }
    }


    // 如果找到了最优路径，则执行
    if (bestGoal.cX != -1) {
        // 从起点冲刺到最优终点
        followAStarPath(finalPath, finalPathLen);

        // （可选）如果需要从终点返回起点，再次调用 A* 或使用反向路径
         MAZECOOR returnPath[OPEN_LIST_MAX_SIZE];
         int returnPathLen = 0;
         if (findPathAStar(bestGoal, start, returnPath, &returnPathLen)) {
             followAStarPath(returnPath, returnPathLen);
         }

         mouseTurnback(); // 回到起点后，可能需要调整姿态
    } else {
         // 未找到路径的处理逻辑 (例如，显示错误)
          Download_7289(2, 0, 0, 0x4f); // 'E'
          Download_7289(2, 1, 0, 0x4f);
          Download_7289(2, 2, 0, 0x4f);
          Download_7289(2, 3, 0, 0x4f);
          while(1); // 停在这里
    }
}
/*********************************************************************************************************
** Function name:       objectGoTo
** Descriptions:        使电脑鼠运动到指定坐标
** input parameters:    cXdst: 目的地的横坐标
**                      cYdst: 目的地的纵坐标
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void objectGoTo (char  cXdst, char  cYdst)
{
    uchar ucStep = 1;
    char  cNBlock = 0, cDirTemp;
    char cX,cY;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdit(cXdst,cYdst);                                           /*  制作等高图                  */
    /*
     *  根据等高值向目标点运动，直到达到目的地
     */
    while ((cX != cXdst) || (cY != cYdst)) {
        ucStep = GucMapStep[cX][cY];
        /*
         *  任选一个等高值比当前自身等高值小的方向前进
         */
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  上方有路                    */
            (GucMapStep[cX][cY + 1] < ucStep)) {                        /*  上方等高值较小              */
            cDirTemp = UP;                                              /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  右方有路                    */
            (GucMapStep[cX + 1][cY] < ucStep)) {                        /*  右方等高值较小              */
            cDirTemp = RIGHT;                                           /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  下方有路                    */
            (GucMapStep[cX][cY - 1] < ucStep)) {                        /*  下方等高值较小              */
            cDirTemp = DOWN;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY--;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  左方有路                    */
            (GucMapStep[cX - 1][cY] < ucStep)) {                        /*  左方等高值较小              */
            cDirTemp = LEFT;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX--;
                continue;                                               /*  跳过本次循环                */
            }
        }
        cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;                      /*  计算方向偏移量              */
        
        if (cNBlock) {
            mouseGoahead(cNBlock);                                      /*  前进cNBlock步               */
        }        
        cNBlock = 0;                                                    /*  任务清零                    */
        
        /*
         *  控制电脑鼠转弯
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
     *  判断任务是否完成，否则继续前进
     */
    if (cNBlock) {
        mouseGoahead(cNBlock);
    }
}
/*********************************************************************************************************
** Function name:       mazeBlockDataGet
** Descriptions:        根据电脑鼠的相对方向，取出该方向上迷宫格的墙壁资料
** input parameters:    ucDir: 电脑鼠的相对方向
** output parameters:   无
** Returned value:      GucMapBlock[cX][cY] : 墙壁资料
*********************************************************************************************************/
uchar mazeBlockDataGet (uchar  ucDirTemp)
{
    char cX = 0,cY = 0;
    
    /*
     *  把电脑鼠的相对方向转换为绝对方向
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
     *  根据绝对方向计算该方向上相邻格的坐标
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
    
    return(GucMapBlock[cX][cY]);                                        /*  返回迷宫格上的资料          */
}
/*********************************************************************************************************
** Function name:       rightMethod
** Descriptions:        右手法则，优先向右前进
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void rightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       leftMethod
** Descriptions:        左手法则，优先向左运动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void leftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontRightMethod
** Descriptions:        中右法则，优先向前运行，其次向右
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontRightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontLeftMethod
** Descriptions:        中左法则，优先向前运行，其次向左
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontLeftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       centralMethod
** Descriptions:        中心法则，根据电脑鼠目前在迷宫中所处的位置觉定使用何种搜索法则
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void centralMethod (void)
{
    if (GmcMouse.cX & 0x08) {
        if (GmcMouse.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的右上角
             */ 
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的右下角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            default:
                break;
            }
        }
    } else {
        if (GmcMouse.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的左上角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                leftMethod();                                           /*  左手法则                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的左下角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                rightMethod();                                          /*  右手法则                    */
                break;

            default:
                break;
            }
        }
    }
}
/*********************************************************************************************************
** Function name:       crosswayCheck
** Descriptions:        统计某坐标存在还未走过的支路数
** input parameters:    ucX，需要检测点的横坐标
**                      ucY，需要检测点的纵坐标
** output parameters:   无
** Returned value:      ucCt，未走过的支路数
*********************************************************************************************************/
uchar crosswayCheck (char  cX, char  cY)
{
    uchar ucCt = 0;
    if ((GucMapBlock[cX][cY] & 0x01) &&                                 /*  绝对方向，迷宫上方有路      */
        (GucMapBlock[cX][cY + 1]) == 0x00) {                            /*  绝对方向，迷宫上方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x02) &&                                 /*  绝对方向，迷宫右方有路      */
        (GucMapBlock[cX + 1][cY]) == 0x00) {                            /*  绝对方向，迷宫右方没有走过  */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x04) &&                                 /*  绝对方向，迷宫下方有路      */
        (GucMapBlock[cX][cY - 1]) == 0x00) {                            /*  绝对方向，迷宫下方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x08) &&                                 /*  绝对方向，迷宫左方有路      */
        (GucMapBlock[cX - 1][cY]) == 0x00) {                            /*  绝对方向，迷宫左方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    return ucCt;
}
/*********************************************************************************************************
** Function name:       crosswayChoice
** Descriptions:        选择一条支路作为前进方向
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        主函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
main (void)
{
    uchar n          = 0;                                               /*  GmcCrossway[]下标           */
    uchar ucRoadStat = 0;                                               /*  统计某一坐标可前进的支路数  */
    uchar ucTemp     = 0;                                               /*  用于START状态中坐标转换     */

    mouseInit();                                                        /*  底层驱动的初始化            */
    Init_7289();                                                      /*  显示模块初始化              */

    while (1) {
        switch (GucMouseTask) {                                         /*  状态机处理                  */
            
        case WAIT:
            sensorDebug();
            voltageDetect();
            delay(100000);
            if (keyCheck() == true) {                                   /*  检测按键等待启动            */
                Reset_7289();                                         /*  复位ZLG7289                 */
                GucMouseTask = START;
            }
            break;
            
        case START:                                                     /*  判断电脑鼠起点的横坐标      */
            mazeSearch();                                               /*  向前搜索                    */
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) {         /*  判断电老鼠左边是否存在出口  */
                if (MAZETYPE == 8) {                                    /*  修改四分之一迷宫的终点坐标  */
                    GucXGoal0 = 1;
                    GucXGoal1 = 0;
                }
                GucXStart   = MAZETYPE - 1;                             /*  修改电脑鼠起点的横坐标      */
                GmcMouse.cX = MAZETYPE - 1;                             /*  修改电脑鼠当前位置的横坐标  */    
                /*
                 *  由于默认的起点为(0,0)，现在需要把已记录的墙壁资料转换过来
                 */
                ucTemp = GmcMouse.cY;
                do {
                    GucMapBlock[MAZETYPE - 1][ucTemp] = GucMapBlock[0][ucTemp];
                    GucMapBlock[0 ][ucTemp] = 0;
                }while (ucTemp--);
                /*
                 *  在OFFSHOOT[0]中保存起点坐标
                 */
                GmcCrossway[n].cX = MAZETYPE - 1;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  状态转换为搜寻状态          */
            }
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02) {         /*  判断电老鼠右边是否存在出口  */
                /*
                 *  在OFFSHOOT[0]中保存起点坐标
                 */
                GmcCrossway[n].cX = 0;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  状态转换为搜寻状态          */
            }
            break;
            
        case MAZESEARCH:
          if (((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal0))||((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal1))
           ||((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal0))||((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal1)))
          {    
             mouseTurnback();
             objectGoTo(GucXStart,GucYStart);
             mouseTurnback();  
             GucMouseTask = SPURT;
             break;
          }          
          else{
            ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);        /*  统计可前进的支路数          */
            if (ucRoadStat) 
            {                                                           /*  有可前进方向                */
                if (ucRoadStat > 1) {                                   /*  有多条可前进方向，保存坐标  */
                    GmcCrossway[n].cX = GmcMouse.cX;
                    GmcCrossway[n].cY = GmcMouse.cY;
                    n++;
                }
                crosswayChoice();                                       /*  用右手法则搜索选择前进方向  */
                mazeSearch();                                           /*  前进一格                    */
            } 
               else if(ucRoadStat==1)
              {
                  crosswayChoice();                                       /*  用右手法则搜索选择前进方向  */
                  mazeSearch();
              }
              else 
             {                                                    /*  没有可前进方向，回到最近支路*/
                // main 函数中 MAZESEARCH 部分的修改示例 (回溯部分)
case MAZESEARCH:
    // ... (检查是否到达终点等逻辑保持不变) ...
    else {
        ucRoadStat = crosswayCheck(GmcMouse.cX, GmcMouse.cY);
        if (ucRoadStat) {
            // ... (处理有未探索路径的逻辑不变) ...
             if (ucRoadStat > 1) {
                GmcCrossway[n].cX = GmcMouse.cX;
                GmcCrossway[n].cY = GmcMouse.cY;
                n++;
            }
            crosswayChoice();
            mazeSearch(); // 继续探索
        } else { // 没有未探索的路径，需要回溯
            //mouseTurnback(); // 原来的回溯可能只需要后转一下
            if (n > 0) { // 确保堆栈中有回溯点
                 n = n - 1;
                 MAZECOOR backtrackGoal = GmcCrossway[n];
                 MAZECOOR backtrackPath[OPEN_LIST_MAX_SIZE];
                 int backtrackPathLen = 0;

                 // 使用 A* 寻找回到最近岔路口的最短已知路径
                 if (findPathAStar(GmcMouse, backtrackGoal, backtrackPath, &backtrackPathLen)) {
                    followAStarPath(backtrackPath, backtrackPathLen); // 执行回溯路径
                 } else {
                     // 无法找到回溯路径，错误处理
                     Download_7289(2, 0, 0, 0x4f); // 'E'
                     while(1);
                 }

                 // 回到岔路口后，重新评估
                 ucRoadStat = crosswayCheck(GmcMouse.cX, GmcMouse.cY);
                 if (ucRoadStat > 1) { // 如果还有其他未探索路径，放回堆栈
                    GmcCrossway[n].cX = GmcMouse.cX;
                    GmcCrossway[n].cY = GmcMouse.cY;
                    n++;
                 }
                 if (ucRoadStat > 0) { // 选择下一个探索方向并前进
                    crosswayChoice();
                    mazeSearch();
                 } else if (n > 0) {
                    // 如果回到岔路口后仍然没有出路 (理论上不应发生，除非地图记录错误或堆栈逻辑问题)，继续向上回溯
                    // 可能需要添加一个循环或递归的回溯逻辑
                 } else {
                     // 堆栈空了，且当前位置无路可走，探索完成或陷入死区
                     // 可以考虑返回起点或进入SPURT状态
                     objectGoTo(GucXStart,GucYStart);
                     mouseTurnback();
                     GucMouseTask = SPURT; // 假设探索完成，进入冲刺
                 }
            } else {
                 // 堆栈为空，且当前位置没有未探索路径，探索完成
                 objectGoTo(GucXStart,GucYStart);
                 mouseTurnback();
                 GucMouseTask = SPURT; // 进入冲刺
            }
        }
    }
    break;
          }
            break;

        case SPURT:
            mouseSpurt();                                               /*  以最优路径冲向终点          */
            objectGoTo(GucXStart,GucYStart);                            /*  回到起点                    */
            mouseTurnback();                                            /*  向后转，恢复出发姿势        */
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



