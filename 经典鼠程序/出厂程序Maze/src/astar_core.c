
/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "astar_core.h"
#include "maze.h"


/*********************************************************************************************************
  常量定义
*********************************************************************************************************/
static const signed char Gneighbors[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
static const uchar Gdirs[4] = {UP, RIGHT, DOWN, LEFT};

/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/
static ASTAR_PARAMS GastarParams = {1.0f, 10, 5, 50, 3};

typedef struct {
    signed char  cX;
    signed char  cY;
    uchar ucDir;
    uint  uiG;
    uint  uiF;
    signed char  cParentX;
    signed char  cParentY;
    uchar ucInClosed;
    uint  uiTimestamp;
} ASTAR_NODE;

static ASTAR_NODE GnodeList[MAZETYPE][MAZETYPE];
static uint GuiCurrentTimestamp = 1;

static char GheapX[MAZETYPE * MAZETYPE];
static char GheapY[MAZETYPE * MAZETYPE];
static uchar GucHeapSize = 0;


/*********************************************************************************************************
** Function name:       astarInit
** Descriptions:        初始化 A* 算法
*********************************************************************************************************/
void astarInit(void)
{
    GuiCurrentTimestamp = 1;
}

/*********************************************************************************************************
** Function name:       astarSetParams
** Descriptions:        设置 A* 算法参数
*********************************************************************************************************/
void astarSetParams(float fHWeight, uchar ucMCost, uchar ucTCost,
                    uchar ucSpeedFactor, uchar ucTurnPenalty)
{
    GastarParams.fHeuristicWeight = fHWeight;
    GastarParams.ucMoveCost = ucMCost;
    GastarParams.ucBaseTurnCost = ucTCost;
    GastarParams.ucSpeedFactor = ucSpeedFactor;
    GastarParams.ucTurnPenalty = ucTurnPenalty;
}

/*********************************************************************************************************
** Function name:       astarGetParams
** Descriptions:        获取 A* 算法参数
*********************************************************************************************************/
void astarGetParams(ASTAR_PARAMS *params)
{
    params->fHeuristicWeight = GastarParams.fHeuristicWeight;
    params->ucMoveCost = GastarParams.ucMoveCost;
    params->ucBaseTurnCost = GastarParams.ucBaseTurnCost;
    params->ucSpeedFactor = GastarParams.ucSpeedFactor;
    params->ucTurnPenalty = GastarParams.ucTurnPenalty;
}

/*********************************************************************************************************
** Function name:       heapSwap
** Descriptions:        交换堆中两个元素
*********************************************************************************************************/
static void heapSwap(uchar i, uchar j)
{
    char cTempX = GheapX[i];
    char cTempY = GheapY[i];
    GheapX[i] = GheapX[j];
    GheapY[i] = GheapY[j];
    GheapX[j] = cTempX;
    GheapY[j] = cTempY;
}

/*********************************************************************************************************
** Function name:       heapUp
** Descriptions:        堆上浮操作
*********************************************************************************************************/
static void heapUp(uchar idx)
{
    while (idx > 0) {
        uchar parent = (idx - 1) >> 1;
        if (GnodeList[GheapX[idx]][GheapY[idx]].uiF >=
            GnodeList[GheapX[parent]][GheapY[parent]].uiF) break;
        heapSwap(idx, parent);
        idx = parent;
    }
}

/*********************************************************************************************************
** Function name:       heapDown
** Descriptions:        堆下沉操作
*********************************************************************************************************/
static void heapDown(uchar idx)
{
    while (1) {
        uchar smallest = idx;
        uchar left = (idx << 1) + 1;
        uchar right = (idx << 1) + 2;

        if (left < GucHeapSize &&
            GnodeList[GheapX[left]][GheapY[left]].uiF <
            GnodeList[GheapX[smallest]][GheapY[smallest]].uiF) {
            smallest = left;
        }
        if (right < GucHeapSize &&
            GnodeList[GheapX[right]][GheapY[right]].uiF <
            GnodeList[GheapX[smallest]][GheapY[smallest]].uiF) {
            smallest = right;
        }
        if (smallest == idx) break;
        heapSwap(idx, smallest);
        idx = smallest;
    }
}

/*********************************************************************************************************
** Function name:       heapPush
** Descriptions:        向堆中添加元素
*********************************************************************************************************/
static void heapPush(char cX, char cY)
{
    GheapX[GucHeapSize] = cX;
    GheapY[GucHeapSize] = cY;
    heapUp(GucHeapSize);
    GucHeapSize++;
}

/*********************************************************************************************************
** Function name:       heapPop
** Descriptions:        从堆中取出最小元素
*********************************************************************************************************/
static uchar heapPop(char *pcX, char *pcY)
{
    if (GucHeapSize == 0) return 0;
    *pcX = GheapX[0];
    *pcY = GheapY[0];
    GucHeapSize--;
    if (GucHeapSize > 0) {
        GheapX[0] = GheapX[GucHeapSize];
        GheapY[0] = GheapY[GucHeapSize];
        heapDown(0);
    }
    return 1;
}

/*********************************************************************************************************
** Function name:       getDynamicTurnCost
** Descriptions:        计算动态转弯代价（考虑速度）
*********************************************************************************************************/
static uint getDynamicTurnCost(uchar ucFromDir, uchar ucToDir, uint uiSpeed)
{
    uchar ucDiff = (ucToDir + 4 - ucFromDir) % 4;
    uint uiBaseCost;

    if (ucDiff == 0) return 0;
    if (ucDiff == 2) uiBaseCost = GastarParams.ucBaseTurnCost * 2;
    else uiBaseCost = GastarParams.ucBaseTurnCost;

    return uiBaseCost * (100 + uiSpeed * GastarParams.ucSpeedFactor / 100) / 100;
}

/*********************************************************************************************************
** Function name:       getImprovedHeuristic
** Descriptions:        改进的启发函数（考虑转弯）
*********************************************************************************************************/
static uint getImprovedHeuristic(char cX1, char cY1, char cX2, char cY2, uchar ucDir)
{
    char cDx = (cX1 > cX2) ? (cX1 - cX2) : (cX2 - cX1);
    char cDy = (cY1 > cY2) ? (cY1 - cY2) : (cY2 - cY1);
    uint uiManhattan = (cDx + cDy) * GastarParams.ucMoveCost;

    uchar ucTargetDir = 0;
    if (cDx > cDy) {
        ucTargetDir = (cX2 > cX1) ? RIGHT : LEFT;
    } else {
        ucTargetDir = (cY2 > cY1) ? UP : DOWN;
    }

    uchar ucTurnDiff = (ucTargetDir + 4 - ucDir) % 4;
    uint uiTurnEstimate = 0;
    if (ucTurnDiff == 1 || ucTurnDiff == 3) uiTurnEstimate = GastarParams.ucTurnPenalty;
    else if (ucTurnDiff == 2) uiTurnEstimate = GastarParams.ucTurnPenalty * 2;

    return (uint)(uiManhattan * GastarParams.fHeuristicWeight) + uiTurnEstimate;
}

/*********************************************************************************************************
** Function name:       reconstructPath
** Descriptions:        重建路径
*********************************************************************************************************/
static void reconstructPath(char cXGoal, char cYGoal, ASTAR_PATH *path)
{
    char cX = cXGoal, cY = cYGoal;
    uchar ucIdx = 0;
    PATH_NODE temp[MAZETYPE * MAZETYPE];

    while (GnodeList[cX][cY].cParentX != -1) {
        temp[ucIdx].cX = cX;
        temp[ucIdx].cY = cY;
        temp[ucIdx].ucDir = GnodeList[cX][cY].ucDir;
        ucIdx++;
        char cTempX = GnodeList[cX][cY].cParentX;
        char cTempY = GnodeList[cX][cY].cParentY;
        cX = cTempX;
        cY = cTempY;
    }

    path->ucLength = ucIdx;
    uchar i;
    for (i = 0; i < ucIdx; i++) {
        path->nodes[i] = temp[ucIdx - 1 - i];
    }
}

/*********************************************************************************************************
** Function name:       astarFindPath
** Descriptions:        A* 路径查找（优化版，使用二叉堆）
*********************************************************************************************************/
uchar astarFindPath(char cXStart, char cYStart, char cXGoal, char cYGoal,
                    uchar ucStartDir, uint uiCurrentSpeed, ASTAR_PATH *path)
{
    char cCurX, cCurY;
    char i;

    GucHeapSize = 0;
    GuiCurrentTimestamp++;

    GnodeList[cXStart][cYStart].uiG = 0;
    GnodeList[cXStart][cYStart].uiF = getImprovedHeuristic(cXStart, cYStart, cXGoal, cYGoal, ucStartDir);
    GnodeList[cXStart][cYStart].ucDir = ucStartDir;
    GnodeList[cXStart][cYStart].cParentX = -1;
    GnodeList[cXStart][cYStart].cParentY = -1;
    GnodeList[cXStart][cYStart].ucInClosed = 0;
    GnodeList[cXStart][cYStart].uiTimestamp = GuiCurrentTimestamp;

    heapPush(cXStart, cYStart);

    while (heapPop(&cCurX, &cCurY)) {
        if (cCurX == cXGoal && cCurY == cYGoal) {
            reconstructPath(cXGoal, cYGoal, path);
            return 1;
        }

        GnodeList[cCurX][cCurY].ucInClosed = 1;

        for (i = 0; i < 4; i++) {
            if (!(GucMapBlock[cCurX][cCurY] & (1 << i))) continue;

            signed char cNX = cCurX + Gneighbors[i][0];
            signed char cNY = cCurY + Gneighbors[i][1];

            if (cNX < 0 || cNX >= MAZETYPE || cNY < 0 || cNY >= MAZETYPE) continue;

            if (GnodeList[cNX][cNY].uiTimestamp == GuiCurrentTimestamp &&
                GnodeList[cNX][cNY].ucInClosed) continue;

            uint uiTurnCost = getDynamicTurnCost(GnodeList[cCurX][cCurY].ucDir, Gdirs[i], uiCurrentSpeed);
            uint uiTentG = GnodeList[cCurX][cCurY].uiG + GastarParams.ucMoveCost + uiTurnCost;

            if (GnodeList[cNX][cNY].uiTimestamp != GuiCurrentTimestamp) {
                GnodeList[cNX][cNY].uiTimestamp = GuiCurrentTimestamp;
                GnodeList[cNX][cNY].uiG = 0xFFFFFFFF;
                GnodeList[cNX][cNY].ucInClosed = 0;
            }

            if (uiTentG < GnodeList[cNX][cNY].uiG) {
                GnodeList[cNX][cNY].cParentX = cCurX;
                GnodeList[cNX][cNY].cParentY = cCurY;
                GnodeList[cNX][cNY].ucDir = Gdirs[i];
                GnodeList[cNX][cNY].uiG = uiTentG;
                GnodeList[cNX][cNY].uiF = uiTentG + getImprovedHeuristic(cNX, cNY, cXGoal, cYGoal, Gdirs[i]);

                heapPush(cNX, cNY);
            }
        }
    }

    path->ucLength = 0;
    return 0;
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
