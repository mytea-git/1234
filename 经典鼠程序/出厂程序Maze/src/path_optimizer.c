
/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "path_optimizer.h"
#include "maze.h"


/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/
static ADAPTIVE_MODE GcurrentMode = ADAPTIVE_MODE_BALANCED;


/*********************************************************************************************************
** Function name:       pathOptimizerInit
** Descriptions:        初始化路径优化器
*********************************************************************************************************/
void pathOptimizerInit(void)
{
    GcurrentMode = ADAPTIVE_MODE_BALANCED;
}

/*********************************************************************************************************
** Function name:       pathGetTurnCount
** Descriptions:        计算路径中的转弯次数
*********************************************************************************************************/
uchar pathGetTurnCount(ASTAR_PATH *path)
{
    uchar ucTurns = 0;
    uchar i;

    if (path->ucLength < 2) return 0;

    for (i = 1; i < path->ucLength; i++) {
        if (path->nodes[i].ucDir != path->nodes[i - 1].ucDir) {
            ucTurns++;
        }
    }
    return ucTurns;
}

/*********************************************************************************************************
** Function name:       pathSmooth
** Descriptions:        路径平滑处理（合并直线段并移除冗余节点）
*********************************************************************************************************/
void pathSmooth(ASTAR_PATH *path)
{
    if (path->ucLength < 2) return;

    PATH_NODE temp[MAZETYPE * MAZETYPE];
    uchar ucNewLen = 0;
    uchar i;

    temp[ucNewLen++] = path->nodes[0];

    for (i = 1; i < path->ucLength; i++) {
        if (path->nodes[i].ucDir != path->nodes[i - 1].ucDir) {
            temp[ucNewLen++] = path->nodes[i];
        } else {
            temp[ucNewLen - 1] = path->nodes[i];
        }
    }

    for (i = 0; i < ucNewLen; i++) {
        path->nodes[i] = temp[i];
    }
    path->ucLength = ucNewLen;
}

/*********************************************************************************************************
** Function name:       adaptiveSetMode
** Descriptions:        设置自适应模式
*********************************************************************************************************/
void adaptiveSetMode(ADAPTIVE_MODE mode)
{
    GcurrentMode = mode;
}

/*********************************************************************************************************
** Function name:       adaptiveAnalyzeMaze
** Descriptions:        分析迷宫复杂度
*********************************************************************************************************/
void adaptiveAnalyzeMaze(MAZE_COMPLEXITY *complexity)
{
    uchar i, j;
    uchar ucBranches = 0;
    uchar ucDeadEnds = 0;
    uchar ucBlock, ucPathCount;

    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            ucBlock = GucMapBlock[i][j];
            if (ucBlock == 0) continue;

            ucPathCount = (ucBlock & 0x01) + ((ucBlock >> 1) & 0x01) +
                         ((ucBlock >> 2) & 0x01) + ((ucBlock >> 3) & 0x01);

            if (ucPathCount >= 3) ucBranches++;
            else if (ucPathCount == 1) ucDeadEnds++;
        }
    }

    complexity->ucBranchCount = ucBranches;
    complexity->ucDeadEndCount = ucDeadEnds;
    complexity->ucComplexity = (ucBranches * 3 + ucDeadEnds) * 100 / (MAZETYPE * MAZETYPE);
    if (complexity->ucComplexity > 100) complexity->ucComplexity = 100;
}

/*********************************************************************************************************
** Function name:       adaptiveAdjustParams
** Descriptions:        根据迷宫复杂度和模式自适应调整参数
*********************************************************************************************************/
void adaptiveAdjustParams(MAZE_COMPLEXITY *complexity, ASTAR_PARAMS *params)
{
    float fHWeight;
    uchar ucTurnCost;

    if (complexity->ucComplexity < 20) {
        fHWeight = 1.5f;
        ucTurnCost = 3;
    } else if (complexity->ucComplexity < 50) {
        fHWeight = 1.2f;
        ucTurnCost = 4;
    } else if (complexity->ucComplexity < 70) {
        fHWeight = 1.0f;
        ucTurnCost = 5;
    } else {
        fHWeight = 0.9f;
        ucTurnCost = 6;
    }

    switch (GcurrentMode) {
    case ADAPTIVE_MODE_SPEED:
        fHWeight *= 1.2f;
        ucTurnCost = (ucTurnCost * 80) / 100;
        break;

    case ADAPTIVE_MODE_SMOOTH:
        fHWeight *= 0.8f;
        ucTurnCost = (ucTurnCost * 150) / 100;
        break;

    case ADAPTIVE_MODE_BALANCED:
    default:
        break;
    }

    params->fHeuristicWeight = fHWeight;
    params->ucBaseTurnCost = ucTurnCost;
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
