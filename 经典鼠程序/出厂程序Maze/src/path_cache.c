
/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "path_cache.h"
#include "Type.h"


/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/
static CACHE_ENTRY GcacheEntries[CACHE_SIZE];
static uchar GucCacheIndex = 0;
static uint GuiHits = 0;
static uint GuiMisses = 0;


/*********************************************************************************************************
** Function name:       pathCacheInit
** Descriptions:        初始化路径缓存
*********************************************************************************************************/
void pathCacheInit(void)
{
    uchar i;
    for (i = 0; i < CACHE_SIZE; i++) {
        GcacheEntries[i].ucValid = 0;
    }
    GucCacheIndex = 0;
    GuiHits = 0;
    GuiMisses = 0;
}

/*********************************************************************************************************
** Function name:       pathCacheLookup
** Descriptions:        查找缓存路径
*********************************************************************************************************/
uchar pathCacheLookup(char cXStart, char cYStart, char cXGoal, char cYGoal, ASTAR_PATH *path)
{
    uchar i, j;

    for (i = 0; i < CACHE_SIZE; i++) {
        if (GcacheEntries[i].ucValid &&
            GcacheEntries[i].cXStart == cXStart &&
            GcacheEntries[i].cYStart == cYStart &&
            GcacheEntries[i].cXGoal == cXGoal &&
            GcacheEntries[i].cYGoal == cYGoal) {

            path->ucLength = GcacheEntries[i].path.ucLength;
            for (j = 0; j < path->ucLength; j++) {
                path->nodes[j] = GcacheEntries[i].path.nodes[j];
            }

            GuiHits++;
            return 1;
        }
    }

    GuiMisses++;
    return 0;
}

/*********************************************************************************************************
** Function name:       pathCacheStore
** Descriptions:        存储路径到缓存
*********************************************************************************************************/
void pathCacheStore(char cXStart, char cYStart, char cXGoal, char cYGoal, ASTAR_PATH *path)
{
    uchar i;

    GcacheEntries[GucCacheIndex].cXStart = cXStart;
    GcacheEntries[GucCacheIndex].cYStart = cYStart;
    GcacheEntries[GucCacheIndex].cXGoal = cXGoal;
    GcacheEntries[GucCacheIndex].cYGoal = cYGoal;
    GcacheEntries[GucCacheIndex].path.ucLength = path->ucLength;

    for (i = 0; i < path->ucLength; i++) {
        GcacheEntries[GucCacheIndex].path.nodes[i] = path->nodes[i];
    }

    GcacheEntries[GucCacheIndex].ucValid = 1;
    GucCacheIndex = (GucCacheIndex + 1) % CACHE_SIZE;
}

/*********************************************************************************************************
** Function name:       pathCacheClear
** Descriptions:        清空缓存
*********************************************************************************************************/
void pathCacheClear(void)
{
    uchar i;
    for (i = 0; i < CACHE_SIZE; i++) {
        GcacheEntries[i].ucValid = 0;
    }
    GucCacheIndex = 0;
}

/*********************************************************************************************************
** Function name:       pathCacheGetStats
** Descriptions:        获取缓存统计信息
*********************************************************************************************************/
void pathCacheGetStats(CACHE_STATS *stats)
{
    stats->uiHits = GuiHits;
    stats->uiMisses = GuiMisses;

    if ((GuiHits + GuiMisses) > 0) {
        stats->uiHitRate = (GuiHits * 100) / (GuiHits + GuiMisses);
    } else {
        stats->uiHitRate = 0;
    }
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
