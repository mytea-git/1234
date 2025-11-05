

#ifndef __PATH_CACHE_H
#define __PATH_CACHE_H

#include "astar_config.h"

#define CACHE_SIZE  8

/* 缓存条目结构 */
typedef struct {
    char  cXStart;
    char  cYStart;
    char  cXGoal;
    char  cYGoal;
    uchar ucValid;
    ASTAR_PATH path;
} CACHE_ENTRY;

/* 缓存统计结构 */
typedef struct {
    uint uiHits;                                                            /*  缓存命中次数                */
    uint uiMisses;                                                          /*  缓存未命中次数              */
    uint uiHitRate;                                                         /*  命中率 (百分比)             */
} CACHE_STATS;

/* 外部函数声明 */
void  pathCacheInit(void);
uchar pathCacheLookup(char cXStart, char cYStart, char cXGoal, char cYGoal, ASTAR_PATH *path);
void  pathCacheStore(char cXStart, char cYStart, char cXGoal, char cYGoal, ASTAR_PATH *path);
void  pathCacheClear(void);
void  pathCacheGetStats(CACHE_STATS *stats);

#endif
