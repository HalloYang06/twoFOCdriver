/*
 * mc_math.c — 所有热路径函数已移至 mc_math.h 做 static inline。
 * 本文件仅保留查找表的编译单元引用（表本体在 mc_sin_table.c）。
 * 如果后续需要非内联的辅助函数，可以加在这里。
 */
#include "mc_math.h"
