#ifndef __PLANE_ROUTE_H__
#define __PLANE_ROUTE_H__

#define ABS(x) ((x) > 0 ? (x) : -(x))

#include <stdint.h>

#define BR_configFOLLOWING_SAFE_DISTANCE 500.0f /* 安全距离[mm] */

typedef struct
{
    float (*pPath)[5];    // 二维数组指针
    float NormalP;        // 法向校正P
    float NormalD;        // 法向校正D
    float ThetaP;         // 自转校正P
    float ThetaD;         // 自转校正D
} BR_Path_t;

#endif
