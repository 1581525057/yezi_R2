#include <plan_route.h>
#include <math.h>
#include "arm_math.h" // CMSIS-DSP

/* 静态变量 */
static int prvCurrentPathIndex = 0; // 标志当前目标点

/* 用于处理第一次进入逻辑时 D项 LastErr=0 */
static uint8_t prvVelCalculateEnterFlag; // 标志第一次进入正常轨迹速度计算逻辑

void BR_vResetPathFollowing(void);
static uint8_t prvSafeCheck(float *pCurTarPoint, float CurrentX, float CurrentY);
static void prvVelCalculate(BR_Path_t pPathStru, float *Vx, float *Vy, float *Omega, float CurrentX, float CurrentY, float CurrentTheta);


/* -------------------------------------------------- Public ----------------------------------------------- */
void BR_vFollowPath(BR_Path_t pPathStru, float *Vx, float *Vy, float *Omega)
{
    float (*pPath)[5] = pPathStru.pPath;
    float CurrentX = 0;
    float CurrentY = 0;
    float CurrentTheta = 0;

    /* 安全检查 */
    if (prvSafeCheck(pPath[prvCurrentPathIndex], CurrentX, CurrentY)) // 若偏离目标
    {
        BR_vResetPathFollowing();
        return;
    }
    /* 判断当前位置, 根据情况设置控制模式和速度 */
    if (prvCurrentPathIndex == 0) // 起始0速点
        prvCurrentPathIndex++;
    else if (pPath[prvCurrentPathIndex][0] != 0 || pPath[prvCurrentPathIndex][1] != 0) // 大多数情况
    {
        /* 判断是否改变目标点 */
        float TargetX = pPath[prvCurrentPathIndex][2];
        float TargetY = pPath[prvCurrentPathIndex][3];
        float NextTargetX = pPath[prvCurrentPathIndex + 1][2];
        float NextTargetY = pPath[prvCurrentPathIndex + 1][3];
        float DotProduct = (TargetX - CurrentX) * (NextTargetX - TargetX) +
                           (TargetY - CurrentY) * (NextTargetY - TargetY);
        if (DotProduct < 0)
            prvCurrentPathIndex += 1;
        /* 正常情况下的速度计算和设置逻辑 */
        prvVelCalculate(pPathStru, Vx, Vy, Omega, CurrentX, CurrentY, CurrentTheta);
    }

}

/** @brief  重置路径跟随至可以开始执行一次跟随的起始状态 */
void BR_vResetPathFollowing(void)
{
    prvCurrentPathIndex = 0;
    prvVelCalculateEnterFlag = 0;
}

/* -------------------------------------------------- Private ----------------------------------------------- */
/** @brief  检查是否偏离当前路径
 *  @retval 0正常, 1偏离
 **/
static uint8_t prvSafeCheck(float *pCurTarPoint, float CurrentX, float CurrentY)
{
    float SafeDistance;
    if (pCurTarPoint[0] == 0 && pCurTarPoint[1] == 0) // 最终0速点增加一定安全距离
        SafeDistance = BR_configFOLLOWING_SAFE_DISTANCE * 4;
    else
        SafeDistance = BR_configFOLLOWING_SAFE_DISTANCE;
    if (ABS(pCurTarPoint[2] - CurrentX) + ABS(pCurTarPoint[3] - CurrentY) > SafeDistance) // 若离目标点太远
        return 1;
    return 0;
}

/** @brief  一般情况的路径跟随逻辑 */
static void prvVelCalculate(BR_Path_t pPathStru, float *Vx, float *Vy, float *Omega, float CurrentX, float CurrentY, float CurrentTheta)
{
   
    float (*pPath)[5] = pPathStru.pPath;
    // 法向修正速度
    float NormalVelX;
    float NormalVelY;

    static float LastErrTheta;
    static float LastVector4[2];
    /* 主方向分速度计算 */
    float MainVelX = pPath[prvCurrentPathIndex][0];
    float MainVelY = pPath[prvCurrentPathIndex][1];
    /* 法向修正分速度计算 */
    float Vector1[2] = {pPath[prvCurrentPathIndex][2] - CurrentX, // 当前到目标
                        pPath[prvCurrentPathIndex][3] - CurrentY};
    float Vector2[2] = {pPath[prvCurrentPathIndex - 1][2] - pPath[prvCurrentPathIndex][2], // 目标点到前一目标点
                        pPath[prvCurrentPathIndex - 1][3] - pPath[prvCurrentPathIndex][3]};
    float Distance = sqrt(pow(Vector2[0], 2) + pow(Vector2[1], 2));                      // 两目标点间距离
    float Shadow = ABS((Vector1[0] * Vector2[0] + Vector1[1] * Vector2[1]) / Distance);  // 投影长度
    float Vector3[2] = {Vector2[0] / Distance * Shadow, Vector2[1] / Distance * Shadow}; // 目标点到前一目标向量的一部分
    float MainVel = sqrt(pow(MainVelX, 2) + pow(MainVelY, 2));
    float NormalP = pPathStru.NormalP * MainVel;                           // 法向修正的P和当前主速度相关
    float NormalD = pPathStru.NormalD * MainVel;                           // 法向修正的D和当前主速度相关
    float Vector4[2] = {Vector1[0] + Vector3[0], Vector1[1] + Vector3[1]}; // 法向修正向量

    if (prvVelCalculateEnterFlag)                                          // PD调节
    {
        NormalVelX = NormalP * Vector4[0] + NormalD * (Vector4[0] - LastVector4[0]);
        NormalVelY = NormalP * Vector4[1] + NormalD * (Vector4[1] - LastVector4[1]);
    }
    else // 第一次进入逻辑时 P调节
    {
        NormalVelX = NormalP * Vector4[0];
        NormalVelY = NormalP * Vector4[1];
    }
    LastVector4[0] = Vector4[0];
    LastVector4[1] = Vector4[1];
    /* 自转角速度计算 */
    float ErrTheta = pPath[prvCurrentPathIndex][4] - CurrentTheta; //这里看角度为0-360°，千万不能负数
    if (prvVelCalculateEnterFlag) // PD调节
    {
        *Omega = pPathStru.ThetaP * ErrTheta + pPathStru.ThetaD * (ErrTheta - LastErrTheta);
    }
    else // 第一次进入逻辑时 P调节
    {
        *Omega = pPathStru.ThetaP * ErrTheta;
        prvVelCalculateEnterFlag = 1;
    }
    LastErrTheta = ErrTheta;
    /* 速度合成 */
    float VelX = MainVelX + NormalVelX;
    float VelY = MainVelY + NormalVelY;
    *Vx = VelX;
    *Vy = VelY;
}
