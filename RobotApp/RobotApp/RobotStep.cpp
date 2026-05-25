#include "RobotStep.h"

#include <fstream>
#include <iostream>
#include <string>

#include <spdlog/spdlog.h>

namespace robotstep {

/**
 * Dfs全局变量
 */
int        g_time[2];
int        g_StepNum[2];
int        g_TheoryStrStep[2];
TheoryStep g_TheorySteps[25];
TheoryStep g_TheorySteps2[2][25];
Rot        g_CubeRot;
HandState  g_HandState;
int        g_MovBuff[2][120];
/**
 * Dfs存储变量
 */
int       s_time[2];
int       s_StepNum[2];
int       s_MovBuff[2][120];
HandState s_HandState[2];
Rot       s_Rot[2];

/**
 * 标志每一层搜索后的魔方状态
 * step state L_0_R_0  x y z
 */
int book[25][2][3][3][2][3][2][3][2];

/**
 * 机械步骤全局变量
 */
struct MechanicalStep M_L1, M_L2, M_L3, M_LC, M_LO;
struct MechanicalStep M_R1, M_R2, M_R3, M_RC, M_RO;
struct MechanicalStep M_END;

/**
 * 旋转矩阵&全局立方体方位
 */
struct Rot R_x1, R_x2, R_x3, R_y1, R_y2, R_y3, R_z1, R_z2, R_z3;

/**
 * 六个面中心在空间点坐标
 */
struct Point3 P_F, P_R, P_U, P_B, P_L, P_D;
struct Point3 P_FRUBLD[6];

/**
 * 操作库变量
 * FRUBLD  F1、F2、F3  L_0_R_0  每种情况下最多保留16个可行解
 */
MechanicalGroup MechanicalGroupLib[6][3][3][16];

/**
 * 初始化操作库
 */
Rot       tempRot;
HandState tempHandState;

/**
 * 初始化
 */
void allInit(void)
{
    spdlog::trace("robotstep::allInit() running...");
    RobotStepsInit();
    RotInit();
    PointInit();
    OperateLibInit();
    TimeLibInit();
}

/**
 * 搜索程序
 */
int search(const std::string& theoryStr)
{
    // 初始化g_TheoryStrStep
    int TheoryStrLength = theoryStr.length() / 3;
    // 分段搜索
    g_TheoryStrStep[0] = TheoryStrLength;
    g_TheoryStrStep[1] = 0;   // 段长为0
    // 初始化g_TheorySteps,g_TheorySteps2
    for (int i = 0; i < TheoryStrLength; i++) {
        for (int j = 0; j < 3; j++) {
            g_TheorySteps[i].face.a[j][0] = P_FRUBLD[char2Int(theoryStr[i * 3])].a[j][0];
        }
        g_TheorySteps[i].distance = theoryStr[i * 3 + 1] - 0x30 - 1;
    }
    for (int i = 0; i < g_TheoryStrStep[0]; i++) {
        g_TheorySteps2[0][i] = g_TheorySteps[i];
    }
    for (int i = 0; i < g_TheoryStrStep[1]; i++) {
        g_TheorySteps2[1][i] = g_TheorySteps[i + g_TheoryStrStep[0]];
    }
    // 初始化搜索用的变量
    bookInit();
    g_HandState.Set(0, 0, 0, 0);
    g_CubeRot.Set(0, 1, 1, 1, 2, 1);
    g_time[0]    = 0;
    g_time[1]    = 0;
    s_time[0]    = 1000000;
    s_time[1]    = 1000000;
    g_StepNum[0] = 0;
    g_StepNum[1] = 0;
    s_StepNum[0] = 1000;
    s_StepNum[1] = 1000;
    for (int i = 0; i < 120; i++) {
        g_MovBuff[0][i] = -1;
        g_MovBuff[1][i] = -1;
        s_MovBuff[0][i] = -1;
        s_MovBuff[1][i] = -1;
    }
    // 深度搜索
    dfs(0, 0);   // 第一阶段
    g_CubeRot   = s_Rot[0];
    g_HandState = s_HandState[0];
    dfs(0, 1);   // 第二阶段

    return s_StepNum[0] + s_StepNum[1];
}

std::string getSteps()
{
    std::string robotSteps;
    for (int i = 0; i < s_StepNum[0]; i++) {
        robotSteps += moveStr[s_MovBuff[0][i]];
    }
    // spdlog::info("robotSteps : {} steps.", robotSteps.length());
    // spdlog::info("robotSteps : {}", robotSteps);
    return robotSteps;
}

void dfs(int step, int state)
{
    // 到达最深处，判断此时g_time是否比s_stime小
    if (step == g_TheoryStrStep[state]) {
        if (g_time[state] < s_time[state]) {
            s_time[state]    = g_time[state];
            s_StepNum[state] = g_StepNum[state];

            for (int i = 0; i < 120; i++) {
                s_MovBuff[state][i] = g_MovBuff[state][i];
                if (g_MovBuff[state][i] == -1) break;
            }
            s_Rot[state]       = g_CubeRot;
            s_HandState[state] = g_HandState;
            // cout << "state:" << state << "Step: " << g_StepNum[state] << "  Min Time:" <<
            // s_time[state]; cout << "Handstate: " <<
            // g_HandState.LeftNotNice<<g_HandState.RightNotNice << endl;
        }
        return;
    }
    // 获取face
    int face;
    int __i;
    for (__i = 0; __i < 3; ++__i) {
        if (g_TheorySteps2[state][step].face.a[__i][0] != 0) break;
    }
    // face.a[i][0] != 0
    int __j;
    for (__j = 0; __j < 3; ++__j) {
        if (g_CubeRot.a[__j][__i] != 0) break;
    }
    if (__j == 0) {
        if (g_CubeRot.a[__j][__i] == g_TheorySteps2[state][step].face.a[__i][0])
            face = F;
        else
            face = B;
    }
    else if (__j == 1) {
        if (g_CubeRot.a[__j][__i] == g_TheorySteps2[state][step].face.a[__i][0])
            face = R;
        else
            face = L;
    }
    else if (__j == 2) {
        if (g_CubeRot.a[__j][__i] == g_TheorySteps2[state][step].face.a[__i][0])
            face = U;
        else
            face = D;
    }
    // 获取face结束

    int j = g_TheorySteps2[state][step].distance;
    int k = g_HandState.LeftNotNice * 2 + g_HandState.RightNotNice;   // 注意此处二进制关系
    for (int l = 0; l < 16; l++) {
        // 保存当前状态
        Rot       _tempRot       = g_CubeRot;
        HandState _tempHandState = g_HandState;
        int       tempStepNum    = g_StepNum[state];
        int       tempTime       = g_time[state];
        int       tempMoveBuff[120]{};
        for (int _i = 0; _i < 120; _i++) {
            tempMoveBuff[_i] = g_MovBuff[state][_i];
            if (g_MovBuff[state][_i] == -1) break;   // 注意此处已经赋值了一个-1
        }
        // 加入本次节点
        g_time[state] += MechanicalGroupLib[face][j][k][l].time;
        g_CubeRot   = RotMtplRot(MechanicalGroupLib[face][j][k][l].rot, g_CubeRot);
        g_HandState = MechanicalGroupLib[face][j][k][l].endHandState;
        for (int _i = 0; _i < MechanicalGroupLib[face][j][k][l].StepNum; _i++) {
            g_MovBuff[state][g_StepNum[state] + _i] =
                MechanicalGroupLib[face][j][k][l].Steps[_i].num;
        }
        g_StepNum[state] += MechanicalGroupLib[face][j][k][l].StepNum;
        // 查看此结果在此深度下有没有发生过
        int row[3]{}, num[3]{};
        for (int _i = 0; _i < 3; _i++)   // 3列
        {
            for (row[_i] = 0; row[_i] < 3; row[_i]++) {
                if (g_CubeRot.a[row[_i]][_i] != 0) {
                    if (g_CubeRot.a[row[_i]][_i] == -1)
                        num[_i] = 0;
                    else if (g_CubeRot.a[row[_i]][_i] == 1)
                        num[_i] = 1;
                    else
                        num[_i] = -1;
                    break;
                }
            }
        }
        int hand = g_HandState.LeftNotNice * 2 + g_HandState.RightNotNice;   // 注意此处二进制关系
        if (g_time[state] < book[step][state][hand][row[0]][num[0]][row[1]][num[1]][row[2]]
                                [num[2]])   // 当前时间比历史最短时间短
        {
            book[step][state][hand][row[0]][num[0]][row[1]][num[1]][row[2]][num[2]] = g_time[state];
            // 深搜
            dfs(step + 1, state);
            // 复原之前保存的状态
            g_CubeRot        = _tempRot;
            g_HandState      = _tempHandState;
            g_time[state]    = tempTime;
            g_StepNum[state] = tempStepNum;
            for (int _i = 0; _i < 120; _i++) {
                g_MovBuff[state][_i] = tempMoveBuff[_i];
                if (tempMoveBuff[_i] == -1) break;
            }
        }
        else {
            // 复原之前保存的状态
            g_CubeRot        = _tempRot;
            g_HandState      = _tempHandState;
            g_time[state]    = tempTime;
            g_StepNum[state] = tempStepNum;
            for (int _i = 0; _i < 120; _i++) {
                g_MovBuff[state][_i] = tempMoveBuff[_i];
                if (tempMoveBuff[_i] == -1) break;
            }
        }
        // 查看此结果在此深度下有没有发生过结束
    }
}

/**
 *
 */
void bookInit(void)
{
    // book[25][3][3][2][3][2][3][2];
    for (int i = 0; i < 25; i++)
        for (int state = 0; state < 2; state++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    for (int l = 0; l < 2; l++)
                        for (int m = 0; m < 3; m++)
                            for (int n = 0; n < 2; n++)
                                for (int o = 0; o < 3; o++)
                                    for (int p = 0; p < 2; p++) {
                                        book[i][state][j][k][l][m][n][o][p] = 1000000;
                                    }
}

/**
 * 字符转换
 */
int char2Int(char inChar)
{
    switch (inChar) {
    case 'F': return F; break;
    case 'R': return R; break;
    case 'U': return U; break;
    case 'B': return B; break;
    case 'L': return L; break;
    case 'D': return D; break;
    default: return -1;
    }
    return -1;
}

/**
 * 机械步骤初始化
 */
void RobotStepsInit(void)
{
    // name
    M_L1.name = "L1";
    M_L2.name = "L2";
    M_L3.name = "L3";
    M_LC.name = "LC";
    M_LO.name = "LO";

    M_R1.name  = "R1";
    M_R2.name  = "R2";
    M_R3.name  = "R3";
    M_RC.name  = "RC";
    M_RO.name  = "RO";
    M_END.name = "M_END";

    // time
    M_L1.time = HandMove90;
    M_L2.time = HandMove180;
    M_L3.time = HandMove90;
    M_LC.time = HandCloseTime;
    M_LO.time = HandOpenTime;

    M_R1.time  = HandMove90;
    M_R2.time  = HandMove180;
    M_R3.time  = HandMove90;
    M_RC.time  = HandCloseTime;
    M_RO.time  = HandOpenTime;
    M_END.time = 0;

    // num
    M_L1.num = L1;
    M_L2.num = L2;
    M_L3.num = L3;
    M_LC.num = LC;
    M_LO.num = LO;

    M_R1.num  = R1;
    M_R2.num  = R2;
    M_R3.num  = R3;
    M_RC.num  = RC;
    M_RO.num  = RO;
    M_END.num = -1;
}

/**
 * 旋转矩阵初始化
 */
void RotInit(void)
{
    R_x1.a[0][0] = 1;
    R_x1.a[2][1] = -1;
    R_x1.a[1][2] = 1;
    R_x2         = RotMtplRot(R_x1, R_x1);
    R_x3         = RotMtplRot(R_x1, R_x2);

    R_y1.a[2][0] = 1;
    R_y1.a[1][1] = 1;
    R_y1.a[0][2] = -1;
    R_y2         = RotMtplRot(R_y1, R_y1);
    R_y3         = RotMtplRot(R_y1, R_y2);

    R_z1.a[1][0] = -1;
    R_z1.a[0][1] = 1;
    R_z1.a[2][2] = 1;
    R_z2         = RotMtplRot(R_z1, R_z1);
    R_z2         = RotMtplRot(R_z1, R_z2);
}

/**
 * 六个面中心在空间点坐标初始化
 */
void PointInit(void)
{
    P_F.a[0][0] = 1;
    P_F.a[1][0] = 0;
    P_F.a[2][0] = 0;
    P_F.name    = "F";
    P_FRUBLD[F] = P_F;

    P_R.a[0][0] = 0;
    P_R.a[1][0] = 1;
    P_R.a[2][0] = 0;
    P_R.name    = "R";
    P_FRUBLD[R] = P_R;

    P_U.a[0][0] = 0;
    P_U.a[1][0] = 0;
    P_U.a[2][0] = 1;
    P_U.name    = "U";
    P_FRUBLD[U] = P_U;

    P_B.a[0][0] = -1;
    P_B.a[1][0] = 0;
    P_B.a[2][0] = 0;
    P_B.name    = "B";
    P_FRUBLD[B] = P_B;

    P_L.a[0][0] = 0;
    P_L.a[1][0] = -1;
    P_L.a[2][0] = 0;
    P_L.name    = "L";
    P_FRUBLD[L] = P_L;

    P_D.a[0][0] = 0;
    P_D.a[1][0] = 0;
    P_D.a[2][0] = -1;
    P_D.name    = "D";
    P_FRUBLD[D] = P_D;
}

/**
 * 矩阵运算
 */
Rot RotMtplRot(Rot l, Rot r)
{
    Rot temp{};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp.a[i][j] = 0;
        }
    }
    for (int k = 0; k < 3; k++) {
        int j;
        for (j = 0; j < 3; j++) {
            if (l.a[k][j] != 0) break;
        }
        int i;
        for (i = 0; i < 3; i++) {
            if (r.a[j][i] != 0) break;
        }
        if (l.a[k][j] == r.a[j][i])
            temp.a[k][i] = 1;
        else
            temp.a[k][i] = -1;
    }
    return temp;
}
Point3 RotMtplPoint3(Rot l, Point3 r)
{
    Point3 temp;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 1; j++) {
            temp.a[i][j] = l.a[i][0] * r.a[0][j] + l.a[i][1] * r.a[1][j] + l.a[i][2] * r.a[2][j];
        }
    }
    return temp;
}

/**
 * 初始化操作库中各个操作组合的总时间
 */
void TimeLibInit(void)
{
    // MechanicalGroupLib[F][_1][L_0_R_0][0]
    for (int i = F; i <= D; i++) {
        for (int j = _1; j <= _3; j++) {
            for (int k = L_0_R_0; k <= L_1_R_0; k++) {
                for (int l = 0; l < 16; l++) {
                    // MechanicalGroupLib[i][j][k][l]
                    MechanicalGroupLib[i][j][k][l].time = 0;
                    int LeftHand                        = CLOSE;
                    int RightHand                       = CLOSE;
                    for (int m = 0; MechanicalGroupLib[i][j][k][l].Steps[m].num != -1; m++) {
                        // 气缸动
                        if (MechanicalGroupLib[i][j][k][l].Steps[m].num == LO) {
                            LeftHand = OPEN;
                            MechanicalGroupLib[i][j][k][l].time += Time_Air;
                        }
                        else if (MechanicalGroupLib[i][j][k][l].Steps[m].num == LC) {
                            LeftHand = CLOSE;
                        }
                        else if (MechanicalGroupLib[i][j][k][l].Steps[m].num == RO) {
                            RightHand = OPEN;
                            MechanicalGroupLib[i][j][k][l].time += Time_Air;
                        }
                        else if (MechanicalGroupLib[i][j][k][l].Steps[m].num == RC) {
                            RightHand = CLOSE;
                        }
                        else   // 电机动
                        {
                            // 拧动
                            if ((RightHand == CLOSE) && (LeftHand == CLOSE)) {
                                if ((MechanicalGroupLib[i][j][k][l].Steps[m].num == L2) ||
                                    (MechanicalGroupLib[i][j][k][l].Steps[m].num == R2)) {
                                    MechanicalGroupLib[i][j][k][l].time += Tim_ND180;
                                }
                                else {
                                    MechanicalGroupLib[i][j][k][l].time += Tim_ND90;
                                }
                            }
                            // 空转
                            else if ((MechanicalGroupLib[i][j][k][l].Steps[m].num == L1) ||
                                     (MechanicalGroupLib[i][j][k][l].Steps[m].num == L3) &&
                                         ((RightHand == CLOSE) && (LeftHand == OPEN))) {
                                MechanicalGroupLib[i][j][k][l].time += Tim_KZ90;
                            }
                            else if ((MechanicalGroupLib[i][j][k][l].Steps[m].num == R1) ||
                                     (MechanicalGroupLib[i][j][k][l].Steps[m].num == R3) &&
                                         ((RightHand == OPEN) && (LeftHand == CLOSE))) {
                                MechanicalGroupLib[i][j][k][l].time += Tim_KZ90;
                            }
                            // 带动
                            else if ((MechanicalGroupLib[i][j][k][l].Steps[m].num == L1) ||
                                     (MechanicalGroupLib[i][j][k][l].Steps[m].num == L3) &&
                                         ((RightHand == OPEN) && (LeftHand == CLOSE))) {
                                MechanicalGroupLib[i][j][k][l].time += Tim_DD90;
                            }
                            else if ((MechanicalGroupLib[i][j][k][l].Steps[m].num == R1) ||
                                     (MechanicalGroupLib[i][j][k][l].Steps[m].num == R3) &&
                                         ((RightHand == CLOSE) && (LeftHand == OPEN))) {
                                MechanicalGroupLib[i][j][k][l].time += Tim_DD90;
                            }
                            else if ((MechanicalGroupLib[i][j][k][l].Steps[m].num == L2) &&
                                     ((RightHand == OPEN) && (LeftHand == CLOSE))) {
                                MechanicalGroupLib[i][j][k][l].time += Tim_DD180;
                            }
                            else if ((MechanicalGroupLib[i][j][k][l].Steps[m].num == R2) &&
                                     ((RightHand == CLOSE) && (LeftHand == OPEN))) {
                                MechanicalGroupLib[i][j][k][l].time += Tim_DD180;
                            }
                        }
                    }
                }
            }
        }
    }
}

/**
 * 操作库初始化 谨慎改动
 */
void OperateLibInit(void)
{
    F1_L0R0Init();
    F1_L0R1Init();
    F1_L1R0Init();
    F2_L0R0Init();
    F2_L0R1Init();
    F2_L1R0Init();
    F3_L0R0Init();
    F3_L0R1Init();
    F3_L1R0Init();
    R1_L0R0Init();
    R1_L0R1Init();
    R1_L1R0Init();
    R2_L0R0Init();
    R2_L0R1Init();
    R2_L1R0Init();
    R3_L0R0Init();
    R3_L0R1Init();
    R3_L1R0Init();
    U1_L0R0Init();
    U1_L0R1Init();
    U1_L1R0Init();
    U2_L0R0Init();
    U2_L0R1Init();
    U2_L1R0Init();
    U3_L0R0Init();
    U3_L0R1Init();
    U3_L1R0Init();
    B1_L0R0Init();
    B1_L0R1Init();
    B1_L1R0Init();
    B2_L0R0Init();
    B2_L0R1Init();
    B2_L1R0Init();
    B3_L0R0Init();
    B3_L0R1Init();
    B3_L1R0Init();
    L1_L0R0Init();
    L1_L0R1Init();
    L1_L1R0Init();
    L2_L0R0Init();
    L2_L0R1Init();
    L2_L1R0Init();
    L3_L0R0Init();
    L3_L0R1Init();
    L3_L1R0Init();
    D1_L0R0Init();
    D1_L0R1Init();
    D1_L1R0Init();
    D2_L0R0Init();
    D2_L0R1Init();
    D2_L1R0Init();
    D3_L0R0Init();
    D3_L0R1Init();
    D3_L1R0Init();
}

void F1_L0R0Init(void)
{
    // F1_L0R0_0
    MechanicalStep F1_L0R0_0[] = {M_L1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][0].Set(1, F1_L0R0_0, tempRot, tempHandState);
    // F1_L0R0_1
    MechanicalStep F1_L0R0_1[] = {M_LO, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][1].Set(4, F1_L0R0_1, tempRot, tempHandState);
    // F1_L0R0_2
    MechanicalStep F1_L0R0_2[] = {M_RO, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][2].Set(4, F1_L0R0_2, tempRot, tempHandState);
    // F1_L0R0_3
    MechanicalStep F1_L0R0_3[] = {M_RO, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][3].Set(4, F1_L0R0_3, tempRot, tempHandState);
    // F1_L0R0_4
    MechanicalStep F1_L0R0_4[] = {M_RO, M_L2, M_RC, M_L1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][4].Set(4, F1_L0R0_4, tempRot, tempHandState);
    // F1_L0R0_5
    MechanicalStep F1_L0R0_5[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_0_R_0][5].Set(11, F1_L0R0_5, tempRot, tempHandState);
    // F1_L0R0_6
    MechanicalStep F1_L0R0_6[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_0_R_0][6].Set(11, F1_L0R0_6, tempRot, tempHandState);
    // F1_L0R0_7
    MechanicalStep F1_L0R0_7[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][7].Set(12, F1_L0R0_7, tempRot, tempHandState);
    // F1_L0R0_8
    MechanicalStep F1_L0R0_8[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_L3, M_LC, M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][8].Set(12, F1_L0R0_8, tempRot, tempHandState);
    // F1_L0R0_9
    MechanicalStep F1_L0R0_9[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][9].Set(12, F1_L0R0_9, tempRot, tempHandState);
    // F1_L0R0_10
    MechanicalStep F1_L0R0_10[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][10].Set(12, F1_L0R0_10, tempRot, tempHandState);
    // F1_L0R0_11
    MechanicalStep F1_L0R0_11[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][11].Set(13, F1_L0R0_11, tempRot, tempHandState);
    // F1_L0R0_12
    MechanicalStep F1_L0R0_12[] = {M_LO,
                                   M_R2,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][12].Set(15, F1_L0R0_12, tempRot, tempHandState);
    // F1_L0R0_13
    MechanicalStep F1_L0R0_13[] = {M_LO,
                                   M_R2,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_0_R_0][13].Set(15, F1_L0R0_13, tempRot, tempHandState);
    // F1_L0R0_14
    MechanicalStep F1_L0R0_14[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R1,
                                   M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_0_R_0][14].Set(16, F1_L0R0_14, tempRot, tempHandState);
    // F1_L0R0_15
    MechanicalStep F1_L0R0_15[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R1,
                                   M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_0_R_0][15].Set(16, F1_L0R0_15, tempRot, tempHandState);
}
void F2_L0R0Init(void)
{
    // F2_L0R0_0
    MechanicalStep F2_L0R0_0[] = {M_L2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][0].Set(1, F2_L0R0_0, tempRot, tempHandState);
    // F2_L0R0_1
    MechanicalStep F2_L0R0_1[] = {M_RO, M_L2, M_RC, M_L2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][1].Set(4, F2_L0R0_1, tempRot, tempHandState);
    // F2_L0R0_2
    MechanicalStep F2_L0R0_2[] = {M_LO, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][2].Set(4, F2_L0R0_2, tempRot, tempHandState);
    // F2_L0R0_3
    MechanicalStep F2_L0R0_3[] = {M_RO, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][3].Set(4, F2_L0R0_3, tempRot, tempHandState);
    // F2_L0R0_4
    MechanicalStep F2_L0R0_4[] = {M_RO, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][4].Set(4, F2_L0R0_4, tempRot, tempHandState);
    // F2_L0R0_5
    MechanicalStep F2_L0R0_5[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][5].Set(11, F2_L0R0_5, tempRot, tempHandState);
    // F2_L0R0_6
    MechanicalStep F2_L0R0_6[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][6].Set(11, F2_L0R0_6, tempRot, tempHandState);
    // F2_L0R0_7
    MechanicalStep F2_L0R0_7[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_L3, M_LC, M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_0_R_0][7].Set(12, F2_L0R0_7, tempRot, tempHandState);
    // F2_L0R0_8
    MechanicalStep F2_L0R0_8[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_0_R_0][8].Set(12, F2_L0R0_8, tempRot, tempHandState);
    // F2_L0R0_9
    MechanicalStep F2_L0R0_9[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_0_R_0][9].Set(12, F2_L0R0_9, tempRot, tempHandState);
    // F2_L0R0_10
    MechanicalStep F2_L0R0_10[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_0_R_0][10].Set(12, F2_L0R0_10, tempRot, tempHandState);
    // F2_L0R0_11
    MechanicalStep F2_L0R0_11[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][11].Set(13, F2_L0R0_11, tempRot, tempHandState);
    // F2_L0R0_12
    MechanicalStep F2_L0R0_12[] = {M_LO,
                                   M_R2,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][12].Set(15, F2_L0R0_12, tempRot, tempHandState);
    // F2_L0R0_13
    MechanicalStep F2_L0R0_13[] = {M_LO,
                                   M_R2,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][13].Set(15, F2_L0R0_13, tempRot, tempHandState);
    // F2_L0R0_14
    MechanicalStep F2_L0R0_14[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R2,
                                   M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][14].Set(16, F2_L0R0_14, tempRot, tempHandState);
    // F2_L0R0_15
    MechanicalStep F2_L0R0_15[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R2,
                                   M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_0][15].Set(16, F2_L0R0_15, tempRot, tempHandState);
}
void F3_L0R0Init(void)
{
    // F3_L0R0_0
    MechanicalStep F3_L0R0_0[] = {M_L3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][0].Set(1, F3_L0R0_0, tempRot, tempHandState);
    // F3_L0R0_1
    MechanicalStep F3_L0R0_1[] = {M_LO, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][1].Set(4, F3_L0R0_1, tempRot, tempHandState);
    // F3_L0R0_2
    MechanicalStep F3_L0R0_2[] = {M_RO, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][2].Set(4, F3_L0R0_2, tempRot, tempHandState);
    // F3_L0R0_3
    MechanicalStep F3_L0R0_3[] = {M_RO, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][3].Set(4, F3_L0R0_3, tempRot, tempHandState);
    // F3_L0R0_4
    MechanicalStep F3_L0R0_4[] = {M_RO, M_L2, M_RC, M_L3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][4].Set(4, F3_L0R0_4, tempRot, tempHandState);
    // F3_L0R0_5
    MechanicalStep F3_L0R0_5[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_0_R_0][5].Set(11, F3_L0R0_5, tempRot, tempHandState);
    // F3_L0R0_6
    MechanicalStep F3_L0R0_6[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_0_R_0][6].Set(11, F3_L0R0_6, tempRot, tempHandState);
    // F3_L0R0_7
    MechanicalStep F3_L0R0_7[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_L3, M_LC, M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][7].Set(12, F3_L0R0_7, tempRot, tempHandState);
    // F3_L0R0_8
    MechanicalStep F3_L0R0_8[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][8].Set(12, F3_L0R0_8, tempRot, tempHandState);
    // F3_L0R0_9
    MechanicalStep F3_L0R0_9[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][9].Set(12, F3_L0R0_9, tempRot, tempHandState);
    // F3_L0R0_10
    MechanicalStep F3_L0R0_10[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][10].Set(12, F3_L0R0_10, tempRot, tempHandState);
    // F3_L0R0_11
    MechanicalStep F3_L0R0_11[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][11].Set(13, F3_L0R0_11, tempRot, tempHandState);
    // F3_L0R0_12
    MechanicalStep F3_L0R0_12[] = {M_LO,
                                   M_R2,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][12].Set(15, F3_L0R0_12, tempRot, tempHandState);
    // F3_L0R0_13
    MechanicalStep F3_L0R0_13[] = {M_LO,
                                   M_R2,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_0_R_0][13].Set(15, F3_L0R0_13, tempRot, tempHandState);
    // F3_L0R0_14
    MechanicalStep F3_L0R0_14[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R3,
                                   M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_0_R_0][14].Set(16, F3_L0R0_14, tempRot, tempHandState);
    // F3_L0R0_15
    MechanicalStep F3_L0R0_15[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R3,
                                   M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_0_R_0][15].Set(16, F3_L0R0_15, tempRot, tempHandState);
}
void R1_L0R0Init(void)
{
    // R1_L0R0_0
    MechanicalStep R1_L0R0_0[] = {M_R1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_0_R_0][0].Set(1, R1_L0R0_0, tempRot, tempHandState);
    // R1_L0R0_1
    MechanicalStep R1_L0R0_1[] = {M_LO, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][1].Set(4, R1_L0R0_1, tempRot, tempHandState);
    // R1_L0R0_2
    MechanicalStep R1_L0R0_2[] = {M_LO, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][2].Set(4, R1_L0R0_2, tempRot, tempHandState);
    // R1_L0R0_3
    MechanicalStep R1_L0R0_3[] = {M_RO, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][3].Set(4, R1_L0R0_3, tempRot, tempHandState);
    // R1_L0R0_4
    MechanicalStep R1_L0R0_4[] = {M_LO, M_R2, M_LC, M_R1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_0_R_0][4].Set(4, R1_L0R0_4, tempRot, tempHandState);
    // R1_L0R0_5
    MechanicalStep R1_L0R0_5[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][5].Set(11, R1_L0R0_5, tempRot, tempHandState);
    // R1_L0R0_6
    MechanicalStep R1_L0R0_6[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][6].Set(11, R1_L0R0_6, tempRot, tempHandState);
    // R1_L0R0_7
    MechanicalStep R1_L0R0_7[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][7].Set(12, R1_L0R0_7, tempRot, tempHandState);
    // R1_L0R0_8
    MechanicalStep R1_L0R0_8[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][8].Set(12, R1_L0R0_8, tempRot, tempHandState);
    // R1_L0R0_9
    MechanicalStep R1_L0R0_9[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][9].Set(12, R1_L0R0_9, tempRot, tempHandState);
    // R1_L0R0_10
    MechanicalStep R1_L0R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][10].Set(12, R1_L0R0_10, tempRot, tempHandState);
    // R1_L0R0_11
    MechanicalStep R1_L0R0_11[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][11].Set(13, R1_L0R0_11, tempRot, tempHandState);
    // R1_L0R0_12
    MechanicalStep R1_L0R0_12[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R1,
                                   M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_0_R_0][12].Set(15, R1_L0R0_12, tempRot, tempHandState);
    // R1_L0R0_13
    MechanicalStep R1_L0R0_13[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R1,
                                   M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_0_R_0][13].Set(15, R1_L0R0_13, tempRot, tempHandState);
    // R1_L0R0_14
    MechanicalStep R1_L0R0_14[] = {M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R2,
                                   M_LC,
                                   M_L1,
                                   M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][14].Set(16, R1_L0R0_14, tempRot, tempHandState);
    // R1_L0R0_15
    MechanicalStep R1_L0R0_15[] = {M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R2,
                                   M_LC,
                                   M_L1,
                                   M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_0_R_0][15].Set(16, R1_L0R0_15, tempRot, tempHandState);
}
void R2_L0R0Init(void)
{
    // R2_L0R0_0
    MechanicalStep R2_L0R0_0[] = {M_R2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][0].Set(1, R2_L0R0_0, tempRot, tempHandState);
    // R2_L0R0_1
    MechanicalStep R2_L0R0_1[] = {M_LO, M_R2, M_LC, M_R2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][1].Set(4, R2_L0R0_1, tempRot, tempHandState);
    // R2_L0R0_2
    MechanicalStep R2_L0R0_2[] = {M_LO, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_0_R_0][2].Set(4, R2_L0R0_2, tempRot, tempHandState);
    // R2_L0R0_3
    MechanicalStep R2_L0R0_3[] = {M_LO, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_0_R_0][3].Set(4, R2_L0R0_3, tempRot, tempHandState);
    // R2_L0R0_4
    MechanicalStep R2_L0R0_4[] = {M_RO, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_0_R_0][4].Set(4, R2_L0R0_4, tempRot, tempHandState);
    // R2_L0R0_5
    MechanicalStep R2_L0R0_5[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][5].Set(11, R2_L0R0_5, tempRot, tempHandState);
    // R2_L0R0_6
    MechanicalStep R2_L0R0_6[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][6].Set(11, R2_L0R0_6, tempRot, tempHandState);
    // R2_L0R0_7
    MechanicalStep R2_L0R0_7[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][7].Set(12, R2_L0R0_7, tempRot, tempHandState);
    // R2_L0R0_8
    MechanicalStep R2_L0R0_8[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][8].Set(12, R2_L0R0_8, tempRot, tempHandState);
    // R2_L0R0_9
    MechanicalStep R2_L0R0_9[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][9].Set(12, R2_L0R0_9, tempRot, tempHandState);
    // R2_L0R0_10
    MechanicalStep R2_L0R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][10].Set(12, R2_L0R0_10, tempRot, tempHandState);
    // R2_L0R0_11
    MechanicalStep R2_L0R0_11[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_0_R_0][11].Set(13, R2_L0R0_11, tempRot, tempHandState);
    // R2_L0R0_12
    MechanicalStep R2_L0R0_12[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R2,
                                   M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][12].Set(15, R2_L0R0_12, tempRot, tempHandState);
    // R2_L0R0_13
    MechanicalStep R2_L0R0_13[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R2,
                                   M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][13].Set(15, R2_L0R0_13, tempRot, tempHandState);
    // R2_L0R0_14
    MechanicalStep R2_L0R0_14[] = {M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R2,
                                   M_LC,
                                   M_L2,
                                   M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][14].Set(16, R2_L0R0_14, tempRot, tempHandState);
    // R2_L0R0_15
    MechanicalStep R2_L0R0_15[] = {M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R2,
                                   M_LC,
                                   M_L2,
                                   M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_0][15].Set(16, R2_L0R0_15, tempRot, tempHandState);
}
void R3_L0R0Init(void)
{
    // R3_L0R0_0
    MechanicalStep R3_L0R0_0[] = {M_R3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_0_R_0][0].Set(1, R3_L0R0_0, tempRot, tempHandState);
    // R3_L0R0_1
    MechanicalStep R3_L0R0_1[] = {M_LO, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][1].Set(4, R3_L0R0_1, tempRot, tempHandState);
    // R3_L0R0_2
    MechanicalStep R3_L0R0_2[] = {M_LO, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][2].Set(4, R3_L0R0_2, tempRot, tempHandState);
    // R3_L0R0_3
    MechanicalStep R3_L0R0_3[] = {M_RO, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][3].Set(4, R3_L0R0_3, tempRot, tempHandState);
    // R3_L0R0_4
    MechanicalStep R3_L0R0_4[] = {M_LO, M_R2, M_LC, M_R3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_0_R_0][4].Set(4, R3_L0R0_4, tempRot, tempHandState);
    // R3_L0R0_5
    MechanicalStep R3_L0R0_5[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][5].Set(11, R3_L0R0_5, tempRot, tempHandState);
    // R3_L0R0_6
    MechanicalStep R3_L0R0_6[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][6].Set(11, R3_L0R0_6, tempRot, tempHandState);
    // R3_L0R0_7
    MechanicalStep R3_L0R0_7[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][7].Set(12, R3_L0R0_7, tempRot, tempHandState);
    // R3_L0R0_8
    MechanicalStep R3_L0R0_8[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][8].Set(12, R3_L0R0_8, tempRot, tempHandState);
    // R3_L0R0_9
    MechanicalStep R3_L0R0_9[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][9].Set(12, R3_L0R0_9, tempRot, tempHandState);
    // R3_L0R0_10
    MechanicalStep R3_L0R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][10].Set(12, R3_L0R0_10, tempRot, tempHandState);
    // R3_L0R0_11
    MechanicalStep R3_L0R0_11[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][11].Set(13, R3_L0R0_11, tempRot, tempHandState);
    // R3_L0R0_12
    MechanicalStep R3_L0R0_12[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R3,
                                   M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_0_R_0][12].Set(15, R3_L0R0_12, tempRot, tempHandState);
    // R3_L0R0_13
    MechanicalStep R3_L0R0_13[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L2,
                                   M_RC,
                                   M_R3,
                                   M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_0_R_0][13].Set(15, R3_L0R0_13, tempRot, tempHandState);
    // R3_L0R0_14
    MechanicalStep R3_L0R0_14[] = {M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R2,
                                   M_LC,
                                   M_L3,
                                   M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][14].Set(16, R3_L0R0_14, tempRot, tempHandState);
    // R3_L0R0_15
    MechanicalStep R3_L0R0_15[] = {M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R2,
                                   M_LC,
                                   M_L3,
                                   M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_0_R_0][15].Set(16, R3_L0R0_15, tempRot, tempHandState);
}
void U1_L0R0Init(void)
{
    // U1_L0R0_0
    MechanicalStep U1_L0R0_0[] = {M_LO, M_L1, M_LC, M_RO, M_L1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_0_R_0][0].Set(7, U1_L0R0_0, tempRot, tempHandState);
    // U1_L0R0_1
    MechanicalStep U1_L0R0_1[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][1].Set(7, U1_L0R0_1, tempRot, tempHandState);
    // U1_L0R0_2
    MechanicalStep U1_L0R0_2[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][2].Set(8, U1_L0R0_2, tempRot, tempHandState);
    // U1_L0R0_3
    MechanicalStep U1_L0R0_3[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][3].Set(8, U1_L0R0_3, tempRot, tempHandState);
    // U1_L0R0_4
    MechanicalStep U1_L0R0_4[] = {M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][4].Set(8, U1_L0R0_4, tempRot, tempHandState);
    // U1_L0R0_5
    MechanicalStep U1_L0R0_5[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][5].Set(8, U1_L0R0_5, tempRot, tempHandState);
    // U1_L0R0_6
    MechanicalStep U1_L0R0_6[] = {M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][6].Set(8, U1_L0R0_6, tempRot, tempHandState);
    // U1_L0R0_7
    MechanicalStep U1_L0R0_7[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][7].Set(8, U1_L0R0_7, tempRot, tempHandState);
    // U1_L0R0_8
    MechanicalStep U1_L0R0_8[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L3, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_0_R_0][8].Set(8, U1_L0R0_8, tempRot, tempHandState);
    // U1_L0R0_9
    MechanicalStep U1_L0R0_9[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_L1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][9].Set(8, U1_L0R0_9, tempRot, tempHandState);
    // U1_L0R0_10
    MechanicalStep U1_L0R0_10[] = {M_RO, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][10].Set(9, U1_L0R0_10, tempRot, tempHandState);
    // U1_L0R0_11
    MechanicalStep U1_L0R0_11[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][11].Set(9, U1_L0R0_11, tempRot, tempHandState);
    // U1_L0R0_12
    MechanicalStep U1_L0R0_12[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][12].Set(12, U1_L0R0_12, tempRot, tempHandState);
    // U1_L0R0_13
    MechanicalStep U1_L0R0_13[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_0_R_0][13].Set(12, U1_L0R0_13, tempRot, tempHandState);
    // U1_L0R0_14
    MechanicalStep U1_L0R0_14[] = {
        M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_0_R_0][14].Set(12, U1_L0R0_14, tempRot, tempHandState);
    // U1_L0R0_15
    MechanicalStep U1_L0R0_15[] = {
        M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_0_R_0][15].Set(12, U1_L0R0_15, tempRot, tempHandState);
}
void U2_L0R0Init(void)
{
    // U2_L0R0_0
    MechanicalStep U2_L0R0_0[] = {M_LO, M_L1, M_LC, M_RO, M_L1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][0].Set(7, U2_L0R0_0, tempRot, tempHandState);
    // U2_L0R0_1
    MechanicalStep U2_L0R0_1[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][1].Set(7, U2_L0R0_1, tempRot, tempHandState);
    // U2_L0R0_2
    MechanicalStep U2_L0R0_2[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_L2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][2].Set(8, U2_L0R0_2, tempRot, tempHandState);
    // U2_L0R0_3
    MechanicalStep U2_L0R0_3[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L3, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][3].Set(8, U2_L0R0_3, tempRot, tempHandState);
    // U2_L0R0_4
    MechanicalStep U2_L0R0_4[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_0_R_0][4].Set(8, U2_L0R0_4, tempRot, tempHandState);
    // U2_L0R0_5
    MechanicalStep U2_L0R0_5[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][5].Set(8, U2_L0R0_5, tempRot, tempHandState);
    // U2_L0R0_6
    MechanicalStep U2_L0R0_6[] = {M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_0_R_0][6].Set(8, U2_L0R0_6, tempRot, tempHandState);
    // U2_L0R0_7
    MechanicalStep U2_L0R0_7[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][7].Set(8, U2_L0R0_7, tempRot, tempHandState);
    // U2_L0R0_8
    MechanicalStep U2_L0R0_8[] = {M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][8].Set(8, U2_L0R0_8, tempRot, tempHandState);
    // U2_L0R0_9
    MechanicalStep U2_L0R0_9[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_0_R_0][9].Set(8, U2_L0R0_9, tempRot, tempHandState);
    // U2_L0R0_10
    MechanicalStep U2_L0R0_10[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_0_R_0][10].Set(9, U2_L0R0_10, tempRot, tempHandState);
    // U2_L0R0_11
    MechanicalStep U2_L0R0_11[] = {M_RO, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][11].Set(9, U2_L0R0_11, tempRot, tempHandState);
    // U2_L0R0_12
    MechanicalStep U2_L0R0_12[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][12].Set(12, U2_L0R0_12, tempRot, tempHandState);
    // U2_L0R0_13
    MechanicalStep U2_L0R0_13[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][13].Set(12, U2_L0R0_13, tempRot, tempHandState);
    // U2_L0R0_14
    MechanicalStep U2_L0R0_14[] = {
        M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][14].Set(12, U2_L0R0_14, tempRot, tempHandState);
    // U2_L0R0_15
    MechanicalStep U2_L0R0_15[] = {
        M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_0][15].Set(12, U2_L0R0_15, tempRot, tempHandState);
}
void U3_L0R0Init(void)
{
    // U3_L0R0_0
    MechanicalStep U3_L0R0_0[] = {M_LO, M_L1, M_LC, M_RO, M_L1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_0_R_0][0].Set(7, U3_L0R0_0, tempRot, tempHandState);
    // U3_L0R0_1
    MechanicalStep U3_L0R0_1[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][1].Set(7, U3_L0R0_1, tempRot, tempHandState);
    // U3_L0R0_2
    MechanicalStep U3_L0R0_2[] = {M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][2].Set(8, U3_L0R0_2, tempRot, tempHandState);
    // U3_L0R0_3
    MechanicalStep U3_L0R0_3[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][3].Set(8, U3_L0R0_3, tempRot, tempHandState);
    // U3_L0R0_4
    MechanicalStep U3_L0R0_4[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][4].Set(8, U3_L0R0_4, tempRot, tempHandState);
    // U3_L0R0_5
    MechanicalStep U3_L0R0_5[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][5].Set(8, U3_L0R0_5, tempRot, tempHandState);
    // U3_L0R0_6
    MechanicalStep U3_L0R0_6[] = {M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][6].Set(8, U3_L0R0_6, tempRot, tempHandState);
    // U3_L0R0_7
    MechanicalStep U3_L0R0_7[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][7].Set(8, U3_L0R0_7, tempRot, tempHandState);
    // U3_L0R0_8
    MechanicalStep U3_L0R0_8[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_L3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][8].Set(8, U3_L0R0_8, tempRot, tempHandState);
    // U3_L0R0_9
    MechanicalStep U3_L0R0_9[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L3, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_0_R_0][9].Set(8, U3_L0R0_9, tempRot, tempHandState);
    // U3_L0R0_10
    MechanicalStep U3_L0R0_10[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][10].Set(9, U3_L0R0_10, tempRot, tempHandState);
    // U3_L0R0_11
    MechanicalStep U3_L0R0_11[] = {M_RO, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][11].Set(9, U3_L0R0_11, tempRot, tempHandState);
    // U3_L0R0_12
    MechanicalStep U3_L0R0_12[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][12].Set(12, U3_L0R0_12, tempRot, tempHandState);
    // U3_L0R0_13
    MechanicalStep U3_L0R0_13[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_0_R_0][13].Set(12, U3_L0R0_13, tempRot, tempHandState);
    // U3_L0R0_14
    MechanicalStep U3_L0R0_14[] = {
        M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_0_R_0][14].Set(12, U3_L0R0_14, tempRot, tempHandState);
    // U3_L0R0_15
    MechanicalStep U3_L0R0_15[] = {
        M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_0_R_0][15].Set(12, U3_L0R0_15, tempRot, tempHandState);
}
void B1_L0R0Init(void)
{
    // B1_L0R0_0
    MechanicalStep B1_L0R0_0[] = {M_LO, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][0].Set(4, B1_L0R0_0, tempRot, tempHandState);
    // B1_L0R0_1
    MechanicalStep B1_L0R0_1[] = {M_LO, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][1].Set(5, B1_L0R0_1, tempRot, tempHandState);
    // B1_L0R0_2
    MechanicalStep B1_L0R0_2[] = {M_RO, M_L2, M_RC, M_LO, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][2].Set(8, B1_L0R0_2, tempRot, tempHandState);
    // B1_L0R0_3
    MechanicalStep B1_L0R0_3[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][3].Set(8, B1_L0R0_3, tempRot, tempHandState);
    // B1_L0R0_4
    MechanicalStep B1_L0R0_4[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][4].Set(8, B1_L0R0_4, tempRot, tempHandState);
    // B1_L0R0_5
    MechanicalStep B1_L0R0_5[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][5].Set(9, B1_L0R0_5, tempRot, tempHandState);
    // B1_L0R0_6
    MechanicalStep B1_L0R0_6[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][6].Set(9, B1_L0R0_6, tempRot, tempHandState);
    // B1_L0R0_7
    MechanicalStep B1_L0R0_7[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][7].Set(11, B1_L0R0_7, tempRot, tempHandState);
    // B1_L0R0_8
    MechanicalStep B1_L0R0_8[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_0_R_0][8].Set(11, B1_L0R0_8, tempRot, tempHandState);
    // B1_L0R0_9
    MechanicalStep B1_L0R0_9[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_0_R_0][9].Set(11, B1_L0R0_9, tempRot, tempHandState);
    // B1_L0R0_10
    MechanicalStep B1_L0R0_10[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][10].Set(12, B1_L0R0_10, tempRot, tempHandState);
    // B1_L0R0_11
    MechanicalStep B1_L0R0_11[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][11].Set(12, B1_L0R0_11, tempRot, tempHandState);
    // B1_L0R0_12
    MechanicalStep B1_L0R0_12[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][12].Set(12, B1_L0R0_12, tempRot, tempHandState);
    // B1_L0R0_13
    MechanicalStep B1_L0R0_13[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_0][13].Set(12, B1_L0R0_13, tempRot, tempHandState);
    // B1_L0R0_14
    MechanicalStep B1_L0R0_14[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_RC,
                                   M_R1,
                                   M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_0_R_0][14].Set(15, B1_L0R0_14, tempRot, tempHandState);
    // B1_L0R0_15
    MechanicalStep B1_L0R0_15[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_RC,
                                   M_R1,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_0_R_0][15].Set(15, B1_L0R0_15, tempRot, tempHandState);
}
void B2_L0R0Init(void)
{
    // B2_L0R0_0
    MechanicalStep B2_L0R0_0[] = {M_LO, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][0].Set(4, B2_L0R0_0, tempRot, tempHandState);
    // B2_L0R0_1
    MechanicalStep B2_L0R0_1[] = {M_LO, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][1].Set(5, B2_L0R0_1, tempRot, tempHandState);
    // B2_L0R0_2
    MechanicalStep B2_L0R0_2[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][2].Set(8, B2_L0R0_2, tempRot, tempHandState);
    // B2_L0R0_3
    MechanicalStep B2_L0R0_3[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][3].Set(8, B2_L0R0_3, tempRot, tempHandState);
    // B2_L0R0_4
    MechanicalStep B2_L0R0_4[] = {M_RO, M_L2, M_RC, M_LO, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][4].Set(8, B2_L0R0_4, tempRot, tempHandState);
    // B2_L0R0_5
    MechanicalStep B2_L0R0_5[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][5].Set(9, B2_L0R0_5, tempRot, tempHandState);
    // B2_L0R0_6
    MechanicalStep B2_L0R0_6[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][6].Set(9, B2_L0R0_6, tempRot, tempHandState);
    // B2_L0R0_7
    MechanicalStep B2_L0R0_7[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][7].Set(11, B2_L0R0_7, tempRot, tempHandState);
    // B2_L0R0_8
    MechanicalStep B2_L0R0_8[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][8].Set(11, B2_L0R0_8, tempRot, tempHandState);
    // B2_L0R0_9
    MechanicalStep B2_L0R0_9[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][9].Set(11, B2_L0R0_9, tempRot, tempHandState);
    // B2_L0R0_10
    MechanicalStep B2_L0R0_10[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_0_R_0][10].Set(12, B2_L0R0_10, tempRot, tempHandState);
    // B2_L0R0_11
    MechanicalStep B2_L0R0_11[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_0_R_0][11].Set(12, B2_L0R0_11, tempRot, tempHandState);
    // B2_L0R0_12
    MechanicalStep B2_L0R0_12[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_0_R_0][12].Set(12, B2_L0R0_12, tempRot, tempHandState);
    // B2_L0R0_13
    MechanicalStep B2_L0R0_13[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_0_R_0][13].Set(12, B2_L0R0_13, tempRot, tempHandState);
    // B2_L0R0_14
    MechanicalStep B2_L0R0_14[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_RC,
                                   M_R2,
                                   M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][14].Set(15, B2_L0R0_14, tempRot, tempHandState);
    // B2_L0R0_15
    MechanicalStep B2_L0R0_15[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_RC,
                                   M_R2,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_0][15].Set(15, B2_L0R0_15, tempRot, tempHandState);
}
void B3_L0R0Init(void)
{
    // B3_L0R0_0
    MechanicalStep B3_L0R0_0[] = {M_LO, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][0].Set(4, B3_L0R0_0, tempRot, tempHandState);
    // B3_L0R0_1
    MechanicalStep B3_L0R0_1[] = {M_LO, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][1].Set(5, B3_L0R0_1, tempRot, tempHandState);
    // B3_L0R0_2
    MechanicalStep B3_L0R0_2[] = {M_RO, M_L2, M_RC, M_LO, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][2].Set(8, B3_L0R0_2, tempRot, tempHandState);
    // B3_L0R0_3
    MechanicalStep B3_L0R0_3[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][3].Set(8, B3_L0R0_3, tempRot, tempHandState);
    // B3_L0R0_4
    MechanicalStep B3_L0R0_4[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][4].Set(8, B3_L0R0_4, tempRot, tempHandState);
    // B3_L0R0_5
    MechanicalStep B3_L0R0_5[] = {M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][5].Set(9, B3_L0R0_5, tempRot, tempHandState);
    // B3_L0R0_6
    MechanicalStep B3_L0R0_6[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][6].Set(9, B3_L0R0_6, tempRot, tempHandState);
    // B3_L0R0_7
    MechanicalStep B3_L0R0_7[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][7].Set(11, B3_L0R0_7, tempRot, tempHandState);
    // B3_L0R0_8
    MechanicalStep B3_L0R0_8[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_0_R_0][8].Set(11, B3_L0R0_8, tempRot, tempHandState);
    // B3_L0R0_9
    MechanicalStep B3_L0R0_9[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_0_R_0][9].Set(11, B3_L0R0_9, tempRot, tempHandState);
    // B3_L0R0_10
    MechanicalStep B3_L0R0_10[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][10].Set(12, B3_L0R0_10, tempRot, tempHandState);
    // B3_L0R0_11
    MechanicalStep B3_L0R0_11[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][11].Set(12, B3_L0R0_11, tempRot, tempHandState);
    // B3_L0R0_12
    MechanicalStep B3_L0R0_12[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][12].Set(12, B3_L0R0_12, tempRot, tempHandState);
    // B3_L0R0_13
    MechanicalStep B3_L0R0_13[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_0][13].Set(12, B3_L0R0_13, tempRot, tempHandState);
    // B3_L0R0_14
    MechanicalStep B3_L0R0_14[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_RC,
                                   M_R3,
                                   M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_0_R_0][14].Set(15, B3_L0R0_14, tempRot, tempHandState);
    // B3_L0R0_15
    MechanicalStep B3_L0R0_15[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_RC,
                                   M_R3,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_0_R_0][15].Set(15, B3_L0R0_15, tempRot, tempHandState);
}
void L1_L0R0Init(void)
{
    // L1_L0R0_0
    MechanicalStep L1_L0R0_0[] = {M_RO, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_0_R_0][0].Set(4, L1_L0R0_0, tempRot, tempHandState);
    // L1_L0R0_1
    MechanicalStep L1_L0R0_1[] = {M_RO, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][1].Set(5, L1_L0R0_1, tempRot, tempHandState);
    // L1_L0R0_2
    MechanicalStep L1_L0R0_2[] = {M_LO, M_R2, M_LC, M_RO, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][2].Set(8, L1_L0R0_2, tempRot, tempHandState);
    // L1_L0R0_3
    MechanicalStep L1_L0R0_3[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_0_R_0][3].Set(8, L1_L0R0_3, tempRot, tempHandState);
    // L1_L0R0_4
    MechanicalStep L1_L0R0_4[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_0_R_0][4].Set(8, L1_L0R0_4, tempRot, tempHandState);
    // L1_L0R0_5
    MechanicalStep L1_L0R0_5[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][5].Set(9, L1_L0R0_5, tempRot, tempHandState);
    // L1_L0R0_6
    MechanicalStep L1_L0R0_6[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][6].Set(9, L1_L0R0_6, tempRot, tempHandState);
    // L1_L0R0_7
    MechanicalStep L1_L0R0_7[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][7].Set(11, L1_L0R0_7, tempRot, tempHandState);
    // L1_L0R0_8
    MechanicalStep L1_L0R0_8[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_R1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_0_R_0][8].Set(11, L1_L0R0_8, tempRot, tempHandState);
    // L1_L0R0_9
    MechanicalStep L1_L0R0_9[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][9].Set(11, L1_L0R0_9, tempRot, tempHandState);
    // L1_L0R0_10
    MechanicalStep L1_L0R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][10].Set(12, L1_L0R0_10, tempRot, tempHandState);
    // L1_L0R0_11
    MechanicalStep L1_L0R0_11[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][11].Set(12, L1_L0R0_11, tempRot, tempHandState);
    // L1_L0R0_12
    MechanicalStep L1_L0R0_12[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][12].Set(12, L1_L0R0_12, tempRot, tempHandState);
    // L1_L0R0_13
    MechanicalStep L1_L0R0_13[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][13].Set(12, L1_L0R0_13, tempRot, tempHandState);
    // L1_L0R0_14
    MechanicalStep L1_L0R0_14[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][14].Set(15, L1_L0R0_14, tempRot, tempHandState);
    // L1_L0R0_15
    MechanicalStep L1_L0R0_15[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_0_R_0][15].Set(15, L1_L0R0_15, tempRot, tempHandState);
}
void L2_L0R0Init(void)
{
    // L2_L0R0_0
    MechanicalStep L2_L0R0_0[] = {M_RO, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][0].Set(4, L2_L0R0_0, tempRot, tempHandState);
    // L2_L0R0_1
    MechanicalStep L2_L0R0_1[] = {M_RO, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_0_R_0][1].Set(5, L2_L0R0_1, tempRot, tempHandState);
    // L2_L0R0_2
    MechanicalStep L2_L0R0_2[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][2].Set(8, L2_L0R0_2, tempRot, tempHandState);
    // L2_L0R0_3
    MechanicalStep L2_L0R0_3[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][3].Set(8, L2_L0R0_3, tempRot, tempHandState);
    // L2_L0R0_4
    MechanicalStep L2_L0R0_4[] = {M_LO, M_R2, M_LC, M_RO, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_0_R_0][4].Set(8, L2_L0R0_4, tempRot, tempHandState);
    // L2_L0R0_5
    MechanicalStep L2_L0R0_5[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_0_R_0][5].Set(9, L2_L0R0_5, tempRot, tempHandState);
    // L2_L0R0_6
    MechanicalStep L2_L0R0_6[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_0_R_0][6].Set(9, L2_L0R0_6, tempRot, tempHandState);
    // L2_L0R0_7
    MechanicalStep L2_L0R0_7[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][7].Set(11, L2_L0R0_7, tempRot, tempHandState);
    // L2_L0R0_8
    MechanicalStep L2_L0R0_8[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][8].Set(11, L2_L0R0_8, tempRot, tempHandState);
    // L2_L0R0_9
    MechanicalStep L2_L0R0_9[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_R2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][9].Set(11, L2_L0R0_9, tempRot, tempHandState);
    // L2_L0R0_10
    MechanicalStep L2_L0R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][10].Set(12, L2_L0R0_10, tempRot, tempHandState);
    // L2_L0R0_11
    MechanicalStep L2_L0R0_11[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][11].Set(12, L2_L0R0_11, tempRot, tempHandState);
    // L2_L0R0_12
    MechanicalStep L2_L0R0_12[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][12].Set(12, L2_L0R0_12, tempRot, tempHandState);
    // L2_L0R0_13
    MechanicalStep L2_L0R0_13[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][13].Set(12, L2_L0R0_13, tempRot, tempHandState);
    // L2_L0R0_14
    MechanicalStep L2_L0R0_14[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][14].Set(15, L2_L0R0_14, tempRot, tempHandState);
    // L2_L0R0_15
    MechanicalStep L2_L0R0_15[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_0][15].Set(15, L2_L0R0_15, tempRot, tempHandState);
}
void L3_L0R0Init(void)
{
    // L3_L0R0_0
    MechanicalStep L3_L0R0_0[] = {M_RO, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_0_R_0][0].Set(4, L3_L0R0_0, tempRot, tempHandState);
    // L3_L0R0_1
    MechanicalStep L3_L0R0_1[] = {M_RO, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][1].Set(5, L3_L0R0_1, tempRot, tempHandState);
    // L3_L0R0_2
    MechanicalStep L3_L0R0_2[] = {M_LO, M_R2, M_LC, M_RO, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][2].Set(8, L3_L0R0_2, tempRot, tempHandState);
    // L3_L0R0_3
    MechanicalStep L3_L0R0_3[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_0_R_0][3].Set(8, L3_L0R0_3, tempRot, tempHandState);
    // L3_L0R0_4
    MechanicalStep L3_L0R0_4[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_0_R_0][4].Set(8, L3_L0R0_4, tempRot, tempHandState);
    // L3_L0R0_5
    MechanicalStep L3_L0R0_5[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][5].Set(9, L3_L0R0_5, tempRot, tempHandState);
    // L3_L0R0_6
    MechanicalStep L3_L0R0_6[] = {M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][6].Set(9, L3_L0R0_6, tempRot, tempHandState);
    // L3_L0R0_7
    MechanicalStep L3_L0R0_7[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][7].Set(11, L3_L0R0_7, tempRot, tempHandState);
    // L3_L0R0_8
    MechanicalStep L3_L0R0_8[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][8].Set(11, L3_L0R0_8, tempRot, tempHandState);
    // L3_L0R0_9
    MechanicalStep L3_L0R0_9[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_R3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_0_R_0][9].Set(11, L3_L0R0_9, tempRot, tempHandState);
    // L3_L0R0_10
    MechanicalStep L3_L0R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][10].Set(12, L3_L0R0_10, tempRot, tempHandState);
    // L3_L0R0_11
    MechanicalStep L3_L0R0_11[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][11].Set(12, L3_L0R0_11, tempRot, tempHandState);
    // L3_L0R0_12
    MechanicalStep L3_L0R0_12[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][12].Set(12, L3_L0R0_12, tempRot, tempHandState);
    // L3_L0R0_13
    MechanicalStep L3_L0R0_13[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][13].Set(12, L3_L0R0_13, tempRot, tempHandState);
    // L3_L0R0_14
    MechanicalStep L3_L0R0_14[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][14].Set(15, L3_L0R0_14, tempRot, tempHandState);
    // L3_L0R0_15
    MechanicalStep L3_L0R0_15[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_0_R_0][15].Set(15, L3_L0R0_15, tempRot, tempHandState);
}
void D1_L0R0Init(void)
{
    // D1_L0R0_0
    MechanicalStep D1_L0R0_0[] = {M_LO, M_L1, M_LC, M_RO, M_L3, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_0_R_0][0].Set(7, D1_L0R0_0, tempRot, tempHandState);
    // D1_L0R0_1
    MechanicalStep D1_L0R0_1[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][1].Set(7, D1_L0R0_1, tempRot, tempHandState);
    // D1_L0R0_2
    MechanicalStep D1_L0R0_2[] = {M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][2].Set(8, D1_L0R0_2, tempRot, tempHandState);
    // D1_L0R0_3
    MechanicalStep D1_L0R0_3[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][3].Set(8, D1_L0R0_3, tempRot, tempHandState);
    // D1_L0R0_4
    MechanicalStep D1_L0R0_4[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][4].Set(8, D1_L0R0_4, tempRot, tempHandState);
    // D1_L0R0_5
    MechanicalStep D1_L0R0_5[] = {M_RO, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][5].Set(8, D1_L0R0_5, tempRot, tempHandState);
    // D1_L0R0_6
    MechanicalStep D1_L0R0_6[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][6].Set(8, D1_L0R0_6, tempRot, tempHandState);
    // D1_L0R0_7
    MechanicalStep D1_L0R0_7[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][7].Set(8, D1_L0R0_7, tempRot, tempHandState);
    // D1_L0R0_8
    MechanicalStep D1_L0R0_8[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_0_R_0][8].Set(8, D1_L0R0_8, tempRot, tempHandState);
    // D1_L0R0_9
    MechanicalStep D1_L0R0_9[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_L1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][9].Set(8, D1_L0R0_9, tempRot, tempHandState);
    // D1_L0R0_10
    MechanicalStep D1_L0R0_10[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][10].Set(9, D1_L0R0_10, tempRot, tempHandState);
    // D1_L0R0_11
    MechanicalStep D1_L0R0_11[] = {M_RO, M_L2, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][11].Set(9, D1_L0R0_11, tempRot, tempHandState);
    // D1_L0R0_12
    MechanicalStep D1_L0R0_12[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_0_R_0][12].Set(12, D1_L0R0_12, tempRot, tempHandState);
    // D1_L0R0_13
    MechanicalStep D1_L0R0_13[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][13].Set(12, D1_L0R0_13, tempRot, tempHandState);
    // D1_L0R0_14
    MechanicalStep D1_L0R0_14[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_0_R_0][14].Set(12, D1_L0R0_14, tempRot, tempHandState);
    // D1_L0R0_15
    MechanicalStep D1_L0R0_15[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_0_R_0][15].Set(12, D1_L0R0_15, tempRot, tempHandState);
}
void D2_L0R0Init(void)
{
    // D2_L0R0_0
    MechanicalStep D2_L0R0_0[] = {M_LO, M_L1, M_LC, M_RO, M_L3, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][0].Set(7, D2_L0R0_0, tempRot, tempHandState);
    // D2_L0R0_1
    MechanicalStep D2_L0R0_1[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][1].Set(7, D2_L0R0_1, tempRot, tempHandState);
    // D2_L0R0_2
    MechanicalStep D2_L0R0_2[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][2].Set(8, D2_L0R0_2, tempRot, tempHandState);
    // D2_L0R0_3
    MechanicalStep D2_L0R0_3[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_L2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][3].Set(8, D2_L0R0_3, tempRot, tempHandState);
    // D2_L0R0_4
    MechanicalStep D2_L0R0_4[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_0_R_0][4].Set(8, D2_L0R0_4, tempRot, tempHandState);
    // D2_L0R0_5
    MechanicalStep D2_L0R0_5[] = {M_RO, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][5].Set(8, D2_L0R0_5, tempRot, tempHandState);
    // D2_L0R0_6
    MechanicalStep D2_L0R0_6[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][6].Set(8, D2_L0R0_6, tempRot, tempHandState);
    // D2_L0R0_7
    MechanicalStep D2_L0R0_7[] = {M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_0_R_0][7].Set(8, D2_L0R0_7, tempRot, tempHandState);
    // D2_L0R0_8
    MechanicalStep D2_L0R0_8[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][8].Set(8, D2_L0R0_8, tempRot, tempHandState);
    // D2_L0R0_9
    MechanicalStep D2_L0R0_9[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_0_R_0][9].Set(8, D2_L0R0_9, tempRot, tempHandState);
    // D2_L0R0_10
    MechanicalStep D2_L0R0_10[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_0_R_0][10].Set(9, D2_L0R0_10, tempRot, tempHandState);
    // D2_L0R0_11
    MechanicalStep D2_L0R0_11[] = {M_RO, M_L2, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][11].Set(9, D2_L0R0_11, tempRot, tempHandState);
    // D2_L0R0_12
    MechanicalStep D2_L0R0_12[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][12].Set(12, D2_L0R0_12, tempRot, tempHandState);
    // D2_L0R0_13
    MechanicalStep D2_L0R0_13[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][13].Set(12, D2_L0R0_13, tempRot, tempHandState);
    // D2_L0R0_14
    MechanicalStep D2_L0R0_14[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][14].Set(12, D2_L0R0_14, tempRot, tempHandState);
    // D2_L0R0_15
    MechanicalStep D2_L0R0_15[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_0][15].Set(12, D2_L0R0_15, tempRot, tempHandState);
}
void D3_L0R0Init(void)
{
    // D3_L0R0_0
    MechanicalStep D3_L0R0_0[] = {M_LO, M_L1, M_LC, M_RO, M_L3, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_0_R_0][0].Set(7, D3_L0R0_0, tempRot, tempHandState);
    // D3_L0R0_1
    MechanicalStep D3_L0R0_1[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][1].Set(7, D3_L0R0_1, tempRot, tempHandState);
    // D3_L0R0_2
    MechanicalStep D3_L0R0_2[] = {M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][2].Set(8, D3_L0R0_2, tempRot, tempHandState);
    // D3_L0R0_3
    MechanicalStep D3_L0R0_3[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][3].Set(8, D3_L0R0_3, tempRot, tempHandState);
    // D3_L0R0_4
    MechanicalStep D3_L0R0_4[] = {M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][4].Set(8, D3_L0R0_4, tempRot, tempHandState);
    // D3_L0R0_5
    MechanicalStep D3_L0R0_5[] = {M_RO, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][5].Set(8, D3_L0R0_5, tempRot, tempHandState);
    // D3_L0R0_6
    MechanicalStep D3_L0R0_6[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][6].Set(8, D3_L0R0_6, tempRot, tempHandState);
    // D3_L0R0_7
    MechanicalStep D3_L0R0_7[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][7].Set(8, D3_L0R0_7, tempRot, tempHandState);
    // D3_L0R0_8
    MechanicalStep D3_L0R0_8[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_0_R_0][8].Set(8, D3_L0R0_8, tempRot, tempHandState);
    // D3_L0R0_9
    MechanicalStep D3_L0R0_9[] = {M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_L3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][9].Set(8, D3_L0R0_9, tempRot, tempHandState);
    // D3_L0R0_10
    MechanicalStep D3_L0R0_10[] = {M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][10].Set(9, D3_L0R0_10, tempRot, tempHandState);
    // D3_L0R0_11
    MechanicalStep D3_L0R0_11[] = {M_RO, M_L2, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][11].Set(9, D3_L0R0_11, tempRot, tempHandState);
    // D3_L0R0_12
    MechanicalStep D3_L0R0_12[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_0_R_0][12].Set(12, D3_L0R0_12, tempRot, tempHandState);
    // D3_L0R0_13
    MechanicalStep D3_L0R0_13[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][13].Set(12, D3_L0R0_13, tempRot, tempHandState);
    // D3_L0R0_14
    MechanicalStep D3_L0R0_14[] = {
        M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_0_R_0][14].Set(12, D3_L0R0_14, tempRot, tempHandState);
    // D3_L0R0_15
    MechanicalStep D3_L0R0_15[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_0_R_0][15].Set(12, D3_L0R0_15, tempRot, tempHandState);
}
void F1_L0R1Init(void)
{
    // F1_L0R1_0
    MechanicalStep F1_L0R1_0[] = {M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][0].Set(4, F1_L0R1_0, tempRot, tempHandState);
    // F1_L0R1_1
    MechanicalStep F1_L0R1_1[] = {M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][1].Set(5, F1_L0R1_1, tempRot, tempHandState);
    // F1_L0R1_2
    MechanicalStep F1_L0R1_2[] = {M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][2].Set(5, F1_L0R1_2, tempRot, tempHandState);
    // F1_L0R1_3
    MechanicalStep F1_L0R1_3[] = {M_RO, M_R1, M_L2, M_RC, M_L1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][3].Set(5, F1_L0R1_3, tempRot, tempHandState);
    // F1_L0R1_4
    MechanicalStep F1_L0R1_4[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_RC, M_R1, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_0_R_1][4].Set(8, F1_L0R1_4, tempRot, tempHandState);
    // F1_L0R1_5
    MechanicalStep F1_L0R1_5[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_RC, M_R1, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_0_R_1][5].Set(8, F1_L0R1_5, tempRot, tempHandState);
    // F1_L0R1_6
    MechanicalStep F1_L0R1_6[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][6].Set(9, F1_L0R1_6, tempRot, tempHandState);
    // F1_L0R1_7
    MechanicalStep F1_L0R1_7[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][7].Set(9, F1_L0R1_7, tempRot, tempHandState);
    // F1_L0R1_8
    MechanicalStep F1_L0R1_8[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][8].Set(11, F1_L0R1_8, tempRot, tempHandState);
    // F1_L0R1_9
    MechanicalStep F1_L0R1_9[] = {
        M_LO, M_R1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][9].Set(11, F1_L0R1_9, tempRot, tempHandState);
    // F1_L0R1_10
    MechanicalStep F1_L0R1_10[] = {
        M_LO, M_R1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][10].Set(11, F1_L0R1_10, tempRot, tempHandState);
    // F1_L0R1_11
    MechanicalStep F1_L0R1_11[] = {
        M_LO, M_R1, M_LC, M_RO, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][11].Set(12, F1_L0R1_11, tempRot, tempHandState);
    // F1_L0R1_12
    MechanicalStep F1_L0R1_12[] = {
        M_LO, M_R2, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][12].Set(12, F1_L0R1_12, tempRot, tempHandState);
    // F1_L0R1_13
    MechanicalStep F1_L0R1_13[] = {
        M_LO, M_R2, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_0_R_1][13].Set(12, F1_L0R1_13, tempRot, tempHandState);
    // F1_L0R1_14
    MechanicalStep F1_L0R1_14[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1,
                                   M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_0_R_1][14].Set(19, F1_L0R1_14, tempRot, tempHandState);
    // F1_L0R1_15
    MechanicalStep F1_L0R1_15[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3,
                                   M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_0_R_1][15].Set(19, F1_L0R1_15, tempRot, tempHandState);
}
void F2_L0R1Init(void)
{
    // F2_L0R1_0
    MechanicalStep F2_L0R1_0[] = {M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][0].Set(4, F2_L0R1_0, tempRot, tempHandState);
    // F2_L0R1_1
    MechanicalStep F2_L0R1_1[] = {M_RO, M_R1, M_L2, M_RC, M_L2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][1].Set(5, F2_L0R1_1, tempRot, tempHandState);
    // F2_L0R1_2
    MechanicalStep F2_L0R1_2[] = {M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][2].Set(5, F2_L0R1_2, tempRot, tempHandState);
    // F2_L0R1_3
    MechanicalStep F2_L0R1_3[] = {M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][3].Set(5, F2_L0R1_3, tempRot, tempHandState);
    // F2_L0R1_4
    MechanicalStep F2_L0R1_4[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_RC, M_R2, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][4].Set(8, F2_L0R1_4, tempRot, tempHandState);
    // F2_L0R1_5
    MechanicalStep F2_L0R1_5[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_RC, M_R2, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][5].Set(8, F2_L0R1_5, tempRot, tempHandState);
    // F2_L0R1_6
    MechanicalStep F2_L0R1_6[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_0_R_1][6].Set(9, F2_L0R1_6, tempRot, tempHandState);
    // F2_L0R1_7
    MechanicalStep F2_L0R1_7[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_0_R_1][7].Set(9, F2_L0R1_7, tempRot, tempHandState);
    // F2_L0R1_8
    MechanicalStep F2_L0R1_8[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][8].Set(11, F2_L0R1_8, tempRot, tempHandState);
    // F2_L0R1_9
    MechanicalStep F2_L0R1_9[] = {
        M_LO, M_R1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_0_R_1][9].Set(11, F2_L0R1_9, tempRot, tempHandState);
    // F2_L0R1_10
    MechanicalStep F2_L0R1_10[] = {
        M_LO, M_R1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_0_R_1][10].Set(11, F2_L0R1_10, tempRot, tempHandState);
    // F2_L0R1_11
    MechanicalStep F2_L0R1_11[] = {
        M_LO, M_R2, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][11].Set(12, F2_L0R1_11, tempRot, tempHandState);
    // F2_L0R1_12
    MechanicalStep F2_L0R1_12[] = {
        M_LO, M_R2, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][12].Set(12, F2_L0R1_12, tempRot, tempHandState);
    // F2_L0R1_13
    MechanicalStep F2_L0R1_13[] = {
        M_LO, M_R1, M_LC, M_RO, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][13].Set(12, F2_L0R1_13, tempRot, tempHandState);
    // F2_L0R1_14
    MechanicalStep F2_L0R1_14[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1,
                                   M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][14].Set(19, F2_L0R1_14, tempRot, tempHandState);
    // F2_L0R1_15
    MechanicalStep F2_L0R1_15[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3,
                                   M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_0_R_1][15].Set(19, F2_L0R1_15, tempRot, tempHandState);
}
void F3_L0R1Init(void)
{
    // F3_L0R1_0
    MechanicalStep F3_L0R1_0[] = {M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][0].Set(4, F3_L0R1_0, tempRot, tempHandState);
    // F3_L0R1_1
    MechanicalStep F3_L0R1_1[] = {M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][1].Set(5, F3_L0R1_1, tempRot, tempHandState);
    // F3_L0R1_2
    MechanicalStep F3_L0R1_2[] = {M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][2].Set(5, F3_L0R1_2, tempRot, tempHandState);
    // F3_L0R1_3
    MechanicalStep F3_L0R1_3[] = {M_RO, M_R1, M_L2, M_RC, M_L3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][3].Set(5, F3_L0R1_3, tempRot, tempHandState);
    // F3_L0R1_4
    MechanicalStep F3_L0R1_4[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_RC, M_R3, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_0_R_1][4].Set(8, F3_L0R1_4, tempRot, tempHandState);
    // F3_L0R1_5
    MechanicalStep F3_L0R1_5[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_RC, M_R3, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_0_R_1][5].Set(8, F3_L0R1_5, tempRot, tempHandState);
    // F3_L0R1_6
    MechanicalStep F3_L0R1_6[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][6].Set(9, F3_L0R1_6, tempRot, tempHandState);
    // F3_L0R1_7
    MechanicalStep F3_L0R1_7[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][7].Set(9, F3_L0R1_7, tempRot, tempHandState);
    // F3_L0R1_8
    MechanicalStep F3_L0R1_8[] = {
        M_LO, M_R1, M_LC, M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][8].Set(11, F3_L0R1_8, tempRot, tempHandState);
    // F3_L0R1_9
    MechanicalStep F3_L0R1_9[] = {
        M_LO, M_R1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][9].Set(11, F3_L0R1_9, tempRot, tempHandState);
    // F3_L0R1_10
    MechanicalStep F3_L0R1_10[] = {
        M_LO, M_R1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][10].Set(11, F3_L0R1_10, tempRot, tempHandState);
    // F3_L0R1_11
    MechanicalStep F3_L0R1_11[] = {
        M_LO, M_R1, M_LC, M_RO, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][11].Set(12, F3_L0R1_11, tempRot, tempHandState);
    // F3_L0R1_12
    MechanicalStep F3_L0R1_12[] = {
        M_LO, M_R2, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][12].Set(12, F3_L0R1_12, tempRot, tempHandState);
    // F3_L0R1_13
    MechanicalStep F3_L0R1_13[] = {
        M_LO, M_R2, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_0_R_1][13].Set(12, F3_L0R1_13, tempRot, tempHandState);
    // F3_L0R1_14
    MechanicalStep F3_L0R1_14[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1,
                                   M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_0_R_1][14].Set(19, F3_L0R1_14, tempRot, tempHandState);
    // F3_L0R1_15
    MechanicalStep F3_L0R1_15[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3,
                                   M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_0_R_1][15].Set(19, F3_L0R1_15, tempRot, tempHandState);
}
void R1_L0R1Init(void)
{
    // R1_L0R1_0
    MechanicalStep R1_L0R1_0[] = {M_R1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][0].Set(1, R1_L0R1_0, tempRot, tempHandState);
    // R1_L0R1_1
    MechanicalStep R1_L0R1_1[] = {M_LO, M_R2, M_LC, M_R1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][1].Set(4, R1_L0R1_1, tempRot, tempHandState);
    // R1_L0R1_2
    MechanicalStep R1_L0R1_2[] = {M_RO, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_0_R_1][2].Set(4, R1_L0R1_2, tempRot, tempHandState);
    // R1_L0R1_3
    MechanicalStep R1_L0R1_3[] = {M_LO, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_0_R_1][3].Set(4, R1_L0R1_3, tempRot, tempHandState);
    // R1_L0R1_4
    MechanicalStep R1_L0R1_4[] = {M_LO, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_0_R_1][4].Set(4, R1_L0R1_4, tempRot, tempHandState);
    // R1_L0R1_5
    MechanicalStep R1_L0R1_5[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_LC, M_L1, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][5].Set(12, R1_L0R1_5, tempRot, tempHandState);
    // R1_L0R1_6
    MechanicalStep R1_L0R1_6[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_LC, M_L1, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][6].Set(12, R1_L0R1_6, tempRot, tempHandState);
    // R1_L0R1_7
    MechanicalStep R1_L0R1_7[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][7].Set(12, R1_L0R1_7, tempRot, tempHandState);
    // R1_L0R1_8
    MechanicalStep R1_L0R1_8[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][8].Set(12, R1_L0R1_8, tempRot, tempHandState);
    // R1_L0R1_9
    MechanicalStep R1_L0R1_9[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][9].Set(13, R1_L0R1_9, tempRot, tempHandState);
    // R1_L0R1_10
    MechanicalStep R1_L0R1_10[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][10].Set(13, R1_L0R1_10, tempRot, tempHandState);
    // R1_L0R1_11
    MechanicalStep R1_L0R1_11[] = {
        M_RO, M_R1, M_L2, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][11].Set(13, R1_L0R1_11, tempRot, tempHandState);
    // R1_L0R1_12
    MechanicalStep R1_L0R1_12[] = {
        M_RO, M_R1, M_L2, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][12].Set(13, R1_L0R1_12, tempRot, tempHandState);
    // R1_L0R1_13
    MechanicalStep R1_L0R1_13[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_0_R_1][13].Set(13, R1_L0R1_13, tempRot, tempHandState);
    // R1_L0R1_14
    MechanicalStep R1_L0R1_14[] = {M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_L1,
                                   M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][14].Set(15, R1_L0R1_14, tempRot, tempHandState);
    // R1_L0R1_15
    MechanicalStep R1_L0R1_15[] = {M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_L1,
                                   M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_0_R_1][15].Set(15, R1_L0R1_15, tempRot, tempHandState);
}
void R2_L0R1Init(void)
{
    // R2_L0R1_0
    MechanicalStep R2_L0R1_0[] = {M_R2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_0_R_1][0].Set(1, R2_L0R1_0, tempRot, tempHandState);
    // R2_L0R1_1
    MechanicalStep R2_L0R1_1[] = {M_LO, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][1].Set(4, R2_L0R1_1, tempRot, tempHandState);
    // R2_L0R1_2
    MechanicalStep R2_L0R1_2[] = {M_RO, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][2].Set(4, R2_L0R1_2, tempRot, tempHandState);
    // R2_L0R1_3
    MechanicalStep R2_L0R1_3[] = {M_LO, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][3].Set(4, R2_L0R1_3, tempRot, tempHandState);
    // R2_L0R1_4
    MechanicalStep R2_L0R1_4[] = {M_LO, M_R2, M_LC, M_R2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_0_R_1][4].Set(4, R2_L0R1_4, tempRot, tempHandState);
    // R2_L0R1_5
    MechanicalStep R2_L0R1_5[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_LC, M_L2, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][5].Set(12, R2_L0R1_5, tempRot, tempHandState);
    // R2_L0R1_6
    MechanicalStep R2_L0R1_6[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_LC, M_L2, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][6].Set(12, R2_L0R1_6, tempRot, tempHandState);
    // R2_L0R1_7
    MechanicalStep R2_L0R1_7[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][7].Set(12, R2_L0R1_7, tempRot, tempHandState);
    // R2_L0R1_8
    MechanicalStep R2_L0R1_8[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][8].Set(12, R2_L0R1_8, tempRot, tempHandState);
    // R2_L0R1_9
    MechanicalStep R2_L0R1_9[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][9].Set(13, R2_L0R1_9, tempRot, tempHandState);
    // R2_L0R1_10
    MechanicalStep R2_L0R1_10[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][10].Set(13, R2_L0R1_10, tempRot, tempHandState);
    // R2_L0R1_11
    MechanicalStep R2_L0R1_11[] = {
        M_RO, M_R1, M_L2, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_0_R_1][11].Set(13, R2_L0R1_11, tempRot, tempHandState);
    // R2_L0R1_12
    MechanicalStep R2_L0R1_12[] = {
        M_RO, M_R1, M_L2, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_0_R_1][12].Set(13, R2_L0R1_12, tempRot, tempHandState);
    // R2_L0R1_13
    MechanicalStep R2_L0R1_13[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][13].Set(13, R2_L0R1_13, tempRot, tempHandState);
    // R2_L0R1_14
    MechanicalStep R2_L0R1_14[] = {M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_L2,
                                   M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][14].Set(15, R2_L0R1_14, tempRot, tempHandState);
    // R2_L0R1_15
    MechanicalStep R2_L0R1_15[] = {M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_L2,
                                   M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_0_R_1][15].Set(15, R2_L0R1_15, tempRot, tempHandState);
}
void R3_L0R1Init(void)
{
    // R3_L0R1_0
    MechanicalStep R3_L0R1_0[] = {M_R3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][0].Set(1, R3_L0R1_0, tempRot, tempHandState);
    // R3_L0R1_1
    MechanicalStep R3_L0R1_1[] = {M_LO, M_R2, M_LC, M_R3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][1].Set(4, R3_L0R1_1, tempRot, tempHandState);
    // R3_L0R1_2
    MechanicalStep R3_L0R1_2[] = {M_RO, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_0_R_1][2].Set(4, R3_L0R1_2, tempRot, tempHandState);
    // R3_L0R1_3
    MechanicalStep R3_L0R1_3[] = {M_LO, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_0_R_1][3].Set(4, R3_L0R1_3, tempRot, tempHandState);
    // R3_L0R1_4
    MechanicalStep R3_L0R1_4[] = {M_LO, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_0_R_1][4].Set(4, R3_L0R1_4, tempRot, tempHandState);
    // R3_L0R1_5
    MechanicalStep R3_L0R1_5[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_LC, M_L3, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][5].Set(12, R3_L0R1_5, tempRot, tempHandState);
    // R3_L0R1_6
    MechanicalStep R3_L0R1_6[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_LC, M_L3, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][6].Set(12, R3_L0R1_6, tempRot, tempHandState);
    // R3_L0R1_7
    MechanicalStep R3_L0R1_7[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][7].Set(12, R3_L0R1_7, tempRot, tempHandState);
    // R3_L0R1_8
    MechanicalStep R3_L0R1_8[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][8].Set(12, R3_L0R1_8, tempRot, tempHandState);
    // R3_L0R1_9
    MechanicalStep R3_L0R1_9[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][9].Set(13, R3_L0R1_9, tempRot, tempHandState);
    // R3_L0R1_10
    MechanicalStep R3_L0R1_10[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][10].Set(13, R3_L0R1_10, tempRot, tempHandState);
    // R3_L0R1_11
    MechanicalStep R3_L0R1_11[] = {
        M_RO, M_R1, M_L2, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][11].Set(13, R3_L0R1_11, tempRot, tempHandState);
    // R3_L0R1_12
    MechanicalStep R3_L0R1_12[] = {
        M_RO, M_R1, M_L2, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][12].Set(13, R3_L0R1_12, tempRot, tempHandState);
    // R3_L0R1_13
    MechanicalStep R3_L0R1_13[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_0_R_1][13].Set(13, R3_L0R1_13, tempRot, tempHandState);
    // R3_L0R1_14
    MechanicalStep R3_L0R1_14[] = {M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_L3,
                                   M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][14].Set(15, R3_L0R1_14, tempRot, tempHandState);
    // R3_L0R1_15
    MechanicalStep R3_L0R1_15[] = {M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_L3,
                                   M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_0_R_1][15].Set(15, R3_L0R1_15, tempRot, tempHandState);
}
void U1_L0R1Init(void)
{
    // U1_L0R1_0
    MechanicalStep U1_L0R1_0[] = {M_LO, M_R3, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][0].Set(4, U1_L0R1_0, tempRot, tempHandState);
    // U1_L0R1_1
    MechanicalStep U1_L0R1_1[] = {M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][1].Set(5, U1_L0R1_1, tempRot, tempHandState);
    // U1_L0R1_2
    MechanicalStep U1_L0R1_2[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_0_R_1][2].Set(8, U1_L0R1_2, tempRot, tempHandState);
    // U1_L0R1_3
    MechanicalStep U1_L0R1_3[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][3].Set(9, U1_L0R1_3, tempRot, tempHandState);
    // U1_L0R1_4
    MechanicalStep U1_L0R1_4[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][4].Set(9, U1_L0R1_4, tempRot, tempHandState);
    // U1_L0R1_5
    MechanicalStep U1_L0R1_5[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_R1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_0_R_1][5].Set(9, U1_L0R1_5, tempRot, tempHandState);
    // U1_L0R1_6
    MechanicalStep U1_L0R1_6[] = {M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][6].Set(9, U1_L0R1_6, tempRot, tempHandState);
    // U1_L0R1_7
    MechanicalStep U1_L0R1_7[] = {
        M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][7].Set(10, U1_L0R1_7, tempRot, tempHandState);
    // U1_L0R1_8
    MechanicalStep U1_L0R1_8[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][8].Set(11, U1_L0R1_8, tempRot, tempHandState);
    // U1_L0R1_9
    MechanicalStep U1_L0R1_9[] = {
        M_RO, M_R1, M_RC, M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][9].Set(11, U1_L0R1_9, tempRot, tempHandState);
    // U1_L0R1_10
    MechanicalStep U1_L0R1_10[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][10].Set(11, U1_L0R1_10, tempRot, tempHandState);
    // U1_L0R1_11
    MechanicalStep U1_L0R1_11[] = {
        M_RO, M_R1, M_RC, M_LO, M_R2, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][11].Set(12, U1_L0R1_11, tempRot, tempHandState);
    // U1_L0R1_12
    MechanicalStep U1_L0R1_12[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_0_R_1][12].Set(13, U1_L0R1_12, tempRot, tempHandState);
    // U1_L0R1_13
    MechanicalStep U1_L0R1_13[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_0_R_1][13].Set(13, U1_L0R1_13, tempRot, tempHandState);
    // U1_L0R1_14
    MechanicalStep U1_L0R1_14[] = {M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][14].Set(15, U1_L0R1_14, tempRot, tempHandState);
    // U1_L0R1_15
    MechanicalStep U1_L0R1_15[] = {M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_0_R_1][15].Set(15, U1_L0R1_15, tempRot, tempHandState);
}
void U2_L0R1Init(void)
{
    // U2_L0R1_0
    MechanicalStep U2_L0R1_0[] = {M_LO, M_R3, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][0].Set(4, U2_L0R1_0, tempRot, tempHandState);
    // U2_L0R1_1
    MechanicalStep U2_L0R1_1[] = {M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][1].Set(5, U2_L0R1_1, tempRot, tempHandState);
    // U2_L0R1_2
    MechanicalStep U2_L0R1_2[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][2].Set(8, U2_L0R1_2, tempRot, tempHandState);
    // U2_L0R1_3
    MechanicalStep U2_L0R1_3[] = {M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][3].Set(9, U2_L0R1_3, tempRot, tempHandState);
    // U2_L0R1_4
    MechanicalStep U2_L0R1_4[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_R2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][4].Set(9, U2_L0R1_4, tempRot, tempHandState);
    // U2_L0R1_5
    MechanicalStep U2_L0R1_5[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_0_R_1][5].Set(9, U2_L0R1_5, tempRot, tempHandState);
    // U2_L0R1_6
    MechanicalStep U2_L0R1_6[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_0_R_1][6].Set(9, U2_L0R1_6, tempRot, tempHandState);
    // U2_L0R1_7
    MechanicalStep U2_L0R1_7[] = {
        M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][7].Set(10, U2_L0R1_7, tempRot, tempHandState);
    // U2_L0R1_8
    MechanicalStep U2_L0R1_8[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][8].Set(11, U2_L0R1_8, tempRot, tempHandState);
    // U2_L0R1_9
    MechanicalStep U2_L0R1_9[] = {
        M_RO, M_R1, M_RC, M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_0_R_1][9].Set(11, U2_L0R1_9, tempRot, tempHandState);
    // U2_L0R1_10
    MechanicalStep U2_L0R1_10[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][10].Set(11, U2_L0R1_10, tempRot, tempHandState);
    // U2_L0R1_11
    MechanicalStep U2_L0R1_11[] = {
        M_RO, M_R1, M_RC, M_LO, M_R2, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_0_R_1][11].Set(12, U2_L0R1_11, tempRot, tempHandState);
    // U2_L0R1_12
    MechanicalStep U2_L0R1_12[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][12].Set(13, U2_L0R1_12, tempRot, tempHandState);
    // U2_L0R1_13
    MechanicalStep U2_L0R1_13[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][13].Set(13, U2_L0R1_13, tempRot, tempHandState);
    // U2_L0R1_14
    MechanicalStep U2_L0R1_14[] = {M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][14].Set(15, U2_L0R1_14, tempRot, tempHandState);
    // U2_L0R1_15
    MechanicalStep U2_L0R1_15[] = {M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_0_R_1][15].Set(15, U2_L0R1_15, tempRot, tempHandState);
}
void U3_L0R1Init(void)
{
    // U3_L0R1_0
    MechanicalStep U3_L0R1_0[] = {M_LO, M_R3, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][0].Set(4, U3_L0R1_0, tempRot, tempHandState);
    // U3_L0R1_1
    MechanicalStep U3_L0R1_1[] = {M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][1].Set(5, U3_L0R1_1, tempRot, tempHandState);
    // U3_L0R1_2
    MechanicalStep U3_L0R1_2[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_0_R_1][2].Set(8, U3_L0R1_2, tempRot, tempHandState);
    // U3_L0R1_3
    MechanicalStep U3_L0R1_3[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][3].Set(9, U3_L0R1_3, tempRot, tempHandState);
    // U3_L0R1_4
    MechanicalStep U3_L0R1_4[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][4].Set(9, U3_L0R1_4, tempRot, tempHandState);
    // U3_L0R1_5
    MechanicalStep U3_L0R1_5[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_R3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_0_R_1][5].Set(9, U3_L0R1_5, tempRot, tempHandState);
    // U3_L0R1_6
    MechanicalStep U3_L0R1_6[] = {M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][6].Set(9, U3_L0R1_6, tempRot, tempHandState);
    // U3_L0R1_7
    MechanicalStep U3_L0R1_7[] = {
        M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][7].Set(10, U3_L0R1_7, tempRot, tempHandState);
    // U3_L0R1_8
    MechanicalStep U3_L0R1_8[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][8].Set(11, U3_L0R1_8, tempRot, tempHandState);
    // U3_L0R1_9
    MechanicalStep U3_L0R1_9[] = {
        M_RO, M_R1, M_RC, M_LO, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][9].Set(11, U3_L0R1_9, tempRot, tempHandState);
    // U3_L0R1_10
    MechanicalStep U3_L0R1_10[] = {
        M_RO, M_R1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][10].Set(11, U3_L0R1_10, tempRot, tempHandState);
    // U3_L0R1_11
    MechanicalStep U3_L0R1_11[] = {
        M_RO, M_R1, M_RC, M_LO, M_R2, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][11].Set(12, U3_L0R1_11, tempRot, tempHandState);
    // U3_L0R1_12
    MechanicalStep U3_L0R1_12[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_0_R_1][12].Set(13, U3_L0R1_12, tempRot, tempHandState);
    // U3_L0R1_13
    MechanicalStep U3_L0R1_13[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_0_R_1][13].Set(13, U3_L0R1_13, tempRot, tempHandState);
    // U3_L0R1_14
    MechanicalStep U3_L0R1_14[] = {M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][14].Set(15, U3_L0R1_14, tempRot, tempHandState);
    // U3_L0R1_15
    MechanicalStep U3_L0R1_15[] = {M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_0_R_1][15].Set(15, U3_L0R1_15, tempRot, tempHandState);
}
void B1_L0R1Init(void)
{
    // B1_L0R1_0
    MechanicalStep B1_L0R1_0[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][0].Set(7, B1_L0R1_0, tempRot, tempHandState);
    // B1_L0R1_1
    MechanicalStep B1_L0R1_1[] = {M_RO, M_R1, M_RC, M_LO, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][1].Set(8, B1_L0R1_1, tempRot, tempHandState);
    // B1_L0R1_2
    MechanicalStep B1_L0R1_2[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][2].Set(8, B1_L0R1_2, tempRot, tempHandState);
    // B1_L0R1_3
    MechanicalStep B1_L0R1_3[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][3].Set(8, B1_L0R1_3, tempRot, tempHandState);
    // B1_L0R1_4
    MechanicalStep B1_L0R1_4[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_RC, M_R1, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_0_R_1][4].Set(8, B1_L0R1_4, tempRot, tempHandState);
    // B1_L0R1_5
    MechanicalStep B1_L0R1_5[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_RC, M_R1, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_0_R_1][5].Set(8, B1_L0R1_5, tempRot, tempHandState);
    // B1_L0R1_6
    MechanicalStep B1_L0R1_6[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L2, M_RC, M_L1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][6].Set(8, B1_L0R1_6, tempRot, tempHandState);
    // B1_L0R1_7
    MechanicalStep B1_L0R1_7[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][7].Set(9, B1_L0R1_7, tempRot, tempHandState);
    // B1_L0R1_8
    MechanicalStep B1_L0R1_8[] = {M_RO, M_R1, M_L2, M_RC, M_LO, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][8].Set(9, B1_L0R1_8, tempRot, tempHandState);
    // B1_L0R1_9
    MechanicalStep B1_L0R1_9[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][9].Set(9, B1_L0R1_9, tempRot, tempHandState);
    // B1_L0R1_10
    MechanicalStep B1_L0R1_10[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][10].Set(9, B1_L0R1_10, tempRot, tempHandState);
    // B1_L0R1_11
    MechanicalStep B1_L0R1_11[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][11].Set(9, B1_L0R1_11, tempRot, tempHandState);
    // B1_L0R1_12
    MechanicalStep B1_L0R1_12[] = {
        M_LO, M_R1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][12].Set(11, B1_L0R1_12, tempRot, tempHandState);
    // B1_L0R1_13
    MechanicalStep B1_L0R1_13[] = {
        M_LO, M_R1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_0_R_1][13].Set(11, B1_L0R1_13, tempRot, tempHandState);
    // B1_L0R1_14
    MechanicalStep B1_L0R1_14[] = {M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R1,
                                   M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_0_R_1][14].Set(16, B1_L0R1_14, tempRot, tempHandState);
    // B1_L0R1_15
    MechanicalStep B1_L0R1_15[] = {M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R1,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_0_R_1][15].Set(16, B1_L0R1_15, tempRot, tempHandState);
}
void B2_L0R1Init(void)
{
    // B2_L0R1_0
    MechanicalStep B2_L0R1_0[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][0].Set(7, B2_L0R1_0, tempRot, tempHandState);
    // B2_L0R1_1
    MechanicalStep B2_L0R1_1[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_RC, M_R2, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][1].Set(8, B2_L0R1_1, tempRot, tempHandState);
    // B2_L0R1_2
    MechanicalStep B2_L0R1_2[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_RC, M_R2, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][2].Set(8, B2_L0R1_2, tempRot, tempHandState);
    // B2_L0R1_3
    MechanicalStep B2_L0R1_3[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L2, M_RC, M_L2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][3].Set(8, B2_L0R1_3, tempRot, tempHandState);
    // B2_L0R1_4
    MechanicalStep B2_L0R1_4[] = {M_RO, M_R1, M_RC, M_LO, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][4].Set(8, B2_L0R1_4, tempRot, tempHandState);
    // B2_L0R1_5
    MechanicalStep B2_L0R1_5[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][5].Set(8, B2_L0R1_5, tempRot, tempHandState);
    // B2_L0R1_6
    MechanicalStep B2_L0R1_6[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][6].Set(8, B2_L0R1_6, tempRot, tempHandState);
    // B2_L0R1_7
    MechanicalStep B2_L0R1_7[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][7].Set(9, B2_L0R1_7, tempRot, tempHandState);
    // B2_L0R1_8
    MechanicalStep B2_L0R1_8[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][8].Set(9, B2_L0R1_8, tempRot, tempHandState);
    // B2_L0R1_9
    MechanicalStep B2_L0R1_9[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_0_R_1][9].Set(9, B2_L0R1_9, tempRot, tempHandState);
    // B2_L0R1_10
    MechanicalStep B2_L0R1_10[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_0_R_1][10].Set(9, B2_L0R1_10, tempRot, tempHandState);
    // B2_L0R1_11
    MechanicalStep B2_L0R1_11[] = {M_RO, M_R1, M_L2, M_RC, M_LO, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][11].Set(9, B2_L0R1_11, tempRot, tempHandState);
    // B2_L0R1_12
    MechanicalStep B2_L0R1_12[] = {
        M_LO, M_R1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_0_R_1][12].Set(11, B2_L0R1_12, tempRot, tempHandState);
    // B2_L0R1_13
    MechanicalStep B2_L0R1_13[] = {
        M_LO, M_R1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_0_R_1][13].Set(11, B2_L0R1_13, tempRot, tempHandState);
    // B2_L0R1_14
    MechanicalStep B2_L0R1_14[] = {M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R2,
                                   M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][14].Set(16, B2_L0R1_14, tempRot, tempHandState);
    // B2_L0R1_15
    MechanicalStep B2_L0R1_15[] = {M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R2,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_0_R_1][15].Set(16, B2_L0R1_15, tempRot, tempHandState);
}
void B3_L0R1Init(void)
{
    // B3_L0R1_0
    MechanicalStep B3_L0R1_0[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][0].Set(7, B3_L0R1_0, tempRot, tempHandState);
    // B3_L0R1_1
    MechanicalStep B3_L0R1_1[] = {M_RO, M_R1, M_RC, M_LO, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][1].Set(8, B3_L0R1_1, tempRot, tempHandState);
    // B3_L0R1_2
    MechanicalStep B3_L0R1_2[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][2].Set(8, B3_L0R1_2, tempRot, tempHandState);
    // B3_L0R1_3
    MechanicalStep B3_L0R1_3[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][3].Set(8, B3_L0R1_3, tempRot, tempHandState);
    // B3_L0R1_4
    MechanicalStep B3_L0R1_4[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_RC, M_R3, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_0_R_1][4].Set(8, B3_L0R1_4, tempRot, tempHandState);
    // B3_L0R1_5
    MechanicalStep B3_L0R1_5[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_RC, M_R3, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_0_R_1][5].Set(8, B3_L0R1_5, tempRot, tempHandState);
    // B3_L0R1_6
    MechanicalStep B3_L0R1_6[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L2, M_RC, M_L3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][6].Set(8, B3_L0R1_6, tempRot, tempHandState);
    // B3_L0R1_7
    MechanicalStep B3_L0R1_7[] = {M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][7].Set(9, B3_L0R1_7, tempRot, tempHandState);
    // B3_L0R1_8
    MechanicalStep B3_L0R1_8[] = {M_RO, M_R1, M_L2, M_RC, M_LO, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][8].Set(9, B3_L0R1_8, tempRot, tempHandState);
    // B3_L0R1_9
    MechanicalStep B3_L0R1_9[] = {M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][9].Set(9, B3_L0R1_9, tempRot, tempHandState);
    // B3_L0R1_10
    MechanicalStep B3_L0R1_10[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][10].Set(9, B3_L0R1_10, tempRot, tempHandState);
    // B3_L0R1_11
    MechanicalStep B3_L0R1_11[] = {M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][11].Set(9, B3_L0R1_11, tempRot, tempHandState);
    // B3_L0R1_12
    MechanicalStep B3_L0R1_12[] = {
        M_LO, M_R1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][12].Set(11, B3_L0R1_12, tempRot, tempHandState);
    // B3_L0R1_13
    MechanicalStep B3_L0R1_13[] = {
        M_LO, M_R1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_0_R_1][13].Set(11, B3_L0R1_13, tempRot, tempHandState);
    // B3_L0R1_14
    MechanicalStep B3_L0R1_14[] = {M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R3,
                                   M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_0_R_1][14].Set(16, B3_L0R1_14, tempRot, tempHandState);
    // B3_L0R1_15
    MechanicalStep B3_L0R1_15[] = {M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R3,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_0_R_1][15].Set(16, B3_L0R1_15, tempRot, tempHandState);
}
void L1_L0R1Init(void)
{
    // L1_L0R1_0
    MechanicalStep L1_L0R1_0[] = {M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_0_R_1][0].Set(5, L1_L0R1_0, tempRot, tempHandState);
    // L1_L0R1_1
    MechanicalStep L1_L0R1_1[] = {M_RO, M_R1, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][1].Set(6, L1_L0R1_1, tempRot, tempHandState);
    // L1_L0R1_2
    MechanicalStep L1_L0R1_2[] = {M_LO, M_R1, M_LC, M_RO, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][2].Set(8, L1_L0R1_2, tempRot, tempHandState);
    // L1_L0R1_3
    MechanicalStep L1_L0R1_3[] = {M_LO, M_R3, M_LC, M_RO, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][3].Set(8, L1_L0R1_3, tempRot, tempHandState);
    // L1_L0R1_4
    MechanicalStep L1_L0R1_4[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_0_R_1][4].Set(8, L1_L0R1_4, tempRot, tempHandState);
    // L1_L0R1_5
    MechanicalStep L1_L0R1_5[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][5].Set(9, L1_L0R1_5, tempRot, tempHandState);
    // L1_L0R1_6
    MechanicalStep L1_L0R1_6[] = {
        M_LO, M_R3, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_0_R_1][6].Set(11, L1_L0R1_6, tempRot, tempHandState);
    // L1_L0R1_7
    MechanicalStep L1_L0R1_7[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_0_R_1][7].Set(11, L1_L0R1_7, tempRot, tempHandState);
    // L1_L0R1_8
    MechanicalStep L1_L0R1_8[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_LC, M_L1, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][8].Set(12, L1_L0R1_8, tempRot, tempHandState);
    // L1_L0R1_9
    MechanicalStep L1_L0R1_9[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_LC, M_L1, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][9].Set(12, L1_L0R1_9, tempRot, tempHandState);
    // L1_L0R1_10
    MechanicalStep L1_L0R1_10[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][10].Set(12, L1_L0R1_10, tempRot, tempHandState);
    // L1_L0R1_11
    MechanicalStep L1_L0R1_11[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][11].Set(12, L1_L0R1_11, tempRot, tempHandState);
    // L1_L0R1_12
    MechanicalStep L1_L0R1_12[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][12].Set(13, L1_L0R1_12, tempRot, tempHandState);
    // L1_L0R1_13
    MechanicalStep L1_L0R1_13[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][13].Set(13, L1_L0R1_13, tempRot, tempHandState);
    // L1_L0R1_14
    MechanicalStep L1_L0R1_14[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][14].Set(15, L1_L0R1_14, tempRot, tempHandState);
    // L1_L0R1_15
    MechanicalStep L1_L0R1_15[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_0_R_1][15].Set(15, L1_L0R1_15, tempRot, tempHandState);
}
void L2_L0R1Init(void)
{
    // L2_L0R1_0
    MechanicalStep L2_L0R1_0[] = {M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][0].Set(5, L2_L0R1_0, tempRot, tempHandState);
    // L2_L0R1_1
    MechanicalStep L2_L0R1_1[] = {M_RO, M_R1, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_0_R_1][1].Set(6, L2_L0R1_1, tempRot, tempHandState);
    // L2_L0R1_2
    MechanicalStep L2_L0R1_2[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][2].Set(8, L2_L0R1_2, tempRot, tempHandState);
    // L2_L0R1_3
    MechanicalStep L2_L0R1_3[] = {M_LO, M_R1, M_LC, M_RO, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_0_R_1][3].Set(8, L2_L0R1_3, tempRot, tempHandState);
    // L2_L0R1_4
    MechanicalStep L2_L0R1_4[] = {M_LO, M_R3, M_LC, M_RO, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_0_R_1][4].Set(8, L2_L0R1_4, tempRot, tempHandState);
    // L2_L0R1_5
    MechanicalStep L2_L0R1_5[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_0_R_1][5].Set(9, L2_L0R1_5, tempRot, tempHandState);
    // L2_L0R1_6
    MechanicalStep L2_L0R1_6[] = {
        M_LO, M_R3, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][6].Set(11, L2_L0R1_6, tempRot, tempHandState);
    // L2_L0R1_7
    MechanicalStep L2_L0R1_7[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][7].Set(11, L2_L0R1_7, tempRot, tempHandState);
    // L2_L0R1_8
    MechanicalStep L2_L0R1_8[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_LC, M_L2, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][8].Set(12, L2_L0R1_8, tempRot, tempHandState);
    // L2_L0R1_9
    MechanicalStep L2_L0R1_9[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_LC, M_L2, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][9].Set(12, L2_L0R1_9, tempRot, tempHandState);
    // L2_L0R1_10
    MechanicalStep L2_L0R1_10[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][10].Set(12, L2_L0R1_10, tempRot, tempHandState);
    // L2_L0R1_11
    MechanicalStep L2_L0R1_11[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][11].Set(12, L2_L0R1_11, tempRot, tempHandState);
    // L2_L0R1_12
    MechanicalStep L2_L0R1_12[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][12].Set(13, L2_L0R1_12, tempRot, tempHandState);
    // L2_L0R1_13
    MechanicalStep L2_L0R1_13[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][13].Set(13, L2_L0R1_13, tempRot, tempHandState);
    // L2_L0R1_14
    MechanicalStep L2_L0R1_14[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][14].Set(15, L2_L0R1_14, tempRot, tempHandState);
    // L2_L0R1_15
    MechanicalStep L2_L0R1_15[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_0_R_1][15].Set(15, L2_L0R1_15, tempRot, tempHandState);
}
void L3_L0R1Init(void)
{
    // L3_L0R1_0
    MechanicalStep L3_L0R1_0[] = {M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_0_R_1][0].Set(5, L3_L0R1_0, tempRot, tempHandState);
    // L3_L0R1_1
    MechanicalStep L3_L0R1_1[] = {M_RO, M_R1, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][1].Set(6, L3_L0R1_1, tempRot, tempHandState);
    // L3_L0R1_2
    MechanicalStep L3_L0R1_2[] = {M_LO, M_R1, M_LC, M_RO, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][2].Set(8, L3_L0R1_2, tempRot, tempHandState);
    // L3_L0R1_3
    MechanicalStep L3_L0R1_3[] = {M_LO, M_R3, M_LC, M_RO, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][3].Set(8, L3_L0R1_3, tempRot, tempHandState);
    // L3_L0R1_4
    MechanicalStep L3_L0R1_4[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_0_R_1][4].Set(8, L3_L0R1_4, tempRot, tempHandState);
    // L3_L0R1_5
    MechanicalStep L3_L0R1_5[] = {M_LO, M_R2, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][5].Set(9, L3_L0R1_5, tempRot, tempHandState);
    // L3_L0R1_6
    MechanicalStep L3_L0R1_6[] = {
        M_LO, M_R3, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_0_R_1][6].Set(11, L3_L0R1_6, tempRot, tempHandState);
    // L3_L0R1_7
    MechanicalStep L3_L0R1_7[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_0_R_1][7].Set(11, L3_L0R1_7, tempRot, tempHandState);
    // L3_L0R1_8
    MechanicalStep L3_L0R1_8[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_LC, M_L3, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][8].Set(12, L3_L0R1_8, tempRot, tempHandState);
    // L3_L0R1_9
    MechanicalStep L3_L0R1_9[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_LC, M_L3, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][9].Set(12, L3_L0R1_9, tempRot, tempHandState);
    // L3_L0R1_10
    MechanicalStep L3_L0R1_10[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][10].Set(12, L3_L0R1_10, tempRot, tempHandState);
    // L3_L0R1_11
    MechanicalStep L3_L0R1_11[] = {
        M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][11].Set(12, L3_L0R1_11, tempRot, tempHandState);
    // L3_L0R1_12
    MechanicalStep L3_L0R1_12[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][12].Set(13, L3_L0R1_12, tempRot, tempHandState);
    // L3_L0R1_13
    MechanicalStep L3_L0R1_13[] = {
        M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][13].Set(13, L3_L0R1_13, tempRot, tempHandState);
    // L3_L0R1_14
    MechanicalStep L3_L0R1_14[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][14].Set(15, L3_L0R1_14, tempRot, tempHandState);
    // L3_L0R1_15
    MechanicalStep L3_L0R1_15[] = {M_LO,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_0_R_1][15].Set(15, L3_L0R1_15, tempRot, tempHandState);
}
void D1_L0R1Init(void)
{
    // D1_L0R1_0
    MechanicalStep D1_L0R1_0[] = {M_LO, M_R1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][0].Set(4, D1_L0R1_0, tempRot, tempHandState);
    // D1_L0R1_1
    MechanicalStep D1_L0R1_1[] = {M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][1].Set(5, D1_L0R1_1, tempRot, tempHandState);
    // D1_L0R1_2
    MechanicalStep D1_L0R1_2[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_0_R_1][2].Set(8, D1_L0R1_2, tempRot, tempHandState);
    // D1_L0R1_3
    MechanicalStep D1_L0R1_3[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][3].Set(9, D1_L0R1_3, tempRot, tempHandState);
    // D1_L0R1_4
    MechanicalStep D1_L0R1_4[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][4].Set(9, D1_L0R1_4, tempRot, tempHandState);
    // D1_L0R1_5
    MechanicalStep D1_L0R1_5[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_R1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_0_R_1][5].Set(9, D1_L0R1_5, tempRot, tempHandState);
    // D1_L0R1_6
    MechanicalStep D1_L0R1_6[] = {M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R3, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][6].Set(9, D1_L0R1_6, tempRot, tempHandState);
    // D1_L0R1_7
    MechanicalStep D1_L0R1_7[] = {
        M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][7].Set(10, D1_L0R1_7, tempRot, tempHandState);
    // D1_L0R1_8
    MechanicalStep D1_L0R1_8[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][8].Set(11, D1_L0R1_8, tempRot, tempHandState);
    // D1_L0R1_9
    MechanicalStep D1_L0R1_9[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][9].Set(11, D1_L0R1_9, tempRot, tempHandState);
    // D1_L0R1_10
    MechanicalStep D1_L0R1_10[] = {
        M_RO, M_R1, M_RC, M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][10].Set(11, D1_L0R1_10, tempRot, tempHandState);
    // D1_L0R1_11
    MechanicalStep D1_L0R1_11[] = {
        M_RO, M_R1, M_RC, M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][11].Set(12, D1_L0R1_11, tempRot, tempHandState);
    // D1_L0R1_12
    MechanicalStep D1_L0R1_12[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_0_R_1][12].Set(13, D1_L0R1_12, tempRot, tempHandState);
    // D1_L0R1_13
    MechanicalStep D1_L0R1_13[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_0_R_1][13].Set(13, D1_L0R1_13, tempRot, tempHandState);
    // D1_L0R1_14
    MechanicalStep D1_L0R1_14[] = {M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][14].Set(15, D1_L0R1_14, tempRot, tempHandState);
    // D1_L0R1_15
    MechanicalStep D1_L0R1_15[] = {M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_0_R_1][15].Set(15, D1_L0R1_15, tempRot, tempHandState);
}
void D2_L0R1Init(void)
{
    // D2_L0R1_0
    MechanicalStep D2_L0R1_0[] = {M_LO, M_R1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][0].Set(4, D2_L0R1_0, tempRot, tempHandState);
    // D2_L0R1_1
    MechanicalStep D2_L0R1_1[] = {M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][1].Set(5, D2_L0R1_1, tempRot, tempHandState);
    // D2_L0R1_2
    MechanicalStep D2_L0R1_2[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][2].Set(8, D2_L0R1_2, tempRot, tempHandState);
    // D2_L0R1_3
    MechanicalStep D2_L0R1_3[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_R2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][3].Set(9, D2_L0R1_3, tempRot, tempHandState);
    // D2_L0R1_4
    MechanicalStep D2_L0R1_4[] = {M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R3, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][4].Set(9, D2_L0R1_4, tempRot, tempHandState);
    // D2_L0R1_5
    MechanicalStep D2_L0R1_5[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_0_R_1][5].Set(9, D2_L0R1_5, tempRot, tempHandState);
    // D2_L0R1_6
    MechanicalStep D2_L0R1_6[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_0_R_1][6].Set(9, D2_L0R1_6, tempRot, tempHandState);
    // D2_L0R1_7
    MechanicalStep D2_L0R1_7[] = {
        M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][7].Set(10, D2_L0R1_7, tempRot, tempHandState);
    // D2_L0R1_8
    MechanicalStep D2_L0R1_8[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][8].Set(11, D2_L0R1_8, tempRot, tempHandState);
    // D2_L0R1_9
    MechanicalStep D2_L0R1_9[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][9].Set(11, D2_L0R1_9, tempRot, tempHandState);
    // D2_L0R1_10
    MechanicalStep D2_L0R1_10[] = {
        M_RO, M_R1, M_RC, M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_0_R_1][10].Set(11, D2_L0R1_10, tempRot, tempHandState);
    // D2_L0R1_11
    MechanicalStep D2_L0R1_11[] = {
        M_RO, M_R1, M_RC, M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_0_R_1][11].Set(12, D2_L0R1_11, tempRot, tempHandState);
    // D2_L0R1_12
    MechanicalStep D2_L0R1_12[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][12].Set(13, D2_L0R1_12, tempRot, tempHandState);
    // D2_L0R1_13
    MechanicalStep D2_L0R1_13[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][13].Set(13, D2_L0R1_13, tempRot, tempHandState);
    // D2_L0R1_14
    MechanicalStep D2_L0R1_14[] = {M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][14].Set(15, D2_L0R1_14, tempRot, tempHandState);
    // D2_L0R1_15
    MechanicalStep D2_L0R1_15[] = {M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_0_R_1][15].Set(15, D2_L0R1_15, tempRot, tempHandState);
}
void D3_L0R1Init(void)
{
    // D3_L0R1_0
    MechanicalStep D3_L0R1_0[] = {M_LO, M_R1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][0].Set(4, D3_L0R1_0, tempRot, tempHandState);
    // D3_L0R1_1
    MechanicalStep D3_L0R1_1[] = {M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][1].Set(5, D3_L0R1_1, tempRot, tempHandState);
    // D3_L0R1_2
    MechanicalStep D3_L0R1_2[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_0_R_1][2].Set(8, D3_L0R1_2, tempRot, tempHandState);
    // D3_L0R1_3
    MechanicalStep D3_L0R1_3[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][3].Set(9, D3_L0R1_3, tempRot, tempHandState);
    // D3_L0R1_4
    MechanicalStep D3_L0R1_4[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][4].Set(9, D3_L0R1_4, tempRot, tempHandState);
    // D3_L0R1_5
    MechanicalStep D3_L0R1_5[] = {M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_R3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_0_R_1][5].Set(9, D3_L0R1_5, tempRot, tempHandState);
    // D3_L0R1_6
    MechanicalStep D3_L0R1_6[] = {M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R3, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][6].Set(9, D3_L0R1_6, tempRot, tempHandState);
    // D3_L0R1_7
    MechanicalStep D3_L0R1_7[] = {
        M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][7].Set(10, D3_L0R1_7, tempRot, tempHandState);
    // D3_L0R1_8
    MechanicalStep D3_L0R1_8[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][8].Set(11, D3_L0R1_8, tempRot, tempHandState);
    // D3_L0R1_9
    MechanicalStep D3_L0R1_9[] = {
        M_RO, M_R1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][9].Set(11, D3_L0R1_9, tempRot, tempHandState);
    // D3_L0R1_10
    MechanicalStep D3_L0R1_10[] = {
        M_RO, M_R1, M_RC, M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][10].Set(11, D3_L0R1_10, tempRot, tempHandState);
    // D3_L0R1_11
    MechanicalStep D3_L0R1_11[] = {
        M_RO, M_R1, M_RC, M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][11].Set(12, D3_L0R1_11, tempRot, tempHandState);
    // D3_L0R1_12
    MechanicalStep D3_L0R1_12[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_0_R_1][12].Set(13, D3_L0R1_12, tempRot, tempHandState);
    // D3_L0R1_13
    MechanicalStep D3_L0R1_13[] = {
        M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_0_R_1][13].Set(13, D3_L0R1_13, tempRot, tempHandState);
    // D3_L0R1_14
    MechanicalStep D3_L0R1_14[] = {M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][14].Set(15, D3_L0R1_14, tempRot, tempHandState);
    // D3_L0R1_15
    MechanicalStep D3_L0R1_15[] = {M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R2,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_0_R_1][15].Set(15, D3_L0R1_15, tempRot, tempHandState);
}
void F1_L1R0Init(void)
{
    // F1_L1R0_0
    MechanicalStep F1_L1R0_0[] = {M_L1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][0].Set(1, F1_L1R0_0, tempRot, tempHandState);
    // F1_L1R0_1
    MechanicalStep F1_L1R0_1[] = {M_RO, M_L2, M_RC, M_L1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][1].Set(4, F1_L1R0_1, tempRot, tempHandState);
    // F1_L1R0_2
    MechanicalStep F1_L1R0_2[] = {M_LO, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][2].Set(4, F1_L1R0_2, tempRot, tempHandState);
    // F1_L1R0_3
    MechanicalStep F1_L1R0_3[] = {M_RO, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][3].Set(4, F1_L1R0_3, tempRot, tempHandState);
    // F1_L1R0_4
    MechanicalStep F1_L1R0_4[] = {M_RO, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][4].Set(4, F1_L1R0_4, tempRot, tempHandState);
    // F1_L1R0_5
    MechanicalStep F1_L1R0_5[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_RC, M_R1, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_1_R_0][5].Set(12, F1_L1R0_5, tempRot, tempHandState);
    // F1_L1R0_6
    MechanicalStep F1_L1R0_6[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_RC, M_R1, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_1_R_0][6].Set(12, F1_L1R0_6, tempRot, tempHandState);
    // F1_L1R0_7
    MechanicalStep F1_L1R0_7[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_1_R_0][7].Set(12, F1_L1R0_7, tempRot, tempHandState);
    // F1_L1R0_8
    MechanicalStep F1_L1R0_8[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_1][L_1_R_0][8].Set(12, F1_L1R0_8, tempRot, tempHandState);
    // F1_L1R0_9
    MechanicalStep F1_L1R0_9[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][9].Set(13, F1_L1R0_9, tempRot, tempHandState);
    // F1_L1R0_10
    MechanicalStep F1_L1R0_10[] = {
        M_LO, M_L1, M_R2, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][10].Set(13, F1_L1R0_10, tempRot, tempHandState);
    // F1_L1R0_11
    MechanicalStep F1_L1R0_11[] = {
        M_LO, M_L1, M_R2, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][11].Set(13, F1_L1R0_11, tempRot, tempHandState);
    // F1_L1R0_12
    MechanicalStep F1_L1R0_12[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][12].Set(13, F1_L1R0_12, tempRot, tempHandState);
    // F1_L1R0_13
    MechanicalStep F1_L1R0_13[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_LC, M_L1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][13].Set(13, F1_L1R0_13, tempRot, tempHandState);
    // F1_L1R0_14
    MechanicalStep F1_L1R0_14[] = {M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_R1,
                                   M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][14].Set(15, F1_L1R0_14, tempRot, tempHandState);
    // F1_L1R0_15
    MechanicalStep F1_L1R0_15[] = {M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_R1,
                                   M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_1][L_1_R_0][15].Set(15, F1_L1R0_15, tempRot, tempHandState);
}
void F2_L1R0Init(void)
{
    // F2_L1R0_0
    MechanicalStep F2_L1R0_0[] = {M_L2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][0].Set(1, F2_L1R0_0, tempRot, tempHandState);
    // F2_L1R0_1
    MechanicalStep F2_L1R0_1[] = {M_LO, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][1].Set(4, F2_L1R0_1, tempRot, tempHandState);
    // F2_L1R0_2
    MechanicalStep F2_L1R0_2[] = {M_RO, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][2].Set(4, F2_L1R0_2, tempRot, tempHandState);
    // F2_L1R0_3
    MechanicalStep F2_L1R0_3[] = {M_RO, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][3].Set(4, F2_L1R0_3, tempRot, tempHandState);
    // F2_L1R0_4
    MechanicalStep F2_L1R0_4[] = {M_RO, M_L2, M_RC, M_L2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][4].Set(4, F2_L1R0_4, tempRot, tempHandState);
    // F2_L1R0_5
    MechanicalStep F2_L1R0_5[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_RC, M_R2, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][5].Set(12, F2_L1R0_5, tempRot, tempHandState);
    // F2_L1R0_6
    MechanicalStep F2_L1R0_6[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_RC, M_R2, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][6].Set(12, F2_L1R0_6, tempRot, tempHandState);
    // F2_L1R0_7
    MechanicalStep F2_L1R0_7[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][7].Set(12, F2_L1R0_7, tempRot, tempHandState);
    // F2_L1R0_8
    MechanicalStep F2_L1R0_8[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][8].Set(12, F2_L1R0_8, tempRot, tempHandState);
    // F2_L1R0_9
    MechanicalStep F2_L1R0_9[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_LC, M_L2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][9].Set(13, F2_L1R0_9, tempRot, tempHandState);
    // F2_L1R0_10
    MechanicalStep F2_L1R0_10[] = {
        M_LO, M_L1, M_R2, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][10].Set(13, F2_L1R0_10, tempRot, tempHandState);
    // F2_L1R0_11
    MechanicalStep F2_L1R0_11[] = {
        M_LO, M_L1, M_R2, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_2][L_1_R_0][11].Set(13, F2_L1R0_11, tempRot, tempHandState);
    // F2_L1R0_12
    MechanicalStep F2_L1R0_12[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_1_R_0][12].Set(13, F2_L1R0_12, tempRot, tempHandState);
    // F2_L1R0_13
    MechanicalStep F2_L1R0_13[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_1_R_0][13].Set(13, F2_L1R0_13, tempRot, tempHandState);
    // F2_L1R0_14
    MechanicalStep F2_L1R0_14[] = {M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_R2,
                                   M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_1_R_0][14].Set(15, F2_L1R0_14, tempRot, tempHandState);
    // F2_L1R0_15
    MechanicalStep F2_L1R0_15[] = {M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_R2,
                                   M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_2][L_1_R_0][15].Set(15, F2_L1R0_15, tempRot, tempHandState);
}
void F3_L1R0Init(void)
{
    // F3_L1R0_0
    MechanicalStep F3_L1R0_0[] = {M_L3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][0].Set(1, F3_L1R0_0, tempRot, tempHandState);
    // F3_L1R0_1
    MechanicalStep F3_L1R0_1[] = {M_RO, M_L2, M_RC, M_L3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][1].Set(4, F3_L1R0_1, tempRot, tempHandState);
    // F3_L1R0_2
    MechanicalStep F3_L1R0_2[] = {M_LO, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][2].Set(4, F3_L1R0_2, tempRot, tempHandState);
    // F3_L1R0_3
    MechanicalStep F3_L1R0_3[] = {M_RO, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][3].Set(4, F3_L1R0_3, tempRot, tempHandState);
    // F3_L1R0_4
    MechanicalStep F3_L1R0_4[] = {M_RO, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][4].Set(4, F3_L1R0_4, tempRot, tempHandState);
    // F3_L1R0_5
    MechanicalStep F3_L1R0_5[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L3, M_RC, M_R3, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_1_R_0][5].Set(12, F3_L1R0_5, tempRot, tempHandState);
    // F3_L1R0_6
    MechanicalStep F3_L1R0_6[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_RO, M_L1, M_RC, M_R3, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_1_R_0][6].Set(12, F3_L1R0_6, tempRot, tempHandState);
    // F3_L1R0_7
    MechanicalStep F3_L1R0_7[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_1_R_0][7].Set(12, F3_L1R0_7, tempRot, tempHandState);
    // F3_L1R0_8
    MechanicalStep F3_L1R0_8[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[F][_3][L_1_R_0][8].Set(12, F3_L1R0_8, tempRot, tempHandState);
    // F3_L1R0_9
    MechanicalStep F3_L1R0_9[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][9].Set(13, F3_L1R0_9, tempRot, tempHandState);
    // F3_L1R0_10
    MechanicalStep F3_L1R0_10[] = {
        M_LO, M_L1, M_R2, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][10].Set(13, F3_L1R0_10, tempRot, tempHandState);
    // F3_L1R0_11
    MechanicalStep F3_L1R0_11[] = {
        M_LO, M_L1, M_R2, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][11].Set(13, F3_L1R0_11, tempRot, tempHandState);
    // F3_L1R0_12
    MechanicalStep F3_L1R0_12[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][12].Set(13, F3_L1R0_12, tempRot, tempHandState);
    // F3_L1R0_13
    MechanicalStep F3_L1R0_13[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_R1, M_RC, M_LO, M_R1, M_LC, M_L3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][13].Set(13, F3_L1R0_13, tempRot, tempHandState);
    // F3_L1R0_14
    MechanicalStep F3_L1R0_14[] = {M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_R3,
                                   M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][14].Set(15, F3_L1R0_14, tempRot, tempHandState);
    // F3_L1R0_15
    MechanicalStep F3_L1R0_15[] = {M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_R3,
                                   M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[F][_3][L_1_R_0][15].Set(15, F3_L1R0_15, tempRot, tempHandState);
}
void R1_L1R0Init(void)
{
    // R1_L1R0_0
    MechanicalStep R1_L1R0_0[] = {M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_1_R_0][0].Set(4, R1_L1R0_0, tempRot, tempHandState);
    // R1_L1R0_1
    MechanicalStep R1_L1R0_1[] = {M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][1].Set(5, R1_L1R0_1, tempRot, tempHandState);
    // R1_L1R0_2
    MechanicalStep R1_L1R0_2[] = {M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][2].Set(5, R1_L1R0_2, tempRot, tempHandState);
    // R1_L1R0_3
    MechanicalStep R1_L1R0_3[] = {M_LO, M_L1, M_R2, M_LC, M_R1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_1_R_0][3].Set(5, R1_L1R0_3, tempRot, tempHandState);
    // R1_L1R0_4
    MechanicalStep R1_L1R0_4[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][4].Set(8, R1_L1R0_4, tempRot, tempHandState);
    // R1_L1R0_5
    MechanicalStep R1_L1R0_5[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][5].Set(8, R1_L1R0_5, tempRot, tempHandState);
    // R1_L1R0_6
    MechanicalStep R1_L1R0_6[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][6].Set(9, R1_L1R0_6, tempRot, tempHandState);
    // R1_L1R0_7
    MechanicalStep R1_L1R0_7[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][7].Set(9, R1_L1R0_7, tempRot, tempHandState);
    // R1_L1R0_8
    MechanicalStep R1_L1R0_8[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][8].Set(11, R1_L1R0_8, tempRot, tempHandState);
    // R1_L1R0_9
    MechanicalStep R1_L1R0_9[] = {
        M_RO, M_L1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][9].Set(11, R1_L1R0_9, tempRot, tempHandState);
    // R1_L1R0_10
    MechanicalStep R1_L1R0_10[] = {
        M_RO, M_L1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][10].Set(11, R1_L1R0_10, tempRot, tempHandState);
    // R1_L1R0_11
    MechanicalStep R1_L1R0_11[] = {
        M_RO, M_L1, M_RC, M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][11].Set(12, R1_L1R0_11, tempRot, tempHandState);
    // R1_L1R0_12
    MechanicalStep R1_L1R0_12[] = {
        M_RO, M_L2, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_1_R_0][12].Set(12, R1_L1R0_12, tempRot, tempHandState);
    // R1_L1R0_13
    MechanicalStep R1_L1R0_13[] = {
        M_RO, M_L2, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_1][L_1_R_0][13].Set(12, R1_L1R0_13, tempRot, tempHandState);
    // R1_L1R0_14
    MechanicalStep R1_L1R0_14[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L1,
                                   M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][14].Set(19, R1_L1R0_14, tempRot, tempHandState);
    // R1_L1R0_15
    MechanicalStep R1_L1R0_15[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L3,
                                   M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_1][L_1_R_0][15].Set(19, R1_L1R0_15, tempRot, tempHandState);
}
void R2_L1R0Init(void)
{
    // R2_L1R0_0
    MechanicalStep R2_L1R0_0[] = {M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][0].Set(4, R2_L1R0_0, tempRot, tempHandState);
    // R2_L1R0_1
    MechanicalStep R2_L1R0_1[] = {M_LO, M_L1, M_R2, M_LC, M_R2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][1].Set(5, R2_L1R0_1, tempRot, tempHandState);
    // R2_L1R0_2
    MechanicalStep R2_L1R0_2[] = {M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_1_R_0][2].Set(5, R2_L1R0_2, tempRot, tempHandState);
    // R2_L1R0_3
    MechanicalStep R2_L1R0_3[] = {M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_1_R_0][3].Set(5, R2_L1R0_3, tempRot, tempHandState);
    // R2_L1R0_4
    MechanicalStep R2_L1R0_4[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][4].Set(8, R2_L1R0_4, tempRot, tempHandState);
    // R2_L1R0_5
    MechanicalStep R2_L1R0_5[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][5].Set(8, R2_L1R0_5, tempRot, tempHandState);
    // R2_L1R0_6
    MechanicalStep R2_L1R0_6[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][6].Set(9, R2_L1R0_6, tempRot, tempHandState);
    // R2_L1R0_7
    MechanicalStep R2_L1R0_7[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][7].Set(9, R2_L1R0_7, tempRot, tempHandState);
    // R2_L1R0_8
    MechanicalStep R2_L1R0_8[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_1_R_0][8].Set(11, R2_L1R0_8, tempRot, tempHandState);
    // R2_L1R0_9
    MechanicalStep R2_L1R0_9[] = {
        M_RO, M_L1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][9].Set(11, R2_L1R0_9, tempRot, tempHandState);
    // R2_L1R0_10
    MechanicalStep R2_L1R0_10[] = {
        M_RO, M_L1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][10].Set(11, R2_L1R0_10, tempRot, tempHandState);
    // R2_L1R0_11
    MechanicalStep R2_L1R0_11[] = {
        M_RO, M_L2, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][11].Set(12, R2_L1R0_11, tempRot, tempHandState);
    // R2_L1R0_12
    MechanicalStep R2_L1R0_12[] = {
        M_RO, M_L2, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][12].Set(12, R2_L1R0_12, tempRot, tempHandState);
    // R2_L1R0_13
    MechanicalStep R2_L1R0_13[] = {
        M_RO, M_L1, M_RC, M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_2][L_1_R_0][13].Set(12, R2_L1R0_13, tempRot, tempHandState);
    // R2_L1R0_14
    MechanicalStep R2_L1R0_14[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L1,
                                   M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][14].Set(19, R2_L1R0_14, tempRot, tempHandState);
    // R2_L1R0_15
    MechanicalStep R2_L1R0_15[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L3,
                                   M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_2][L_1_R_0][15].Set(19, R2_L1R0_15, tempRot, tempHandState);
}
void R3_L1R0Init(void)
{
    // R3_L1R0_0
    MechanicalStep R3_L1R0_0[] = {M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_1_R_0][0].Set(4, R3_L1R0_0, tempRot, tempHandState);
    // R3_L1R0_1
    MechanicalStep R3_L1R0_1[] = {M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][1].Set(5, R3_L1R0_1, tempRot, tempHandState);
    // R3_L1R0_2
    MechanicalStep R3_L1R0_2[] = {M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][2].Set(5, R3_L1R0_2, tempRot, tempHandState);
    // R3_L1R0_3
    MechanicalStep R3_L1R0_3[] = {M_LO, M_L1, M_R2, M_LC, M_R3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_1_R_0][3].Set(5, R3_L1R0_3, tempRot, tempHandState);
    // R3_L1R0_4
    MechanicalStep R3_L1R0_4[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][4].Set(8, R3_L1R0_4, tempRot, tempHandState);
    // R3_L1R0_5
    MechanicalStep R3_L1R0_5[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][5].Set(8, R3_L1R0_5, tempRot, tempHandState);
    // R3_L1R0_6
    MechanicalStep R3_L1R0_6[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][6].Set(9, R3_L1R0_6, tempRot, tempHandState);
    // R3_L1R0_7
    MechanicalStep R3_L1R0_7[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][7].Set(9, R3_L1R0_7, tempRot, tempHandState);
    // R3_L1R0_8
    MechanicalStep R3_L1R0_8[] = {
        M_RO, M_L1, M_RC, M_LO, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 1, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][8].Set(11, R3_L1R0_8, tempRot, tempHandState);
    // R3_L1R0_9
    MechanicalStep R3_L1R0_9[] = {
        M_RO, M_L1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][9].Set(11, R3_L1R0_9, tempRot, tempHandState);
    // R3_L1R0_10
    MechanicalStep R3_L1R0_10[] = {
        M_RO, M_L1, M_RC, M_LO, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][10].Set(11, R3_L1R0_10, tempRot, tempHandState);
    // R3_L1R0_11
    MechanicalStep R3_L1R0_11[] = {
        M_RO, M_L1, M_RC, M_LO, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][11].Set(12, R3_L1R0_11, tempRot, tempHandState);
    // R3_L1R0_12
    MechanicalStep R3_L1R0_12[] = {
        M_RO, M_L2, M_RC, M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_1_R_0][12].Set(12, R3_L1R0_12, tempRot, tempHandState);
    // R3_L1R0_13
    MechanicalStep R3_L1R0_13[] = {
        M_RO, M_L2, M_RC, M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[R][_3][L_1_R_0][13].Set(12, R3_L1R0_13, tempRot, tempHandState);
    // R3_L1R0_14
    MechanicalStep R3_L1R0_14[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L1,
                                   M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][14].Set(19, R3_L1R0_14, tempRot, tempHandState);
    // R3_L1R0_15
    MechanicalStep R3_L1R0_15[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L3,
                                   M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(1, 1, 0, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[R][_3][L_1_R_0][15].Set(19, R3_L1R0_15, tempRot, tempHandState);
}
void U1_L1R0Init(void)
{
    // U1_L1R0_0
    MechanicalStep U1_L1R0_0[] = {M_RO, M_L1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_1_R_0][0].Set(4, U1_L1R0_0, tempRot, tempHandState);
    // U1_L1R0_1
    MechanicalStep U1_L1R0_1[] = {M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][1].Set(5, U1_L1R0_1, tempRot, tempHandState);
    // U1_L1R0_2
    MechanicalStep U1_L1R0_2[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][2].Set(8, U1_L1R0_2, tempRot, tempHandState);
    // U1_L1R0_3
    MechanicalStep U1_L1R0_3[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][3].Set(9, U1_L1R0_3, tempRot, tempHandState);
    // U1_L1R0_4
    MechanicalStep U1_L1R0_4[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][4].Set(9, U1_L1R0_4, tempRot, tempHandState);
    // U1_L1R0_5
    MechanicalStep U1_L1R0_5[] = {M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L3, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_1_R_0][5].Set(9, U1_L1R0_5, tempRot, tempHandState);
    // U1_L1R0_6
    MechanicalStep U1_L1R0_6[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_L1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][6].Set(9, U1_L1R0_6, tempRot, tempHandState);
    // U1_L1R0_7
    MechanicalStep U1_L1R0_7[] = {
        M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][7].Set(10, U1_L1R0_7, tempRot, tempHandState);
    // U1_L1R0_8
    MechanicalStep U1_L1R0_8[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][8].Set(11, U1_L1R0_8, tempRot, tempHandState);
    // U1_L1R0_9
    MechanicalStep U1_L1R0_9[] = {
        M_LO, M_L1, M_LC, M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][9].Set(11, U1_L1R0_9, tempRot, tempHandState);
    // U1_L1R0_10
    MechanicalStep U1_L1R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][10].Set(11, U1_L1R0_10, tempRot, tempHandState);
    // U1_L1R0_11
    MechanicalStep U1_L1R0_11[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][11].Set(12, U1_L1R0_11, tempRot, tempHandState);
    // U1_L1R0_12
    MechanicalStep U1_L1R0_12[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][12].Set(13, U1_L1R0_12, tempRot, tempHandState);
    // U1_L1R0_13
    MechanicalStep U1_L1R0_13[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_1][L_1_R_0][13].Set(13, U1_L1R0_13, tempRot, tempHandState);
    // U1_L1R0_14
    MechanicalStep U1_L1R0_14[] = {M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R1,
                                   M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_1_R_0][14].Set(15, U1_L1R0_14, tempRot, tempHandState);
    // U1_L1R0_15
    MechanicalStep U1_L1R0_15[] = {M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R1,
                                   M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_1][L_1_R_0][15].Set(15, U1_L1R0_15, tempRot, tempHandState);
}
void U2_L1R0Init(void)
{
    // U2_L1R0_0
    MechanicalStep U2_L1R0_0[] = {M_RO, M_L1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][0].Set(4, U2_L1R0_0, tempRot, tempHandState);
    // U2_L1R0_1
    MechanicalStep U2_L1R0_1[] = {M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_1_R_0][1].Set(5, U2_L1R0_1, tempRot, tempHandState);
    // U2_L1R0_2
    MechanicalStep U2_L1R0_2[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][2].Set(8, U2_L1R0_2, tempRot, tempHandState);
    // U2_L1R0_3
    MechanicalStep U2_L1R0_3[] = {M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L3, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][3].Set(9, U2_L1R0_3, tempRot, tempHandState);
    // U2_L1R0_4
    MechanicalStep U2_L1R0_4[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_L2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][4].Set(9, U2_L1R0_4, tempRot, tempHandState);
    // U2_L1R0_5
    MechanicalStep U2_L1R0_5[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][5].Set(9, U2_L1R0_5, tempRot, tempHandState);
    // U2_L1R0_6
    MechanicalStep U2_L1R0_6[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][6].Set(9, U2_L1R0_6, tempRot, tempHandState);
    // U2_L1R0_7
    MechanicalStep U2_L1R0_7[] = {
        M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_1_R_0][7].Set(10, U2_L1R0_7, tempRot, tempHandState);
    // U2_L1R0_8
    MechanicalStep U2_L1R0_8[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_1_R_0][8].Set(11, U2_L1R0_8, tempRot, tempHandState);
    // U2_L1R0_9
    MechanicalStep U2_L1R0_9[] = {
        M_LO, M_L1, M_LC, M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][9].Set(11, U2_L1R0_9, tempRot, tempHandState);
    // U2_L1R0_10
    MechanicalStep U2_L1R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_2][L_1_R_0][10].Set(11, U2_L1R0_10, tempRot, tempHandState);
    // U2_L1R0_11
    MechanicalStep U2_L1R0_11[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][11].Set(12, U2_L1R0_11, tempRot, tempHandState);
    // U2_L1R0_12
    MechanicalStep U2_L1R0_12[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][12].Set(13, U2_L1R0_12, tempRot, tempHandState);
    // U2_L1R0_13
    MechanicalStep U2_L1R0_13[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][13].Set(13, U2_L1R0_13, tempRot, tempHandState);
    // U2_L1R0_14
    MechanicalStep U2_L1R0_14[] = {M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R2,
                                   M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][14].Set(15, U2_L1R0_14, tempRot, tempHandState);
    // U2_L1R0_15
    MechanicalStep U2_L1R0_15[] = {M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R2,
                                   M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_2][L_1_R_0][15].Set(15, U2_L1R0_15, tempRot, tempHandState);
}
void U3_L1R0Init(void)
{
    // U3_L1R0_0
    MechanicalStep U3_L1R0_0[] = {M_RO, M_L1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_1_R_0][0].Set(4, U3_L1R0_0, tempRot, tempHandState);
    // U3_L1R0_1
    MechanicalStep U3_L1R0_1[] = {M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 2, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][1].Set(5, U3_L1R0_1, tempRot, tempHandState);
    // U3_L1R0_2
    MechanicalStep U3_L1R0_2[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][2].Set(8, U3_L1R0_2, tempRot, tempHandState);
    // U3_L1R0_3
    MechanicalStep U3_L1R0_3[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][3].Set(9, U3_L1R0_3, tempRot, tempHandState);
    // U3_L1R0_4
    MechanicalStep U3_L1R0_4[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][4].Set(9, U3_L1R0_4, tempRot, tempHandState);
    // U3_L1R0_5
    MechanicalStep U3_L1R0_5[] = {M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L3, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_1_R_0][5].Set(9, U3_L1R0_5, tempRot, tempHandState);
    // U3_L1R0_6
    MechanicalStep U3_L1R0_6[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_L3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][6].Set(9, U3_L1R0_6, tempRot, tempHandState);
    // U3_L1R0_7
    MechanicalStep U3_L1R0_7[] = {
        M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L3, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][7].Set(10, U3_L1R0_7, tempRot, tempHandState);
    // U3_L1R0_8
    MechanicalStep U3_L1R0_8[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][8].Set(11, U3_L1R0_8, tempRot, tempHandState);
    // U3_L1R0_9
    MechanicalStep U3_L1R0_9[] = {
        M_LO, M_L1, M_LC, M_RO, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 1, 1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][9].Set(11, U3_L1R0_9, tempRot, tempHandState);
    // U3_L1R0_10
    MechanicalStep U3_L1R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][10].Set(11, U3_L1R0_10, tempRot, tempHandState);
    // U3_L1R0_11
    MechanicalStep U3_L1R0_11[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][11].Set(12, U3_L1R0_11, tempRot, tempHandState);
    // U3_L1R0_12
    MechanicalStep U3_L1R0_12[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(1, 1, 2, 1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][12].Set(13, U3_L1R0_12, tempRot, tempHandState);
    // U3_L1R0_13
    MechanicalStep U3_L1R0_13[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[U][_3][L_1_R_0][13].Set(13, U3_L1R0_13, tempRot, tempHandState);
    // U3_L1R0_14
    MechanicalStep U3_L1R0_14[] = {M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R3,
                                   M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_1_R_0][14].Set(15, U3_L1R0_14, tempRot, tempHandState);
    // U3_L1R0_15
    MechanicalStep U3_L1R0_15[] = {M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R3,
                                   M_END};
    tempRot.Set(2, 1, 0, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[U][_3][L_1_R_0][15].Set(15, U3_L1R0_15, tempRot, tempHandState);
}
void B1_L1R0Init(void)
{
    // B1_L1R0_0
    MechanicalStep B1_L1R0_0[] = {M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][0].Set(5, B1_L1R0_0, tempRot, tempHandState);
    // B1_L1R0_1
    MechanicalStep B1_L1R0_1[] = {M_LO, M_L1, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][1].Set(6, B1_L1R0_1, tempRot, tempHandState);
    // B1_L1R0_2
    MechanicalStep B1_L1R0_2[] = {M_RO, M_L3, M_RC, M_LO, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][2].Set(8, B1_L1R0_2, tempRot, tempHandState);
    // B1_L1R0_3
    MechanicalStep B1_L1R0_3[] = {M_RO, M_L1, M_RC, M_LO, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][3].Set(8, B1_L1R0_3, tempRot, tempHandState);
    // B1_L1R0_4
    MechanicalStep B1_L1R0_4[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][4].Set(8, B1_L1R0_4, tempRot, tempHandState);
    // B1_L1R0_5
    MechanicalStep B1_L1R0_5[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][5].Set(9, B1_L1R0_5, tempRot, tempHandState);
    // B1_L1R0_6
    MechanicalStep B1_L1R0_6[] = {
        M_RO, M_L3, M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][6].Set(11, B1_L1R0_6, tempRot, tempHandState);
    // B1_L1R0_7
    MechanicalStep B1_L1R0_7[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][7].Set(11, B1_L1R0_7, tempRot, tempHandState);
    // B1_L1R0_8
    MechanicalStep B1_L1R0_8[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_RC, M_R1, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_1_R_0][8].Set(12, B1_L1R0_8, tempRot, tempHandState);
    // B1_L1R0_9
    MechanicalStep B1_L1R0_9[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_RC, M_R1, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_1_R_0][9].Set(12, B1_L1R0_9, tempRot, tempHandState);
    // B1_L1R0_10
    MechanicalStep B1_L1R0_10[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_1_R_0][10].Set(12, B1_L1R0_10, tempRot, tempHandState);
    // B1_L1R0_11
    MechanicalStep B1_L1R0_11[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_1][L_1_R_0][11].Set(12, B1_L1R0_11, tempRot, tempHandState);
    // B1_L1R0_12
    MechanicalStep B1_L1R0_12[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][12].Set(13, B1_L1R0_12, tempRot, tempHandState);
    // B1_L1R0_13
    MechanicalStep B1_L1R0_13[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][13].Set(13, B1_L1R0_13, tempRot, tempHandState);
    // B1_L1R0_14
    MechanicalStep B1_L1R0_14[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_R1,
                                   M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][14].Set(15, B1_L1R0_14, tempRot, tempHandState);
    // B1_L1R0_15
    MechanicalStep B1_L1R0_15[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_R1,
                                   M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_1][L_1_R_0][15].Set(15, B1_L1R0_15, tempRot, tempHandState);
}
void B2_L1R0Init(void)
{
    // B2_L1R0_0
    MechanicalStep B2_L1R0_0[] = {M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][0].Set(5, B2_L1R0_0, tempRot, tempHandState);
    // B2_L1R0_1
    MechanicalStep B2_L1R0_1[] = {M_LO, M_L1, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][1].Set(6, B2_L1R0_1, tempRot, tempHandState);
    // B2_L1R0_2
    MechanicalStep B2_L1R0_2[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][2].Set(8, B2_L1R0_2, tempRot, tempHandState);
    // B2_L1R0_3
    MechanicalStep B2_L1R0_3[] = {M_RO, M_L3, M_RC, M_LO, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][3].Set(8, B2_L1R0_3, tempRot, tempHandState);
    // B2_L1R0_4
    MechanicalStep B2_L1R0_4[] = {M_RO, M_L1, M_RC, M_LO, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][4].Set(8, B2_L1R0_4, tempRot, tempHandState);
    // B2_L1R0_5
    MechanicalStep B2_L1R0_5[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][5].Set(9, B2_L1R0_5, tempRot, tempHandState);
    // B2_L1R0_6
    MechanicalStep B2_L1R0_6[] = {
        M_RO, M_L3, M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][6].Set(11, B2_L1R0_6, tempRot, tempHandState);
    // B2_L1R0_7
    MechanicalStep B2_L1R0_7[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][7].Set(11, B2_L1R0_7, tempRot, tempHandState);
    // B2_L1R0_8
    MechanicalStep B2_L1R0_8[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_RC, M_R2, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][8].Set(12, B2_L1R0_8, tempRot, tempHandState);
    // B2_L1R0_9
    MechanicalStep B2_L1R0_9[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_RC, M_R2, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][9].Set(12, B2_L1R0_9, tempRot, tempHandState);
    // B2_L1R0_10
    MechanicalStep B2_L1R0_10[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][10].Set(12, B2_L1R0_10, tempRot, tempHandState);
    // B2_L1R0_11
    MechanicalStep B2_L1R0_11[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_2][L_1_R_0][11].Set(12, B2_L1R0_11, tempRot, tempHandState);
    // B2_L1R0_12
    MechanicalStep B2_L1R0_12[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_1_R_0][12].Set(13, B2_L1R0_12, tempRot, tempHandState);
    // B2_L1R0_13
    MechanicalStep B2_L1R0_13[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_1_R_0][13].Set(13, B2_L1R0_13, tempRot, tempHandState);
    // B2_L1R0_14
    MechanicalStep B2_L1R0_14[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_R2,
                                   M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_1_R_0][14].Set(15, B2_L1R0_14, tempRot, tempHandState);
    // B2_L1R0_15
    MechanicalStep B2_L1R0_15[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_R2,
                                   M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_2][L_1_R_0][15].Set(15, B2_L1R0_15, tempRot, tempHandState);
}
void B3_L1R0Init(void)
{
    // B3_L1R0_0
    MechanicalStep B3_L1R0_0[] = {M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][0].Set(5, B3_L1R0_0, tempRot, tempHandState);
    // B3_L1R0_1
    MechanicalStep B3_L1R0_1[] = {M_LO, M_L1, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 1, 1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][1].Set(6, B3_L1R0_1, tempRot, tempHandState);
    // B3_L1R0_2
    MechanicalStep B3_L1R0_2[] = {M_RO, M_L3, M_RC, M_LO, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][2].Set(8, B3_L1R0_2, tempRot, tempHandState);
    // B3_L1R0_3
    MechanicalStep B3_L1R0_3[] = {M_RO, M_L1, M_RC, M_LO, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][3].Set(8, B3_L1R0_3, tempRot, tempHandState);
    // B3_L1R0_4
    MechanicalStep B3_L1R0_4[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][4].Set(8, B3_L1R0_4, tempRot, tempHandState);
    // B3_L1R0_5
    MechanicalStep B3_L1R0_5[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][5].Set(9, B3_L1R0_5, tempRot, tempHandState);
    // B3_L1R0_6
    MechanicalStep B3_L1R0_6[] = {
        M_RO, M_L3, M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][6].Set(11, B3_L1R0_6, tempRot, tempHandState);
    // B3_L1R0_7
    MechanicalStep B3_L1R0_7[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R2, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(0, -1, 2, 1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][7].Set(11, B3_L1R0_7, tempRot, tempHandState);
    // B3_L1R0_8
    MechanicalStep B3_L1R0_8[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_RO, M_L3, M_RC, M_R3, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_1_R_0][8].Set(12, B3_L1R0_8, tempRot, tempHandState);
    // B3_L1R0_9
    MechanicalStep B3_L1R0_9[] = {
        M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_RO, M_L1, M_RC, M_R3, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_1_R_0][9].Set(12, B3_L1R0_9, tempRot, tempHandState);
    // B3_L1R0_10
    MechanicalStep B3_L1R0_10[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_1_R_0][10].Set(12, B3_L1R0_10, tempRot, tempHandState);
    // B3_L1R0_11
    MechanicalStep B3_L1R0_11[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[B][_3][L_1_R_0][11].Set(12, B3_L1R0_11, tempRot, tempHandState);
    // B3_L1R0_12
    MechanicalStep B3_L1R0_12[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(1, -1, 0, 1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][12].Set(13, B3_L1R0_12, tempRot, tempHandState);
    // B3_L1R0_13
    MechanicalStep B3_L1R0_13[] = {
        M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][13].Set(13, B3_L1R0_13, tempRot, tempHandState);
    // B3_L1R0_14
    MechanicalStep B3_L1R0_14[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L3,
                                   M_R1,
                                   M_RC,
                                   M_R3,
                                   M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][14].Set(15, B3_L1R0_14, tempRot, tempHandState);
    // B3_L1R0_15
    MechanicalStep B3_L1R0_15[] = {M_LO,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_R3,
                                   M_END};
    tempRot.Set(1, -1, 2, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[B][_3][L_1_R_0][15].Set(15, B3_L1R0_15, tempRot, tempHandState);
}
void L1_L1R0Init(void)
{
    // L1_L1R0_0
    MechanicalStep L1_L1R0_0[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_1_R_0][0].Set(7, L1_L1R0_0, tempRot, tempHandState);
    // L1_L1R0_1
    MechanicalStep L1_L1R0_1[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][1].Set(8, L1_L1R0_1, tempRot, tempHandState);
    // L1_L1R0_2
    MechanicalStep L1_L1R0_2[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][2].Set(8, L1_L1R0_2, tempRot, tempHandState);
    // L1_L1R0_3
    MechanicalStep L1_L1R0_3[] = {M_LO, M_L1, M_LC, M_RO, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][3].Set(8, L1_L1R0_3, tempRot, tempHandState);
    // L1_L1R0_4
    MechanicalStep L1_L1R0_4[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][4].Set(8, L1_L1R0_4, tempRot, tempHandState);
    // L1_L1R0_5
    MechanicalStep L1_L1R0_5[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][5].Set(8, L1_L1R0_5, tempRot, tempHandState);
    // L1_L1R0_6
    MechanicalStep L1_L1R0_6[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_R1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_1_R_0][6].Set(8, L1_L1R0_6, tempRot, tempHandState);
    // L1_L1R0_7
    MechanicalStep L1_L1R0_7[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][7].Set(9, L1_L1R0_7, tempRot, tempHandState);
    // L1_L1R0_8
    MechanicalStep L1_L1R0_8[] = {M_LO, M_L1, M_R2, M_LC, M_RO, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][8].Set(9, L1_L1R0_8, tempRot, tempHandState);
    // L1_L1R0_9
    MechanicalStep L1_L1R0_9[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][9].Set(9, L1_L1R0_9, tempRot, tempHandState);
    // L1_L1R0_10
    MechanicalStep L1_L1R0_10[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_1_R_0][10].Set(9, L1_L1R0_10, tempRot, tempHandState);
    // L1_L1R0_11
    MechanicalStep L1_L1R0_11[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_1][L_1_R_0][11].Set(9, L1_L1R0_11, tempRot, tempHandState);
    // L1_L1R0_12
    MechanicalStep L1_L1R0_12[] = {
        M_RO, M_L1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][12].Set(11, L1_L1R0_12, tempRot, tempHandState);
    // L1_L1R0_13
    MechanicalStep L1_L1R0_13[] = {
        M_RO, M_L1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][13].Set(11, L1_L1R0_13, tempRot, tempHandState);
    // L1_L1R0_14
    MechanicalStep L1_L1R0_14[] = {M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][14].Set(16, L1_L1R0_14, tempRot, tempHandState);
    // L1_L1R0_15
    MechanicalStep L1_L1R0_15[] = {M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L1,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_1][L_1_R_0][15].Set(16, L1_L1R0_15, tempRot, tempHandState);
}
void L2_L1R0Init(void)
{
    // L2_L1R0_0
    MechanicalStep L2_L1R0_0[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][0].Set(7, L2_L1R0_0, tempRot, tempHandState);
    // L2_L1R0_1
    MechanicalStep L2_L1R0_1[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][1].Set(8, L2_L1R0_1, tempRot, tempHandState);
    // L2_L1R0_2
    MechanicalStep L2_L1R0_2[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][2].Set(8, L2_L1R0_2, tempRot, tempHandState);
    // L2_L1R0_3
    MechanicalStep L2_L1R0_3[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_R2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][3].Set(8, L2_L1R0_3, tempRot, tempHandState);
    // L2_L1R0_4
    MechanicalStep L2_L1R0_4[] = {M_LO, M_L1, M_LC, M_RO, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_1_R_0][4].Set(8, L2_L1R0_4, tempRot, tempHandState);
    // L2_L1R0_5
    MechanicalStep L2_L1R0_5[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_1_R_0][5].Set(8, L2_L1R0_5, tempRot, tempHandState);
    // L2_L1R0_6
    MechanicalStep L2_L1R0_6[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_1_R_0][6].Set(8, L2_L1R0_6, tempRot, tempHandState);
    // L2_L1R0_7
    MechanicalStep L2_L1R0_7[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][7].Set(9, L2_L1R0_7, tempRot, tempHandState);
    // L2_L1R0_8
    MechanicalStep L2_L1R0_8[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R2, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][8].Set(9, L2_L1R0_8, tempRot, tempHandState);
    // L2_L1R0_9
    MechanicalStep L2_L1R0_9[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][9].Set(9, L2_L1R0_9, tempRot, tempHandState);
    // L2_L1R0_10
    MechanicalStep L2_L1R0_10[] = {M_LO, M_L1, M_R2, M_LC, M_RO, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_2][L_1_R_0][10].Set(9, L2_L1R0_10, tempRot, tempHandState);
    // L2_L1R0_11
    MechanicalStep L2_L1R0_11[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][11].Set(9, L2_L1R0_11, tempRot, tempHandState);
    // L2_L1R0_12
    MechanicalStep L2_L1R0_12[] = {
        M_RO, M_L1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][12].Set(11, L2_L1R0_12, tempRot, tempHandState);
    // L2_L1R0_13
    MechanicalStep L2_L1R0_13[] = {
        M_RO, M_L1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][13].Set(11, L2_L1R0_13, tempRot, tempHandState);
    // L2_L1R0_14
    MechanicalStep L2_L1R0_14[] = {M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][14].Set(16, L2_L1R0_14, tempRot, tempHandState);
    // L2_L1R0_15
    MechanicalStep L2_L1R0_15[] = {M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L2,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_2][L_1_R_0][15].Set(16, L2_L1R0_15, tempRot, tempHandState);
}
void L3_L1R0Init(void)
{
    // L3_L1R0_0
    MechanicalStep L3_L1R0_0[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_LC, M_R3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_1_R_0][0].Set(7, L3_L1R0_0, tempRot, tempHandState);
    // L3_L1R0_1
    MechanicalStep L3_L1R0_1[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][1].Set(8, L3_L1R0_1, tempRot, tempHandState);
    // L3_L1R0_2
    MechanicalStep L3_L1R0_2[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][2].Set(8, L3_L1R0_2, tempRot, tempHandState);
    // L3_L1R0_3
    MechanicalStep L3_L1R0_3[] = {M_LO, M_L1, M_LC, M_RO, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 1, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][3].Set(8, L3_L1R0_3, tempRot, tempHandState);
    // L3_L1R0_4
    MechanicalStep L3_L1R0_4[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][4].Set(8, L3_L1R0_4, tempRot, tempHandState);
    // L3_L1R0_5
    MechanicalStep L3_L1R0_5[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][5].Set(8, L3_L1R0_5, tempRot, tempHandState);
    // L3_L1R0_6
    MechanicalStep L3_L1R0_6[] = {M_RO, M_L2, M_RC, M_LO, M_L1, M_R2, M_LC, M_R3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_1_R_0][6].Set(8, L3_L1R0_6, tempRot, tempHandState);
    // L3_L1R0_7
    MechanicalStep L3_L1R0_7[] = {M_RO, M_L1, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 0, -1, 1, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][7].Set(9, L3_L1R0_7, tempRot, tempHandState);
    // L3_L1R0_8
    MechanicalStep L3_L1R0_8[] = {M_LO, M_L1, M_R2, M_LC, M_RO, M_L2, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 1, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][8].Set(9, L3_L1R0_8, tempRot, tempHandState);
    // L3_L1R0_9
    MechanicalStep L3_L1R0_9[] = {M_RO, M_L3, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][9].Set(9, L3_L1R0_9, tempRot, tempHandState);
    // L3_L1R0_10
    MechanicalStep L3_L1R0_10[] = {M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, 1, 1, -1, 0, 1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_1_R_0][10].Set(9, L3_L1R0_10, tempRot, tempHandState);
    // L3_L1R0_11
    MechanicalStep L3_L1R0_11[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_R3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[L][_3][L_1_R_0][11].Set(9, L3_L1R0_11, tempRot, tempHandState);
    // L3_L1R0_12
    MechanicalStep L3_L1R0_12[] = {
        M_RO, M_L1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][12].Set(11, L3_L1R0_12, tempRot, tempHandState);
    // L3_L1R0_13
    MechanicalStep L3_L1R0_13[] = {
        M_RO, M_L1, M_RC, M_LO, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][13].Set(11, L3_L1R0_13, tempRot, tempHandState);
    // L3_L1R0_14
    MechanicalStep L3_L1R0_14[] = {M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L1,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R3,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(1, 1, 0, -1, 2, 1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][14].Set(16, L3_L1R0_14, tempRot, tempHandState);
    // L3_L1R0_15
    MechanicalStep L3_L1R0_15[] = {M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_L3,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_R1,
                                   M_LC,
                                   M_RO,
                                   M_R1,
                                   M_RC,
                                   M_L3,
                                   M_END};
    tempRot.Set(1, -1, 0, -1, 2, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[L][_3][L_1_R_0][15].Set(16, L3_L1R0_15, tempRot, tempHandState);
}
void D1_L1R0Init(void)
{
    // D1_L1R0_0
    MechanicalStep D1_L1R0_0[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_RC, M_L1, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][0].Set(8, D1_L1R0_0, tempRot, tempHandState);
    // D1_L1R0_1
    MechanicalStep D1_L1R0_1[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L1, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][1].Set(9, D1_L1R0_1, tempRot, tempHandState);
    // D1_L1R0_2
    MechanicalStep D1_L1R0_2[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L1, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][2].Set(9, D1_L1R0_2, tempRot, tempHandState);
    // D1_L1R0_3
    MechanicalStep D1_L1R0_3[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_L1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][3].Set(9, D1_L1R0_3, tempRot, tempHandState);
    // D1_L1R0_4
    MechanicalStep D1_L1R0_4[] = {M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_1_R_0][4].Set(9, D1_L1R0_4, tempRot, tempHandState);
    // D1_L1R0_5
    MechanicalStep D1_L1R0_5[] = {
        M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][5].Set(10, D1_L1R0_5, tempRot, tempHandState);
    // D1_L1R0_6
    MechanicalStep D1_L1R0_6[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_LC, M_R1, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_1_R_0][6].Set(10, D1_L1R0_6, tempRot, tempHandState);
    // D1_L1R0_7
    MechanicalStep D1_L1R0_7[] = {
        M_LO, M_L1, M_LC, M_RO, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][7].Set(11, D1_L1R0_7, tempRot, tempHandState);
    // D1_L1R0_8
    MechanicalStep D1_L1R0_8[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R1, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][8].Set(11, D1_L1R0_8, tempRot, tempHandState);
    // D1_L1R0_9
    MechanicalStep D1_L1R0_9[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R1, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][9].Set(11, D1_L1R0_9, tempRot, tempHandState);
    // D1_L1R0_10
    MechanicalStep D1_L1R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L1, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][10].Set(12, D1_L1R0_10, tempRot, tempHandState);
    // D1_L1R0_11
    MechanicalStep D1_L1R0_11[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][11].Set(13, D1_L1R0_11, tempRot, tempHandState);
    // D1_L1R0_12
    MechanicalStep D1_L1R0_12[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L1, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][12].Set(13, D1_L1R0_12, tempRot, tempHandState);
    // D1_L1R0_13
    MechanicalStep D1_L1R0_13[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_LC, M_RO, M_L2, M_R1, M_RC, M_R1, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_1][L_1_R_0][13].Set(14, D1_L1R0_13, tempRot, tempHandState);
    // D1_L1R0_14
    MechanicalStep D1_L1R0_14[] = {M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R1,
                                   M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_1_R_0][14].Set(15, D1_L1R0_14, tempRot, tempHandState);
    // D1_L1R0_15
    MechanicalStep D1_L1R0_15[] = {M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R1,
                                   M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_1][L_1_R_0][15].Set(15, D1_L1R0_15, tempRot, tempHandState);
}
void D2_L1R0Init(void)
{
    // D2_L1R0_0
    MechanicalStep D2_L1R0_0[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_RC, M_L2, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][0].Set(8, D2_L1R0_0, tempRot, tempHandState);
    // D2_L1R0_1
    MechanicalStep D2_L1R0_1[] = {M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][1].Set(9, D2_L1R0_1, tempRot, tempHandState);
    // D2_L1R0_2
    MechanicalStep D2_L1R0_2[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_L2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][2].Set(9, D2_L1R0_2, tempRot, tempHandState);
    // D2_L1R0_3
    MechanicalStep D2_L1R0_3[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L2, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][3].Set(9, D2_L1R0_3, tempRot, tempHandState);
    // D2_L1R0_4
    MechanicalStep D2_L1R0_4[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L2, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][4].Set(9, D2_L1R0_4, tempRot, tempHandState);
    // D2_L1R0_5
    MechanicalStep D2_L1R0_5[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_LC, M_R2, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][5].Set(10, D2_L1R0_5, tempRot, tempHandState);
    // D2_L1R0_6
    MechanicalStep D2_L1R0_6[] = {
        M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_1_R_0][6].Set(10, D2_L1R0_6, tempRot, tempHandState);
    // D2_L1R0_7
    MechanicalStep D2_L1R0_7[] = {
        M_LO, M_L1, M_LC, M_RO, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][7].Set(11, D2_L1R0_7, tempRot, tempHandState);
    // D2_L1R0_8
    MechanicalStep D2_L1R0_8[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R2, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_1_R_0][8].Set(11, D2_L1R0_8, tempRot, tempHandState);
    // D2_L1R0_9
    MechanicalStep D2_L1R0_9[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R2, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_1_R_0][9].Set(11, D2_L1R0_9, tempRot, tempHandState);
    // D2_L1R0_10
    MechanicalStep D2_L1R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L2, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][10].Set(12, D2_L1R0_10, tempRot, tempHandState);
    // D2_L1R0_11
    MechanicalStep D2_L1R0_11[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][11].Set(13, D2_L1R0_11, tempRot, tempHandState);
    // D2_L1R0_12
    MechanicalStep D2_L1R0_12[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L2, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][12].Set(13, D2_L1R0_12, tempRot, tempHandState);
    // D2_L1R0_13
    MechanicalStep D2_L1R0_13[] = {
        M_LO, M_L1, M_LC, M_RO, M_L1, M_RC, M_LO, M_L1, M_LC, M_RO, M_L2, M_R1, M_RC, M_R2, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_2][L_1_R_0][13].Set(14, D2_L1R0_13, tempRot, tempHandState);
    // D2_L1R0_14
    MechanicalStep D2_L1R0_14[] = {M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R2,
                                   M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][14].Set(15, D2_L1R0_14, tempRot, tempHandState);
    // D2_L1R0_15
    MechanicalStep D2_L1R0_15[] = {M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R2,
                                   M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_2][L_1_R_0][15].Set(15, D2_L1R0_15, tempRot, tempHandState);
}
void D3_L1R0Init(void)
{
    // D3_L1R0_0
    MechanicalStep D3_L1R0_0[] = {M_RO, M_L3, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_1_R_0][0].Set(4, D3_L1R0_0, tempRot, tempHandState);
    // D3_L1R0_1
    MechanicalStep D3_L1R0_1[] = {M_RO, M_L3, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, 1, 2, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][1].Set(5, D3_L1R0_1, tempRot, tempHandState);
    // D3_L1R0_2
    MechanicalStep D3_L1R0_2[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_RC, M_L3, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][2].Set(8, D3_L1R0_2, tempRot, tempHandState);
    // D3_L1R0_3
    MechanicalStep D3_L1R0_3[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L1, M_RC, M_L3, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][3].Set(9, D3_L1R0_3, tempRot, tempHandState);
    // D3_L1R0_4
    MechanicalStep D3_L1R0_4[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L3, M_RC, M_L3, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][4].Set(9, D3_L1R0_4, tempRot, tempHandState);
    // D3_L1R0_5
    MechanicalStep D3_L1R0_5[] = {M_LO, M_L1, M_R1, M_LC, M_RO, M_R1, M_L2, M_RC, M_L3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][5].Set(9, D3_L1R0_5, tempRot, tempHandState);
    // D3_L1R0_6
    MechanicalStep D3_L1R0_6[] = {M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_1_R_0][6].Set(9, D3_L1R0_6, tempRot, tempHandState);
    // D3_L1R0_7
    MechanicalStep D3_L1R0_7[] = {
        M_LO, M_L1, M_R2, M_L1, M_LC, M_RO, M_L1, M_R1, M_RC, M_R3, M_END};
    tempRot.Set(0, -1, 2, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][7].Set(10, D3_L1R0_7, tempRot, tempHandState);
    // D3_L1R0_8
    MechanicalStep D3_L1R0_8[] = {
        M_LO, M_L1, M_LC, M_RO, M_R1, M_RC, M_LO, M_R1, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, 1, 1, 1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][8].Set(11, D3_L1R0_8, tempRot, tempHandState);
    // D3_L1R0_9
    MechanicalStep D3_L1R0_9[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R1, M_LC, M_R3, M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][9].Set(11, D3_L1R0_9, tempRot, tempHandState);
    // D3_L1R0_10
    MechanicalStep D3_L1R0_10[] = {
        M_LO, M_L1, M_LC, M_RO, M_L3, M_RC, M_LO, M_L1, M_R3, M_LC, M_R3, M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][10].Set(11, D3_L1R0_10, tempRot, tempHandState);
    // D3_L1R0_11
    MechanicalStep D3_L1R0_11[] = {
        M_LO, M_L1, M_LC, M_RO, M_L2, M_R1, M_RC, M_LO, M_R3, M_L1, M_LC, M_L3, M_END};
    tempRot.Set(2, -1, 1, -1, 0, -1);
    tempHandState.Set(0, 0, 0, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][11].Set(12, D3_L1R0_11, tempRot, tempHandState);
    // D3_L1R0_12
    MechanicalStep D3_L1R0_12[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L1, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(1, -1, 2, 1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][12].Set(13, D3_L1R0_12, tempRot, tempHandState);
    // D3_L1R0_13
    MechanicalStep D3_L1R0_13[] = {
        M_LO, M_L1, M_R3, M_LC, M_RO, M_R1, M_L3, M_RC, M_LO, M_L1, M_R2, M_LC, M_L3, M_END};
    tempRot.Set(1, 1, 2, -1, 0, -1);
    tempHandState.Set(0, 0, 1, 0);
    MechanicalGroupLib[D][_3][L_1_R_0][13].Set(13, D3_L1R0_13, tempRot, tempHandState);
    // D3_L1R0_14
    MechanicalStep D3_L1R0_14[] = {M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R1,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R3,
                                   M_END};
    tempRot.Set(2, -1, 0, 1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_1_R_0][14].Set(15, D3_L1R0_14, tempRot, tempHandState);
    // D3_L1R0_15
    MechanicalStep D3_L1R0_15[] = {M_RO,
                                   M_L1,
                                   M_R1,
                                   M_RC,
                                   M_LO,
                                   M_R3,
                                   M_L1,
                                   M_LC,
                                   M_RO,
                                   M_L2,
                                   M_RC,
                                   M_LO,
                                   M_L1,
                                   M_LC,
                                   M_R3,
                                   M_END};
    tempRot.Set(2, 1, 0, -1, 1, -1);
    tempHandState.Set(0, 0, 0, 1);
    MechanicalGroupLib[D][_3][L_1_R_0][15].Set(15, D3_L1R0_15, tempRot, tempHandState);
}

}   // namespace robotstep
