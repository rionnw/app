#pragma once
#include <iostream>
#include <string>

namespace robotstep {
/**
 * 电机动作常量 L/R  1/2/3/O/C  90/180/270/OPEN/CLOSE
 *
 * "M_L1", "M_L2", "M_L3", "M_LC", "M_LO", "M_R1", "M_R2", "M_R3", "M_RC", "M_RO"
 */
const std::string moveStr[10] = {"4", "3", "2", "0", "1", "9", "8", "7", "5", "6"};

/**
 * 运动时间常量
 */
const int HandCloseTime    = 300;   // 气缸合并时间
const int HandOpenTime     = 300;   // 气缸打开时间
const int HandMove90       = 300;   // 手臂拧动90度所需时间
const int HandMove180      = 300;   // 手臂拧动180度所需时间
const int DelayBetwen2Step = 200;   // 两个连续步骤之间的延迟时间

/**
 * 初始化操作库中各个操作组合的总时间
 */
const int Time_Air  = 120;   // 气缸动
const int Tim_KZ90  = 53;    // 空转
const int Tim_ND90  = 54;    // 拧动 90 度
const int Tim_ND180 = 90;    // 拧动 180 度
const int Tim_DD90  = 122;   // 带动 90 度
const int Tim_DD180 = 194;   // 带动 180 度

const int CLOSE = 0;
const int OPEN  = 1;

/**
 * 手臂状态结构体 定义初始手臂状态
 */
struct HandState
{
    int IsLeftOpen  = 0;   // 0为闭合
    int IsRightOpen = 0;

    int LeftNotNice  = 0;   // 0为nice
    int RightNotNice = 0;

    void Set(int _IsLeftOpen, int _IsRightOpen, int _LeftNotNice, int _RightNotNice)
    {
        IsLeftOpen   = _IsLeftOpen;
        IsRightOpen  = _IsRightOpen;
        LeftNotNice  = _LeftNotNice;
        RightNotNice = _RightNotNice;
    }
};

/**
 * 旋转矩阵相关结构体
 */
struct Rot
{
    int  a[3][3];
    void Set(int row0, int num0, int row1, int num1, int row2, int num2)
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) a[i][j] = 0;

        a[row0][0] = num0;
        a[row1][1] = num1;
        a[row2][2] = num2;
    }
};

/**
 * 定义空间点
 */
struct Point3
{
    int         a[3][1];
    std::string name;
};

/**
 * 理论步骤
 */
struct TheoryStep
{
    Point3 face;
    int    distance;
};

/**
 * 机械步骤结构体
 */
struct MechanicalStep   // 机械步骤单步
{
    std::string name;
    int         time;
    int         num;
};

/**
 * 机械步骤组合结构体
 */
struct MechanicalGroup   // 机械步骤
{
    int              time    = 0;    // 此Group运行所需时间
    int              StepNum = 0;    // 步骤数量
    struct Rot       rot;            // 此步骤的旋转矩阵
    struct HandState endHandState;   // 末点时刻手臂的状态
    // 每个理论步骤解算出来的机械步骤不超过15步，加上最后一步M_END应<=16
    struct MechanicalStep Steps[20];

    void Set(int _StepNum, struct MechanicalStep* _Steps, struct Rot _rot, struct HandState _state)
    {
        StepNum      = _StepNum;
        rot          = _rot;
        endHandState = _state;
        int i;
        for (i = 0; _Steps[i].name != "M_END"; i++) {
            if (i >= 20) {
                std::cout << "SetError!" << std::endl;
                break;
            }

            Steps[i] = _Steps[i];
        }
        Steps[i] = {"M_END", 0, -1};
    }
};

// 宏
constexpr auto L_0_R_0 = 0;   // 两个机械臂都竖着
constexpr auto L_0_R_1 = 1;   // 左边竖着，右边横着;
constexpr auto L_1_R_0 = 2;   // 右边竖着，左边横着;

constexpr auto F = 0;
constexpr auto R = 1;
constexpr auto U = 2;
constexpr auto B = 3;
constexpr auto L = 4;
constexpr auto D = 5;

constexpr auto _1 = 0;
constexpr auto _2 = 1;
constexpr auto _3 = 2;

constexpr auto L1 = 0;
constexpr auto L2 = 1;
constexpr auto L3 = 2;
constexpr auto LC = 3;
constexpr auto LO = 4;
constexpr auto R1 = 5;
constexpr auto R2 = 6;
constexpr auto R3 = 7;
constexpr auto RC = 8;
constexpr auto RO = 9;

/**
 * 调用层函数
 */
void        allInit(void);
int         search(const std::string& theoryStr);
std::string getSteps();
void        dfs(int step, int state);
void        bookInit(void);
int         char2Int(char inChar);

/**
 * 底层封装函数 谨慎改动
 */
void RobotStepsInit(void);
void RotInit(void);
void PointInit(void);
void TimeLibInit(void);
void OperateLibInit(void);

Rot    RotMtplRot(Rot l, Rot r);
Point3 RotMtplPoint3(Rot l, Point3 r);

void F1_L0R0Init();
void F1_L0R1Init();
void F1_L1R0Init();
void F2_L0R0Init();
void F2_L0R1Init();
void F2_L1R0Init();
void F3_L0R0Init();
void F3_L0R1Init();
void F3_L1R0Init();
void R1_L0R0Init();
void R1_L0R1Init();
void R1_L1R0Init();
void R2_L0R0Init();
void R2_L0R1Init();
void R2_L1R0Init();
void R3_L0R0Init();
void R3_L0R1Init();
void R3_L1R0Init();
void U1_L0R0Init();
void U1_L0R1Init();
void U1_L1R0Init();
void U2_L0R0Init();
void U2_L0R1Init();
void U2_L1R0Init();
void U3_L0R0Init();
void U3_L0R1Init();
void U3_L1R0Init();
void B1_L0R0Init();
void B1_L0R1Init();
void B1_L1R0Init();
void B2_L0R0Init();
void B2_L0R1Init();
void B2_L1R0Init();
void B3_L0R0Init();
void B3_L0R1Init();
void B3_L1R0Init();
void L1_L0R0Init();
void L1_L0R1Init();
void L1_L1R0Init();
void L2_L0R0Init();
void L2_L0R1Init();
void L2_L1R0Init();
void L3_L0R0Init();
void L3_L0R1Init();
void L3_L1R0Init();
void D1_L0R0Init();
void D1_L0R1Init();
void D1_L1R0Init();
void D2_L0R0Init();
void D2_L0R1Init();
void D2_L1R0Init();
void D3_L0R0Init();
void D3_L0R1Init();
void D3_L1R0Init();

}   // namespace robotstep