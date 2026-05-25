#pragma once

#include <string>

#include "HandStep.h"

class StepOptimizer
{
public:
	StepOptimizer();
	~StepOptimizer();

    std::string getHandSteps(const std::string& cubeFace);
    std::string getRobotSteps(const std::string& cubeFace);

private:
    int threadsNum = 6;      // 并行求解线程数（启用超线程）
    int timeLimit = 25;      // 单次求解超时限制（毫秒）
    int solutionNum = 3;     // 期望获取的最优解数量
    int maxStepLen = -1;     // 最大搜索步数（-1表示不限制）
    int n_splits = 1;        // 搜索空间分割组数（threadsNum/n_splits ≈ 6）

    handstep::Engine* handStepSolver;  // 人手拧动步骤求解器
};
