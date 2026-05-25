#include "StepOptimizer.h"

#include <sstream>

#include <spdlog/spdlog.h>

#include "Coord.h"
#include "Cubie.h"
#include "Face.h"
#include "Move.h"
#include "Prun.h"
#include "RobotStep.h"
#include "Sym.h"

StepOptimizer::StepOptimizer()
{
	// 初始化求解器参数
	threadsNum = std::thread::hardware_concurrency() - 2;    // 自动检测CPU核心数（含超线程）
	timeLimit = 25;   // 单次求解最大时间限制(毫秒)
	solutionNum = 3;  // 期望获取的最优解数量
	maxStepLen = -1;  // 最大搜索步数(-1表示不限制)

	// 计算并行搜索分组数，确保每个分组约有6个线程
	n_splits = std::max(1, threadsNum / 6);  // 保证n_splits至少为1

	// 记录初始化信息
	spdlog::info("Initializing StepOptimizer solver...");
	spdlog::info("Configuration parameters:\n"
		"  Threads: {}\n"
		"  Time Limit: {} ms\n"
		"  Solutions: {}\n"
		"  Max Steps: {}\n"
		"  Search Groups: {}",
		threadsNum, timeLimit, solutionNum, maxStepLen, n_splits);

	// 初始化魔方基础数据结构
	face::init();     // 初始化面数据
	move::init();     // 初始化移动操作表
	coord::init();    // 初始化坐标系统
	sym::init();      // 初始化对称性操作

	// 初始化剪枝表(如果失败则终止程序)
	if (prun::init(true)) {
		spdlog::critical("Pruning table initialization failed!");
		exit(EXIT_FAILURE);
	}

	// 创建手动解法求解引擎
	handStepSolver = new handstep::Engine(threadsNum, timeLimit, solutionNum, maxStepLen, n_splits);

	// 初始化机器人解法相关组件
	robotstep::allInit();

	// 记录初始化成功
	spdlog::info("StepOptimizer initialization complete. Ready for solving!");
}

std::string StepOptimizer::getHandSteps(const std::string& cubeFace)
{
	// 创建魔方状态对象
	cubie::cube c;

	// 存储找到的所有解
	std::vector<std::vector<int>> sols;

	// 将面表示法转换为立方体表示法
	auto err = face::to_cubie(cubeFace, c);
	if (err != 0) {
		spdlog::error("Face Error {}", err);
		return ""; // 转换失败，返回空解
	}

	// 检查立方体表示的合法性
	err = cubie::check(c);
	if (err != 0) {
		spdlog::error("Cubie Error {}", err);
		return ""; // 状态非法，返回空解
	}

	// 使用启发式搜索算法求解魔方
	handStepSolver->solve(c, sols);

	// 处理无解的情况
	if (sols.empty()) {
		return ""; // 未找到解，返回空字符串
	}

	// 找到步数最少的解
	auto minIt = std::min_element(sols.begin(), sols.end(),
		[](const auto& a, const auto& b) {
			return a.size() < b.size(); // 比较解的步数
		});

	// 将最优解转换为标准魔方操作序列字符串
	std::stringstream ss;
	for (int moveIdx : *minIt) {
		// 确保操作索引在有效范围内
		if (moveIdx >= 0 && moveIdx < move::COUNT) {
			ss << move::names[moveIdx] << " "; // 转换为标准符号表示
		}
	}

	// 重置求解器，为下一次求解做准备
	handStepSolver->prepare();

	// 返回最优解的操作序列
	return ss.str();
}

std::string StepOptimizer::getRobotSteps(const std::string& cubeFace)
{
	// 生成人手拧动步骤
	auto handSteps = getHandSteps(cubeFace);

	// 检查是否生成了解法
	if (handSteps.empty()) {
		return ""; // 未找到解法，返回空字符串
	}

	// 基于人手解法生成机器人可执行的优化步骤
	// 该函数会分析 handSteps 并生成适合机器人执行的动作序列
	robotstep::search(handSteps);

	// 返回双臂二指机器人优化后的解法步骤
	return robotstep::getSteps();
}

StepOptimizer::~StepOptimizer()
{
	handStepSolver->finish();
	delete handStepSolver;
}