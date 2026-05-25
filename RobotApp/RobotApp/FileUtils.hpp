#pragma once

#include <vector>

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

using json = nlohmann::json;


static const std::string roiFile = "roi.json";

/*
 * 保存 ROI
 * @param rois ROI
 * @return bool 是否成功
 */
bool saveRois(std::vector<cv::Rect>& rois)
{
	json roiJson;
	for (const auto& roi : rois) {
		json roiData;
		roiData["x"] = roi.x;
		roiData["y"] = roi.y;
		roiData["width"] = roi.width;
		roiData["height"] = roi.height;
		roiJson["rois"].push_back(roiData);
	}

	std::ofstream f(roiFile);
	if (!f.is_open()) {
		return false;
	}
	f << roiJson.dump(2);   // 4 spaces for indentation
	f.close();
	return true;
}

/*
 * 读取 ROI
 * @return std::vector<cv::Rect> ROI
 */
std::vector<cv::Rect> readRois()
{
	std::ifstream f(roiFile);
	if (!f.is_open()) {
		return {};
	}
	json roiJson;
	f >> roiJson;
	f.close();

	std::vector<cv::Rect> rois;
	// 检查是否存在 "rois" 键 以及 是否是数组
	if (!roiJson.contains("rois") || !roiJson["rois"].is_array()) {
		return {};
	}
	for (const auto& roiData : roiJson["rois"]) {
		cv::Rect roi;
		roi.x = roiData["x"];
		roi.y = roiData["y"];
		roi.width = roiData["width"];
		roi.height = roiData["height"];
		rois.push_back(roi);
	}

	// 检查是否读取到数据
	if (rois.size() != 54) {
		spdlog::warn("Failed to read ROIs from file. Expected 54, got {}", rois.size());
		spdlog::warn("Please relabel the rois.");
	}

	return rois;
}

struct CtrlParams {
	int right = 0;   // 右侧控制参数
	int left = 0;    // 左侧控制参数

	double motorSpeed = 0.0; // 电机速度
	double airSpeed = 0.0; 
};

/*
 * 初始化 ROI
 */
std::vector<cv::Rect> initRois(void)
{
	std::vector<cv::Rect> rois;
	// 读取文件
	rois = readRois();
	if (rois.empty()) {
		spdlog::info("Failed to read ROIs from file. Initializing with default values.");
		// 如果文件不存在，则初始化
		for (int i = 0; i < 54; ++i) {
			int x = (i % 9) * 50;
			int y = (i / 9) * 50;
			int width = 10;
			int height = 10;
			rois.push_back(cv::Rect(x, y, width, height));
		}
		saveRois(rois);
	}
	return rois;
}

bool saveCtrlParams(const CtrlParams& params) {
	json ctrlJson;
	ctrlJson["ControlParams"] = true;
	std::ofstream f("params.json");
	if (!f.is_open()) {
		return false;
	}

	ctrlJson["params"]["right"] = params.right;
	ctrlJson["params"]["left"] = params.left;
	ctrlJson["params"]["motorSpeed"] = params.motorSpeed;
	ctrlJson["params"]["airSpeed"] = params.airSpeed;

	f << ctrlJson.dump(2);   // 4 spaces for indentation
	f.close();
	return true;
}

CtrlParams readCtrlParams() {
	CtrlParams params;
	std::ifstream f("params.json");
	if (!f.is_open()) {
		return params; // 返回默认值
	}
	json ctrlJson;
	f >> ctrlJson;
	f.close();
	if (ctrlJson.contains("params") && ctrlJson.size() == 4) {
		params.right = ctrlJson.value("right", 0);
		params.left = ctrlJson.value("left", 0);
		params.motorSpeed = ctrlJson.value("motorSpeed", 0.0);
		params.airSpeed = ctrlJson.value("airSpeed", 0.0);
	}
	return params;
}

CtrlParams initCtrlParams() {
	return readCtrlParams();
}