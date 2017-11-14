///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#pragma once

#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>

#ifdef _WIN32
#ifdef COMMON_EXPORTS
#define COMMON_API __declspec(dllexport)
#else
#define COMMON_API __declspec(dllimport)
#endif
#else
#define COMMON_API
#endif

namespace ar {
	enum VObjType {
		SCREEN
	};

	//! Data collected by the motion sensors.
	struct MotionData {
		std::chrono::steady_clock::time_point shot_time;
		//TODO: Fill the data here.
	};

	cv::Mat COMMON_API RefineFundamentalMatrix(const cv::Mat& fundamentalMatrix,
		const std::vector<cv::Point2d>& point1,
		const std::vector<cv::Point2d>& point2);
}
