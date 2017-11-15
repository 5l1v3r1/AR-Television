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
		TV
	};

	//! Data collected by the motion sensors.
	struct MotionData {
		std::chrono::steady_clock::time_point shot_time;
		//TODO: Fill the data here.
	};

	cv::Mat COMMON_API RefineFundamentalMatrix(const cv::Mat& fundamentalMatrix,
		const std::vector<cv::Point2d>& point1,
		const std::vector<cv::Point2d>& point2);

	//! Recover rotation and translation from an essential matrix.
	//	This operation produces four posible results, each is a pair of
	//	rotation matrix and translation vector.
	std::vector<std::pair<cv::Mat, cv::Mat>> RecoverRotationAndTranslation(const cv::Mat& essential_matrix);

	//! Calculate the relative rotation and translation from camera 1 to camera 2,
	//	given their own rotations and translations with respect to the world coordinate.
	std::pair<cv::Mat, cv::Mat> CalculateRelativeRotationAndTranslation(cv::Mat R1, cv::Mat t1, cv::Mat R2, cv::Mat t2);
}
