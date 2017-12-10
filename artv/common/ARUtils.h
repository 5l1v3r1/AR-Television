////////////////////////////////////////////////////////////
/// AR Television
/// Copyright(c) 2017 Carnegie Mellon University
/// Licensed under The MIT License[see LICENSE for details]
/// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
////////////////////////////////////////////////////////////
#pragma once

#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>

#include <common/ErrorCodes.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

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

	/// Data collected by the motion sensors.
	struct MotionData {
		std::chrono::steady_clock::time_point shot_time;
		//TODO: Fill the data here.
	};

    /// Plot matched points between two images.
    void PlotMatches(const cv::Mat& img1,
					 const cv::Mat& img2,
					 const std::vector<cv::Point2f>& pts1,
					 const std::vector<cv::Point2f>& pts2,
					 cv::Mat& out);

	/// Recover rotation and translation from an essential matrix.
	///	This operation produces four posible results, each is a 3x4 matrix that combines
	///	rotation and translation.
	std::vector<cv::Mat> COMMON_API RecoverRotAndTranslation(const cv::Mat& essential_matrix);

	/// Calculate the relative rotation and translation from camera 1 to camera 2,
	///	given their own rotations and translations with respect to the world coordinate.
	std::pair<cv::Mat, cv::Mat> COMMON_API CalRelRotAndTranslation(cv::Mat R1, cv::Mat t1, cv::Mat R2, cv::Mat t2);

	/// Input a series of camera matrices and 2D points. The 2D points are all matched in order to relate to some 3D points.
	///	Output the estimation of 3D points and estimation error.
	ERROR_CODE COMMON_API Triangulate(const std::vector<std::pair<cv::Mat, cv::Mat>> &camera_matrices_and_2d_points,
									  cv::Mat &points3d,
									  double *error = nullptr);

    void COMMON_API BundleAdjustment(cv::Mat K1, cv::Mat M1, cv::Mat p1,
                                     cv::Mat K2, cv::Mat& M2, cv::Mat p2,
                                     cv::Mat K3, cv::Mat& M3, cv::Mat p3,
                                     cv::Mat &Points3d);
	cv::Mat COMMON_API CombineExtrinsics(const cv::Mat& base, const cv::Mat& rel);
}
