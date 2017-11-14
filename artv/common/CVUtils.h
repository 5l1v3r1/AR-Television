///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#pragma once

#ifndef CVUTILS_H
#define CVUTILS_H

#include <vector>
#include <string>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>

#include <common/ErrorCodes.h>

#ifdef _WIN32
#ifdef COMMON_EXPORTS
#define COMMON_API __declspec(dllexport)
#else
#define COMMON_API __declspec(dllimport)
#endif
#else
#define COMMON_API
#endif

namespace ar
{
	//! The interface FrameStream specifies the routine of frame stream classes.
	//	Subclasses should retrieve video content from various sources, and return
	//	a required frame on call of the nextFrame method.
	class COMMON_API FrameStream {
	public:
		virtual int NextFrame(cv::Mat& outputBuf) = 0;
	};

	class COMMON_API RealtimeLocalVideoStream : public FrameStream {
		cv::VideoCapture cap_;
		double fps_;
		std::chrono::steady_clock::time_point start_time_;
		int frame_cnt_;
	public:
		inline RealtimeLocalVideoStream() { Restart(); }
		void Restart();
		ERROR_CODE Open(const char* videoPath);
		ERROR_CODE NextFrame(cv::Mat& outputBuf);
	};

	//! Detect interest points in an image by DoG.
	std::vector<cv::Point> COMMON_API DetectInterestPointsDoG(cv::Mat image);

	class COMMON_API InterestPointsTracker
	{
	public:
		struct Stats {
			int keypoints;
			int matches;
			int inliers;
			int ratio;
		};

		InterestPointsTracker(cv::Ptr<cv::Feature2D> _detector, cv::Ptr<cv::DescriptorMatcher> _matcher) :
			detector_(_detector),
			matcher_(_matcher)
		{}

		void GenKeypointsDesc(const cv::Mat& frame, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
		void MatchNewKeypoints(const cv::Mat& frame, cv::Mat& former_desc, std::vector<cv::KeyPoint>& former_kp);
	protected:
		const double RANSAC_THRESH = 2.5f; // RANSAC inlier threshold
		const double NN_MATCH_RATIO = 0.8f; // Nearest-neighbour matching ratio
		const int STATS_UPDATE_PERIOD = 10; // On-screen statistics are updated every 10 frames
		cv::Ptr<cv::Feature2D> detector_;
		cv::Ptr<cv::DescriptorMatcher> matcher_;
	};
}

#endif //CVUTILS_H