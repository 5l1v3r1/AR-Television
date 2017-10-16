///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <opencv2/opencv.hpp>

#include <common/ErrorCodes.h>

namespace ar
{
	//! The interface FrameStream specifies the routine of frame stream classes.
	//	Subclasses should retrieve video content from various sources, and return
	//	a required frame on call of the nextFrame method.
	class FrameStream {
	public:
		virtual int nextFrame(cv::Mat& outputBuf) = 0;
	};

	class RealtimeLocalVideoStream : public FrameStream {
		cv::VideoCapture cap_;
		int fps_;
		std::chrono::steady_clock::time_point start_time_;
		int frame_cnt_;
	public:
		inline RealtimeLocalVideoStream() { restart(); }
		void restart();
		ERROR_CODE open(const char* videoPath);
		ERROR_CODE nextFrame(cv::Mat& outputBuf);
	};
}