///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <chrono>

#include <common/CVUtils.h>
#include <common/ErrorCodes.h>

using timepoint = std::chrono::steady_clock::time_point;

using namespace std;
using namespace cv;

namespace ar {
	void RealtimeLocalVideoStream::restart() {
		start_time_ = chrono::steady_clock::now();
		frame_cnt_ = 0;
	}

	ERROR_CODE RealtimeLocalVideoStream::open(const char* video_path) {
		cap_ = VideoCapture(video_path);
		if (!cap_.isOpened())
			return AR_FILE_NOT_FOUND;

		fps_ = cap_.get(CAP_PROP_FPS);

		return AR_SUCCESS;
	}

	ERROR_CODE RealtimeLocalVideoStream::nextFrame(Mat& output_buf) {
		if (!cap_.isOpened())
			return AR_UNINITIALIZED;

		auto now = chrono::steady_clock::now();
		int dest_frame_ind = (now - start_time_).count() * fps_ / 1000000000;
		while (frame_cnt_++ < dest_frame_ind)
			cap_.grab();
		bool ret = cap_.retrieve(output_buf);
		if (!ret || output_buf.empty())
			return AR_NO_MORE_FRAMES;
		else
			return AR_SUCCESS;
	}
}