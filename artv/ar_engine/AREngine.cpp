///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <ar_engine/AREngine.h>

using namespace cv;

namespace ar {
	ERROR_CODE AREngine::GetMixedScene(const Mat& raw_scene, Mat& mixed_scene) {
		// TODO: Accumulate the motion data.
		accumulated_motion_data_.clear();

		// TODO: This is only a fake function. Need real implementation.
		mixed_scene = raw_scene;
		return AR_SUCCESS;
	}

	ERROR_CODE AREngine::CreateScreen(cv::Point2i location, FrameStream& content_stream) {
		// TODO: This is only a fake function. Need real implementation.
		return AR_SUCCESS;
	}

	int AREngine::GetTopVObj(int x, int y) const {
		// TODO: This is only a fake function. Need real implementation.
		return -1;
	}

	//!	Drag a virtual object to a location. The virtual object is stripped from the
	//	real world by then, and its shape and size in the scene remain the same during
	//	the dragging. Call FixVObj to fix the virtual object onto the real world again.
	ERROR_CODE AREngine::DragVObj(int id, int x, int y) {
		// TODO: This is only a fake function. Need real implementation.
		return AR_SUCCESS;
	}

	//! Fix a virtual object that is floating to the real world. The orientation
	//	and size might be adjusted to fit the new location.
	ERROR_CODE AREngine::FixVObj(int id) {
		// TODO: This is only a fake function. Need real implementation.
		return AR_SUCCESS;
	}
}