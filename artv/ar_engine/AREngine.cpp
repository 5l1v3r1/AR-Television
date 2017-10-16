///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <ar_engine/AREngine.h>

using namespace cv;

namespace ar {
	ERROR_CODE AREngine::getMixedScene(const Mat& raw_scene, Mat& mixed_scene) {
		// TODO: This is only a fake function. Need real implementation.
		mixed_scene = raw_scene;
		return AR_SUCCESS;
	}

	ERROR_CODE AREngine::createScreen(cv::Point2i location, FrameStream& content_stream) {
		// TODO: This is only a fake function. Need real implementation.
		return AR_SUCCESS;
	}
}