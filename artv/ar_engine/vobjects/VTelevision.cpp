///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <ar_engine/vobjects/VTelevision.h>

using namespace std;
using namespace cv;

namespace ar
{
	const double VTelevision::MEAN_TV_SIZE_RATE = 0.1;

	VTelevision::VTelevision(AREngine& engine,
							 int id,
							 FrameStream& content_stream) :
		VObject(engine, id),
		content_stream_(content_stream)
	{
		// TODO: Need implementation.
	}

	void VTelevision::locate(const shared_ptr<InterestPoint>& left_upper,
							 const shared_ptr<InterestPoint>& left_lower,
							 const shared_ptr<InterestPoint>& right_upper,
							 const shared_ptr<InterestPoint>& right_lower) {
		// TODO: Need implementation.
	}

	void VTelevision::Draw(cv::Mat& scene, const cv::Mat& camera_matrix) {
		// TODO: Need implementation.
	}
}