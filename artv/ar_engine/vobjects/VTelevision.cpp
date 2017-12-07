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
		VObject(engine, id, INT_MAX),
		content_stream_(content_stream)
	{
	}

	bool VTelevision::IsSelected(Point2f pt2d, int frame_id) {
		Point2f lu = left_upper_->observation(frame_id)->loc();
		Point2f ll = left_lower_->observation(frame_id)->loc();
		Point2f ru = right_upper_->observation(frame_id)->loc();
		Point2f rl = right_lower_->observation(frame_id)->loc();

		return (ru - lu).cross(pt2d - lu) > 0
			&& (rl - ru).cross(pt2d - ru) > 0
			&& (ll - rl).cross(pt2d - rl) > 0
			&& (lu - ll).cross(pt2d - ll) > 0;
	}

	void VTelevision::locate(const shared_ptr<const InterestPoint>& left_upper,
							 const shared_ptr<const InterestPoint>& left_lower,
							 const shared_ptr<const InterestPoint>& right_upper,
							 const shared_ptr<const InterestPoint>& right_lower) {
		left_upper_ = left_upper;
		left_lower_ = left_lower;
		right_upper_ = right_upper;
		right_lower_ = right_lower;
	}

	void VTelevision::Draw(cv::Mat& scene, const cv::Mat& camera_matrix) {
		// TODO: Need implementation. Use the camera matrix to project the television and the
		// video content on the scene.
	}
}