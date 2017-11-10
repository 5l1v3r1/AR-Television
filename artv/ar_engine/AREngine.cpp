///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <ar_engine/AREngine.h>

using namespace std;
using namespace cv;

namespace ar {
	//! Find in the current 2D frame the bounder surrounding the surface specified by a given point.
	vector<Point> AREngine::FindSurroundingBounder(const cv::Point& point) {
		// TODO: This is only a fake function. Need real implementation.
		return vector<Point>();
	}

	//! Input the interest points in the current 2D frame, match them with the stored interest points,
	//	and update their estimated 3D locations. Return the camera matrix.
	Mat AREngine::UpdateInterestPoints(std::vector<cv::Point>& interest_points2d) {
		// TODO: Match the interest points with the stored ones.

		// TODO: Update 3D locations, and simultaneously calculate the camera matrix.
		return Mat();
	}

	ERROR_CODE AREngine::GetMixedScene(const Mat& raw_scene, Mat& mixed_scene) {
		last_raw_frame_ = raw_scene;

		// TODO: Accumulate the motion data.
		accumulated_motion_data_.clear();

		auto interest_points2d = DetectInterestPointsDoG(raw_scene);
		auto cam_mat = UpdateInterestPoints(interest_points2d);

		mixed_scene = raw_scene;
		for (auto vobj : virtual_objects_) {
			// TODO: Draw the virtual object on the mixed_scene.
		}

		// Remove the oldest location record of the interest points, and remove the 
		// interest points that are determined not visible anymore.
		int new_size = interest_points_.size();
		for (int i = 0; i < new_size; ++i) {
			interest_points_[i].RemoveOldestLoc();
			if (interest_points_[i].ToDiscard()) {
				interest_points_[i] = interest_points_[--new_size];
				--i;
			}
		}
		interest_points_.resize(new_size);

		return AR_SUCCESS;
	}

	ERROR_CODE AREngine::CreateTelevision(cv::Point location, FrameStream& content_stream) {
		// TODO: This is only a fake function. Need real implementation.

		// TODO: Find the 

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

	void AREngine::InterestPoint::AddLatestLoc(optional<Point> p) {
		loc2d_seq_.push(p);
		if (p.has_value())
			++vis_cnt;
	}

	void AREngine::InterestPoint::RemoveOldestLoc() {
		if (loc2d_seq_.front.has_value())
			--vis_cnt;
		loc2d_seq_.pop();
	}
}