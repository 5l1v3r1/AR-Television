///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <ar_engine/AREngine.h>
#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;

namespace ar {
	//! Find in the current 2D frame the bounder surrounding the surface specified by a given point.
	//	@return the indices of the interest points.
	vector<int> AREngine::FindSurroundingBounder(const Point& point) {
		// TODO: This is only a fake function. Need real implementation.
		return vector<int>();
	}

	//! Update the estimated 3D locations of the interest points. Return the camera matrix.
	Mat AREngine::Estimate3DPointsAndCamMatrix() {
		// TODO: Update 3D locations, and simultaneously calculate the camera matrix.
		return Mat();
	}

	AREngine::AREngine() : interest_points_tracker_(ORB::create(), DescriptorMatcher::create("FLANNBASED")) {

	}

	ERROR_CODE AREngine::GetMixedScene(const Mat& raw_scene, Mat& mixed_scene) {
		last_raw_frame_ = raw_scene;
		cvtColor(last_raw_frame_, last_gray_frame_, COLOR_BGR2GRAY);

		// TODO: Accumulate the motion data.
		accumulated_motion_data_.clear();

		// Generate new keypoints.
		std::vector<cv::KeyPoint> keypoints;
		cv::Mat descriptors;
		interest_points_tracker_.GenKeypointsDesc(raw_scene, keypoints, descriptors);

		// Match the new keypoints to the stored keypoints.
		// TODO: Assemble the currently stored keypoints and average descriptors.
		std::vector<cv::KeyPoint> stored_keypoints;
		std::vector<int> corr_indices;
		stored_keypoints.reserve(interest_points_.size());
		corr_indices.reserve(interest_points_.size());
		cv::Mat stored_descriptors;
		for (int i = 0; i < interest_points_.size(); ++i) {
			auto loc = interest_points_[i].loc2d_seq().back();
			if (loc.has_value()) {
				corr_indices.push_back(i);
				stored_keypoints.push_back(loc.value());
				vconcat(stored_descriptors, interest_points_[i].average_desc_);
			}
		}
		auto matches = interest_points_tracker_.MatchKeypoints(keypoints, descriptors, stored_keypoints, stored_descriptors);
		// TODO: Update the stored keypoints.
		bool* matched = new bool[keypoints.size()];
		memset(matched, 0, sizeof(bool) * keypoints.size());

		delete[] matched;
		
		auto cam_mat = Estimate3DPointsAndCamMatrix();

		mixed_scene = raw_scene;
		for (auto vobj : virtual_objects_) {
			// TODO: Draw the virtual object on the mixed_scene.
		}

		// If we have stored too many interest points, we remove the oldest location record
		// of the interest points, and remove the interest points that are determined not visible anymore.
		if (interest_points_.size() > MAX_INTEREST_POINTS) {
			int new_size = int(interest_points_.size());
			for (int i = 0; i < new_size; ++i) {
				interest_points_[i].RemoveOldestLoc();
				if (interest_points_[i].ToDiscard()) {
					interest_points_[i] = interest_points_[--new_size];
					--i;
				}
			}
			interest_points_.resize(new_size);
		}

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

	void AREngine::InterestPoint::AddLatestLoc(optional<KeyPoint> p) {
		loc2d_seq_.push(p);
		if (p.has_value())
			++vis_cnt;
	}

	void AREngine::InterestPoint::RemoveOldestLoc() {
		if (loc2d_seq_.front().has_value())
			--vis_cnt;
		loc2d_seq_.pop();
	}
}