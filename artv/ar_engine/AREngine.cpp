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

	//! If we have stored too many interest points, we remove the oldest location record
	//	of the interest points, and remove the interest points that are determined not visible anymore.
	void AREngine::ReduceInterestPoints() {
		if (interest_points_.size() > MAX_INTEREST_POINTS) {
			int new_size = int(interest_points_.size());
			// We only remove points with status sequence length larger than the half of the maximum sequence
			// length of all interest points. We initialize this as 2, so interest points with only one status
			// will not be affected.
			int max_seq_len = 2;
			for (int i = 0; i < new_size; ++i) {
				int len = interest_points_[i].status_seq().size();
				if (len > max_seq_len)
					max_seq_len = len;
				if (len > max_seq_len >> 1) {
					interest_points_[i].RemoveEarliestStatus();
					if (interest_points_[i].ToDiscard()) {
						interest_points_[i] = interest_points_[--new_size];
						--i;
					}
				}
			}
			interest_points_.resize(new_size);
		}
	}


	void AREngine::UpdateInterestPoints(const cv::Mat& scene) {
		// Generate new keypoints.
		std::vector<cv::KeyPoint> keypoints;
		cv::Mat descriptors;
		interest_points_tracker_.GenKeypointsDesc(scene, keypoints, descriptors);

		// Match the new keypoints to the stored keypoints.
		cv::Mat stored_descriptors;
		for (int i = 0; i < interest_points_.size(); ++i)
			vconcat(stored_descriptors, interest_points_[i].average_desc_);
		auto matches = interest_points_tracker_.MatchKeypoints(descriptors, stored_descriptors);

		// Update the stored keypoints.
		bool* matched_new = new bool[keypoints.size()];
		bool* matched_stored = new bool[interest_points_.size()];
		memset(matched_new, 0, sizeof(bool) * keypoints.size());
		memset(matched_stored, 0, sizeof(bool) * interest_points_.size());
		for (auto match : matches) {
			matched_new[match.first] = true;
			matched_stored[match.second] = true;
			interest_points_[match.second].AddNewStatus(InterestPoint::Status({ keypoints[match.first], descriptors.row(match.first) }));
		}
		// These interest points are not ever visible in the previous frames.
		for (int i = 0; i < keypoints.size(); ++i)
			if (!matched_new[i])
				interest_points_.push_back(InterestPoint(keypoints[i], descriptors.row(i)));
		// These interest points are not visible at this frame.
		for (int i = 0; i < interest_points_.size(); ++i)
			interest_points_[i].AddNewStatus(InterestPoint::Status());
		delete[] matched_new;
		delete[] matched_stored;

		ReduceInterestPoints();
	}

	ERROR_CODE AREngine::GetMixedScene(const Mat& raw_scene, Mat& mixed_scene) {
		last_raw_frame_ = raw_scene;
		cvtColor(last_raw_frame_, last_gray_frame_, COLOR_BGR2GRAY);

		// TODO: Accumulate the motion data.
		accumulated_motion_data_.clear();

		UpdateInterestPoints(raw_scene);
		
		auto cam_mat = Estimate3DPointsAndCamMatrix();

		mixed_scene = raw_scene;
		for (auto vobj : virtual_objects_) {
			// TODO: Draw the virtual object on the mixed_scene.
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

	InterestPoint::InterestPoint(): vis_cnt(0) {}

	InterestPoint::InterestPoint(const KeyPoint& initial_loc,
								 const cv::Mat& initial_desc): vis_cnt(1) {
		status_seq_.push(Status({ initial_loc, initial_desc }));
		average_desc_ = initial_desc;
	}

	void InterestPoint::AddNewStatus(const Status& p) {
		status_seq_.push(p);
		if (p.has_value()) {
			if (average_desc_.empty())
				average_desc_ = p.value().second;
			else
				average_desc_ = (average_desc_ * vis_cnt + p.value().second) / (vis_cnt + 1);
			++vis_cnt;
		}
	}

	void InterestPoint::RemoveEarliestStatus() {
		if (status_seq_.front().has_value()) {
			--vis_cnt;
			if (vis_cnt)
				average_desc_ = (average_desc_ * (vis_cnt + 1) - status_seq_.front().value().second) / vis_cnt;
			else
				average_desc_ = Mat();
		}
		status_seq_.pop();
	}
}