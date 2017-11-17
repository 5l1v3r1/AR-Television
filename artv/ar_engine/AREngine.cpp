///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <opencv2/features2d.hpp>

#include <ar_engine/AREngine.h>
#include <common/OSUtils.h>

using namespace std;
using namespace cv;

namespace ar {
	//! Estimate the 3D location of the interest points with the latest keyframe asynchronously.
	void AREngine::EstimateMap() {
		// TODO: Need implementation. Remember to calculate the average depth!
	}

	void AREngine::MapEstimationLoop() {
		++thread_cnt_;
		while (interest_points_.empty() && !to_terminate_)
			AR_SLEEP(1);
		while (!to_terminate_)
			EstimateMap();
		--thread_cnt_;
	}

	void AREngine::CallMapEstimationLoop(AREngine* engine) {
		engine->MapEstimationLoop();
	}

	AREngine::~AREngine() {
		to_terminate_ = true;
		do {
			AR_SLEEP(1);
		} while (thread_cnt_);
	}

	AREngine::AREngine() : interest_points_tracker_(ORB::create(), DescriptorMatcher::create("FLANNBASED")) {
		mapping_thread_ = thread(AREngine::CallMapEstimationLoop, this);
	}

	//! If we have stored too many interest points, we remove the oldest location record
	//	of the interest points, and remove the interest points that are determined not visible anymore.
	void AREngine::ReduceInterestPoints() {
		if (interest_points_.size() > MAX_INTEREST_POINTS) {
			int new_size = int(interest_points_.size());
			for (int i = 0; i < new_size; ++i) {
				int len = interest_points_[i]->observation_seq().size();
				// Interest points being used somewhere else should also remain.
				if (len > MAX_OBSERVATIONS && interest_points_[i].use_count() <= 1) {
					interest_points_[i]->RemoveEarlyObservations(len - MAX_OBSERVATIONS);
					if (interest_points_[i]->ToDiscard()) {
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
			vconcat(stored_descriptors, interest_points_[i]->average_desc_);
		auto matches = interest_points_tracker_.MatchKeypoints(descriptors, stored_descriptors);

		// Update the stored keypoints.
		bool* matched_new = new bool[keypoints.size()];
		bool* matched_stored = new bool[interest_points_.size()];
		memset(matched_new, 0, sizeof(bool) * keypoints.size());
		memset(matched_stored, 0, sizeof(bool) * interest_points_.size());
		for (auto match : matches) {
			matched_new[match.first] = true;
			matched_stored[match.second] = true;
			interest_points_[match.second]->AddObservation(InterestPoint::Observation(keypoints[match.first], descriptors.row(match.first)));
		}
		// These interest points are not ever visible in the previous frames.
		for (int i = 0; i < keypoints.size(); ++i)
			if (!matched_new[i])
				interest_points_.push_back(shared_ptr<InterestPoint>(new InterestPoint(keypoints[i], descriptors.row(i))));
		// These interest points are not visible at this frame.
		for (int i = 0; i < interest_points_.size(); ++i)
			interest_points_[i]->AddObservation(InterestPoint::Observation());
		delete[] matched_new;
		delete[] matched_stored;

		ReduceInterestPoints();
	}

	AREngine::Keyframe::Keyframe(Mat _intrinsics,
								 vector<shared_ptr<InterestPoint>> _interest_points,
								 Mat _R,
								 Mat _t,
								 double _average_depth) :
		intrinsics(_intrinsics), interest_points(_interest_points), R(_R), t(_t), average_depth(_average_depth) {
	}

	ERROR_CODE AREngine::FeedScene(const Mat& raw_scene) {
		last_raw_frame_ = raw_scene;
		cvtColor(last_raw_frame_, last_gray_frame_, COLOR_BGR2GRAY);

		UpdateInterestPoints(raw_scene);

		if (recent_keyframes_.empty())
			// Initial keyframe.
			recent_keyframes_.push(Keyframe(intrinsics_,
											interest_points_,
											Mat::eye(3, 3, CV_64F),
											Mat::zeros(3, 1, CV_64F),
											0));
		else {
			auto& last_keyframe = recent_keyframes_.back();

			// TODO: Estimate the fundamental matrix from the last keyframe.
			Mat fundamental_matrix;

			// Estimate the essential matrix.
			Mat essential_matrix = intrinsics_.t() * fundamental_matrix * last_keyframe.intrinsics;

			// TODO: Call RecoverRotAndTranslation to recover rotation and translation.
			auto candidates = RecoverRotAndTranslation(essential_matrix);
			Mat R, t;

			// TODO: Estimate the average depth.
			int average_depth;

			// If the translation from the last keyframe is greater than some proportion of the depth, update the keyframes.
			double distance = cv::norm(t, cv::NormTypes::NORM_L2);
			if (distance > last_keyframe.average_depth / 5) {
				recent_keyframes_.push(Keyframe(intrinsics_,
												interest_points_,
												last_keyframe.R * R,
												last_keyframe.t + t,
												average_depth));
				if (recent_keyframes_.size() > MAX_KEYFRAMES)
					recent_keyframes_.pop();
			}
		}
	}

	ERROR_CODE AREngine::GetMixedScene(const Mat& raw_scene, Mat& mixed_scene) {
		FeedScene(raw_scene);

		// TODO: Accumulate the motion data.
		accumulated_motion_data_.clear();

		mixed_scene = raw_scene;
		for (auto vobj : virtual_objects_) {
			// TODO: Draw the virtual object on the mixed_scene.
		}

		return AR_SUCCESS;
	}

	//! Find in the current 2D frame the bounder surrounding the surface specified by a given point.
	//	@return the indices of the interest points.
	vector<int> AREngine::FindSurroundingBounder(const Point& point) {
		// TODO: This is only a fake function. Need real implementation.
		return vector<int>();
	}

	ERROR_CODE AREngine::CreateTelevision(cv::Point location, FrameStream& content_stream) {
		// TODO: Find the interest points that roughly form a rectangle in the real world that surrounds the given location.

		// TODO: Create a virtual television, and locate it with respect to these interest points.

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
		observation_seq_.push(Observation(initial_loc, initial_desc));
		average_desc_ = initial_desc;
	}

	void InterestPoint::AddObservation(const Observation& p) {
		observation_seq_.push(p);
		if (p.visible) {
			if (average_desc_.empty())
				average_desc_ = p.desc;
			else
				average_desc_ = (average_desc_ * vis_cnt + p.desc) / (vis_cnt + 1);
			++vis_cnt;
		}
	}

	InterestPoint::Observation::Observation(): visible(false) {}

	InterestPoint::Observation::Observation(const cv::KeyPoint& _pt,
											const cv::Mat& _desc):
		pt(_pt), desc(_desc), visible(true) {}

	void InterestPoint::RemoveEarlyObservations(int cnt) {
		Mat removed_desc_acc;
		int removed_visible_cnt = 0;
		for (int i = 0; i < cnt; ++i) {
			if (observation_seq_.front().visible) {
				++removed_visible_cnt;
				removed_desc_acc += observation_seq_.front().desc;
			}
			observation_seq_.pop();
		}
		average_desc_ = (average_desc_ * vis_cnt - removed_desc_acc) / (vis_cnt - removed_visible_cnt);
		vis_cnt -= removed_visible_cnt;
	}
}