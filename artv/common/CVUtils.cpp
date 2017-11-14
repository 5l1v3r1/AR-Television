///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <chrono>

#include <common/CVUtils.h>
#include <common/ErrorCodes.h>

using namespace std;
using namespace cv;

using timepoint = chrono::steady_clock::time_point;

namespace ar {
	vector<Point> DetectInterestPointsDoG(Mat image) {
		// TODO: This is only a fake function. Need real implementation.
		return vector<Point>();
	}

	void RealtimeLocalVideoStream::Restart() {
		start_time_ = chrono::steady_clock::now();
		frame_cnt_ = 0;
	}

	ERROR_CODE RealtimeLocalVideoStream::Open(const char* video_path) {
		cap_ = VideoCapture(video_path);
		if (!cap_.isOpened())
			return AR_FILE_NOT_FOUND;

		fps_ = cap_.get(CAP_PROP_FPS);

		return AR_SUCCESS;
	}

	ERROR_CODE RealtimeLocalVideoStream::NextFrame(Mat& output_buf) {
		if (!cap_.isOpened())
			return AR_UNINITIALIZED;

		auto now = chrono::steady_clock::now();
		int dest_frame_ind = int((now - start_time_).count() * fps_ / 1000000000);
		while (frame_cnt_++ < dest_frame_ind)
			cap_.grab();
		bool ret = cap_.retrieve(output_buf);
		if (!ret || output_buf.empty())
			return AR_NO_MORE_FRAMES;
		else
			return AR_SUCCESS;
	}

	void InterestPointsTracker::GenKeypointsDesc(const Mat& frame, vector<KeyPoint>& keypoints, Mat& descriptors)
	{
		detector_->detectAndCompute(frame, noArray(), keypoints, descriptors);
	}

	void InterestPointsTracker::MatchNewKeypoints(const Mat& frame, Mat& former_desc, vector<KeyPoint>& former_kp)
	{
		vector<KeyPoint> kp;
		Mat desc;
		detector_->detectAndCompute(frame, noArray(), kp, desc);

		vector<vector<DMatch>> matches;
		vector<KeyPoint> matched1, matched2;
		matcher_->knnMatch(former_desc, desc, matches, 2);
		for (unsigned i = 0; i < matches.size(); i++) {
			if (matches[i][0].distance < NN_MATCH_RATIO * matches[i][1].distance) {
				matched1.push_back(former_kp[matches[i][0].queryIdx]);
				matched2.push_back(kp[matches[i][0].trainIdx]);
			}
		}

		Mat inlier_mask, homography;
		vector<KeyPoint> inliers1, inliers2;
		vector<DMatch> inlier_matches;
		if (matched1.size() >= 4) {
			homography = findHomography(matched1, matched2,
				RANSAC, RANSAC_THRESH, inlier_mask);
		}

		if (matched1.size() < 4 || homography.empty()) {
		}
		for (unsigned i = 0; i < matched1.size(); i++) {
			if (inlier_mask.at<uchar>(i)) {
				int new_i = static_cast<int>(inliers1.size());
				inliers1.push_back(matched1[i]);
				inliers2.push_back(matched2[i]);
				inlier_matches.push_back(DMatch(new_i, new_i, 0));
			}
		}
	}
}