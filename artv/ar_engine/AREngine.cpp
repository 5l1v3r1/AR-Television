////////////////////////////////////////////////////////////
/// AR Television
/// Copyright(c) 2017 Carnegie Mellon University
/// Licensed under The MIT License[see LICENSE for details]
/// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
////////////////////////////////////////////////////////////
#include <opencv2/features2d.hpp>

#include <common/OSUtils.h>
#include <common/Utils.h>
#include <ar_engine/AREngine.h>
#include <ar_engine/vobjects/VTelevision.h>

using namespace std;
using namespace cv;

namespace ar {
    const int AREngine::MAX_KEYFRAMES;
    const int InterestPoint::MAX_OBSERVATIONS;
    shared_ptr<InterestPoint::Observation> EMPTY_OBSERVATION = make_shared<InterestPoint::Observation>();

    /// Estimate the 3D location of the interest points with the latest keyframe asynchronously.
    ///	Perform bundle adjustment based on the rough estimation of the extrinsics.
    void AREngine::EstimateMap() {
        while (!interest_points_mutex_.try_lock())
            AR_SLEEP(1);

        auto last_frame1 = keyframe(keyframe_id_);
        auto last_frame2 = keyframe(keyframe_id_ - 1);
        auto K1 = last_frame1.intrinsics;
        auto K2 = last_frame2.intrinsics;
        Mat M1, M2;
        hconcat(last_frame1.R, last_frame1.t, M1);
        hconcat(last_frame2.R, last_frame2.t, M2);

        vector<int> utilized_interest_points;
        // Find utilized_interest_points.
        utilized_interest_points.reserve(interest_points_.size());
        for (int i = 0; i < interest_points_.size(); ++i) {
            bool usable = true;
            for (int j = 0; j <= max(1, keyframe_id_); ++j) {
                if (!interest_points_[i]->observation(keyframe_id_ - 1)->visible) {
                    usable = false;
                    break;
                }
            }
            if (usable)
                utilized_interest_points.push_back(i);
        }
        // Fill the data.
        Mat pts1(static_cast<int>(utilized_interest_points.size()), 2, CV_32F);
        Mat pts2(static_cast<int>(utilized_interest_points.size()), 2, CV_32F);
        Mat Points3d(static_cast<int>(utilized_interest_points.size()), 3, CV_32F);
        for (auto ip_id : utilized_interest_points) {
            pts1.row(ip_id) = Mat(interest_points_[ip_id]->observation(keyframe_id_ - 1)->pt.pt, false);
            pts2.row(ip_id) = Mat(interest_points_[ip_id]->observation(keyframe_id_)->pt.pt, false);
            Points3d.row(ip_id) = Mat(interest_points_[ip_id]->loc3d_, false);
        }
        BundleAdjustment(K1, M1, pts1, K2, M2, pts2, Points3d);

        // TODO: Recalculate the average depth.

        interest_points_mutex_.unlock();
    }

    void AREngine::MapEstimationLoop() {
        ++thread_cnt_;
        while (interest_points_.empty() && keyframe_id_ < 1 && !to_terminate_)
            AR_SLEEP(1);
        int last_keyframe_ind = keyframe_id_;
        while (!to_terminate_) {
//            EstimateMap();
            while (!to_terminate_ && last_keyframe_ind == keyframe_id_)
                AR_SLEEP(1);
        }

        --thread_cnt_;
    }

    void AREngine::CallMapEstimationLoop(AREngine *engine) {
        engine->MapEstimationLoop();
    }

    AREngine::~AREngine() {
        to_terminate_ = true;
        do {
            AR_SLEEP(1);
        } while (thread_cnt_);
    }

    shared_ptr<InterestPoint::Observation> InterestPoint::observation(int keyframe_id) const {
        for (int i = 0; i < min(observation_seq_tail_ + 1, MAX_OBSERVATIONS); ++i)
            if (observation_seq_[i]->keyframe_id == keyframe_id)
                return observation_seq_[i];
        return EMPTY_OBSERVATION;
    }

    AREngine::AREngine() : interest_points_tracker_(ORB::create(),
                                                    new BFMatcher(NORM_HAMMING)) {
        mapping_thread_ = thread(AREngine::CallMapEstimationLoop, this);

        float default_intrinsics[][3] = {{1071.8, 0,      639.5},
                                         {0,      1071.8, 359.5},
                                         {0,      0,      1}};
        intrinsics_ = Mat(3, 3, CV_32F);
        memcpy(intrinsics_.data, default_intrinsics, sizeof(float) * 3 * 3);
    }

    /// If we have stored too many interest points, we remove the oldest location record
    ///	of the interest points, and remove the interest points that are determined not visible anymore.
    void AREngine::ReduceInterestPoints() {
        while (!interest_points_mutex_.try_lock())
            AR_SLEEP(1);

        auto new_size = interest_points_.size();
        for (int i = 0; i < new_size; ++i)
            if (interest_points_[i]->ToDiscard())
                interest_points_[i--] = interest_points_[--new_size];
        interest_points_.resize(new_size);

        interest_points_mutex_.unlock();

        cout << "Currently there are " << interest_points_.size() << " points." << endl;

        // Also remove virtual objects that are based on the removed interest points.
        vector<int> to_remove;
        for (auto &vobj : virtual_objects_)
            if (!vobj.second->IsAlive())
                to_remove.push_back(vobj.first);
        for (auto id : to_remove)
            virtual_objects_.erase(id);
    }

    Keyframe::Keyframe(Mat _intrinsics,
                       Mat _R,
                       Mat _t,
                       double _average_depth) :
            intrinsics(std::move(_intrinsics)),
            R(std::move(_R)), t(std::move(_t)),
            average_depth(_average_depth) {}

    void AREngine::AddKeyframe(Keyframe &kf) {
        last_key_scene_ = last_raw_frame_.clone();

        keyframe(++keyframe_id_) = kf;
        if (keyframe_id_ >= (MAX_KEYFRAMES << 1))
            keyframe_id_ -= MAX_KEYFRAMES;
    }

    void AREngine::FindExtrinsics(const vector<Mat> &candidates,
                                  vector<pair<Mat, Mat>> &data,
                                  Mat &M2,
                                  Mat &pts3d,
                                  Mat &mask) const {
        // Try each candidate of extrinsics.
        auto least_error = DBL_MAX;
        for (auto &candidateM2 : candidates) {

            data.back().first = intrinsics_ * candidateM2;
            Mat estimated_pts3d;
            double err = 0;

#ifdef USE_OPENCV_TRIANGULATE
            Mat pts4d;
            triangulatePoints(data.front().first, data.back().first, data.front().second.t(), data.back().second.t(), pts4d);
            for (int i = 0; i < 3; ++i)
                Mat(pts4d.row(i) / pts4d.row(3)).copyTo(pts4d.row(i));
            estimated_pts3d = pts4d.rowRange(0, 3).t();
#else
            Triangulate(data, estimated_pts3d, &err);
#endif
            assert(estimated_pts3d.rows == data.back().second.rows);
            // These 3D points are valid if they are in front of the camera in the previous keyframes.
            bool valid = true;
            Mat inlier_mask = Mat::ones(estimated_pts3d.rows, 1, CV_8U);
            int invalid_cnt = 0;
            for (int j = 0; j <= min(int(data.size() - 2), keyframe_id_) && valid; ++j) {
                auto &kf = keyframe(keyframe_id_ - j);
                Mat T = Mat(estimated_pts3d.rows, 3, CV_32F);
                for (int k = 0; k < estimated_pts3d.rows; ++k)
                    ((Mat) kf.t.t()).copyTo(T.row(k));
                Mat transformed_pts3d = estimated_pts3d * kf.R.t() + T;
                for (int k = 0; k < transformed_pts3d.rows; ++k)
                    if (inlier_mask.at<bool>(k) && transformed_pts3d.at<float>(k, 2) < 1) {
                        ++invalid_cnt;
                        inlier_mask.at<bool>(k) = false;
                    }
                // We allow some errors.
//                if (invalid_cnt >= (transformed_pts3d.rows >> 2))
//                    valid = false;
                if (invalid_cnt != 0)
                    valid = false;
            }
            if (valid) {
                // Also check with the current frame.
                Mat R = candidateM2.colRange(0, 3);
                Mat t = candidateM2.col(3);
                Mat T = Mat(estimated_pts3d.rows, 3, CV_32F);
                for (int k = 0; k < estimated_pts3d.rows; ++k)
                    ((Mat) t.t()).copyTo(T.row(k));
                Mat transformed_pts3d = estimated_pts3d * R.t() + T;
                for (int k = 0; k < transformed_pts3d.rows; ++k)
                    if (inlier_mask.at<bool>(k) && transformed_pts3d.at<float>(k, 2) < 1) {
                        ++invalid_cnt;
                        inlier_mask.at<bool>(k) = false;
                    }
                if (invalid_cnt >= (transformed_pts3d.rows >> 2))
                    valid = false;

                if (valid && err < least_error) {
                    least_error = err;
                    M2 = candidateM2;
                    pts3d = estimated_pts3d;
                    mask = inlier_mask;

                    cout << "Found a valid solution! Error=" << err << endl;
                }
            }
        }
    }

    ERROR_CODE AREngine::FeedScene(const Mat &raw_scene) {
        last_raw_frame_ = raw_scene;
        cvtColor(last_raw_frame_, last_gray_frame_, COLOR_BGR2GRAY);

        // Generate new keypoints.
        vector<KeyPoint> keypoints;
        Mat descriptors;
        interest_points_tracker_.GenKeypointsDesc(raw_scene, keypoints, descriptors);
        if (!desc_length_)
            desc_length_ = descriptors.cols;

        // Visualize the keypoints for debugging.
        int thickness = -1;
        int lineType = 8;
        for (auto &keypoint : keypoints) {
            Point center(keypoint.pt);
            circle(raw_scene, center, 2, Scalar(0, 0, 255), thickness, lineType);
        }

        // Check whether this is the first frame.
        if (keyframe_id_ == -1) {
            // Initial keyframe.
            auto kf = Keyframe(intrinsics_,
                               Mat::eye(3, 3, CV_32F),
                               Mat::zeros(3, 1, CV_32F),
                               DBL_MAX);
            AddKeyframe(kf);

            // Make all keypoints as initial interest points.
            interest_points_.reserve(keypoints.size());
            for (int i = 0; i < keypoints.size(); ++i)
                interest_points_.push_back(make_shared<InterestPoint>(keyframe_id_, keypoints[i], descriptors.row(i)));

            assert(interest_points_[0]->is_visible(0));
        } else {
            // Perform matching between the new keypoints and those in the last keyframes.
            auto &last_keyframe = keyframe(keyframe_id_);

            Mat stored_descriptors(static_cast<int>(interest_points_.size()), desc_length_, CV_8U);
            for (int i = 0; i < interest_points_.size(); ++i)
                interest_points_[i]->last_desc().copyTo(stored_descriptors.row(i));
            auto matches = interest_points_tracker_.MatchKeypoints(stored_descriptors, descriptors);

            // Estimate the fundamental matrix from the last keyframe.
            vector<Point2f> points1, points2;
            for (auto &match : matches) {
                if (interest_points_[match.first]->is_visible(keyframe_id_)) {
                    points1.emplace_back(interest_points_[match.first]->last_loc());
                    points2.emplace_back(keypoints[match.second].pt);
                }
            }
            // Too few matches. Skip this scene.
            if (points1.size() < 8)
                return AR_SUCCESS;

            // Estimate the fundamental matrix using the matched points.
            Mat inlier_mask;
            Mat fundamental_matrix = findFundamentalMat(points1, points2, FM_RANSAC, 3., 0.99, inlier_mask);

            // If fail to compute a solution of fundamental matrix, this scene might be problematic. We skip it.
            if (fundamental_matrix.empty())
                return AR_SUCCESS;
            if (fundamental_matrix.rows > 3)
                fundamental_matrix = fundamental_matrix.rowRange(0, 3);
            fundamental_matrix.convertTo(fundamental_matrix, CV_32F);

            // Remove outliers from the matches.
            size_t new_size = 0;
            for (int i = 0; i < matches.size(); ++i)
                if (inlier_mask.at<bool>(i))
                    matches[new_size++] = matches[i];
            // If there are too few inliers, this scene is problematic. Skip it.
            if (new_size < 4)
                return AR_SUCCESS;
            matches.resize(new_size);
            // The new matches consist of all inliers.
            inlier_mask = Mat::ones(static_cast<int>(matches.size()), 1, CV_8U);

            // Plot matches.
            Mat plot;
            PlotMatches(last_key_scene_, raw_scene, points1, points2, plot);
            imshow("Matches", plot);

            // Estimate the essential matrix.
            Mat essential_matrix = intrinsics_.t() * fundamental_matrix * intrinsics_;

            // Call RecoverRotAndTranslation to recover rotation and translation.
            auto candidates = RecoverRotAndTranslation(essential_matrix);

            Mat pts3d;
            // Test for the only valid rotation and translation combination.
            {
                // Fill the data for 3D reconstruction.
                bool done = false;
#ifndef USE_OPENCV_TRIANGULATE
                if (keyframe_id_ >= 1) {
                    // We maybe can use 2 keyframes.
                    auto &last_keyframe2 = keyframe(keyframe_id_ - 1);
                    int cnt = 0;
                    for (auto &match : matches)
                        if (interest_points_[match.first]->is_visible(keyframe_id_ - 1) &&
                            interest_points_[match.first]->is_visible(keyframe_id_))
                            ++cnt;
                    if (cnt >= (matches.size() >> 2)) {
                        // There are enough points for triangulate.
                        Mat stored_pts1(cnt, 2, CV_32F);
                        Mat stored_pts2(cnt, 2, CV_32F);
                        Mat new_pts(cnt, 2, CV_32F);
                        cnt = 0;
                        for (auto &match : matches)
                            if (interest_points_[match.first]->is_visible(keyframe_id_ - 1) &&
                                interest_points_[match.first]->is_visible(keyframe_id_)) {
                                // Stored keypoints.
                                auto *dest1 = reinterpret_cast<float *>(stored_pts1.ptr(cnt));
                                auto &pt1 = interest_points_[match.first]->loc(keyframe_id_ - 1);
                                dest1[0] = pt1.x;
                                dest1[1] = pt1.y;
                                auto *dest2 = reinterpret_cast<float *>(stored_pts2.ptr(cnt));
                                auto &pt2 = interest_points_[match.first]->loc(keyframe_id_);
                                dest2[0] = pt2.x;
                                dest2[1] = pt2.y;
                                // New keypoints.
                                auto *dest = reinterpret_cast<float *>(new_pts.ptr(cnt));
                                dest[0] = keypoints[match.second].pt.x;
                                dest[1] = keypoints[match.second].pt.y;
                                ++cnt;
                            }
                        vector<pair<Mat, Mat>> data;
                        data.emplace_back(ComputeCameraMatrix(last_keyframe2.intrinsics,
                                                              last_keyframe2.R,
                                                              last_keyframe2.t),
                                          stored_pts1);
                        data.emplace_back(ComputeCameraMatrix(last_keyframe.intrinsics,
                                                              last_keyframe.R,
                                                              last_keyframe.t),
                                          stored_pts2);
                        data.emplace_back(Mat(), new_pts);

                        Mat M2, mask;
                        for (size_t j = 0; j < candidates.size(); ++j) {
                            Mat(candidates[j].col(3) + candidates[j].colRange(0, 3) * last_keyframe.t).copyTo(candidates[j].col(3));
                            Mat(candidates[j].colRange(0, 3) * last_keyframe.R).copyTo(candidates[j].colRange(0, 3));
                        }
                        FindExtrinsics(candidates, data, M2, pts3d, mask);
                        if (!M2.empty()) {
                            extrinsics_ = M2;
                            inlier_mask = mask;
                            done = true;
                        }
                    }
                }
#endif

                // We can only use 1 keyframe.
                for (int i = 0; i <= min(2, keyframe_id_) && !done; ++i) {
                    // Try to use another keyframe.
                    int id = keyframe_id_ - i;
                    int cnt = 0;
                    for (auto &match : matches)
                        if (interest_points_[match.first]->is_visible(id))
                            ++cnt;
                    if (cnt) {
                        Mat stored_pts = Mat(cnt, 2, CV_32F);
                        Mat new_pts = Mat(cnt, 2, CV_32F);
                        cnt = 0;
                        for (auto &match : matches) {
                            if (interest_points_[match.first]->is_visible(id)) {
                                // Stored keypoints.
                                auto *dest = reinterpret_cast<float *>(stored_pts.ptr(cnt));
                                auto &pt = interest_points_[match.first]->loc(id);
                                dest[0] = pt.x;
                                dest[1] = pt.y;
                                // New keypoints.
                                dest = reinterpret_cast<float *>(new_pts.ptr(cnt));
                                dest[0] = keypoints[match.second].pt.x;
                                dest[1] = keypoints[match.second].pt.y;
                                ++cnt;
                            }
                        }
                        vector<pair<Mat, Mat>> data;
                        data.emplace_back(
                                ComputeCameraMatrix(keyframe(id).intrinsics, keyframe(id).R, keyframe(id).t),
                                stored_pts);
                        data.emplace_back(Mat(), new_pts);

                        Mat M2, mask;
                        for (size_t j = 0; j < candidates.size(); ++j) {
                            Mat(candidates[j].col(3) + candidates[j].colRange(0, 3) * last_keyframe.t).copyTo(candidates[j].col(3));
                            Mat(candidates[j].colRange(0, 3) * last_keyframe.R).copyTo(candidates[j].colRange(0, 3));
                        }
                        FindExtrinsics(candidates, data, M2, pts3d, mask);
                        if (!M2.empty()) {
                            extrinsics_ = M2;
                            inlier_mask = mask;
                            done = true;
                        }
                    }
                }

                if (!done) {
                    // This frame is problematic. Skip it.
                    return AR_SUCCESS;
                }
            }

            // Remain only the points with correct estimated 3D locations.
            new_size = 0;
            for (int k = 0; k < pts3d.rows; ++k)
                if (inlier_mask.at<bool>(k)) {
                    pts3d.row(k).copyTo(pts3d.row(static_cast<int>(new_size)));
                    matches[new_size++] = matches[k];
                }
            // If there are no points left, this scene is problematic. Skip it.
            if (!new_size) {
                cout << inlier_mask << endl;
                return AR_SUCCESS;
            }
            matches.resize(new_size);
            pts3d = pts3d.rowRange(0, static_cast<int>(new_size));

            Mat R = extrinsics_.colRange(0, 3);
            Mat t = extrinsics_.col(3);

            cout << "Det :" << abs(determinant(R)) << endl;

            // Estimate the average depth.
            Mat T = Mat(pts3d.rows, 3, CV_32F);
            for (int k = 0; k < pts3d.rows; ++k)
                Mat(t.t()).copyTo(T.row(k));
            Mat transformed_pts3d = pts3d * R.t() + T;
            double average_depth = sum(transformed_pts3d.col(2))[0] / pts3d.rows;

            // If the translation from the last keyframe is greater than some proportion of the depth,
            // this is a new keyframe!
            Mat t_rel = t - R * last_keyframe.R.t() * last_keyframe.t;
//            Mat t_rel = last_keyframe.t - last_keyframe.R * R.t() * t;
            double distance = cv::norm(t_rel, cv::NormTypes::NORM_L2);

//            double distance = cv::norm(last_keyframe.t -  t, cv::NormTypes::NORM_L2);
            cout << "Distance=" << distance << " vs AverageDepth=" << last_keyframe.average_depth << endl;
            if (distance / min(average_depth, last_keyframe.average_depth) > 0.1) {
                auto kf = Keyframe(intrinsics_,
                                   last_keyframe.R * R,
                                   last_keyframe.t + t,
                                   average_depth);
                AddKeyframe(kf);

                // Try to match new keypoints to the stored keypoints.
                vector<bool> matched_new(keypoints.size(), 0);
                vector<bool> matched_stored(interest_points_.size(), 0);

                // Add an observation in this keyframe to the interest points.
                for (auto match : matches) {
                    matched_stored[match.first] = true;
                    matched_new[match.second] = true;
                    interest_points_[match.first]->AddObservation(
                            make_shared<InterestPoint::Observation>(keyframe_id_,
                                                                    keypoints[match.second],
                                                                    descriptors.row(match.second)));
                    interest_points_[match.first]->loc3d_.x = pts3d.at<float>(match.first, 0);
                    interest_points_[match.first]->loc3d_.y = pts3d.at<float>(match.first, 1);
                    interest_points_[match.first]->loc3d_.z = pts3d.at<float>(match.first, 2);
                }

                // These interest points are not visible at this frame.
                for (int i = 0; i < interest_points_.size(); ++i)
                    if (!matched_stored[i])
                        interest_points_[i]->AddObservation(make_shared<InterestPoint::Observation>());

                // These interest points are not ever visible in the previous frames.
                for (int i = 0; i < keypoints.size(); ++i)
                    if (!matched_new[i])
                        interest_points_.push_back(
                                make_shared<InterestPoint>(keyframe_id_, keypoints[i], descriptors.row(i)));

                ReduceInterestPoints();
            }
        }

        return AR_SUCCESS;
    }

    ERROR_CODE AREngine::GetMixedScene(const Mat &raw_scene, Mat &mixed_scene) {
        FeedScene(raw_scene);

        // TODO: Accumulate the motion data.
        accumulated_motion_data_.clear();

        mixed_scene = raw_scene;
        for (auto vobj : virtual_objects_) {
            switch (vobj.second->GetType()) {
                default:
                    vobj.second->Draw(mixed_scene, intrinsics_ * extrinsics_);
            }
        }

        return AR_SUCCESS;
    }

    double InterestPoint::Observation::l2dist_sqr(const Observation &o) const {
        return l2dist_sqr(o.loc());
    }

    double InterestPoint::Observation::l2dist_sqr(const Point2f &p) const {
        return pow(loc().x - p.x, 2) + pow(loc().y - p.y, 2);
    }

    int AREngine::GetTopVObj(int x, int y) const {
        int highest_level = 0;
        int top = -1;
        for (auto vobj : virtual_objects_) {
            if (vobj.second->IsSelected(Point2f(x, y), keyframe_id_)) {
                if (vobj.second->layer_ind_ > highest_level) {
                    top = vobj.first;
                    highest_level = vobj.second->layer_ind_;
                }
            }
        }
        return top;
    }

    ///	Drag a virtual object to a location. The virtual object is stripped from the
    //	real world by then, and its shape and size in the scene remain the same during
    //	the dragging. Call FixVObj to fix the virtual object onto the real world again.
    ERROR_CODE AREngine::DragVObj(int id, int x, int y) {
        // TODO: This is only a fake function. Need real implementation.
        return AR_SUCCESS;
    }

    /// Fix a virtual object that is floating to the real world. The orientation
    ///	and size might be adjusted to fit the new location.
    ERROR_CODE AREngine::FixVObj(int id) {
        // TODO: This is only a fake function. Need real implementation.
        return AR_SUCCESS;
    }

    InterestPoint::InterestPoint(int initial_keyframe_id,
                                 const KeyPoint &initial_loc,
                                 const cv::Mat &initial_desc) :
            vis_cnt_(1), last_desc_(initial_desc) {
        observation_seq_[observation_seq_tail_ = 0] = make_shared<Observation>(initial_keyframe_id,
                                                                               initial_loc,
                                                                               initial_desc);
    }

    /// Add an observation to the interest point.
    /// In the current system setting, only observations from the keyframes shall be added.
    void InterestPoint::AddObservation(shared_ptr<Observation> p) {
        ++observation_seq_tail_;
        // Remove the information of the discarded observation.
        if (observation_seq_tail_ >= MAX_OBSERVATIONS) {
            auto old = observation_seq_[observation_seq_tail_ % MAX_OBSERVATIONS];
            if (old->visible)
                --vis_cnt_;
        }
        // Add the information of the new observation.
        if (p->visible) {
            ++vis_cnt_;
            last_desc_ = p->desc;
        }
        observation_seq_[observation_seq_tail_ % MAX_OBSERVATIONS] = p;
        if (observation_seq_tail_ >= (MAX_OBSERVATIONS << 1))
            observation_seq_tail_ -= MAX_OBSERVATIONS;
        assert(last_observation().get() == p.get());
    }

    InterestPoint::Observation::Observation() : visible(false) {}

    InterestPoint::Observation::Observation(int _keyframe_id,
                                            const cv::KeyPoint &_pt,
                                            const cv::Mat &_desc) :
            pt(_pt), desc(_desc), visible(true), keyframe_id(_keyframe_id) {}
}
