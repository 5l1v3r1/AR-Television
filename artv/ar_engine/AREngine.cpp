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
        interest_points_mutex_.lock();

        keyframe_mutex_.lock();
        int keyframe_id = keyframe_id_;
        keyframe_mutex_.unlock();
        cout << "Key Frame #" << keyframe_id << endl;
        const auto &last_frame1 = keyframe(keyframe_id - 2);
        auto &last_frame2 = keyframe(keyframe_id - 1);
        auto &last_frame3 = keyframe(keyframe_id);
        auto K1 = last_frame1.intrinsics().clone();
        auto K2 = last_frame2.intrinsics().clone();
        auto K3 = last_frame3.intrinsics().clone();
        auto M1 = last_frame1.extrinsics().clone();
        auto M2 = last_frame2.extrinsics().clone();
        auto M3 = last_frame3.extrinsics().clone();

        vector<shared_ptr<InterestPoint>> used_points;
        // Find usable interest points.
        for (auto &interest_point : interest_points_) {
            bool usable = true;
            for (int j = 0; j <= max(2, keyframe_id_); ++j) {
                if (!interest_point->observation(keyframe_id_ - j)->visible) {
                    usable = false;
                    break;
                }
            }
            if (usable)
                used_points.push_back(interest_point);
        }

        if (used_points.empty()) {
            interest_points_mutex_.unlock();
            return;
        }

        cout << "Using " << used_points.size() << " points for BA." << endl;
        // Fill the data.
        auto *p1 = new double[used_points.size() << 1];
        auto *p2 = new double[used_points.size() << 1];
        auto *p3 = new double[used_points.size() << 1];
        auto *pts3d = new double[used_points.size() * 3];
        int ind2d = 0, ind3d = 0;
        for (auto &ip : used_points) {
            assert(ip);
            p1[ind2d] = ip->observation(keyframe_id - 2)->pt.pt.x;
            p1[ind2d + 1] = ip->observation(keyframe_id - 2)->pt.pt.y;
            p2[ind2d] = ip->observation(keyframe_id - 1)->pt.pt.x;
            p2[ind2d + 1] = ip->observation(keyframe_id - 1)->pt.pt.y;
            p3[ind2d] = ip->observation(keyframe_id)->pt.pt.x;
            p3[ind2d + 1] = ip->observation(keyframe_id)->pt.pt.y;
            ind2d += 2;
            pts3d[ind3d] = ip->loc3d().x;
            pts3d[ind3d + 1] = ip->loc3d().y;
            pts3d[ind3d + 2] = ip->loc3d().z;
            ind3d += 3;
        }

        interest_points_mutex_.unlock();
        bool converged = BundleAdjustment(static_cast<int>(used_points.size()), K1, M1, p1, K2, M2, p2, K3, M3, p3,
                                          pts3d);
        delete[] p1;
        delete[] p2;
        delete[] p3;

        if (converged) {
            last_frame2.extrinsics(M2);
            last_frame3.extrinsics(M3);
            // Recalculate the depth.
            double total_depth = 0;
            ind3d = 0;
            for (auto &ip : used_points) {
                ip->loc3d(static_cast<float>(pts3d[ind3d]),
                          static_cast<float>(pts3d[ind3d + 1]),
                          static_cast<float>(pts3d[ind3d + 2]));
                ind3d += 3;
                float data[] = {ip->loc3d().x, ip->loc3d().y, ip->loc3d().z, 1};
                auto depth = Mat(M3.row(2) * Mat(4, 1, CV_32F, data)).at<float>(0);
                total_depth += depth;
            }
            last_frame3.average_depth = total_depth / used_points.size();
        }
        delete[] pts3d;
    }

    void AREngine::MapEstimationLoop() {
        while ((interest_points_.empty() || keyframe_id_ < 2) && !to_terminate_)
            AR_SLEEP(1);
        int last_keyframe_ind;
        while (!to_terminate_) {
            last_keyframe_ind = keyframe_id_;
            EstimateMap();
            while (!to_terminate_ && last_keyframe_ind == keyframe_id_)
                AR_SLEEP(1);
        }
        cout << "Exiting map estimation loop!" << endl;
    }

    void AREngine::CallMapEstimationLoop(AREngine *engine) {
        engine->MapEstimationLoop();
    }

    AREngine::~AREngine() {
        to_terminate_ = true;
        mapping_thread_.join();
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

    void InterestPoint::Combine(const shared_ptr<InterestPoint> &another) {
        int last_keyframe_id = observation_seq_[observation_seq_tail_ % MAX_OBSERVATIONS]->keyframe_id;
        for (int i = 0; i < MAX_OBSERVATIONS; ++i)
            if (another->observation(last_keyframe_id - i)->visible &&
                (!observation_seq_[(observation_seq_tail_ + MAX_OBSERVATIONS - i) % MAX_OBSERVATIONS] ||
                 !observation_seq_[(observation_seq_tail_ + MAX_OBSERVATIONS - i) % MAX_OBSERVATIONS]->visible)) {
                observation_seq_[(observation_seq_tail_ + MAX_OBSERVATIONS - i) % MAX_OBSERVATIONS] =
                        another->observation(last_keyframe_id - i);
            }
    }

    /// If we have stored too many interest points, we remove the oldest location record
    ///	of the interest points, and remove the interest points that are determined not visible anymore.
    void AREngine::ReduceInterestPoints() {
        interest_points_mutex_.lock();

        auto new_size = interest_points_.size();
        for (int i = 0; i < new_size; ++i)
            // Check whether the interest point is not visible in several keyframes.
            if (interest_points_[i]->ToDiscard())
                interest_points_[i--] = interest_points_[--new_size];
            else if (interest_points_[i]->has_estimated_3d_loc_) {
                // Check whether the interest point can be combined to another existing point.
                for (int j = 0; j < i; ++j)
                    if ((interest_points_[j]->has_estimated_3d_loc_) &&
                        norm(interest_points_[i]->loc3d() - interest_points_[j]->loc3d()) < 0.01) {
//                        cout << "Combining points with distance " << norm(interest_points_[i]->loc3d() - interest_points_[j]->loc3d()) << endl;
                        interest_points_[j]->Combine(interest_points_[i]);
                        interest_points_[i--] = interest_points_[--new_size];
                        break;
                    }
            }
        interest_points_.resize(new_size);

        interest_points_mutex_.unlock();

        cout << "Currently there are " << interest_points_.size() << " points." << endl;

        // Also remove virtual objects that are based on the removed interest points.
        vector<int> to_remove;
        to_remove.reserve(virtual_objects_.size());
        for (auto &vobj : virtual_objects_)
            if (!vobj.second->IsAlive())
                to_remove.push_back(vobj.first);
        for (auto id : to_remove)
            virtual_objects_.erase(id);
    }

    Keyframe::Keyframe(Mat _intrinsics,
                       Mat _extrinsics,
                       double _average_depth) :
            intrinsics_(_intrinsics.clone()),
            extrinsics_(_extrinsics.clone()),
            average_depth(_average_depth) {
        assert(abs(determinant(extrinsics_.colRange(0, 3)) - 1) < 0.001);
    }

    void AREngine::AddKeyframe(Keyframe &kf) {
        last_key_scene_ = last_raw_frame_.clone();

        keyframe_mutex_.lock();
        keyframe(++keyframe_id_) = kf;
        if (keyframe_id_ >= (MAX_KEYFRAMES << 1))
            keyframe_id_ -= MAX_KEYFRAMES;
        keyframe_mutex_.unlock();
    }

    void AREngine::FindExtrinsics(const vector<Mat> &candidates,
                                  const Mat &baseExtrinsics,
                                  vector<pair<Mat, Mat>> &data,
                                  Mat &M2,
                                  Mat &pts3d,
                                  Mat &mask) const {
        // Try each candidate of extrinsics.
        auto least_error = DBL_MAX;
        for (auto &candidate : candidates) {
            Mat combined = CombineExtrinsics(baseExtrinsics, candidate);
            data.back().first = intrinsics_ * combined;
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
            Mat inliers = Mat::ones(estimated_pts3d.rows, 1, CV_8U);
            // These 3D points are valid if they are in front of the camera in the previous keyframes.
            bool valid = true;
            int invalid_cnt = 0;
            for (int j = 0; j <= min(int(data.size() - 2), keyframe_id_) && valid; ++j) {
                auto &kf = keyframe(keyframe_id_ - j);
                Mat T = Mat(estimated_pts3d.rows, 3, CV_32F);
                for (int k = 0; k < estimated_pts3d.rows; ++k)
                    ((Mat) kf.translation().t()).copyTo(T.row(k));
                Mat transformed_pts3d = estimated_pts3d * kf.rotation().t() + T;

                for (int k = 0; k < transformed_pts3d.rows; ++k)
                    if (transformed_pts3d.at<float>(k, 2) < 1) {
                        inliers.at<bool>(k) = false;
                        ++invalid_cnt;
                    }
                if (invalid_cnt > transformed_pts3d.rows >> 2) {
                    valid = false;
                    break;
                }
            }
            if (valid) {
                // Also check with the current frame.
                Mat R = candidate.colRange(0, 3);
                Mat t = candidate.col(3);
                Mat T = Mat(estimated_pts3d.rows, 3, CV_32F);
                for (int k = 0; k < estimated_pts3d.rows; ++k)
                    ((Mat) t.t()).copyTo(T.row(k));
                Mat transformed_pts3d = estimated_pts3d * R.t() + T;
                for (int k = 0; k < transformed_pts3d.rows; ++k)
                    if (transformed_pts3d.at<float>(k, 2) < 1) {
                        inliers.at<bool>(k) = false;
                        ++invalid_cnt;
                    }
                if (invalid_cnt > transformed_pts3d.rows >> 2)
                    valid = false;

                if (valid && err < least_error) {
                    least_error = err;
                    M2 = combined;
                    pts3d = estimated_pts3d;
                    mask = inliers;
//                    cout << "Found a valid solution! Error=" << err << endl;
                }
            }
        }
    }

    Point2f InterestPoint::loc(Mat camera_matrix) const {
        float raw_homogenous[] = {loc3d_.x, loc3d_.y, loc3d_.z, 1};
        Mat homogenous(4, 1, CV_32F, raw_homogenous);
        auto proj = Mat(camera_matrix * homogenous);
        return Point2f(proj.at<float>(0) / proj.at<float>(2), proj.at<float>(1) / proj.at<float>(2));
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
                               Mat::eye(3, 4, CV_32F),
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
            if (points1.size() < 8) {
                cout << "Too few matched!" << endl;
                return AR_SUCCESS;
            }

            // Estimate the fundamental matrix using the matched points.
            Mat inlier_mask;
            Mat fundamental_matrix = findFundamentalMat(points1, points2, FM_RANSAC, 3., 0.99, inlier_mask);

            // If fail to compute a solution of fundamental matrix, this scene might be problematic. We skip it.
            if (fundamental_matrix.empty()) {
                cout << "Failed to estimate fundamental matrix!" << endl;
                return AR_SUCCESS;
            }
            if (fundamental_matrix.rows > 3)
                fundamental_matrix = fundamental_matrix.rowRange(0, 3);
            fundamental_matrix.convertTo(fundamental_matrix, CV_32F);

            // Remove outliers from the matches.
            size_t new_size = 0;
            for (int i = 0; i < matches.size(); ++i)
                if (inlier_mask.at<bool>(i))
                    matches[new_size++] = matches[i];
            // If there are too few inliers, this scene is problematic. Skip it.
            if (new_size < 4) {
                cout << "Too few inliers!" << endl;
                return AR_SUCCESS;
            }
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
                        data.emplace_back(last_keyframe2.intrinsics() * last_keyframe2.extrinsics(),
                                          stored_pts1);
                        data.emplace_back(last_keyframe.intrinsics() * last_keyframe.extrinsics(),
                                          stored_pts2);
                        data.emplace_back(Mat(), new_pts);

                        Mat M2;
                        FindExtrinsics(candidates, last_keyframe.extrinsics(), data, M2, pts3d, inlier_mask);
                        if (!M2.empty()) {
                            extrinsics_ = M2;
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
                        data.emplace_back(keyframe(id).intrinsics() * keyframe(id).extrinsics(),
                                          stored_pts);
                        data.emplace_back(Mat(), new_pts);

                        Mat M2;
                        FindExtrinsics(candidates, keyframe(id).extrinsics(), data, M2, pts3d, inlier_mask);
                        if (!M2.empty()) {
                            extrinsics_ = M2;
                            done = true;
                        }
                    }
                }

                if (!done) {
                    // This frame is problematic. Skip it.
                    cout << "Cannot find valid solution for extrinsics!" << endl;
                    return AR_SUCCESS;
                }
            }

            // Remove outliers from the matches.
            new_size = 0;
            for (int i = 0; i < matches.size(); ++i)
                if (inlier_mask.at<bool>(i))
                    matches[new_size++] = matches[i];
            // If there are too few inliers, this scene is problematic. Skip it.
            if (new_size < 4) {
                cout << "Too few inliers!" << endl;
                return AR_SUCCESS;
            }
            matches.resize(new_size);
            // The new matches consist of all inliers.
            inlier_mask = Mat::ones(static_cast<int>(matches.size()), 1, CV_8U);

            Mat R = extrinsics_.colRange(0, 3);
            Mat t = extrinsics_.col(3);
            assert(abs(determinant(R) - 1) < 0.001);

            // Estimate the average depth.
            Mat T = Mat(pts3d.rows, 3, CV_32F);
            for (int k = 0; k < pts3d.rows; ++k)
                Mat(t.t()).copyTo(T.row(k));
            Mat transformed_pts3d = pts3d * R.t() + T;
            double average_depth = sum(transformed_pts3d.col(2))[0] / pts3d.rows;

            // If the translation from the last keyframe is greater than some proportion of the depth,
            // this is a new keyframe!
            Mat t_rel = t - R * last_keyframe.rotation().t() * last_keyframe.translation();
            double distance = cv::norm(t_rel, cv::NormTypes::NORM_L2);
            cout << "Distance=" << distance << " vs AverageDepth=" << last_keyframe.average_depth << endl;
            if (distance / min(average_depth, last_keyframe.average_depth) > 0.1 || last_keyframe.average_depth < 0) {
                auto kf = Keyframe(intrinsics_,
                                   extrinsics_,
                                   average_depth);
                AddKeyframe(kf);

                // Try to match new keypoints to the stored keypoints.
                vector<bool> matched_new(keypoints.size(), false);
                vector<bool> matched_stored(interest_points_.size(), false);

                interest_points_mutex_.lock();

                // Add an observation in this keyframe to the interest points.
                for (auto match :matches) {
                    matched_stored[match.first] = true;
                    matched_new[match.second] = true;
                    interest_points_[match.first]->AddObservation(
                            make_shared<InterestPoint::Observation>(keyframe_id_,
                                                                    keypoints[match.second],
                                                                    descriptors.row(match.second)));
                    interest_points_[match.first]->loc3d(pts3d.at<float>(match.first, 0),
                                                         pts3d.at<float>(match.first, 1),
                                                         pts3d.at<float>(match.first, 2));
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

                interest_points_mutex_.unlock();
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
    ///	real world by then, and its shape and size in the scene remain the same during
    ///	the dragging. Call FixVObj to fix the virtual object onto the real world again.
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

    void InterestPoint::loc3d(float x, float y, float z) {
        loc3d_.x = x;
        loc3d_.y = y;
        loc3d_.z = z;
        has_estimated_3d_loc_ = true;
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

    void InterestPoint::loc3d(const Point3f &pt3d) {
        loc3d_ = pt3d;
        has_estimated_3d_loc_ = true;
    }

    InterestPoint::Observation::Observation() : visible(false) {}

    InterestPoint::Observation::Observation(int _keyframe_id,
                                            const cv::KeyPoint &_pt,
                                            const cv::Mat &_desc) :
            pt(_pt), desc(_desc), visible(true), keyframe_id(_keyframe_id) {}

}
