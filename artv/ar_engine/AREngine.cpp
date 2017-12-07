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
    const int AREngine::MAX_INTEREST_POINTS;
    const int AREngine::MAX_KEYFRAMES;
    const int InterestPoint::MAX_OBSERVATIONS;
    shared_ptr<InterestPoint::Observation> EMPTY_OBSERVATION = make_shared<InterestPoint::Observation>();

    /// Estimate the 3D location of the interest points with the latest keyframe asynchronously.
    //	Perform bundle adjustment based on the rough estimation of the extrinsics.
    void AREngine::EstimateMap() {
        while (!interest_points_mutex_.try_lock())
            AR_SLEEP(1);
        auto interest_points = interest_points_;
        interest_points_mutex_.unlock();

        // Remember to calculate the average depth!
        auto last_frame1 = keyframe(keyframe_seq_tail_);
        auto last_frame2 = keyframe(keyframe_seq_tail_ - 1);
        int frame_id1 = last_frame1.frame_id;
        int frame_id2 = last_frame2.frame_id;
        auto K1 = last_frame1.intrinsics;
        auto K2 = last_frame2.intrinsics;
        Mat M1, M2;
        hconcat(last_frame1.R, last_frame1.t, M1);
        hconcat(last_frame2.R, last_frame2.t, M2);

        vector<int> utilized_interest_points;
        //find utilized_interest_points
        utilized_interest_points.reserve(interest_points_.size());
        for (int i = 0; i < interest_points_.size(); ++i) {
            bool usable = true;
            for (int j = 0; j <= max(1, keyframe_seq_tail_); ++j) {
                int frame_id = keyframe(keyframe_seq_tail_ - j).frame_id;
                if (!interest_points_[i]->observation(frame_id)->visible) {
                    usable = false;
                    break;
                }
            }
            if (usable)
                utilized_interest_points.push_back(i);
        }
        //Fill the data
        Mat pts1(static_cast<int>(utilized_interest_points.size()), 2, CV_32F);
        Mat pts2(static_cast<int>(utilized_interest_points.size()), 2, CV_32F);
        Mat Points3d(static_cast<int>(utilized_interest_points.size()), 3, CV_32F);
        for (auto ip_id : utilized_interest_points) {
            pts1.row(ip_id) = Mat(interest_points_[ip_id]->observation(frame_id1)->pt.pt, false);
            pts2.row(ip_id) = Mat(interest_points_[ip_id]->observation(frame_id2)->pt.pt, false);
            Points3d.row(ip_id) = Mat(interest_points_[ip_id]->loc3d_, false);
        }
        BundleAdjustment(K1, M1, pts1, K2, M2, pts2, Points3d);
    }

    void AREngine::MapEstimationLoop() {
        ++thread_cnt_;
        while (interest_points_.empty() && keyframe_seq_tail_ < 1 && !to_terminate_)
            AR_SLEEP(1);
        int last_keyframe_ind = keyframe_seq_tail_;
        while (!to_terminate_) {
//            EstimateMap();
            while (!to_terminate_ && last_keyframe_ind == keyframe_seq_tail_)
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

    shared_ptr<InterestPoint::Observation> InterestPoint::observation(int frame_id) const {
        for (int i = 0; i < min(observation_seq_tail_, MAX_OBSERVATIONS); ++i)
            if (observation_seq_[i]->frame_id == frame_id)
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
    //	of the interest points, and remove the interest points that are determined not visible anymore.
    void AREngine::ReduceInterestPoints() {
        while (!interest_points_mutex_.try_lock())
            AR_SLEEP(1);

        if (interest_points_.size() > MAX_INTEREST_POINTS) {
            auto new_size = interest_points_.size();
            for (int i = 0; i < new_size; ++i) {
                if (interest_points_[i]->ToDiscard()) {
                    interest_points_[i] = interest_points_[--new_size];
                    --i;
                }
            }
            interest_points_.resize(new_size);
        }
        interest_points_mutex_.unlock();

        cout << "Currently there are " << interest_points_.size() << " points." << endl;
    }

    bool AREngine::IsKeyframe(int frame_id) {
        for (int i = 0; i < min(keyframe_seq_tail_, MAX_KEYFRAMES); ++i) {
            if (recent_keyframes_[i].frame_id == frame_id)
                return true;
        }
        return false;
    }

    Keyframe::Keyframe(int _frame_id,
                       Mat _intrinsics,
                       Mat _R,
                       Mat _t,
                       double _average_depth) :
            frame_id(_frame_id),
            intrinsics(std::move(_intrinsics)),
            R(std::move(_R)), t(std::move(_t)),
            average_depth(_average_depth) {}

    void AREngine::AddKeyframe(Keyframe &kf) {
        keyframe(++keyframe_seq_tail_) = kf;
        if (keyframe_seq_tail_ >= (MAX_KEYFRAMES << 1))
            keyframe_seq_tail_ -= MAX_KEYFRAMES;
    }

    ERROR_CODE AREngine::FeedScene(const Mat &raw_scene) {
        ++frame_id_;

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
        if (keyframe_seq_tail_ == -1) {
            // Initial keyframe.
            auto kf = Keyframe(frame_id_,
                               intrinsics_,
                               Mat::eye(3, 3, CV_32F),
                               Mat::zeros(3, 1, CV_32F),
                               0);
            AddKeyframe(kf);

            // Make all keypoints as initial interest points.
            interest_points_.reserve(keypoints.size());
            for (int i = 0; i < keypoints.size(); ++i)
                interest_points_.push_back(make_shared<InterestPoint>(frame_id_, keypoints[i], descriptors.row(i)));
        } else {
            // Perform matching between the new keypoints and those in the last keyframes.
            auto &last_keyframe = keyframe(keyframe_seq_tail_);

            Mat stored_descriptors(static_cast<int>(interest_points_.size()), desc_length_, CV_8U);
            for (int i = 0; i < interest_points_.size(); ++i)
                interest_points_[i]->last_desc().copyTo(stored_descriptors.row(i));
            auto matches = interest_points_tracker_.MatchKeypoints(stored_descriptors, descriptors);

            // Estimate the fundamental matrix from the last keyframe.
            vector<Point2f> points1, points2;
            for (auto &match : matches) {
                points1.emplace_back(interest_points_[match.first]->last_loc());
                points2.emplace_back(keypoints[match.second].pt);
            }
            Mat fundamental_matrix = findFundamentalMat(points1, points2, FM_8POINT);
            fundamental_matrix.convertTo(fundamental_matrix, CV_32F);

            // Estimate the essential matrix.
            Mat essential_matrix = intrinsics_.t() * fundamental_matrix * intrinsics_;

            // Call RecoverRotAndTranslation to recover rotation and translation.
            auto candidates = RecoverRotAndTranslation(essential_matrix);
            Mat R, t;
            Mat pts3d;
            // Test for the only valid rotation and translation combination.
            {
                // Fill the data for 3D reconstruction.
                vector<pair<Mat, Mat>> data;
                if (keyframe_seq_tail_ >= 1) {
                    // We can use 2 keyframes.
                    auto &last_keyframe2 = keyframe(keyframe_seq_tail_ - 1);
                    int cnt = 0;
                    for (auto &match : matches)
                        if (interest_points_[match.first]->is_visible(last_keyframe2.frame_id))
                            ++cnt;
                    Mat stored_pts1(cnt, 2, CV_32F);
                    Mat stored_pts2(cnt, 2, CV_32F);
                    Mat new_pts(cnt, 2, CV_32F);
                    cnt = 0;
                    for (auto &match : matches)
                        if (interest_points_[match.first]->is_visible(last_keyframe2.frame_id)) {
                            // Stored keypoints.
                            auto *dest1 = reinterpret_cast<float *>(stored_pts1.ptr(cnt));
                            auto &pt1 = interest_points_[match.first]->loc(last_keyframe2.frame_id);
                            dest1[0] = pt1.x;
                            dest1[1] = pt1.y;
                            auto *dest2 = reinterpret_cast<float *>(stored_pts2.ptr(cnt));
                            auto &pt2 = interest_points_[match.first]->loc(last_keyframe.frame_id);
                            dest2[0] = pt2.x;
                            dest2[1] = pt2.y;
                            // New keypoints.
                            auto *dest = reinterpret_cast<float *>(new_pts.ptr(cnt));
                            dest[0] = keypoints[match.second].pt.x;
                            dest[1] = keypoints[match.second].pt.y;
                            ++cnt;
                        }
                    assert(cnt > 0);
                    data.emplace_back(
                            ComputeCameraMatrix(last_keyframe2.intrinsics, last_keyframe2.R, last_keyframe2.t),
                            stored_pts1);
                    data.emplace_back(ComputeCameraMatrix(last_keyframe.intrinsics, last_keyframe.R, last_keyframe.t),
                                      stored_pts2);
                    data.emplace_back(Mat(), new_pts);
                } else {
                    // We can only use 1 keyframe.
                    Mat stored_pts(static_cast<int>(matches.size()), 2, CV_32F);
                    Mat new_pts(static_cast<int>(matches.size()), 2, CV_32F);
                    for (int i = 0; i < matches.size(); ++i) {
                        auto &match = matches[i];
                        // Stored keypoints.
                        auto *dest = reinterpret_cast<float *>(stored_pts.ptr(i));
                        auto &pt = interest_points_[match.first]->last_loc();
                        dest[0] = pt.x;
                        dest[1] = pt.y;
                        // New keypoints.
                        dest = reinterpret_cast<float *>(new_pts.ptr(i));
                        dest[0] = keypoints[match.second].pt.x;
                        dest[1] = keypoints[match.second].pt.y;
                    }
                    data.emplace_back(ComputeCameraMatrix(last_keyframe.intrinsics, last_keyframe.R, last_keyframe.t),
                                      stored_pts);
                    data.emplace_back(Mat(), new_pts);
                }
                // Try each candidate of extrinsics.
                Mat bestM2;
                auto least_error = DBL_MAX;
                for (auto &M2 : candidates) {
                    data.back().first = intrinsics_ * M2;
                    Mat estimated_pts3d;
                    double err = 0;

                    Triangulate(data, estimated_pts3d, &err);
                    assert(estimated_pts3d.rows == data.back().second.rows);
                    // These 3D points are valid if they are in front of the camera in the previous keyframes.
                    bool valid = true;
//                    for (int j = 0; j <= max(1, keyframe_seq_tail_) && valid; ++j) {
//                        auto &kf = keyframe(keyframe_seq_tail_ - j);
//                        Mat T = Mat(estimated_pts3d.rows, 3, CV_32F);
//                        for (int k = 0; k < estimated_pts3d.rows; ++k)
//                            ((Mat) kf.t.t()).copyTo(T.row(k));
//                        Mat transformed_pts3d = estimated_pts3d * kf.R.t() + T;
//                        for (int k = 0; k < transformed_pts3d.rows; ++k)
//                            if (transformed_pts3d.at<float>(k, 3) < 0) {
//                                valid = false;
//                                break;
//                            }
//                    }
                    if (valid) {
                        if (err < least_error) {
                            least_error = err;
                            bestM2 = M2;
                            pts3d = estimated_pts3d;
                        }
                    }
                }
                R = bestM2.colRange(0, 3);
                t = bestM2.col(3);
            }

            // Estimate the average depth.
            Mat T = Mat(pts3d.rows, 3, CV_32F);
            for (int k = 0; k < pts3d.rows; ++k)
                ((Mat) t.t()).copyTo(T.row(k));
            Mat transformed_pts3d = pts3d * R.t() + T;
            double average_depth = sum(transformed_pts3d.col(2))[0];

            // If the translation from the last keyframe is greater than some proportion of the depth,
            // this is a new keyframe!
            double distance = cv::norm(t, cv::NormTypes::NORM_L2);
            if (distance > last_keyframe.average_depth / 5) {
                auto kf = Keyframe(frame_id_,
                                   intrinsics_,
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
                            make_shared<InterestPoint::Observation>(frame_id_, keypoints[match.second],
                                                                    descriptors.row(match.second)));
                }

                // These interest points are not visible at this frame.
                for (int i = 0; i < interest_points_.size(); ++i)
                    if (!matched_stored[i])
                        interest_points_[i]->AddObservation(make_shared<InterestPoint::Observation>());

                // These interest points are not ever visible in the previous frames.
                for (int i = 0; i < keypoints.size(); ++i)
                    if (!matched_new[i])
                        interest_points_.push_back(
                                make_shared<InterestPoint>(frame_id_, keypoints[i], descriptors.row(i)));
            }

            ReduceInterestPoints();
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
                case VObjType::TV:
                    // TODO: Draw the virtual television on the mixed_scene.
                    break;
                default:
                    return AR_UNIMPLEMENTED;
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
            if (vobj.second->IsSelected(Point2f(x, y), frame_id_)) {
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
    //	and size might be adjusted to fit the new location.
    ERROR_CODE AREngine::FixVObj(int id) {
        // TODO: This is only a fake function. Need real implementation.
        return AR_SUCCESS;
    }

    InterestPoint::InterestPoint() : vis_cnt_(0), observation_seq_tail_(-1) {}

    InterestPoint::InterestPoint(int initial_frame_id,
                                 const KeyPoint &initial_loc,
                                 const cv::Mat &initial_desc) :
            vis_cnt_(1), last_desc_(initial_desc) {
        observation_seq_[observation_seq_tail_ = 0] = make_shared<Observation>(initial_frame_id, initial_loc,
                                                                               initial_desc);
        assert(observation_seq_[0]->visible);
    }

    /// Add an observation to the interest point.
    /// In the current system setting, only observations from the keyframes shall be added.
    void InterestPoint::AddObservation(shared_ptr<Observation> p) {
        // Remove the information of the discarded observation.
        if (observation_seq_tail_ + 1 >= MAX_OBSERVATIONS) {
            auto old = observation(observation_seq_tail_ + 1);
            if (old->visible)
                --vis_cnt_;
        }
        // Add the information of the new observation.
        if (p->visible) {
            ++vis_cnt_;
            last_desc_ = p->desc;
        }
        observation_seq_[++observation_seq_tail_ % MAX_OBSERVATIONS] = p;
        if (observation_seq_tail_ >= (MAX_OBSERVATIONS << 1))
            observation_seq_tail_ -= MAX_OBSERVATIONS;
    }

    InterestPoint::Observation::Observation() : visible(false) {}

    InterestPoint::Observation::Observation(int _frame_id,
                                            const cv::KeyPoint &_pt,
                                            const cv::Mat &_desc) :
            pt(_pt), desc(_desc), visible(true), frame_id(_frame_id) {}
}
