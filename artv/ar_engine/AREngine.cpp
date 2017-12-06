///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <opencv2/features2d.hpp>
#include <utility>

#include <common/OSUtils.h>
#include <ar_engine/AREngine.h>
#include <ar_engine/vobjects/VTelevision.h>

using namespace std;
using namespace cv;

namespace ar {
    InterestPoint::Observation EMPTY_OBSERVATION;

    //! Estimate the 3D location of the interest points with the latest keyframe asynchronously.
    //	Perform bundle adjustment based on the rough estimation of the extrinsics.
    void AREngine::EstimateMap() {
        // TODO: Need implementation. Remember to calculate the average depth!
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
                if (!interest_points_[i]->observation(frame_id).visible) {
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
            pts1.row(ip_id) = Mat(interest_points_[ip_id]->observation(frame_id1).pt.pt, false);
            pts2.row(ip_id) = Mat(interest_points_[ip_id]->observation(frame_id2).pt.pt, false);
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

    InterestPoint::Observation &InterestPoint::observation(int frame_id) {
        return frame_id < initial_frame_id_ ? EMPTY_OBSERVATION :
               observation_seq_[(frame_id - initial_frame_id_) % MAX_OBSERVATIONS];
    }

    const InterestPoint::Observation &InterestPoint::observation(int frame_id) const {
        return frame_id < initial_frame_id_ ? EMPTY_OBSERVATION :
               observation_seq_[(frame_id - initial_frame_id_) % MAX_OBSERVATIONS];
    }

    AREngine::AREngine() : interest_points_tracker_(ORB::create(),
                                                    DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE)) {
        mapping_thread_ = thread(AREngine::CallMapEstimationLoop, this);

        float default_intrinsics[][3] = {{1071.8, 0,      639.5},
                                         {0,      1071.8, 359.5},
                                         {0,      0,      1}};
        intrinsics_ = Mat(3, 3, CV_32F);
        memcpy(intrinsics_.data, default_intrinsics, sizeof(float) * 3 * 3);
    }

    //! If we have stored too many interest points, we remove the oldest location record
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
    }

    void AREngine::UpdateInterestPoints(const cv::Mat &scene) {
        // Generate new keypoints.
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        interest_points_tracker_.GenKeypointsDesc(scene, keypoints, descriptors);
        if (!desc_length_)
            desc_length_ = descriptors.cols;

        // Try to match new keypoints to the stored keypoints.
        auto *matched_new = new bool[keypoints.size()];
        auto *matched_stored = new bool[interest_points_.size()];
        memset(matched_new, 0, sizeof(bool) * keypoints.size());
        memset(matched_stored, 0, sizeof(bool) * interest_points_.size());

        if (frame_id_) {
            // Perform matching.
            cv::Mat stored_descriptors(interest_points_.size(), desc_length_, CV_32F);
            for (int i = 0; i < interest_points_.size(); ++i)
                interest_points_[i]->average_desc_.copyTo(stored_descriptors.row(i));
            /*memcpy(stored_descriptors.ptr<float>(i),
                   interest_points_[i]->average_desc_.ptr<float>(0),
                   sizeof(float) * desc_length_);*/
            auto matches = interest_points_tracker_.MatchKeypoints(descriptors, stored_descriptors);

            // Update the stored keypoints.
            for (auto match : matches) {
                matched_new[match.first] = true;
                matched_stored[match.second] = true;
                interest_points_[match.second]->AddObservation(
                        InterestPoint::Observation(keypoints[match.first], descriptors.row(match.first)));
            }

            // These interest points are not visible at this frame.
            for (int i = 0; i < interest_points_.size(); ++i)
                if (!matched_stored[i])
                    interest_points_[i]->AddObservation(InterestPoint::Observation());
        }

        // These interest points are not ever visible in the previous frames.
        for (int i = 0; i < keypoints.size(); ++i) {
            if (!matched_new[i]) {
                interest_points_.push_back(
                        std::make_shared<InterestPoint>(frame_id_, keypoints[i], descriptors.row(i)));
            }
        }
        delete[] matched_new;
        delete[] matched_stored;

        ReduceInterestPoints();
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

        UpdateInterestPoints(raw_scene);

        if (keyframe_seq_tail_ == -1) {
            // Initial keyframe.
            auto kf = Keyframe(frame_id_,
                               intrinsics_,
                               Mat::eye(3, 3, CV_32F),
                               Mat::zeros(3, 1, CV_32F),
                               0);
            AddKeyframe(kf);
        } else {
            auto &last_keyframe = keyframe(keyframe_seq_tail_);

            // TODO: Estimate the fundamental matrix from the last keyframe.
            vector<Point2f> points1, points2;
            for (const auto &ip : interest_points_) {
                if (ip->observation(last_keyframe.frame_id).visible && ip->observation(frame_id_).visible) {
                    Point2f loc1 = ip->observation(last_keyframe.frame_id).pt.pt;
                    Point2f loc2 = ip->observation(frame_id_).pt.pt;
                    points1.push_back(loc1);
                    points2.push_back(loc2);
                }
            }
            Mat fundamental_matrix = findFundamentalMat(points1, points2, FM_8POINT);
            fundamental_matrix.convertTo(fundamental_matrix, CV_32F);

            // Estimate the essential matrix.
            Mat essential_matrix = intrinsics_.t() * fundamental_matrix * last_keyframe.intrinsics;

            // Call RecoverRotAndTranslation to recover rotation and translation.
            auto candidates = RecoverRotAndTranslation(essential_matrix);
            Mat R, t;
            Mat pts3d;
            // Test for the only valid rotation and translation combination.
            {
                // Utilize at most 2 previous keyframes for bundled estimation.
                // Find the interest points that are visible in these keyframes.
                vector<int> utilized_interest_points;
                utilized_interest_points.reserve(interest_points_.size());
                for (int i = 0; i < interest_points_.size(); ++i) {
                    bool usable = true;
                    for (int j = 0; j <= max(1, keyframe_seq_tail_); ++j) {
                        int frame_id = keyframe(keyframe_seq_tail_ - j).frame_id;
                        if (!interest_points_[i]->observation(frame_id).visible) {
                            usable = false;
                            break;
                        }
                    }
                    if (usable)
                        utilized_interest_points.push_back(i);
                }
                // Fill the data for 3D reconstruction from the previous keyframes.
                vector<pair<Mat, Mat>> data;
                for (int i = 0; i < max(1, keyframe_seq_tail_); ++i) {
                    auto &kf = keyframe(keyframe_seq_tail_ - i);
                    int frame_id = kf.frame_id;
                    Mat pts(static_cast<int>(utilized_interest_points.size()), 2, CV_32F);
                    for (auto ip_id : utilized_interest_points)
                        pts.row(ip_id) = Mat(interest_points_[ip_id]->observation(frame_id).pt.pt, false);
                    Mat extrinsics;
                    hconcat(kf.R, kf.t, extrinsics);
                    data.push_back(make_pair(kf.intrinsics * extrinsics, pts));
                }
                // Fill the data from the current frame.
                Mat pts(static_cast<int>(utilized_interest_points.size()), 2, CV_32F);
                for (auto ip_id : utilized_interest_points)
                    pts.row(ip_id) = Mat(interest_points_[ip_id]->observation(frame_id_).pt.pt, false);
                data.push_back(make_pair(Mat(), pts));
                // Try each candidate of extrinsics.
                Mat bestM2;
                auto least_error = DBL_MAX;
                for (auto &M2 : candidates) {
                    data.back().first = intrinsics_ * M2;
                    Mat estimated_pts3d;
                    double err = 0;

                    triangulate(data, estimated_pts3d, &err);
                    // These 3D points are valid if they are in front of the camera in the previous keyframes.
                    bool valid = true;
                    for (int j = 0; j <= max(1, keyframe_seq_tail_) && valid; ++j) {
                        auto &kf = keyframe(keyframe_seq_tail_ - j);
                        Mat T = Mat(estimated_pts3d.rows, 3, CV_32F);
                        for (int k = 0; k < estimated_pts3d.rows; ++k)
                            Mat(kf.t.t()).copyTo(T.row(k));
                        Mat transformed_pts3d = estimated_pts3d * kf.R.t() + T;
                        for (int k = 0; k < transformed_pts3d.rows; ++k)
                            if (transformed_pts3d.at<float>(k, 3) < 0) {
                                valid = false;
                                break;
                            }
                    }
                    if (valid) {
                        if (err < least_error) {
                            least_error = err;
                            bestM2 = M2;
                            pts3d = estimated_pts3d;
                        }
                    }
                    cout << err << endl;
                }
                R = bestM2.colRange(0, 3);
                t = bestM2.col(3);
            }

            // Estimate the average depth.
            Mat T = Mat(pts3d.rows, 3, CV_32F);
            for (int k = 0; k < pts3d.rows; ++k)
                T.row(k) = t.t();
            Mat transformed_pts3d = pts3d * R.t() + T;
            double average_depth = sum(transformed_pts3d.col(2))[0];

            // If the translation from the last keyframe is greater than some proportion of the depth, update the keyframes.
            double distance = cv::norm(t, cv::NormTypes::NORM_L2);
            if (distance > last_keyframe.average_depth / 5) {
                auto kf = Keyframe(frame_id_,
                                   intrinsics_,
                                   last_keyframe.R * R,
                                   last_keyframe.t + t,
                                   average_depth);
                AddKeyframe(kf);
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
        return l2dist_sqr(o.pt.pt);
    }

    double InterestPoint::Observation::l2dist_sqr(const Point2f &p) const {
        return pow(pt.pt.x - p.x, 2) + pow(pt.pt.y - p.y, 2);
    }

    ERROR_CODE AREngine::CreateTelevision(cv::Point location, FrameStream &content_stream) {
        Canny(last_gray_frame_, last_canny_map_, 100, 200);
        Mat dilated_canny;
        dilate(last_canny_map_, dilated_canny, NULL);

        // Find the interest points that roughly form a rectangle in the real world that surrounds the given location.
        vector<pair<double, shared_ptr<InterestPoint>>> left_uppers, left_lowers, right_uppers, right_lowers;
        for (auto &ip : interest_points_) {
            double dist_sqr = ip->last_observation().l2dist_sqr(location);
            if (dist_sqr > min(last_gray_frame_.rows, last_gray_frame_.cols) * VTelevision::MEAN_TV_SIZE_RATE) {
                if (ip->last_loc().x < location.x && ip->last_loc().y < location.y)
                    left_uppers.emplace_back(dist_sqr, ip);
                else if (ip->last_loc().x > location.x && ip->last_loc().y < location.y)
                    right_uppers.emplace_back(dist_sqr, ip);
                else if (ip->last_loc().x < location.x && ip->last_loc().y > location.y)
                    left_lowers.emplace_back(dist_sqr, ip);
                else if (ip->last_loc().x > location.x && ip->last_loc().y > location.y)
                    right_lowers.emplace_back(dist_sqr, ip);
            }
        }
        sort(left_uppers.begin(), left_uppers.end());
        sort(right_uppers.begin(), right_uppers.end());
        sort(left_lowers.begin(), left_lowers.end());
        sort(right_lowers.begin(), right_lowers.end());
        auto CountEdgeOnLine = [dilated_canny](const Point2f &start, const Point2f &end) {
            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double dist = sqrt(dx * dx + dy * dy);
            dx /= dist;
            dy /= dist;
            auto x = static_cast<int>(start.x + dx);
            auto y = static_cast<int>(start.y + dy);
            int edge_cnt = 0;
            for (int i = 1; i < dist; ++i)
                if (dilated_canny.at<double>(y, x) > DBL_EPSILON)
                    ++edge_cnt;
            return edge_cnt / dist;
        };
        shared_ptr<InterestPoint> lu_corner, ru_corner, ll_corner, rl_corner;
        bool found = false;
        for (auto &lu : left_uppers) {
            if (found)
                break;
            for (auto &ru : right_uppers) {
                if (found)
                    break;
                if (CountEdgeOnLine(lu.second->last_loc(), ru.second->last_loc()) < 0.8)
                    break;
                for (auto &ll : left_lowers) {
                    if (found)
                        break;
                    if (CountEdgeOnLine(lu.second->last_loc(), ll.second->last_loc()) < 0.8)
                        break;
                    for (auto &rl : right_lowers) {
                        if (CountEdgeOnLine(ru.second->last_loc(), rl.second->last_loc()) < 0.8)
                            break;
                        if (CountEdgeOnLine(ll.second->last_loc(), rl.second->last_loc()) < 0.8)
                            break;
                        found = true;
                        lu_corner = lu.second;
                        ru_corner = ru.second;
                        ll_corner = ll.second;
                        rl_corner = rl.second;
                    }
                }
            }
        }

        // Create a virtual television, and locate it with respect to these interest points.
        int id = rand();
        while (virtual_objects_.count(id))
            id = rand();
        auto handle = new VTelevision(*this, id, content_stream);
        handle->locate(lu_corner, ll_corner, ru_corner, rl_corner);
        virtual_objects_[id] = handle;

        max_idle_period_ = static_cast<int>(60000 - virtual_objects_.size());

        return AR_SUCCESS;
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

    InterestPoint::InterestPoint(int initial_frame_id) : vis_cnt_(0), initial_frame_id_(initial_frame_id) {}

    InterestPoint::InterestPoint(int initial_frame_id,
                                 const KeyPoint &initial_loc,
                                 const cv::Mat &initial_desc) : vis_cnt_(1), initial_frame_id_(initial_frame_id) {
        observation(++observation_seq_tail_) = Observation(initial_loc, initial_desc);
        average_desc_ = initial_desc;
    }

    void InterestPoint::AddObservation(const Observation &p) {
        // Remove the information of the discarded observation.
        if (observation_seq_tail_ + 1 >= MAX_OBSERVATIONS) {
            auto &old = observation(observation_seq_tail_ + 1);
            if (old.visible) {
                if (--vis_cnt_)
                    average_desc_ = (average_desc_ * (vis_cnt_ + 1) - p.desc) / vis_cnt_;
                else
                    average_desc_ = Mat();
            }
        }
        // Add the information of the new observation.
        if (p.visible) {
            if (average_desc_.empty())
                average_desc_ = p.desc;
            else
                average_desc_ = (average_desc_ * vis_cnt_ + p.desc) / (vis_cnt_ + 1);
            ++vis_cnt_;
        }
        observation(++observation_seq_tail_) = p;
        if (observation_seq_tail_ >= (MAX_OBSERVATIONS << 1))
            observation_seq_tail_ -= MAX_OBSERVATIONS;
    }

    InterestPoint::Observation::Observation() : visible(false) {}

    InterestPoint::Observation::Observation(const cv::KeyPoint &_pt,
                                            const cv::Mat &_desc) :
            pt(_pt), desc(_desc), visible(true) {}
}
