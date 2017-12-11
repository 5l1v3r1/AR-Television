#include <common/OSUtils.h>
#include <ar_engine/AREngine.h>
#include <ar_engine/vobjects/VTelevision.h>

namespace ar {
    ERROR_CODE AREngine::CreateTelevision(cv::Point2f location, FrameStream &content_stream) {
        Mat camera_matrix = intrinsics_ * extrinsics_;

        Canny(last_gray_frame_, last_canny_map_, 60, 60 * 3);
        Mat dilated_canny;
        dilate(last_canny_map_, dilated_canny, getStructuringElement(MORPH_RECT, Size(4, 4)));

        Mat canvas = dilated_canny.clone();
        cvtColor(canvas, canvas, CV_GRAY2RGB);

        interest_points_mutex_.lock();
        cout << "Lock!" << endl;

        // Find the interest points that roughly form a rectangle in the real world that surrounds the given location.
        vector<pair<double, shared_ptr<InterestPoint>>> candidates;
        for (auto &ip : interest_points_)
            if (ip->visible_in_last_frame_) {
                auto dist_sqr = norm(ip->last_loc_ - location);
                if (dist_sqr > min(last_gray_frame_.rows, last_gray_frame_.cols) * VTelevision::MIN_TV_SIZE_RATE) {
                    candidates.emplace_back(dist_sqr, ip);
                    circle(canvas, ip->last_loc_, 2, Scalar(0, 0, 255), 2);
                }
            }

        circle(canvas, location, 4, Scalar(0, 255, 0), 4);
        imshow("Canny", canvas);
        waitKey(1);

        interest_points_mutex_.unlock();
        cout << "Unlock!" << endl;

        sort(candidates.begin(), candidates.end());
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
                if (dilated_canny.at<uchar>(static_cast<int>(round(y + i * dy)), static_cast<int>(round(x + i * dx))) >
                    0)
                    ++edge_cnt;
            return edge_cnt / dist;
        };
        cout << "Finding boundary..." << endl;
        shared_ptr<InterestPoint> lu_corner, ru_corner, ll_corner, rl_corner;
        bool found = false;
        for (auto &left_upper : candidates) {
            if (found)
                break;
            for (auto &right_upper : candidates) {
                if (found)
                    break;
                if (norm(right_upper.second->last_loc_ - left_upper.second->last_loc_) < 1)
                    continue;
                if ((right_upper.second->last_loc_ - left_upper.second->last_loc_).cross(
                        location - right_upper.second->last_loc_) < 0)
                    continue;
                if (CountEdgeOnLine(left_upper.second->last_loc_, right_upper.second->last_loc_) < 0.8)
                    continue;
                for (auto &right_lower : candidates) {
                    if (found)
                        break;
                    if (norm(right_lower.second->last_loc_ - left_upper.second->last_loc_) < 1 ||
                        norm(right_lower.second->last_loc_ - right_upper.second->last_loc_) < 1)
                        continue;
                    if ((right_lower.second->last_loc_ - right_upper.second->last_loc_).cross(
                            location - right_lower.second->last_loc_) < 0)
                        continue;
                    if (CountEdgeOnLine(right_upper.second->last_loc_, right_lower.second->last_loc_) < 0.8)
                        continue;
                    for (auto &left_lower : candidates) {
                        if (norm(left_lower.second->last_loc_ - left_upper.second->last_loc_) < 1 ||
                            norm(left_lower.second->last_loc_ - right_upper.second->last_loc_) < 1 ||
                            norm(left_lower.second->last_loc_ - right_lower.second->last_loc_) < 1)
                            continue;
                        if ((left_lower.second->last_loc_ - right_lower.second->last_loc_).cross(
                                location - left_lower.second->last_loc_) < 0)
                            continue;
                        if ((left_upper.second->last_loc_ - left_lower.second->last_loc_).cross(
                                location - left_lower.second->last_loc_) < 0)
                            continue;
                        if (CountEdgeOnLine(right_lower.second->last_loc_, left_lower.second->last_loc_) < 0.8)
                            continue;
                        if (CountEdgeOnLine(left_lower.second->last_loc_, left_upper.second->last_loc_) < 0.8)
                            continue;
                        found = true;
                        lu_corner = left_upper.second;
                        ru_corner = right_upper.second;
                        ll_corner = left_lower.second;
                        rl_corner = right_lower.second;
                    }
                }
            }
        }

        if (!found) {
            cout << "Cannot find valid TV boundary." << endl;
            return AR_OPERATION_FAILED;
        }
        cout << "Found!" << endl;

        putText(canvas, "1", lu_corner->last_loc_, HersheyFonts::FONT_HERSHEY_PLAIN, 8, Scalar(255, 0, 0));
        putText(canvas, "2", ru_corner->last_loc_, HersheyFonts::FONT_HERSHEY_PLAIN, 8, Scalar(255, 0, 0));
        putText(canvas, "3", rl_corner->last_loc_, HersheyFonts::FONT_HERSHEY_PLAIN, 8, Scalar(255, 0, 0));
        putText(canvas, "4", ll_corner->last_loc_, HersheyFonts::FONT_HERSHEY_PLAIN, 8, Scalar(255, 0, 0));
        line(canvas, lu_corner->last_loc_, ru_corner->last_loc_, Scalar(255, 0, 0), 4);
        line(canvas, ru_corner->last_loc_, rl_corner->last_loc_, Scalar(255, 0, 0), 4);
        line(canvas, rl_corner->last_loc_, ll_corner->last_loc_, Scalar(255, 0, 0), 4);
        line(canvas, ll_corner->last_loc_, lu_corner->last_loc_, Scalar(255, 0, 0), 4);
        imshow("Canny", canvas);
        waitKey(1);

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
}