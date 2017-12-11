#include <common/OSUtils.h>
#include <ar_engine/AREngine.h>
#include <ar_engine/vobjects/VTelevision.h>

namespace ar {
    ERROR_CODE AREngine::CreateTelevision(cv::Point2f location, FrameStream &content_stream) {
        Mat camera_matrix = intrinsics_ * extrinsics_;

        Canny(last_gray_frame_, last_canny_map_, 60, 60 * 3);
        Mat dilated_canny;
        dilate(last_canny_map_, dilated_canny, getStructuringElement(MORPH_RECT, Size(8, 8)));

        Mat canvas = dilated_canny.clone();
        cvtColor(canvas, canvas, CV_GRAY2RGB);

        interest_points_mutex_.lock();

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
        for (auto &ip : transient_interest_points_) {
            assert(ip->visible_in_last_frame_);
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

        // Sort by distance to the selected location.
        sort(candidates.begin(), candidates.end());

        auto CountEdgeOnLine = [dilated_canny](const Point2f &start, const Point2f &end) {
            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double dist = sqrt(dx * dx + dy * dy);
            dx /= dist;
            dy /= dist;
            auto x = static_cast<int>(start.x + dx);
            auto y = static_cast<int>(start.y + dy);
            int non_edge_cnt = 0;
            for (int i = 1; i < dist && non_edge_cnt < dist * 0.2; ++i)
                if (!dilated_canny.at<uchar>(static_cast<int>(round(y + i * dy)), static_cast<int>(round(x + i * dx))))
                    ++non_edge_cnt;
            return 1 - non_edge_cnt / dist;
        };

        // Pre-compute the connections.
        vector<vector<int>> next(candidates.size());
        for (int i = 0; i < candidates.size(); ++i)
            for (int j = 0; j < candidates.size(); ++j)
                if (i != j) {
                    auto &a = candidates[i].second->last_loc_;
                    auto &b = candidates[j].second->last_loc_;
                    if ((b - a).cross(location - b) > 0 &&
                        CountEdgeOnLine(a, b) >= 0.8)
                        next[i].push_back(j);
                }

        shared_ptr<InterestPoint> lu_corner, ru_corner, rl_corner, ll_corner;
        bool found = false;
        for (int lu =0; lu < candidates.size(); ++lu) {
            auto &left_upper = candidates[lu].second->last_loc_;
            if (left_upper.x >= location.x || left_upper.y >= location.y)
                continue;
            for (int ru : next[lu]) {
                for (int rl : next[ru]) {
                    for (int ll : next[rl]) {
                        for (int end : next[ll])
                            if (end == lu) {
                                lu_corner = candidates[lu].second;
                                ru_corner = candidates[ru].second;
                                rl_corner = candidates[rl].second;
                                ll_corner = candidates[ll].second;
                                found = true;
                                break;
                            }
                        if (found)
                            break;
                    }
                    if (found)
                        break;
                }
                if (found)
                    break;
            }
            if (found)
                break;
        }

        if (!found) {
            cout << "Cannot find valid TV boundary." << endl;
            return AR_OPERATION_FAILED;
        }
        cout << "Found!" << endl;

        interest_points_mutex_.lock();
        interest_points_.push_back(lu_corner);
        interest_points_.push_back(ru_corner);
        interest_points_.push_back(ll_corner);
        interest_points_.push_back(rl_corner);
        interest_points_mutex_.unlock();

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