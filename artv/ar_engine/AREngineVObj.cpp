#include <common/OSUtils.h>
#include <ar_engine/AREngine.h>
#include <ar_engine/vobjects/VTelevision.h>

namespace ar {
    ERROR_CODE AREngine::CreateTelevision(cv::Point location, FrameStream &content_stream) {
        Mat camera_matrix = intrinsics_ * extrinsics_;

        Canny(last_gray_frame_, last_canny_map_, 60, 60 * 3);
        Mat dilated_canny;
        dilate(last_canny_map_, dilated_canny, noArray());

        Mat canvas = dilated_canny.clone();
        cvtColor(canvas, canvas, CV_GRAY2RGB);

        interest_points_mutex_.lock();

        // Find the interest points that roughly form a rectangle in the real world that surrounds the given location.
        vector<pair<double, shared_ptr<InterestPoint>>> left_uppers, left_lowers, right_uppers, right_lowers;
        cout << camera_matrix << endl;
        for (auto &ip : interest_points_)
            if (ip->has_estimated_3d_loc_) {
                double dist_sqr = ip->last_observation()->l2dist_sqr(location);
                if (dist_sqr > min(last_gray_frame_.rows, last_gray_frame_.cols) * VTelevision::MIN_TV_SIZE_RATE) {
                    auto loc = ip->loc(camera_matrix);
                    if (loc.x < location.x && loc.y < location.y) {
                        left_uppers.emplace_back(dist_sqr, ip);
                        circle(canvas, loc, 2, Scalar(0, 0, 255), 2);
                    } else if (loc.x > location.x && loc.y < location.y) {
                        right_uppers.emplace_back(dist_sqr, ip);
                        circle(canvas, loc, 2, Scalar(0, 255, 255), 2);
                    } else if (loc.x < location.x && loc.y > location.y) {
                        left_lowers.emplace_back(dist_sqr, ip);
                        circle(canvas, loc, 2, Scalar(255, 0, 255), 2);
                    } else if (loc.x > location.x && loc.y > location.y) {
                        right_lowers.emplace_back(dist_sqr, ip);
                        circle(canvas, loc, 2, Scalar(255, 255, 0), 2);
                    }
                }
            }

        imshow("Canny", canvas);
        waitKey(1);

        interest_points_mutex_.unlock();

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

        if (!found) {
            cout << "Cannot find valid TV boundary." << endl;
            return AR_OPERATION_FAILED;
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
}