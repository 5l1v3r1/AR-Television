////////////////////////////////////////////////////////////
/// AR Television
/// Copyright(c) 2017 Carnegie Mellon University
/// Licensed under The MIT License[see LICENSE for details]
/// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
////////////////////////////////////////////////////////////
#include <common/Utils.h>
#include <ar_engine/vobjects/VTelevision.h>

using namespace std;
using namespace cv;

namespace ar {
    const double VTelevision::MIN_TV_SIZE_RATE = 0.01;

    VTelevision::VTelevision(AREngine &engine,
                             int id,
                             FrameStream &content_stream) :
            VObject(engine, id, INT_MAX),
            content_stream_(content_stream) {
    }

    bool VTelevision::IsAlive() {
        return !(left_upper_->ToDiscard() || left_lower_->ToDiscard() ||
                 right_upper_->ToDiscard() || right_lower_->ToDiscard());
    }

    bool VTelevision::IsSelected(Point2f pt2d, int frame_id) {
        assert(left_upper_ && left_lower_ && right_upper_ && right_lower_);
        Point2f lu = left_upper_->observation(frame_id)->loc();
        Point2f ll = left_lower_->observation(frame_id)->loc();
        Point2f ru = right_upper_->observation(frame_id)->loc();
        Point2f rl = right_lower_->observation(frame_id)->loc();

        return (ru - lu).cross(pt2d - lu) > 0
               && (rl - ru).cross(pt2d - ru) > 0
               && (ll - rl).cross(pt2d - rl) > 0
               && (lu - ll).cross(pt2d - ll) > 0;
    }

    void VTelevision::locate(shared_ptr<InterestPoint> left_upper,
                             shared_ptr<InterestPoint> left_lower,
                             shared_ptr<InterestPoint> right_upper,
                             shared_ptr<InterestPoint> right_lower) {
        left_upper_ = left_upper;
        left_lower_ = left_lower;
        right_upper_ = right_upper;
        right_lower_ = right_lower;
        assert(left_upper_ && left_lower_ && right_upper_ && right_lower_);
        assert(left_upper_.use_count() > 1);
    }

    void VTelevision::Draw(cv::Mat &scene, const cv::Mat &camera_matrix) {
        // Use the camera matrix to project the television and the
        // video content on the scene.

//        Point2f lu = left_upper_->last_loc_;
//        Point2f ll = left_lower_->last_loc_;
//        Point2f ru = right_upper_->last_loc_;
//        Point2f rl = right_lower_->last_loc_;

        Point2f lu = left_upper_->loc(camera_matrix);
        Point2f ll = left_lower_->loc(camera_matrix);
        Point2f ru = right_upper_->loc(camera_matrix);
        Point2f rl = right_lower_->loc(camera_matrix);

        Mat frame;
        AR_SAFE_CALL(content_stream_.NextFrame(frame));
        vector<Point2f> pts_src = {Point(0, 0), Point(frame.cols, 0), Point(0, frame.rows),
                                   Point(frame.cols, frame.rows)};
        vector<Point2f> pts_dst = {lu, ru, ll, rl};
        DrawPolygon(scene, frame, pts_src, pts_dst);
    }

    void VTelevision::DrawPolygon(Mat &scene, Mat &frame, vector<Point2f> &pts_src, vector<Point2f> &pts_dst) {
        Mat dst;
        Mat h = findHomography(pts_src, pts_dst);
        Mat mask(scene.size(), scene.type(), Scalar(0, 0, 0));
        warpPerspective(frame, dst, h, scene.size());
        mask = (mask == dst);
        bitwise_and(scene, mask, scene);
        scene = scene + dst;
    }
}