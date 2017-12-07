///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <ar_engine/vobjects/VTelevision.h>

using namespace std;
using namespace cv;

namespace ar
{
	const double VTelevision::MEAN_TV_SIZE_RATE = 0.1;

	VTelevision::VTelevision(AREngine& engine,
							 int id,
							 FrameStream& content_stream) :
		VObject(engine, id, INT_MAX),
		content_stream_(content_stream)
	{
	}

	bool VTelevision::IsSelected(Point2f pt2d, int frame_id) {
		Point2f lu = left_upper_->observation(frame_id).pt.pt;
		Point2f ll = left_lower_->observation(frame_id).pt.pt;
		Point2f ru = right_upper_->observation(frame_id).pt.pt;
		Point2f rl = right_lower_->observation(frame_id).pt.pt;

		return (ru - lu).cross(pt2d - lu) > 0
			&& (rl - ru).cross(pt2d - ru) > 0
			&& (ll - rl).cross(pt2d - rl) > 0
			&& (lu - ll).cross(pt2d - ll) > 0;
	}

	void VTelevision::locate(const shared_ptr<const InterestPoint>& left_upper,
							 const shared_ptr<const InterestPoint>& left_lower,
							 const shared_ptr<const InterestPoint>& right_upper,
							 const shared_ptr<const InterestPoint>& right_lower) {
		left_upper_ = left_upper;
		left_lower_ = left_lower;
		right_upper_ = right_upper;
		right_lower_ = right_lower;
	}

	void VTelevision::Draw(cv::Mat& scene, const cv::Mat& camera_matrix) {
		// TODO: Need implementation. Use the camera matrix to project the television and the
		// video content on the scene.
		Point2f lu = ProjectPoint(camera_matrix, left_upper_);
		Point2f ll = ProjectPoint(camera_matrix, left_lower_);
		Point2f ru = ProjectPoint(camera_matrix, right_upper_);
		Point2f rl = ProjectPoint(camera_matrix, right_lower_);
		Mat frame;
		content_stream_.NextFrame(frame);
		vector<Point2f> pts_src = { Point(0,0), Point(frame.cols,0), Point(0,frame.rows), Point(frame.cols,frame.rows) };
		vector<Point2f> pts_dst = { lu, ru, ll, rl };
		DrawPolygon(scene, frame, pts_src, pts_dst);
	}

	inline Point2f VTelevision::ProjectPoint(const cv::Mat& camera_matrix, shared_ptr<const InterestPoint> point3d) {
		Point2f result;
		Mat origin(4, 1, CV_64F);
		Mat projected(3, 1, CV_64F);
		origin.data[0] = point3d->loc3d_.x;
		origin.data[0] = point3d->loc3d_.y;
		origin.data[0] = point3d->loc3d_.z;
		origin.data[0] = double(1);
		projected = camera_matrix * origin;
		result.x = projected.data[0] / projected.data[2];
		result.y = projected.data[1] / projected.data[2];
		return result;
	}

	void VTelevision::DrawPolygon(Mat &scene, Mat &frame, vector<Point2f> &pts_src, vector<Point2f> &pts_dst) {
		Mat dst = scene.clone();
		Mat h = findHomography(pts_src, pts_dst);
		Mat mask(scene.rows, scene.cols, CV_8UC3, Scalar(0, 0, 0));
		warpPerspective(frame, dst, h, dst.size());
		mask = (mask == dst);
		bitwise_and(scene, mask, scene);
		scene = scene + dst;
	}
}