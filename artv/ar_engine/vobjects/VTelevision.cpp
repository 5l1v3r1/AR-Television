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

	void VTelevision::Draw(cv::Mat& scene, const cv::Mat& camera_matrix) {
		// TODO: Need implementation. Use the camera matrix to project the television and the
		// video content on the scene.

		Point3f lu = left_upper_->loc3d();
		Point3f ll = left_lower_->loc3d();
		Point3f ru = right_upper_->loc3d();
		Point3f rl = right_lower_->loc3d();
		Mat frame;
		content_stream_.NextFrame(frame);
		vector<Point3f> corners_ori = { lu, ru, rl, ll };
		//Point3f normal_vector = Point3f(0, 0, -0.1);
		Point3f normal_vector = get_normal_vector(corners_ori);
		vector<Point3f> corners_dst = add_vector(corners_ori, normal_vector);
		vector<Point2f> pts_src = { Point(0,0), Point(frame.cols,0), Point(frame.cols,frame.rows), Point(0,frame.rows) };
		//vector<Point2f> pts_ori = { Point(100,50), Point(600,100), Point(550,300), Point(120,200) };
		//vector<Point2f> pts_dst = { Point(120,70), Point(500,100), Point(450,350), Point(150,250) };
		vector<Point2f> pts_ori = ProjectCorners(camera_matrix, corners_ori);//{ lu, ru, rl, ll };
		vector<Point2f> pts_dst = ProjectCorners(camera_matrix, corners_dst);
	}

	void VTelevision::DrawPolygon(cv::Mat &scene, cv::Mat &frame, vector<Point2f> &pts_src, vector<Point2f> &pts_dst) {
		Mat dst = scene.clone();
		Mat h = findHomography(pts_src, pts_dst);
		Mat mask(scene.rows, scene.cols, CV_8UC3, Scalar(0, 0, 0));
		warpPerspective(frame, dst, h, dst.size());
		mask = (mask == dst);
		bitwise_and(scene, mask, scene);
		scene = scene + dst;
	}

	void VTelevision::DrawBoards(cv::Mat &scene, vector<Point2f> &pts_ori, vector<Point2f> &pts_dst, cv::Scalar color) {
		vector<vector<Point>> boards[4];
		boards[0].push_back({ pts_ori[0], pts_dst[0], pts_dst[1], pts_ori[1] });
		boards[1].push_back({ pts_ori[1], pts_dst[1], pts_dst[2], pts_ori[2] });
		boards[2].push_back({ pts_ori[2], pts_dst[2], pts_dst[3], pts_ori[3] });
		boards[3].push_back({ pts_ori[3], pts_dst[3], pts_dst[0], pts_ori[0] });
		for (int i = 0; i < 4; i++)
			cv::fillPoly(scene, boards[i], color);
		for (int i = 0; i < 4; i++) {
			cv::line(scene, pts_ori[i], pts_dst[i], Scalar(0, 0, 0));
			//cv::line(scene, pts_ori[i], pts_ori[(i + 1) % 4], Scalar(0, 0, 0));
			cv::line(scene, pts_dst[i], pts_dst[(i + 1) % 4], Scalar(0, 0, 0));
		}
	}

	inline Point2f VTelevision::ProjectPoint(const cv::Mat& camera_matrix, cv::Point3f point3f) {
		Point2f result;
		Mat origin(4, 1, CV_32F);
		Mat projected(3, 1, CV_32F);
		origin.at<float>(0, 0) = point3f.x;
		origin.at<float>(1, 0) = point3f.y;
		origin.at<float>(2, 0) = point3f.z;
		origin.at<float>(3, 0) = double(1);
		projected = camera_matrix * origin;
		result.x = projected.at<float>(0, 0) / projected.at<float>(2, 0);
		result.y = projected.at<float>(1, 0) / projected.at<float>(2, 0);
		return result;
	}

	inline vector<Point2f> VTelevision::ProjectCorners(const cv::Mat& camera_matrix, vector<Point3f> &corners) {
		vector<Point2f> result(4);
		for (int i = 0; i < 4; i++) {
			result[i] = ProjectPoint(camera_matrix, corners[i]);
		}
		return result;
	}

	void VTelevision::normalize_vector(Point3f &v, float width) {
		float length = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
		float times = width / length;
		v = v * times;
		if (v.z > 0)
			v = -v;
	}

	Point3f VTelevision::get_normal_vector(const vector<Point3f> &ori) {
		Mat A = Mat(6, 4, CV_32F);
		int index = 0;
		for (int i = 0; i < 4; i++)
			for (int j = i + 1; j < 4; j++, index++) {
				A.at<float>(index, 0) = ori[i].x - ori[j].x;
				A.at<float>(index, 1) = ori[i].y - ori[j].y;
				A.at<float>(index, 2) = ori[i].z - ori[j].z;
				A.at<float>(index, 3) = 1;
			}
		Mat U, S, V;
		SVD::compute(A, U, S, V);
		Point3f result;
		result.x = V.at<float>(3, 0);
		result.y = V.at<float>(3, 1);
		result.z = V.at<float>(3, 2);
		normalize_vector(result, 0.1);
		return result;
	}

	inline vector<Point3f> VTelevision::add_vector(const vector<Point3f> &ori, const Point3f &normal_vector) {
		vector<Point3f> result(4);
		for (int i = 0; i < 4; i++)
			result[i] = ori[i] + normal_vector;
		return result;
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
}