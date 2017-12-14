////////////////////////////////////////////////////////////
/// AR Television
/// Copyright(c) 2017 Carnegie Mellon University
/// Licensed under The MIT License[see LICENSE for details]
/// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
////////////////////////////////////////////////////////////
#pragma once

#ifndef VTELEVISION_H
#define VTELEVISION_H

#include <opencv2/opencv.hpp>

#include <common/CVUtils.h>
#include <ar_engine/VObject.h>

namespace ar
{
	class VTelevision : public VObject
	{
		FrameStream& content_stream_;

		bool alive_ = true;

		shared_ptr<const InterestPoint> left_upper_;
		shared_ptr<const InterestPoint> left_lower_;
		shared_ptr<const InterestPoint> right_upper_;
		shared_ptr<const InterestPoint> right_lower_;
	public:
		static const double MIN_TV_SIZE_RATE;

		VTelevision(AREngine& engine,
					int id,
					FrameStream& content_stream);

		void locate(std::shared_ptr<InterestPoint> left_upper,
					std::shared_ptr<InterestPoint> left_lower,
					std::shared_ptr<InterestPoint> right_upper,
					std::shared_ptr<InterestPoint> right_lower);

		bool IsAlive() override;

		inline VObjType GetType() { return TV; }

		bool IsSelected(cv::Point2f pt2d, 
						int frame_id);

		void Draw(cv::Mat& scene, 
				  const cv::Mat& camera_matrix);

		void DrawPolygon(Mat &src, 
						 Mat &frame, 
						 vector<Point2f> &pts_src, 
						 vector<Point2f> &pts_dst);

		void DrawBoards(cv::Mat &scene, 
						vector<Point2f> &pts_ori, 
						vector<Point2f> &pts_dst, 
						cv::Scalar color);

		inline Point2f ProjectPoint(const cv::Mat& camera_matrix, 
									cv::Point3f point3f);

		inline vector<Point2f> ProjectCorners(const cv::Mat& camera_matrix, 
											  vector<Point3f> &corners);

		void normalize_vector(Point3f &v, 
							  float width);

		cv::Point3f get_normal_vector(const vector<Point3f> &ori);

		inline vector<Point3f> add_vector(const vector<Point3f> &ori, 
										  const Point3f &normal_vector);
	};
}

#endif // !VTELEVISION_H
