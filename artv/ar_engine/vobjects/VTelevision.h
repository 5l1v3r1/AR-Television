///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
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

		shared_ptr<const InterestPoint> left_upper_;
		shared_ptr<const InterestPoint> left_lower_;
		shared_ptr<const InterestPoint> right_upper_;
		shared_ptr<const InterestPoint> right_lower_;
	public:
		static const double MEAN_TV_SIZE_RATE;

		VTelevision(AREngine& engine,
					int id,
					FrameStream& content_stream);

		void locate(const std::shared_ptr<const InterestPoint>& left_upper,
					const std::shared_ptr<const InterestPoint>& left_lower,
					const std::shared_ptr<const InterestPoint>& right_upper,
					const std::shared_ptr<const InterestPoint>& right_lower);

		inline VObjType GetType() { return TV; }
		bool IsSelected(cv::Point2f pt2d, int frame_id);
		void Draw(cv::Mat& scene, const cv::Mat& camera_matrix);
		void DrawPolygon(Mat &src, Mat &frame, vector<Point2f> &pts_src, vector<Point2f> &pts_dst);
		inline Point2f ProjectPoint(const cv::Mat& camera_matrix, shared_ptr<const InterestPoint> point3d);		
	};
}

#endif // !VTELEVISION_H
