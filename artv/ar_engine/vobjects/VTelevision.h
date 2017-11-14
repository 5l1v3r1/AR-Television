///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#pragma once
#include <opencv2/opencv.hpp>

#include <common/CVUtils.h>
#include <ar_engine/VObject.h>

namespace ar
{
	class VTelevision : public VObject
	{
		FrameStream& content_stream_;
		cv::Point3d base_centroid_;	// Coordinator of the centroid of the TV base in real-world coordinate system.
		cv::Vec3d normal_;			// The normal of the TV surface.
		int width_;					// Width of the TV in real-world coordinate system.
		int height_;				// Height of the TV in real-world coordinate system.
	public:
		VTelevision(AREngine& engine,
					int id,
					FrameStream& content_stream,
					const cv::Point3d& base_centroid,
					const cv::Vec3d& normal,
					int width,
					int height);
		inline virtual VObjType GetType() { return TV; }
	};
}
