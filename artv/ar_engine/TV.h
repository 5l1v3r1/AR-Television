#pragma once
#include <opencv2/opencv.hpp>

#include <common/CVUtils.h>
#include <ar_engine/Object.h>

namespace ar
{
	class TV : public Object
	{
		FrameStream& content_stream_;
		cv::Point3d base_centroid_;	// Coordinator of the centroid of the TV base in real-world coordinate system.
		cv::Vec3d normal_;			// The normal of the TV surface.
		int width_;					// Width of the TV in real-world coordinate system.
		int height_;				// Height of the TV in real-world coordinate system.
	public:
		TV(FrameStream& content_stream, cv::Point3d base_centroid, cv::Vec3d normal, int width, int height);
	};
}
