///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <ar_engine/vobjects/VTelevision.h>

using namespace cv;

namespace ar
{
	VTelevision::VTelevision(AREngine& engine,
							 int id,
							 FrameStream& content_stream,
							 const Point3d& base_centroid,
							 const Vec3d& normal,
							 int width, int height) :
		VObject(engine, id),
		content_stream_(content_stream), base_centroid_(base_centroid), normal_(normal), width_(width), height_(height)
	{
		// TODO: Need implementation.
	}
}