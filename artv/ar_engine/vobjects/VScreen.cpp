///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <ar_engine/vobjects/VScreen.h>

using namespace cv;

namespace ar
{
	VScreen::VScreen(AREngine& engine, int id,
		FrameStream& content_stream, Point3d base_centroid, Vec3d normal, int width, int height) :
		VObject(engine, id),
		content_stream_(content_stream), base_centroid_(base_centroid), normal_(normal), width_(width), height_(height)
	{
		// TODO: Need implementation.
	}
}