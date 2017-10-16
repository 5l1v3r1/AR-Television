#include <ar_engine/TV.h>

using namespace cv;

namespace ar
{
	TV::TV(FrameStream& content_stream, Point3d base_centroid, Vec3d normal, int width, int height) :
		content_stream_(content_stream), base_centroid_(base_centroid), normal_(normal), width_(width), height_(height)
	{
		// TODO: Need implementation.
	}
}