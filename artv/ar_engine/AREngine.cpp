#include "AREngine.h"

using namespace cv;

namespace ar {
	ERROR_CODE AREngine::getMixedScene(const Mat& raw_scene, Mat& mixed_scene) {
		// TODO: This is only a fake function. Need real implementation.
		mixed_scene = raw_scene;
		return AR_SUCCESS;
	}
}