#pragma once

#include <common/CVUtils.h>

namespace ar {
	//!	The class AREngine maintains the information of the percepted real world and
	//	the living hologram objects. Raw scene images and user operation events should
	//	be fed into the engine, and the engine computes the mixed-reality scene with
	//	holograms projected into the real world.
	class AREngine {
	public:
		ERROR_CODE getMixedScene(const cv::Mat& raw_scene, cv::Mat& mixed_scene);
	};
}