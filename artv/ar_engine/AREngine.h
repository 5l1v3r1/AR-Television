#pragma once

#include <common/CVUtils.h>
#include <unordered_map>

namespace ar {
	class Object;
	//!	The class AREngine maintains the information of the percepted real world and
	//	the living hologram objects. Raw scene images and user operation events should
	//	be fed into the engine, and the engine computes the mixed-reality scene with
	//	holograms projected into the real world.
	class AREngine {
		// For objects in this engine, they should automatically disappear if not viewed
		// for this long period (in milliseconds). This period might be dynamically
		// adjusted according to the number of objects there are in the engine.
		int max_idle_period_;
		std::unordered_map<int, Object*> objects_;
	public:
		ERROR_CODE getMixedScene(const cv::Mat& raw_scene, cv::Mat& mixed_scene);
		// Create a TV at the location in the last input scene.
		ERROR_CODE createTV(cv::Point2i location, FrameStream& content_stream);
		void removeObject(int id) { objects_.erase(id); }
		inline int getMaxIdlePeriod() const { return max_idle_period_; }
	};
}