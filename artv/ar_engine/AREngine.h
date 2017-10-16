///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#pragma once

#include <common/ARUtils.hpp>
#include <common/CVUtils.h>
#include <unordered_map>

namespace ar {
	class VObject;
	//!	The class AREngine maintains the information of the percepted real world and
	//	the living hologram objects. Raw scene images and user operation events should
	//	be fed into the engine, and the engine computes the mixed-reality scene with
	//	holograms projected into the real world.
	class AREngine {
		//! For objects in this engine, they should automatically disappear if not viewed
		//	for this long period (in milliseconds). This period might be dynamically
		//	adjusted according to the number of objects there are in the engine.
		int max_idle_period_;
		std::unordered_map<int, VObject*> virtual_objects_;
	public:
		///////////////////////////////// General methods /////////////////////////////////
		ERROR_CODE GetMixedScene(const cv::Mat& raw_scene, cv::Mat& mixed_scene);
		void RemoveVObject(int id) { virtual_objects_.erase(id); }
		inline int GetMaxIdlePeriod() const { return max_idle_period_; }

		///////////////////////// Special object creating methods /////////////////////////
		//!	Create a screen displaying the content at the location in the last input scene.
		ERROR_CODE CreateScreen(cv::Point2i location, FrameStream& content_stream);
	};
}