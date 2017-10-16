///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#pragma once

#include <chrono>

namespace ar {
	enum VObjType {
		SCREEN
	};

	//! Data collected by the motion sensors.
	struct MotionData {
		std::chrono::steady_clock::time_point shot_time;
		//TODO: Fill the data here.
	};
}
