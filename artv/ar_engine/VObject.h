///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#pragma once
#include <chrono>
#include <thread>

#include <ar_engine/AREngine.h>

namespace ar {
	//! Base class for any virtual objects in the AR engine.
	class VObject {
		int id_;
		bool alive_ = false;
		AREngine& engine_;
		std::chrono::steady_clock::time_point last_viewed_time_;
		// The object will tell the engine to remove itself after being idle for sometime.
		std::thread monitor_thread_;
		static void monitor(VObject* obj);
	public:
		VObject(int id, AREngine& engine);
		inline void updateViewedTime() { last_viewed_time_ = std::chrono::steady_clock::now(); }
		void disappear();
	};
}