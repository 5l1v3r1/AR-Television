#pragma once
#include <chrono>
#include <thread>

#include "AREngine.h"

namespace ar {
	class Object {
		int id_;
		bool alive_ = false;
		AREngine& engine_;
		std::chrono::steady_clock::time_point last_viewed_time_;
		// The object will tell the engine to remove itself after being idle for sometime.
		std::thread monitor_thread_;
		static void monitor(Object* obj);
	public:
		Object(int id, AREngine& engine);
		inline void updateViewedTime() { last_viewed_time_ = std::chrono::steady_clock::now(); }
		void disappear();
	};
}