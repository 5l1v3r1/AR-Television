///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#pragma once
#include <chrono>
#include <thread>
#include <mutex>

#include <ar_engine/AREngine.h>

namespace ar {
	//! Base class for any virtual objects in the AR engine.
	class VObject {
		class TimerKiller {
		public:
			//! @returns False if killed.
			template<class R, class P>
			inline bool WaitFor(std::chrono::duration<R, P> const& time) {
				std::unique_lock<std::mutex> lock(mutex_);
				return !cond_var_.wait_for(lock, time, [&] {return terminate_; });
			}
			inline void Kill() {
				std::unique_lock<std::mutex> lock(mutex_);
				terminate_ = true;
				cond_var_.notify_all();
			}
		private:
			std::condition_variable cond_var_;
			std::mutex mutex_;
			bool terminate_ = false;
		};

		AREngine& engine_;
		int id_;
		bool alive_ = true;
		std::chrono::steady_clock::time_point last_viewed_time_;
		// The object will tell the engine to remove itself after being idle for sometime.
		std::thread monitor_thread_;
		TimerKiller monitor_killer_;
		static void Monitor(VObject* obj);

		//! Whether the object is held.
		bool on_hold_;
		//! When the object is held, its appearance on the 2D scene is frozen
		//	and can be moved along with the handle with no respect to the real world.
		cv::Mat frozen_appearance_;
	public:
		//! Layer index for dealing with virtual objects' overlapping.
		//	INT_MAX means the object is not overlappable.
		int layer_ind_;
		VObject(AREngine& engine, int id, int layer_ind);
		virtual ~VObject();
		inline void UpdateViewedTime() { last_viewed_time_ = std::chrono::steady_clock::now(); }
		//!	Remove itself from the engine that maintains it.
		void Disappear();
		virtual bool IsSelected(cv::Point2f pt2d, int frame_id) = 0;
		virtual void Draw(cv::Mat& scene, const cv::Mat& camera_matrix) = 0;
		virtual VObjType GetType() = 0;
	};
}