////////////////////////////////////////////////////////////
/// AR Television
/// Copyright(c) 2017 Carnegie Mellon University
/// Licensed under The MIT License[see LICENSE for details]
/// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
////////////////////////////////////////////////////////////
#pragma once
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <ar_engine/AREngine.h>

namespace ar {
	/// Base class for any virtual objects in the AR engine.
	class VObject {
		AREngine& engine_;
		int id_;

		/// Whether the object is held.
		bool on_hold_;
		/// When the object is held, its appearance on the 2D scene is frozen
		///	and can be moved along with the handle with no respect to the real world.
		cv::Mat frozen_appearance_;
	public:
		/// Layer index for dealing with virtual objects' overlapping.
		///	INT_MAX means the object is not overlappable.
		int layer_ind_;
		VObject(AREngine& engine, int id, int layer_ind);
		virtual ~VObject();
		///	Remove itself from the engine that maintains it.
		virtual bool IsAlive() = 0;
		virtual bool IsSelected(cv::Point2f pt2d, int frame_id) = 0;
		virtual void Draw(cv::Mat& scene, const cv::Mat& camera_matrix) = 0;
		virtual VObjType GetType() = 0;
	};
}