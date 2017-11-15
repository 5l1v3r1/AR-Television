///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#pragma once

#include <unordered_map>
#include <vector>
#include <queue>
#include <thread>
#include <common/ARUtils.h>
#include <common/CVUtils.h>

#ifdef _WIN32
#ifdef ARENGINE_EXPORTS
#define ARENGINE_API __declspec(dllexport)
#else
#define ARENGINE_API __declspec(dllimport)
#endif
#else
#define ARENGINE_API
#endif

namespace ar {
	//! The class InterestPoint represents an interest point in the real world.
	//	It stores the 2D locations of it in some consecutive frames as well as the currently
	//	estimated 3D location of it in the real world.
	class ARENGINE_API InterestPoint {
	public:
		struct Observation {
			bool visible;
			cv::KeyPoint pt;
			cv::Mat desc;
			Observation();
			Observation(const cv::KeyPoint& _pt,
						const cv::Mat& _desc);
		};
		InterestPoint();
		InterestPoint(const cv::KeyPoint& initial_loc,
					  const cv::Mat& initial_desc);
		void AddObservation(const Observation& p);
		void RemoveEarlyObservations(int cnt = 1);
		inline bool ToDiscard() const { return vis_cnt; }
		inline auto& observation_seq() const { return observation_seq_; }
		// The weighted average feature for the interest point.
		cv::Mat average_desc_;
		//! The estimated 3D location of the point.
		cv::Point3d loc3d;
	private:
		//! Sequence of observations (2D locations and descriptors) in the frames.
		std::queue<Observation> observation_seq_;
		//! Count the number of frames in which this point is visible.
		int vis_cnt;
	};

	class VObject;
	//!	The class AREngine maintains the information of the percepted real world and
	//	the living hologram objects. Raw scene images and user operation events should
	//	be fed into the engine, and the engine computes the mixed-reality scene with
	//	holograms projected into the real world.
	class ARENGINE_API AREngine {
		bool to_terminate_ = false;
		int thread_cnt_ = 0;

		struct KeyFrame {
			cv::Mat scene;
			std::vector<std::shared_ptr<InterestPoint>> interest_points;
			//! Rotation relative to the world coordinate.
			cv::Mat rotation;
			//! Translation relative to the world coordinate.
			cv::Mat t;
		};

		static const int MAX_INTEREST_POINTS = 100;
		static const int MAX_OBSERVATIONS = 100;

		//! For objects in this engine, they should automatically disappear if not viewed
		//	for this long period (in milliseconds). This period might be dynamically
		//	adjusted according to the number of objects there are in the engine.
		int max_idle_period_;
		//!	Virtual objects are labeled with random positive integers in the AR engine.
		//	The virtual_objects_ is a map from IDs to virtual object pointers.
		std::unordered_map<int, VObject*> virtual_objects_;
		std::vector<MotionData> accumulated_motion_data_;

		//! Buffered last frame and feature maps.
		cv::Mat last_raw_frame_;
		cv::Mat last_gray_frame_;

		//! The interest points in recent frames. The observation sequence.
		std::vector<std::shared_ptr<InterestPoint>> interest_points_;
		InterestPointsTracker interest_points_tracker_;
		void UpdateInterestPoints(const cv::Mat& scene);
		//! If we have stored too many interest points, we remove the oldest location record
		//	of the interest points, and remove the interest points that are determined not visible anymore.
		void ReduceInterestPoints();

		//! Find in the current 2D frame the bounder surrounding the surface specified by a given point.
		//	@return the indices of the interest points.
		std::vector<int> FindSurroundingBounder(const cv::Point& point);
		
		//! Estimate the 3D location of the interest points with the latest keyframe asynchronously.
		void EstimateMap();
		void MapEstimationLoop();
		static void CallMapEstimationLoop(AREngine* engine);

		KeyFrame last_keyframe_;
		std::thread mapping_thread_;
	public:
		///////////////////////////////// General methods /////////////////////////////////
		AREngine();
		~AREngine();
		void RemoveVObject(int id) { virtual_objects_.erase(id); }
		inline int GetMaxIdlePeriod() const { return max_idle_period_; }

		//! Get the ID of the top virtual object at location (x, y) in the last scene.
		//	@return ID of the top virtual object. -1 for no object at the location.
		int GetTopVObj(int x, int y) const;

		//!	Drag a virtual object to a location. The virtual object is stripped from the
		//	real world by then, and its shape and size in the scene remain the same during
		//	the dragging. Call FixVObj to fix the virtual object onto the real world again.
		ERROR_CODE DragVObj(int id, int x, int y);

		//! Fix a virtual object that is floating to the real world. The orientation
		//	and size might be adjusted to fit the new location.
		ERROR_CODE FixVObj(int id);

		//! Return a mixed scene with both fixed and floating virtual objects overlaid to
		//	the raw scene.
		ERROR_CODE GetMixedScene(const cv::Mat& raw_scene, cv::Mat& mixed_scene);

		//! Feed the motion data collected by the motion sensors at the moment.
		//	The data will be accumulated and used on computing the next mixed scene,
		//	so whenever the motion data of a moment is ready, immediately input it into
		//	the AR engine with this function.
		inline void FeedMotionData(const MotionData& data) { accumulated_motion_data_.push_back(data); }

		///////////////////////// Special object creating methods /////////////////////////
		//!	Create a screen displaying the content at the location in the last input scene.
		ERROR_CODE CreateTelevision(cv::Point location, FrameStream& content_stream);
	};
}