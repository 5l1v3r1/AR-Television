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
#include <mutex>
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
	using namespace std;
	using namespace cv;

	//! The class InterestPoint represents an interest point in the real world.
	//	It stores the 2D locations of it in some consecutive frames as well as the currently
	//	estimated 3D location of it in the real world.
	class ARENGINE_API InterestPoint {
	public:
		static const int MAX_OBSERVATIONS = 100;
		struct Observation {
			bool visible;
			KeyPoint pt;
			Mat desc;
			Observation();
			Observation(const KeyPoint& _pt,
						const Mat& _desc);
			double l2dist_sqr(const Observation& o) const;
			double l2dist_sqr(const Point2f& p) const;
		};
		InterestPoint::Observation& observation(int frame_id);
		const InterestPoint::Observation& observation(int frame_id) const;
		inline auto& last_observation() { return observation_seq_[observation_seq_tail_ % MAX_OBSERVATIONS]; }
		inline auto& last_loc() { return last_observation().pt.pt; }
		InterestPoint(int initial_frame_id);
		InterestPoint(int initial_frame_id,
					  const KeyPoint& initial_loc,
					  const Mat& initial_desc);
		void AddObservation(const Observation& p);
		inline bool ToDiscard() const { return vis_cnt_; }
		//! The weighted average feature for the interest point.
		Mat average_desc_;
		//! The estimated 3D location of the point.
		Point3d loc3d_;
	private:
		int initial_frame_id_;
		//! Looped queue of observations (2D locations and descriptors) in the frames.
		Observation observation_seq_[MAX_OBSERVATIONS];
		int observation_seq_tail_ = -1;
		//! Count the number of frames in which this point is visible.
		int vis_cnt_;
	};

	struct Keyframe {
		int frame_id = 0;
		Mat intrinsics;
		//! Rotation relative to the world coordinate.
		Mat R;
		//! Translation relative to the world coordinate.
		Mat t;
		double average_depth = 0;
		Keyframe(int frame_id, 
				 Mat intrinsics,
				 Mat R,
				 Mat t,
				 double average_depth);
		Keyframe() {}
	};

	class VObject;
	//!	The class AREngine maintains the information of the percepted real world and
	//	the living hologram objects. Raw scene images and user operation events should
	//	be fed into the engine, and the engine computes the mixed-reality scene with
	//	holograms projected into the real world.
	class ARENGINE_API AREngine {
		bool to_terminate_ = false;
		int thread_cnt_ = 0;

		static const int MAX_INTEREST_POINTS = 100;
		static const int MAX_KEYFRAMES = 5;

		//! For objects in this engine, they should automatically disappear if not viewed
		//	for this long period (in milliseconds). This period might be dynamically
		//	adjusted according to the number of objects there are in the engine.
		int max_idle_period_;
		//!	Virtual objects are labeled with random positive integers in the AR engine.
		//	The virtual_objects_ is a map from IDs to virtual object pointers.
		unordered_map<int, VObject*> virtual_objects_;
		vector<MotionData> accumulated_motion_data_;
		Mat intrinsics_;

		Mat last_raw_frame_;
		Mat last_gray_frame_;
		Mat last_canny_map_;
		//! Rotation of the camera at the last frame with respect to the world coordinate.
		Mat last_R_;
		//! Translation of the camera at the last frame with respect to the world coordinate.
		Mat last_t_;

		//! The interest points in recent frames. The observation sequence.
		vector<shared_ptr<InterestPoint>> interest_points_;
		InterestPointsTracker interest_points_tracker_;
		void UpdateInterestPoints(const Mat& scene);
		//! If we have stored too many interest points, we remove the oldest location record
		//	of the interest points, and remove the interest points that are determined not visible anymore.
		void ReduceInterestPoints();
		
		//! Estimate the 3D location of the interest points with the latest keyframe asynchronously.
		void EstimateMap();
		void MapEstimationLoop();
		static void CallMapEstimationLoop(AREngine* engine);

		int frame_id_ = -1;
		Keyframe recent_keyframes_[MAX_KEYFRAMES];
		int keyframe_seq_tail_ = -1;
		void AddKeyframe(Keyframe& keyframe);
		inline auto& keyframe(int ind) { return recent_keyframes_[keyframe_seq_tail_ % MAX_KEYFRAMES]; }
		thread mapping_thread_;
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

		//! Feed a scene but do not get mixed scene. Should at least call this once before calling
		//	the GetMixedScene.
		ERROR_CODE FeedScene(const Mat& raw_scene);
		//! Return a mixed scene with both fixed and floating virtual objects overlaid to
		//	the raw scene.
		ERROR_CODE GetMixedScene(const Mat& raw_scene, Mat& mixed_scene);

		//! Feed the motion data collected by the motion sensors at the moment.
		//	The data will be accumulated and used on computing the next mixed scene,
		//	so whenever the motion data of a moment is ready, immediately input it into
		//	the AR engine with this function.
		inline void FeedMotionData(const MotionData& data) { accumulated_motion_data_.push_back(data); }

		///////////////////////// Special object creating methods /////////////////////////
		//!	Create a screen displaying the content at the location in the last input scene.
		ERROR_CODE CreateTelevision(Point location, FrameStream& content_stream);
	};
}
