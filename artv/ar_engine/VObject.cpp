///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <chrono>
#include <thread>

#include <ar_engine/VObject.h>
#include <common/OSUtils.h>

using namespace std;

namespace ar {
	VObject::~VObject() {

	}

	VObject::VObject(AREngine& engine, int id): engine_(engine), id_(id) {
		UpdateViewedTime();
		monitor_thread_ = thread(Monitor, this);
	}
	
	void VObject::Disappear() {
		alive_ = false;
		monitor_killer_.Kill();
		monitor_thread_.join();
		engine_.RemoveVObject(id_);
	}

	void VObject::Monitor(VObject* obj) {
		while (obj->alive_) {
			int max_idle_period = obj->engine_.GetMaxIdlePeriod();
			auto now = chrono::steady_clock::now();
			int time_left = max_idle_period - (now - obj->last_viewed_time_).count() / 1000000;
			if (time_left <= 0) {
				obj->Disappear();
				break;
			}

			if (!obj->monitor_killer_.WaitFor(std::chrono::milliseconds(time_left)))
				break;
		}
	}
}