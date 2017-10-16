///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <ar_engine/VObject.h>
#include <common/OSUtils.h>

using namespace std;

namespace ar {
	VObject::VObject(AREngine& engine, int id): engine_(engine), id_(id) {
		updateViewedTime();
		monitor_thread_ = thread(monitor, this);
	}
	
	void VObject::disappear() {
		engine_.removeVObject(id_);
		alive_ = true;
	}

	void VObject::monitor(VObject* obj) {
		while (obj->alive_) {
			int max_idle_period = obj->engine_.getMaxIdlePeriod();
			auto now = chrono::steady_clock::now();
			int time_left = max_idle_period - (now - obj->last_viewed_time_).count() / 1000000;
			if (time_left <= 0) {
				obj->disappear();
				return;
			}
			AR_SLEEP(time_left);
		}
	}
}