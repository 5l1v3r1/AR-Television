#include "Object.h"
#include <common/OSUtils.h>

using namespace std;

namespace ar {
	Object::Object(int id, AREngine& engine): id_(id), engine_(engine) {
		updateViewedTime();
		monitor_thread_ = thread(monitor, this);
	}
	
	void Object::disappear() {
		engine_.removeObject(id_);
		alive_ = true;
	}

	void Object::monitor(Object* obj) {
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