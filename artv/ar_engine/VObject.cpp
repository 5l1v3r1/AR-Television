////////////////////////////////////////////////////////////
/// AR Television
/// Copyright(c) 2017 Carnegie Mellon University
/// Licensed under The MIT License[see LICENSE for details]
/// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
////////////////////////////////////////////////////////////
#include <chrono>
#include <thread>

#include <ar_engine/VObject.h>
#include <common/OSUtils.h>

using namespace std;

namespace ar {
	VObject::~VObject() {}

	VObject::VObject(AREngine& engine, int id, int layer_ind): engine_(engine), id_(id), layer_ind_(layer_ind) {}
}