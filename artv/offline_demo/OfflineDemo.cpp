///////////////////////////////////////////////////////////
// AR Television
// Copyright(c) 2017 Carnegie Mellon University
// Licensed under The MIT License[see LICENSE for details]
// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
///////////////////////////////////////////////////////////
#include <iostream>

#include <opencv2/opencv.hpp>

#include <common/OSUtils.h>
#include <ar_engine/AREngine.h>

using namespace std;
using namespace cv;
using namespace ar;

struct MouseListenerMemory {
	bool left_down = false;
	bool middle_down = false;
	bool right_down = false;
	int ldx, ldy;
	int mdx, mdy;
	int rdx, rdy;
	AREngine* ar_engine;
	FrameStream* tv_show;
};

void RespondMouseAction(int event, int x, int y, int flags, void* p) {
	MouseListenerMemory* mem = (MouseListenerMemory*)p;
	switch (event) {
	case EVENT_LBUTTONDOWN:
		break;
	case EVENT_RBUTTONDOWN:
		break;
	case EVENT_MBUTTONDOWN:
		break;
	case EVENT_LBUTTONUP:
		break;
	case EVENT_RBUTTONUP:
		break;
	case EVENT_MBUTTONUP:
		break;
	case EVENT_MOUSEMOVE:
		break;
	default:
		break;
	}
}

int main(int argc, char* argv[]) {
	if (argc < 3) {
		cout << "Usage: offline_demo [scene_video_path] [tv_show_path]" << endl;
		AR_PAUSE;
		return 0;
	}

	const char* scene_video_path = argv[1];
	const char* movie_path = argv[2];
	
	VideoCapture cap(scene_video_path);
	if (!cap.isOpened()) {
		cerr << "Cannot open the scene video at " << scene_video_path
			<< "! Please check the path and read permission of the video" << endl;
		AR_PAUSE;
		return -1;
	}
	
	RealtimeLocalVideoStream tv_show;
	auto ret = tv_show.Open(movie_path);
	if (ret < 0) {
		cerr << "Cannot initialize the TV show: " << ErrCode2Msg(ret) << endl;
		AR_PAUSE;
		return -1;
	}

	AREngine ar_engine;

	Mat raw_scene, mixed_scene;
	//Create the windows
	namedWindow("Origin scene");
	namedWindow("Mixed scene");
	//set the callback function for any mouse event.
	MouseListenerMemory mem;
	mem.ar_engine = &ar_engine;
	mem.tv_show = &tv_show;
	setMouseCallback("Origin scene", RespondMouseAction, &mem);
	setMouseCallback("Mixed scene", RespondMouseAction, &mem);
	while (true) {
		cap >> raw_scene;
		if (raw_scene.empty())
			break;
		imshow("Origin scene", raw_scene);

		ar_engine.GetMixedScene(raw_scene, mixed_scene);
		imshow("Mixed scene", mixed_scene);

		waitKey(1);
	}

	return 0;
}