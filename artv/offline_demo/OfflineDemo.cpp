#include <iostream>

#include <opencv2/opencv.hpp>

#include <common/OSUtils.h>
#include <ar_engine/AREngine.h>

using namespace std;
using namespace cv;
using namespace ar;

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
	auto ret = tv_show.open(movie_path);
	if (ret < 0) {
		cerr << "Cannot initialize the TV show: " << code2Message(ret) << endl;
		AR_PAUSE;
		return -1;
	}

	AREngine arEngine;
	Mat raw_scene, mixed_scene;
	while (true) {
		cap >> raw_scene;
		if (raw_scene.empty())
			break;
		imshow("Origin scene", raw_scene);

		arEngine.getMixedScene(raw_scene, mixed_scene);
		imshow("Mixed scene", mixed_scene);

		// TODO: Listen mouse click action, and register TVs into the AR engine.
	}

	return 0;
}