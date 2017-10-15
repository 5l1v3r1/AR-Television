#include <iostream>

#include <opencv2/opencv.hpp>
#include <common/utils.hpp>

using namespace std;
using namespace cv;

// Input the path of a test video.
int main(int argc, char* argv[])
{
	if (argc == 1) {
		cout << "Usage: offline_demo [video_path]" << endl;
		ARTV_PAUSE;
		return 0;
	}

	const char* video_path = argv[1];
	VideoCapture cap(video_path);
	if (!cap.isOpened()) {
		cerr << "Cannot open the video at " << video_path
			<< "! Please check the path and read permission of the video" << endl;
		ARTV_PAUSE;
		return -1;
	}

	Mat frame;
	while (true) {
		cap >> frame;
		if (frame.empty())
			break;
		imshow("Origin frame", frame);
	}

	return 0;
}