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
    int holding_obj_id;
    bool left_down = false;
    bool middle_down = false;
    bool right_down = false;
    // Location of the mouse on pressing down the left button.
    int ldx, ldy;
    // Location of the mouse on pressing down the middle button.
    int mdx, mdy;
    // Location of the mouse on pressing down the right button.
    int rdx, rdy;
    AREngine *ar_engine;
    FrameStream *tv_show;
};

void RespondMouseAction(int event, int x, int y, int flags, void *p) {
    auto *mem = (MouseListenerMemory *) p;
    auto ar_engine = mem->ar_engine;
    switch (event) {
        case EVENT_LBUTTONDOWN:
            if (!mem->right_down) {
                mem->left_down = true;
                mem->ldx = x;
                mem->ldy = y;
                mem->holding_obj_id = ar_engine->GetTopVObj(x, y);
            }
            break;
        case EVENT_RBUTTONDOWN:
            if (!mem->left_down) {
                mem->right_down = true;
                mem->rdx = x;
                mem->rdy = y;
                mem->holding_obj_id = ar_engine->GetTopVObj(x, y);
            }
            break;
        case EVENT_MBUTTONDOWN:
            mem->middle_down = true;
            mem->mdx = x;
            mem->mdy = y;
            break;
        case EVENT_LBUTTONUP:
            if (mem->left_down) {
                // Place a television here!
                ar_engine->CreateTelevision(cv::Point(x, y), *mem->tv_show);
                mem->left_down = false;
                break;
            }
        case EVENT_RBUTTONUP:
            if (mem->right_down) {
                if (mem->holding_obj_id != -1)
                    ar_engine->RemoveVObject(mem->holding_obj_id);
                mem->right_down = false;
            }
            break;
        case EVENT_MBUTTONUP:
            break;
        case EVENT_MOUSEMOVE:
            if (mem->left_down)
                ar_engine->DragVObj(mem->holding_obj_id, x, y);
            break;
        default:
            break;
    }
}

std::vector<std::pair<cv::Mat, cv::Mat>> generate_data(cv::Mat &points3d, int N, int n) {
    double low = 100.0;
    double high = 200.0;
    randu(points3d, Scalar(low), Scalar(high));
    std::vector<std::pair<cv::Mat, cv::Mat>> pts(n);
    Mat a = Mat::ones(N, 1, CV_32F);
    for (int i = 0; i < n; ++i) {
        Mat P = Mat(3, 4, CV_32F);
        randu(P, Scalar(low), Scalar(high));
        Mat b;
        hconcat(points3d, a, b);
        Mat pts_2D = b * P.colRange(0, 4).t();
        pts_2D = pts_2D.colRange(0, 2) / pts_2D.colRange(3, 3);
        pts[i] = make_pair(P, pts_2D);
    }
    return pts;
}

double test(int numPoints, int numCam) {
    cout << "Testing triangulate with " << numPoints << " points and " << numCam << " cameras." << endl;
    cv::Mat points3d, gen_3d_pts;
    gen_3d_pts = Mat(numPoints, 3, CV_32F);
    vector<std::pair<cv::Mat, cv::Mat>> cameras = generate_data(gen_3d_pts, numPoints, numCam);
    cout << "Finish Generating Random Data" << endl;
    double error;
    triangulate(cameras, points3d, &error);
    Mat diff = abs(gen_3d_pts - points3d);
    return error;
}

int main(int argc, char *argv[]) {

//    double error = test(100, 2);
//    cout << error << endl;

    if (argc < 3) {
        cout << "Usage: offline_demo [scene_video_path] [tv_show_path]" << endl;
#ifdef _WIN32
        AR_PAUSE;
#endif
        return 0;
    }

    const char *scene_video_path = argv[1];
    const char *movie_path = argv[2];

    VideoCapture cap(scene_video_path);
    if (!cap.isOpened()) {
        cerr << "Cannot open the scene video at " << scene_video_path
             << "! Please check the path and read permission of the video" << endl;
#ifdef _WIN32
        AR_PAUSE;
#endif
        return -1;
    }

    auto *tv_show = new RealtimeLocalVideoStream;
    auto ret = tv_show->Open(movie_path);
    if (ret < 0) {
        cerr << "Cannot initialize the TV show: " << ErrCode2Msg(ret) << endl;
#ifdef _WIN32
        AR_PAUSE;
#endif
        return -1;
    }

    auto *ar_engine = new AREngine();

    auto width = static_cast<int>(cap.get(VideoCaptureProperties::CAP_PROP_FRAME_WIDTH));
    auto height = static_cast<int>(cap.get(VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT));
    VideoWriter recorder("demo.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(width, height));

    Mat raw_scene, mixed_scene;
    //Create the windows
    namedWindow("Origin scene");
    namedWindow("Mixed scene");
    //set the callback function for any mouse event.
    MouseListenerMemory mem;
    mem.ar_engine = ar_engine;
    mem.tv_show = tv_show;
    setMouseCallback("Origin scene", RespondMouseAction, &mem);
    setMouseCallback("Mixed scene", RespondMouseAction, &mem);
    while (true) {
        cap >> raw_scene;
        if (raw_scene.empty())
            break;
        imshow("Origin scene", raw_scene);

        ar_engine->GetMixedScene(raw_scene, mixed_scene);
        recorder << mixed_scene;
        imshow("Mixed scene", mixed_scene);
        waitKey(1);
    }

    delete ar_engine;
    delete tv_show;

    return 0;
}
