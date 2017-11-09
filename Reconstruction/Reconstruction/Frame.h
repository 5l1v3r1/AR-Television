//
//  Frame.h
//  Reconstruction
//
//  Created by Qiqi Xiao on 08/11/2017.
//  Copyright © 2017 Qiqi Xiao. All rights reserved.
//

#ifndef Frame_h
#define Frame_h
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace cv;
using namespace std;
class Frame{
public:
    Frame(Mat& img, int32_t index = -1): frame_id(index), image(img){};
    vector<Point> pts;
private:
    int32_t frame_id;
    Mat image;
};

#endif /* Frame_h */
