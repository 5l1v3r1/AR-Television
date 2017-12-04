//
//  main.cpp
//  Reconstruction
//
//  Created by Qiqi Xiao on 08/11/2017.
//  Copyright © 2017 Qiqi Xiao. All rights reserved.
//
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <ceres/ceres.h>
#include <iostream>
#include "Frame.h"
using namespace cv;
using namespace std;

int main( int argc, char** argv)
{
    Mat img1, img2;
    img1 = imread("testdata/img1.png");
    img2 = imread("testdata/img2.png");
    M = max(img1.size());
    Frame frame1 = Frame(img1, 0);
    Frame frame2 = Frame(img2, 1);
    vector<vector<double>> K1, K2;
    vector<vector<double>> pts1, pts2;
    vector<double> x1, y1;
    vector<vector<double>> F = eightpoint(pts1, pts2, M);
    vector<vector<double>> E = essentialMatrix(F, K1, K2);
    
}
