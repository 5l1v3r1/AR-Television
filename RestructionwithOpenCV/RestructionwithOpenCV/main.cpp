//
//  main.cpp
//  RestructionwithOpenCV
//
//  Created by Qiqi Xiao on 09/11/2017.
//  Copyright © 2017 Qiqi Xiao. All rights reserved.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "Frame.h"
#include <map>

using namespace std;
using namespace cv;

void readCameraMatrix(const char* filename,
                      const Mat& cameraMatrix){
    FileStorage fs(filename, FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
}

vector<Mat> camera2(Mat& E){
    vector<Mat> M2s;
    SVD svd(E);
    Matx33d W(0, -1, 0, 1, 0, 0, 0, 0, 1);
    Mat_<double> R = svd.u * Mat(W) * svd.vt;
    Mat_<double> t = svd.u.col(2);
    Mat M = (Mat_<double>(3,4) << R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0), R(1, 1), R(1, 2), t(1), R(2, 0), R(2, 1), R(2, 2), t(2));
    return M;
}

void build3dmodel(const vector<Mat>& imgs,
                  const Ptr<FeatureDetector>& detector,
                  const Ptr<DescriptorExtractor>& descriptorExtrator,
                  const Mat& K,
                  const vector<Point3f>& model){
    size_t nimgs = imgs.size();
    
    //find keypoints and descriptors
    vector<vector<KeyPoint>> allkeypoints;
    vector<Mat> alldescriptors;
    vector<Vec2i> pairwiseMatches;
    for (size_t i = 0; i < nimgs; ++i){
        Mat img = imgs[i];
        Mat gray;
        cvtColor(img, gray, CV_BGR2GRAY);
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detect(img, keypoints);
        descriptorExtrator->compute(gray, keypoints, descriptor);
        allkeypoints.push_back(keypoints);
        alldescriptors.push_back(descriptor);
    }
    
    //find pairwise correspondences
    for (size_t i = 0; i < nimgs; ++i){
        for (size_t j = i+1; j < nimgs; ++j){
            vector<KeyPoint> keypoints1;
            vector<KeyPoint> keypoints2;
            const Mat descriptor1 = alldescriptors[i];
            const Mat descriptor2 = alldescriptors[j];
            BFMatcher matcher(NORM_L2);
            vector<DMatch> matches;
            matcher.match(descriptor1, descriptor2, matches);
            for (size_t m = 0; m < matches.size(); ++m){
                keypoints1.push_back(allkeypoints[i][matches[m].queryIdx]);
                keypoints2.push_back(allkeypoints[j][matches[m].trainIdx]);
            }
            
            Mat F = findFundamentalMat(keypoints1, keypoints2, FM_RANSAC, 3, 0.99);
            Mat E = K.t() * F * K; //get essential matrix
            vector<Mat> M2s = camera2(E);
            for (size_t m = 0; m < M2s.size(); ++m){
                Mat M1 = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
                Mat M2 = M2s[m];
                vector<Mat> points4d;
                triangulatePoints(K * M1, K * M2, keypoints1, keypoints2, points4d);
            }
        }
    }
    
    //visualize
    
}

int main(int argc, const char * argv[]) {
    // insert code here...
    const char* intrinsticsFilename = "intrinstics.txt";
    const char* detectorName = "SURF";
    const char* descriptorExtractorName = "ORB";
    
    Mat cameraMatrix;
    Size imageSize;
    readCameraMatrix(intrinsticsFilename, cameraMatrix);
    //Ptr<FeatureDetector> detector = FeatureDetector::create(detectorName);
    Ptr<FeatureDetector> detector = KAZE::create(detectorName);
    Ptr<DescriptorExtractor> descriptorExtractor = KAZE::create(descriptorExtractorName);
    
    Mat img1, img2;
    img1 = imread("img1.png", CV_LOAD_IMAGE_GRAYSCALE);
    img2 = imread("img2.png", CV_LOAD_IMAGE_GRAYSCALE);
    if (!img1.data || !img2.data)
        return 0;
    vector<Mat> imgs;
    imgs.push_back(img1);
    img2.push_back(img2);
    vector<Point3f> model;
    build3dmodel(imgs, detector, descriptorExtractor, cameraMatrix, model);
    
    return 0;
}
