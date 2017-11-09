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
#include <iostream>
#include "Frame.h"
using namespace cv;
using namespace std;

vector<vector<double>> eightpoint(vector<vector<double>> pts1,
                                  vector<vector<double>> pts2,
                                  double M){
    int pointNo = pts1.size();
    for (int i = 0; i <= pointNo; i++){
        pts1[i] /= M;
        pts2[i] /= M;
    }
    
}

vector<vector<double>> rodrigues(vector<double> r){
    double theta = norm(r);
    int dmatrix = 3;
    vector<vector<double>> R(dmatrix, vector<double>(dmatrix, 0.));
    if (theta < DBL_EPSILON){
        R[0][0] = R[1][1] = R[2][2] = 1.;
    }
    else{
        vector<double> u(dmatrix, 0.);
        for (int i = 0; i < dmatrix; ++i){
            u[i] = r[i] / theta;
        }
        double cth = cos(theta);
        double cth_comp = 1 - cth;
        double sth = sin(theta);
        R[0][0] = cth + u[0] * u[0] * cth_comp;
        R[0][1] =       u[0] * u[1] * cth_comp - u[2] * sth;
        R[0][2] =       u[0] * u[2] * cth_comp + u[1] * sth;
        R[1][0] =       u[0] * u[1] * cth_comp + u[2] * sth;
        R[1][1] = cth + u[1] * u[1] * cth_comp;
        R[1][2] =       u[1] * u[2] * cth_comp - u[0] * sth;
        R[2][0] =       u[0] * u[2] * cth_comp - u[1] * sth;
        R[2][1] =       u[1] * u[2] * cth_comp + u[0] * sth;
        R[2][2] = cth + u[2] * u[2] * cth_comp;
    }
    return R;
}

vector<double> invRodrigues(vector<vector<double>> R){
    vector<vector<double>> S, U, V;
    SVD Rsvd = SVD(R);
    int dmatrix = 3;
    vector<double> r(dmatrix, 0.);
    SVD::compute(R, S, U, V);
    double tr = 0.;
    for (int i = 0; i < dmatrix; i++){
        for (int j = 0; j < dmatrix; j++){
            tr += U[i][j] * V[i][j];
        }
    }
    tr = (tr - 1.)/2.;
    double theta = acos(tr);
    if (sin(theta) < 1e-4)
        ;
    else{
        double vth = 1./(2 * sin(theta));
        r[0] = vth * (R[2][1] - R[1][2]) * theta;
        r[1] = vth * (R[0][2] - R[2][0]) * theta;
        r[2] = vth * (R[1][0] - R[0][1]) * theta;
    }
    return r;
}

vector<double> rodriguesResidual(vector<vector<double>> K1,
                       vector<vector<double>> M1,
                       vector<vector<double>> p1, //N*2
                       vector<vector<double>> K2,
                       vector<vector<double>> p2,
                       vector<vector<double>> P, //N*3
                       vector<double> r2,
                       vector<double> t2){
    int dmatrix = 3;
    size_t pointNo = p1.size();
    vector<vector<double>> R2 = rodrigues(r2);
    vector<vector<double>> C1(dmatrix, vector<double>(dmatrix, 0.)),
                           C2(dmatrix, vector<double>(dmatrix, 0.));
    for (int i = 0; i < dmatrix; ++i){
        for (int j = 0; j < dmatrix+1; ++j){
            for (int k = 0; k < dmatrix; k++){
                C1[i][j] += K1[i][k] * M1[k][j];
            }
        }
        for (int j = 0; j < dmatrix; ++j){
            for (int k = 0; k < dmatrix; ++k){
                C2[i][j] += K2[i][k] * R2[k][j];
            }
        }
        for (int k = 0; k < dmatrix; ++k){
            C2[i][dmatrix] += K2[i][k] * t2[k];
        }
    }

    vector<vector<double>> p1_hat(dmatrix, vector<double>(pointNo, 0.)),
                            p2_hat(dmatrix, vector<double>(pointNo, 0.));
    for (int i = 0; i < dmatrix; ++i){
        for (int j = 0; j < pointNo; ++j){
            for (int k = 0; k < dmatrix; k++){
                p1_hat[i][j] += C1[i][k] * P[j][k];
                p2_hat[i][j] += C2[i][k] * P[j][k];
            }
            p1_hat[i][j] += C1[i][dmatrix];
            p2_hat[i][j] += C2[i][dmatrix];
        }
    }
    
    vector<double> resuduals; // (N*2) * 2
    for (int i = 0; i < pointNo; i++){
        resuduals.push_back(p1[i][0] - p1_hat[i][0]);
        resuduals.push_back(p1[i][1] - p1_hat[i][1]);
        resuduals.push_back(p2[i][0] - p2_hat[i][0]);
        resuduals.push_back(p2[i][1] - p2_hat[i][1]);
    }
    return resuduals;
}

void bundleAdjustment(vector<vector<double>> K1,
                      vector<vector<double>> M1,
                      vector<vector<double>> p1,
                      vector<vector<double>> K2,
                      vector<vector<double>> &R2,
                      vector<double> &t2,
                      vector<vector<double>> p2,
                      vector<vector<double>> &P){
    vector<double> r2 = invRodrigues(R2);
    vector<double> residuals = rodriguesResidual(K1, M1, p1, K2, p2, P, r2, t2);
    
    //somehow optimize to get new P, r2 and t2
    R2 = rodrigues(r2);
}

vector<Point3d> reconstruction(vector<Point> pts1, vector<Point> pts2, int M)
{
    double threshold = 1e-3;
    
}

int main( int argc, char** argv)
{
    Mat img1, img2;
    img1 = imread("testdata/img1.png");
    img2 = imread("testdata/img2.png");
    Frame frame1 = Frame(img1, 0);
    Frame frame2 = Frame(img2, 1);
    
}
