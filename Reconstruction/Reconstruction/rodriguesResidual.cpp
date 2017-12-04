//
//  rodriguesResidual.cpp
//  Reconstruction
//
//  Created by Qiqi Xiao on 09/11/2017.
//  Copyright © 2017 Qiqi Xiao. All rights reserved.
//

#include "rodriguesResidual.h"
using namespace std;

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
