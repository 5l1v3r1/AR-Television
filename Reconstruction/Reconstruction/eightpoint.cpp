//
//  eightpoint.cpp
//  Reconstruction
//
//  Created by Qiqi Xiao on 09/11/2017.
//  Copyright © 2017 Qiqi Xiao. All rights reserved.
//

#include "eightpoint.h"

vector<vector<double>> eightpoint(vector<vector<double>> pts1, //N*2
                                  vector<vector<double>> pts2,
                                  double M){
    int pointNo = pts1.size();
    for (int i = 0; i < pointNo; i++){
        for (int j = 0; j < 2; j++){
            pts1[i][j] /= M;
            pts2[i][j] /= M;
        }
    }
    vector<vector<double>> A(9, vector<double>(pointNo, 0.));
    for (int i = 0; i < pointNo; i++){
        A[0][i] = pts2[i][0] * pts1[i][0];
        A[1][i] = pts2[i][0] * pts1[i][1];
        A[2][i] = pts2[i][0];
        A[3][i] = pts2[i][1] * pts1[i][0];
        A[4][i] = pts2[i][1] * pts1[i][1];
        A[5][i] = pts2[i][1];
        A[6][i] = pts1[i][0];
        A[7][i] = pts1[i][1];
        A[8][i] = 1.;
    }
    vector<vector<double>> S, U, V;
    SVD::compute(A, S, U, V);
    vector<vector<double>> F(3, vector<double>(3, 0.));
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            F[i][j] = V[V.size()-1][i*3+j];
        }
    }
    vector<vector<double>> uS, uU, uV;
    SVD::compute(A, uS, uU, uV);
    
    return F;
}
