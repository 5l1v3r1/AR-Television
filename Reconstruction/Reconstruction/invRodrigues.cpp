//
//  invRodrigues.cpp
//  Reconstruction
//
//  Created by Qiqi Xiao on 09/11/2017.
//  Copyright © 2017 Qiqi Xiao. All rights reserved.
//

#include "invRodrigues.h"
using namespace std;
vector<double> invRodrigues(vector<vector<double>> R){
    vector<vector<double>> S, U, V;
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
