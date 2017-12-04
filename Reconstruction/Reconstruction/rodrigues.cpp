//
//  rodrigues.cpp
//  Reconstruction
//
//  Created by Qiqi Xiao on 09/11/2017.
//  Copyright © 2017 Qiqi Xiao. All rights reserved.
//

#include "rodrigues.h"
usign namspace std;

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
