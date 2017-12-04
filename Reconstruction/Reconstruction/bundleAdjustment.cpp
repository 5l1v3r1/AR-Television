//
//  bundleAdjustment.cpp
//  Reconstruction
//
//  Created by Qiqi Xiao on 09/11/2017.
//  Copyright © 2017 Qiqi Xiao. All rights reserved.
//

#include "bundleAdjustment.h"
void bundleAdjustment(vector<vector<double>> K1,
                      vector<vector<double>> M1,
                      vector<vector<double>> p1,
                      vector<vector<double>> K2,
                      vector<vector<double>> &R2,
                      vector<double> &t2,
                      vector<vector<double>> p2,
                      vector<vector<double>> &P)
{
    vector<double> r2 = invRodrigues(R2);
    vector<double> residuals = rodriguesResidual(K1, M1, p1, K2, p2, P, r2, t2);
    //somehow optimize to get new P, r2 and t2 R2 = rodrigues(r2); }
    return;
}
