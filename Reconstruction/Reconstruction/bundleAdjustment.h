//
//  bundleAdjustment.h
//  Reconstruction
//
//  Created by Qiqi Xiao on 09/11/2017.
//  Copyright © 2017 Qiqi Xiao. All rights reserved.
//

#ifndef bundleAdjustment_h
#define bundleAdjustment_h
#include <vector>
using std::vector;
void bundleAdjustment(vector<vector<double>> K1,
                      vector<vector<double>> M1,
                      vector<vector<double>> p1,
                      vector<vector<double>> K2,
                      vector<vector<double>> &R2,
                      vector<double> &t2,
                      vector<vector<dousble>> p2,
                      vector<vector<double>> &P);

#endif /* bundleAdjustment_h */
