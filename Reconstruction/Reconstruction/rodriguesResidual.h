//
//  rodriguesResidual.h
//  Reconstruction
//
//  Created by Qiqi Xiao on 09/11/2017.
//  Copyright © 2017 Qiqi Xiao. All rights reserved.
//

#ifndef rodriguesResidual_h
#define rodriguesResidual_h
#include <vector>
vector<double> rodriguesResidual(vector<vector<double>> K1,
                                 vector<vector<double>> M1,
                                 vector<vector<double>> p1, //N*2
                                 vector<vector<double>> K2,
                                 vector<vector<double>> p2,
                                 vector<vector<double>> P, //N*3
                                 vector<double> r2,
                                 vector<double> t2);
#endif /* rodriguesResidual_h */
