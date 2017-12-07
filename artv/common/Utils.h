////////////////////////////////////////////////////////////
/// AR Television
/// Copyright(c) 2017 Carnegie Mellon University
/// Licensed under The MIT License[see LICENSE for details]
/// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
////////////////////////////////////////////////////////////
#pragma once

#ifndef ARTV_UTILS_H
#define ARTV_UTILS_H

#include <common/ErrorCodes.h>
#include <cstdio>
#include <cstdlib>

namespace ar {
#define ARSafeCall(err) __ARSafeCall( err, __FILE__, __LINE__ )

    inline void __ARSafeCall(ERROR_CODE err, const char *file, const int line) {
        if (AR_SUCCESS != err) {
            fprintf(stderr, "ARSafeCall() failed at %s:%i : %s\n",
                    file, line, ErrCode2Msg(err));
            exit(-1);
        }
    }
}

#endif //ARTV_UTILS_H
