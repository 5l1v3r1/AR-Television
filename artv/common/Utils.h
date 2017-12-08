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
#define AR_SAFE_CALL(err) __AR_SAFE_CALL( err, __FILE__, __LINE__ )

    inline void __AR_SAFE_CALL(ERROR_CODE err, const char *file, const int line) {
        if (AR_SUCCESS != err) {
            fprintf(stderr, "AR_SAFE_CALL() failed at %s:%i : %s\n",
                    file, line, ErrCode2Msg(err));
            exit(-1);
        }
    }
}

#endif //ARTV_UTILS_H
