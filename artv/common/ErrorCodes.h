////////////////////////////////////////////////////////////
/// AR Television
/// Copyright(c) 2017 Carnegie Mellon University
/// Licensed under The MIT License[see LICENSE for details]
/// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
////////////////////////////////////////////////////////////
#pragma once

#ifndef ERRORCODES_H
#define ERRORCODES_H

#define AR_SUCCESS			0
#define AR_FILE_NOT_FOUND	-1
#define AR_UNINITIALIZED	-2
#define AR_NO_MORE_FRAMES	-3
#define AR_INVALID_INPUT    -4
#define AR_UNIMPLEMENTED    -5
#define AR_OPERATION_FAILED	-6

namespace ar {
	typedef int ERROR_CODE;

	inline const char* ErrCode2Msg(ERROR_CODE errorCode) {
		switch (errorCode)
		{
		case 0:
			return "Success.";
		case 1:
			return "File not found.";
		case 2:
			return "Instance not initialized.";
		case 3:
			return "No more frames.";
		default:
			return "Unknown error.";
		}
	}
}

#endif // !ERRORCODES_H