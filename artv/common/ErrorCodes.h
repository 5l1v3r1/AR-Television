#pragma once

#define AR_SUCCESS			0
#define AR_FILE_NOT_FOUND	-1
#define AR_UNINITIALIZED	-2
#define AR_NO_MORE_FRAMES	-3

namespace ar {
	typedef int ERROR_CODE;

	const char* code2Message(ERROR_CODE errorCode) {
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