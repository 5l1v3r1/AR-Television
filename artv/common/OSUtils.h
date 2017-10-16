// Cross-platform micros.
#ifdef _WIN32
#include <Windows.h>
#define AR_PAUSE system("PAUSE")
#define AR_SLEEP(x) Sleep(x)
#else
#define AR_PAUSE getc()
#define AR_SLEEP(X) usleep(x * 1000)
#endif