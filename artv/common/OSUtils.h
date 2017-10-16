// Cross-platform micros.
#ifdef _WIN32
#define AR_PAUSE system("PAUSE")
#else
#define AR_PAUSE getc()
#endif