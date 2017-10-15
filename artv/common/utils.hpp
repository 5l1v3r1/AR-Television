// Cross-platform micros.
#ifdef _WIN32
#define ARTV_PAUSE system("PAUSE")
#else
#define ARTV_PAUSE getc()
#endif