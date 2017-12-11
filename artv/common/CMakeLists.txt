file(GLOB_RECURSE tmp *.cpp *.cc *.c *.hpp *.h)
set(COMMON_SRCS ${COMMON_SRCS} ${tmp} Utils.h)

add_library(common SHARED ${COMMON_SRCS})

target_link_libraries(common LINK_PUBLIC ${OpenCV_LIBS} ${CERES_LIBRARIES})