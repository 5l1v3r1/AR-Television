file(GLOB tmp *.cpp)
set(COMMON_SRCS ${COMMON_SRCS} ${tmp})

add_library(common SHARED ${COMMON_SRCS})

target_link_libraries(common LINK_PUBLIC ${OpenCV_LIBS} ${CERES_LIBRARIES})