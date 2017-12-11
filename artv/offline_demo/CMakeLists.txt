file(GLOB_RECURSE tmp *.cpp *.cc *.c *.hpp *.h)
set(OFFLINE_DEMO_SRCS ${OFFLINE_DEMO_SRCS} ${tmp})

add_executable(demo ${OFFLINE_DEMO_SRCS})

target_link_libraries(demo LINK_PUBLIC engine common  ${OpenCV_LIBS} ${Ceres_LIBS})