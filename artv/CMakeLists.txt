# *_SRCS are the lists that will have all the related source
# files. They will be filled with the CMakeLists.txt files
# under the folders.
set(COMMON_SRCS)
set(ENGINE_SRCS)
set(OFFLINE_DEMO_SRCS)

# ---[ Add respective subdirectories
add_subdirectory(ar_engine)
add_subdirectory(common)
add_subdirectory(offline_demo)

add_library(common ${COMMON_SRCS})
add_library(ar_engine ${ENGINE_SRCS})
add_executable(offline_demo ${OFFLINE_DEMO_SRCS})

find_package(OpenCV REQUIRED PATHS /usr/local/Cellar/opencv/3.3.0_3/share/OpenCV NO_DEFAULT_PATH)
link_libraries(${OpenCV_LIBS})
link_libraries(common)
link_libraries(ar_engine)