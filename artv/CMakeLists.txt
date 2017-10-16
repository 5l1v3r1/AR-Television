# *_SRCS are the lists that will have all the related source
# files. They will be filled with the CMakeLists.txt files
# under the folders.
set(CORE_SRCS)
set(OFFLINE_DEMO_SRCS)

# ---[ Add respective subdirectories
add_subdirectory(ar_engine)
add_subdirectory(common)
add_subdirectory(offline_demo)

add_library(artv_core ${CORE_SRCS})
add_executable(artv_offline_demo ${OFFLINE_DEMO_SRCS})