find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

# ---[ Add respective subdirectories
add_subdirectory(ar_engine)
add_subdirectory(common)
add_subdirectory(offline_demo)
