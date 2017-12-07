# AR Television #

This project implements an augmented reality (AR) application: virtual television. It enables users to watch TV shows or movies on AR devices.

Though the current goal of this project is limited to a specific application, we are adopting general designs for the fundamental implementation, so this project might be turned to a universal AR platform, if it works well.

Author: Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao

## How to build ##

This project requires C++17 standard.

### Ubuntu ###

1. Install Ceres-Solver according to the [tutorial](http://ceres-solver.org/installation.html).
2. Set environment "EIGEN3_INCLUDE_DIR" as "/usr/include/eigen3".
3. Install Intel Math Kernel Libraries 2018.1 or later.
4. Install OpenCV 3.3.0 or higher. Enable MKL support during building.
5. ```bash
   clone https://github.com/kyu-sz/AR-Television.git
   mkdir build
   cd build
   cmake ..
   make -j 4
   ```

### Windows ###
1. Install Visual Studio 2017.
2. Install Intel Math Kernel Libraries 2018.1 or later.
2. Install Ceres-solver with its dependencies.
2. Install OpenCV 3.3.0 or higher. Better recompile it with MKL support.
3. Create environment variable "OPENCV3", pointing to the installation directory of OpenCV, which contains the "include" and "bin" folders. This path usually ends at the "opencv/build" directory.
4. Add "%OPENCV3%/x64/vc14/bin" to the "PATH" environment variable.
5. Open the solution file "winbuild/artv.sln" in this project with VS2017.
6. Build the whole solution.