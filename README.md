# AR Television #

This project implements an augmented reality (AR) application: virtual television. It enables users to watch TV shows or movies on AR devices.

Though the current goal of this project is limited to a specific application, we are adopting general designs for the fundamental implementation, so this project might be turned to a universal AR platform, if it works well.

Author: Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao

## How to build ##

This project requires C++17 standard.

### Windows ###
1. Install Visual Studio 2017.
2. Install OpenCV 3.3.0.
3. Create environment variable "OPENCV3", pointing to the installation directory of OpenCV, which contains the "include" and "bin" folders. This path usually ends at the "opencv/build" directory.
4. Add "%OPENCV3%/x64/vc14/bin" to the "PATH" environment variable.
5. Open the solution file "winbuild/artv.sln" in this project with VS2017.
6. Build the whole solution.