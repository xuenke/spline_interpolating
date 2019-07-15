cmake_minimum_required (VERSION 2.8)

project(spline_interpolating)

## 把我们需要的package通过catkin components的形式找到，
## 找到的catkin信息都被放在了以下变量中：
## ... catkin_INCLUDE_DIRS: 找到的package的include路径
## ... catkin_LIBRARIES: 找到的package的library
## 各个component的信息都被放在了catkin的信息中了
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  cmake_modules
)
find_package(Eigen REQUIRED)

## catkin_package是一个macro，为生成包的pkg-config和CMake指定提供信息，有5个选项：
## INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package
## LIBRARIES - The exported libraries from the project
## CATKIN_DEPENDS - Other catkin projects that this project depends on
## DEPENDS - Non-catkin CMake projects that this project depends on. For a better understanding, see this explanation.
## CFG_EXTRAS - Additional configuration options
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs rospy
  # DEPENDS
)

## 把path的信息和library信息包含进来
## 这里不需要library信息（也不推荐）
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## 指定target文件需要的cpp文件和依赖的库
## 我们用find_package找到的库都在catkin_LIBRARIES里面，链接过来就可以了
## 为何不用链接到eigen库？？好像eigen库只需要把头文件信息包含进来就可以了
add_executable(spline_interpolating src/smooth_cspline.cpp)
target_link_libraries(spline_interpolating ${catkin_LIBRARIES})
