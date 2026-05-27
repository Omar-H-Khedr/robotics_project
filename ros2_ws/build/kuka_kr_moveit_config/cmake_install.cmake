# Install script for directory: /home/omar/code/robotics_project/ros2_ws/src/external/kuka_robot_descriptions/kuka_kr_moveit_config

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/omar/code/robotics_project/ros2_ws/install/kuka_kr_moveit_config")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr4_r600.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr6_r700_sixx.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr6_r700_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr6_r900_sixx.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr6_r900_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr8_r1440_2_arc_hw.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr8_r2100_2_arc_hw.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr10_r900_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr10_r1100_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr12_r1450_3_hw.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr16_r1610_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr16_r2010_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr20_r1810_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr20_r3100.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr30_r2100.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr50_r2500.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr70_r2100.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr150_r3100_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr210_r2700_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr210_r3100_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr210_r3100_ultra.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr210_r3300_2_k.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr240_r2900_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr240_r3330.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr300_r2700_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr300_r2800_2_mt.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr360_r2830.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr500_r2800_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr560_r3100_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/urdf" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/kr800_r2800_2.srdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config" TYPE DIRECTORY FILES
    "/home/omar/code/robotics_project/ros2_ws/src/external/kuka_robot_descriptions/kuka_kr_moveit_config/config"
    "/home/omar/code/robotics_project/ros2_ws/src/external/kuka_robot_descriptions/kuka_kr_moveit_config/srdf"
    "/home/omar/code/robotics_project/ros2_ws/src/external/kuka_robot_descriptions/kuka_kr_moveit_config/launch"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/kuka_kr_moveit_config")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/kuka_kr_moveit_config")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/environment" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/environment" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_index/share/ament_index/resource_index/packages/kuka_kr_moveit_config")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config/cmake" TYPE FILE FILES
    "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_core/kuka_kr_moveit_configConfig.cmake"
    "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/ament_cmake_core/kuka_kr_moveit_configConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_kr_moveit_config" TYPE FILE FILES "/home/omar/code/robotics_project/ros2_ws/src/external/kuka_robot_descriptions/kuka_kr_moveit_config/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/omar/code/robotics_project/ros2_ws/build/kuka_kr_moveit_config/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
