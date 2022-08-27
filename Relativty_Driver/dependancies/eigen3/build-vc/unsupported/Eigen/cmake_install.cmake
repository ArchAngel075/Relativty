# Install script for directory: C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/build-vc/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/AdolcForward"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/AlignedVector3"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/ArpackSupport"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/AutoDiff"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/BVH"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/EulerAngles"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/FFT"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/IterativeSolvers"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/KroneckerProduct"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/LevenbergMarquardt"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/MatrixFunctions"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/MoreVectorization"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/MPRealSupport"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/NonLinearOptimization"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/NumericalDiff"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/OpenGLSupport"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/Polynomials"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/Skyline"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/SparseExtra"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/SpecialFunctions"
    "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/Splines"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/Users/Arch/Documents/SteamVR/Relativty/Relativty/Relativty_Driver/dependancies/eigen3/build-vc/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

