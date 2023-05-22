cmake_minimum_required(VERSION 3.14)

include(GNUInstallDirs)


#############
## Version ##
#############

file(READ "${CMAKE_SOURCE_DIR}/include/motion3d/motion3d.hpp" _motion3d_version_header)
string(REGEX MATCH "define[ \t]+MOTION3D_MAJOR_VERSION[ \t]+([0-9]+)" _motion3d_major_version_match "${_motion3d_version_header}")
set(MOTION3D_MAJOR_VERSION "${CMAKE_MATCH_1}")
string(REGEX MATCH "define[ \t]+MOTION3D_MINOR_VERSION[ \t]+([0-9]+)" _motion3d_minor_version_match "${_motion3d_version_header}")
set(MOTION3D_MINOR_VERSION "${CMAKE_MATCH_1}")
string(REGEX MATCH "define[ \t]+MOTION3D_PATCH_VERSION[ \t]+([0-9]+)" _motion3d_patch_version_match "${_motion3d_version_header}")
set(MOTION3D_PATCH_VERSION "${CMAKE_MATCH_1}")
set(MOTION3D_VERSION_NUMBER ${MOTION3D_MAJOR_VERSION}.${MOTION3D_MINOR_VERSION}.${MOTION3D_PATCH_VERSION})

message(STATUS "Motion3d version ${MOTION3D_VERSION_NUMBER}")


#############
## Project ##
#############

project(motion3d VERSION ${MOTION3D_VERSION_NUMBER}
                 DESCRIPTION "A header-only C++ library with full Python bindings for handling, converting, and storing 3D motions and poses.")

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif (NOT CMAKE_BUILD_TYPE)

option(COVERAGE "Activate coverage in unit tests" OFF)
option(TESTS "Build unit tests" OFF)
option(TIDY "Enable Clang-Tidy" OFF)


##################
## Dependencies ##
##################

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


#############
## Library ##
#############
 
add_library(motion3d INTERFACE)

target_include_directories(motion3d
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(motion3d INTERFACE Eigen3::Eigen)

target_compile_features(motion3d INTERFACE cxx_std_17)


#############
## Install ##
#############

install(TARGETS motion3d
        EXPORT ${PROJECT_NAME}_Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

include(CMakePackageConfigHelpers)
write_basic_package_version_file("motion3dConfigVersion.cmake"
                                 VERSION ${PROJECT_VERSION}
                                 COMPATIBILITY SameMajorVersion)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

install(EXPORT ${PROJECT_NAME}_Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/motion3d DESTINATION include)


###############
## Uninstall ##
###############

configure_file(
  "${PROJECT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)


################
## Unit Tests ##
################

if(${TESTS})
  ## Dependencies
  find_package(GTest REQUIRED)

  ## Test configurations
  enable_testing()

  add_executable(motion3d_test
    ${PROJECT_SOURCE_DIR}/test/cpp/main.cpp
  )

  target_link_libraries(motion3d_test
    PRIVATE
      motion3d
      GTest::GTest
  )

  target_include_directories(motion3d_test PRIVATE)

  target_compile_options(motion3d_test PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>)

  if(${COVERAGE})
    message(STATUS "Building unit tests for coverage analysis")
    target_link_libraries(motion3d_test PRIVATE gcov)
    target_compile_options(motion3d_test PRIVATE --coverage -g -O0)
  endif()

  if(${TIDY})
    message(STATUS "Building unit tests with clang-tidy enabled")
    target_compile_options(motion3d_test PRIVATE -fdiagnostics-color=always)
    set_property(TARGET motion3d_test PROPERTY CXX_CLANG_TIDY "clang-tidy;--use-color")
  endif()

  gtest_discover_tests(motion3d_test)
endif()
