cmake_minimum_required(VERSION 2.8.3)
project(asr_fake_object_recognition)

# C++11 is needed
add_definitions(-Wall -std=gnu++0x -ggdb)
set(CMAKE_BUILD_TYPE Debug)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  std_msgs
  message_generation
  roslib
  tf
  asr_msgs
  cmake_modules
  asr_rapidxml
)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(cfg/FakeObjectRecognition.cfg)
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

add_service_files(
   DIRECTORY srv
   FILES GetRecognizer.srv ReleaseRecognizer.srv GetAllRecognizers.srv ClearAllRecognizers.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS dynamic_reconfigure std_msgs roscpp roslib asr_msgs asr_rapidxml
    DEPENDS Eigen
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

FILE(GLOB_RECURSE LIBFILES "include/*.h")
add_executable(asr_fake_object_recognition src/fake_object_recognition.cpp
                                       src/object_config.cpp
                                       src/rating.cpp
                                       src/error_simulation.cpp
                            ${LIBFILES})

TARGET_LINK_LIBRARIES(asr_fake_object_recognition
      ${catkin_LIBRARIES}
)

add_dependencies(asr_fake_object_recognition
        ${PROJECT_NAME}_gencfg
        asr_fake_object_recognition_gencpp
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        asr_msgs_generate_messages_cpp
)

add_executable(config_marker_rotator src/config_marker_rotator.cpp)
target_link_libraries(config_marker_rotator ${catkin_LIBRARIES})