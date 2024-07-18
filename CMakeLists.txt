cmake_minimum_required(VERSION 3.15)
project(franka_with_rrt)
include(CTest)

set(CMAKE_CXX_STANDARD 17)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-no-as-needed")


LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake" )
INCLUDE(CMake/FindRBDL.cmake)

find_package(Boost REQUIRED)
find_package(RBDL REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Eigen3 3.3.7 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

message(STATUS "Eigen3 : " ${EIGEN3_INCLUDE_DIR})
message(STATUS "current : " ${CMAKE_CURRENT_SOURCE_DIR})

message(STATUS "Boost : " ${Boost_INCLUDE_DIR})





add_definitions(-DMJSIMULATE_STATIC) # use static =,solve the "error LNK2001/LNK2019" in windows

find_package (X11)
set(MUJOCO_HOME ${CMAKE_SOURCE_DIR})
message(STATUS "MUJOCO_HOME: " ${MUJOCO_HOME})


link_directories(${Boost_LIBRARY_DIR})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GLEW_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${MUJOCO_HOME}/lib
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
  #/usr/local/lib/pkgconfig
  #/usr/local/include/trajectory
  
)

set(SRCS
simulate/ArmPlanner.cpp
simulate/RRTFunction.cpp
simulate/Utils.cpp
)

set(HEADS
    include/ArmPlanner.h
    include/Fwd.h
    include/NodeTree.h
    include/RRTFunction.h
    include/RRTNode.h
    include/Utils.h
)
add_library(RRT 
  ${SRCS} 
)

target_link_libraries(RRT ${Boost_LIBRARIES} rbdl rbdl_urdfreader yaml-cpp trajectory)


set(COM_SRC
        simulate/glfw_dispatch.cc
        simulate/simulate.cc
        simulate/uitools.cc
        simulate/main.cc
        
        simulate/controller.cpp
        simulate/robotmodel.cpp
        simulate/trajectory.cpp
        
        )

add_executable(${PROJECT_NAME} ${COM_SRC})


target_link_libraries(${PROJECT_NAME} 
	${CMAKE_DL_LIBS} 
	${X11_LIBRARIES} 
  libmujoco.so.2.3.1
  glfw
  -lrbdl
    	-lrbdl_urdfreader
    	-lpthread
    	OpenGL::OpenGL
	OpenGL::EGL
	GLEW::GLEW
	${GLEW_LIBRARIES}
	${OPENGL_LIBRARIES}
  RRT
	
)



target_link_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  /home/kist-robot2/trajectory_smoothing/build
)
