cmake_minimum_required(VERSION 3.8)
project(prioritized_inverse_kinematics_solver)

#add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

#find_package(catkin REQUIRED COMPONENTS
#  ik_constraint
#  prioritized_qp_osqp
#  )
find_package(ament_cmake REQUIRED)
find_package(ik_constraint REQUIRED)
find_package(prioritized_qp_osqp REQUIRED)
find_package(choreonoid REQUIRED)
find_package(OsqpEigen REQUIRED)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES prioritized_inverse_kinematics_solver
#  CATKIN_DEPENDS ik_constraint prioritized_qp_osqp
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${ik_constraint_INCLUDE_DIRS}
 ${prioritized_qp_osqp_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

### 相対パスを絶対パスに直す
##set(CHOREONOID_BODY_LIBRARIES_ABS)
##foreach(lib ${CHOREONOID_BODY_LIBRARIES})
##  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
##  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
##endforeach(lib)

add_library(${PROJECT_NAME} SHARED
  src/PrioritizedInverseKinematicsSolver.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${ik_constraint_LIBRARIES}
  ${prioritized_qp_osqp_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  OsqpEigen::OsqpEigen
  )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

##add_subdirectory(sample) ## TODO
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_prioritized_inverse_kinematics_solver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

ament_export_libraries(prioritized_inverse_kinematics_solver)
ament_export_dependencies(prioritized_qp_osqp ik_constraint)
ament_export_include_directories(include)
ament_package()
