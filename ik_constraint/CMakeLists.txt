cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0057 NEW)

project(ik_constraint)

#add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

#find_package(catkin REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(choreonoid REQUIRED)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ik_constraint
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS CHOREONOID
#)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)

### 相対パスを絶対パスに直す
##set(CHOREONOID_BODY_LIBRARIES_ABS)
##foreach(lib ${CHOREONOID_BODY_LIBRARIES})
##  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
##  if (NOT (${lib}_abs))
##    set(${lib}_abs ${lib})
##  endif()
##  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
##endforeach(lib)
##message ("cnoid_body_lib: ${CHOREONOID_BODY_LIBRARIES_ABS}")

add_library(${PROJECT_NAME} SHARED
  src/Jacobian.cpp
  src/IKConstraint.cpp
  src/PositionConstraint.cpp
  src/COMConstraint.cpp
  src/COMVelocityConstraint.cpp
  src/AngularMomentumConstraint.cpp
  src/AngularMomentumLimitConstraint.cpp
  src/JointAngleConstraint.cpp
  src/JointLimitConstraint.cpp
  src/JointVelocityConstraint.cpp
  src/CollisionConstraint.cpp
  src/ClientCollisionConstraint.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ik_constraint.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

ament_export_libraries(ik_constraint)
ament_export_include_directories(include)
ament_package()
