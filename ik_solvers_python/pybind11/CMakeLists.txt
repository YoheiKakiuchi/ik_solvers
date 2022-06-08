set(TARGET PyIKSolvers)

add_cnoid_python_module(${TARGET} PyIKSolvers.cpp)

target_include_directories(${TARGET} PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)

target_link_libraries(${TARGET}
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  OsqpEigen::OsqpEigen
  )

target_compile_definitions(${TARGET} PUBLIC ${CHOREONOID_COMPILE_DEFINITIONS})

set(choreonoid_PYTHON_DIR ${CHOREONOID_ROOT_DIR}/${CHOREONOID_PYTHON_SUBDIR}/cnoid)## copy from jsk_choreonoid
set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PYTHON_DIR})# copy the library for calling 'import cnoid.hoge'
