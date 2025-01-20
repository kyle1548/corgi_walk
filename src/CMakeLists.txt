add_executable(walk_gait walk_gait.cpp)
target_link_libraries(walk_gait ${catkin_LIBRARIES} ${CORGI_UTILS})

add_executable(walk_test walk_test.cpp)
target_link_libraries(walk_test ${catkin_LIBRARIES} ${CORGI_UTILS})