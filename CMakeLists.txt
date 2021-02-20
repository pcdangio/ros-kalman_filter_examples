cmake_minimum_required(VERSION 3.0.2)
project(kalman_filter_examples)

# Find catkin and catkin package dependencies.
# NOTE: Make sure to include kalman_filter here to build against it's libraries.
# Make sure to link executables against ${catkin_LIBRARIES} to link to kalman_filter libraries.
find_package(catkin REQUIRED COMPONENTS
  kalman_filter)

# Set up catkin output package.
catkin_package()

# Set up include directories.
include_directories(${catkin_INCLUDE_DIRS})

# Build the kf_simple example.
add_executable(${PROJECT_NAME}_kf_simple src/kf/simple.cpp)
set_target_properties(${PROJECT_NAME}_kf_simple PROPERTIES OUTPUT_NAME kf_simple PREFIX "")
target_link_libraries(${PROJECT_NAME}_kf_simple
  ${catkin_LIBRARIES})

# Build the ukf_simple example.
add_executable(${PROJECT_NAME}_ukf_simple src/ukf/simple.cpp)
set_target_properties(${PROJECT_NAME}_ukf_simple PROPERTIES OUTPUT_NAME ukf_simple PREFIX "")
target_link_libraries(${PROJECT_NAME}_ukf_simple
  ${catkin_LIBRARIES})

# Build the ukf_profile example.
add_executable(${PROJECT_NAME}_ukf_profile src/ukf/profile.cpp)
set_target_properties(${PROJECT_NAME}_ukf_profile PROPERTIES OUTPUT_NAME ukf_profile PREFIX "")
target_link_libraries(${PROJECT_NAME}_ukf_profile
  ${catkin_LIBRARIES})

# Build the ukfa_profile example.
add_executable(${PROJECT_NAME}_ukfa_profile src/ukfa/profile.cpp)
set_target_properties(${PROJECT_NAME}_ukfa_profile PROPERTIES OUTPUT_NAME ukfa_profile PREFIX "")
target_link_libraries(${PROJECT_NAME}_ukfa_profile
  ${catkin_LIBRARIES})