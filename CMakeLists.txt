cmake_minimum_required(VERSION 2.8.3)
project(spatiotemporalexploration)

find_package(catkin REQUIRED COMPONENTS  pcl_conversions pcl_ros roscpp rospy std_msgs visualization_msgs geometry_msgs genmsg actionlib_msgs actionlib sensor_msgs image_transport)
#set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")
set ( CMAKE_CXX_FLAGS "-ggdb")

add_message_files(
  FILES
  Reachable.msg
)

add_service_files(
		FILES
		EditValue.srv
		Entropy.srv
		SaveLoad.srv
		AddView.srv
		Visualize.srv
		FlatGrid.srv
		
)

add_action_files(
	DIRECTORY 
	action 
	FILES 
	Plan.action 
	Execution.action
)

generate_messages(
	DEPENDENCIES
	std_msgs geometry_msgs actionlib_msgs
)

catkin_package(CATKIN_DEPENDS roscpp message_runtime pcl_conversions pcl_ros std_msgs genmsg actionlib_msgs strands_navigation)

include_directories(${catkin_INCLUDE_DIRS}) 


add_executable(exploration_monitor src/exploration_monitor.cpp)
add_executable(fremengrid src/fremenGrid.cpp)
add_executable(fremenutil src/fremenUtil.cpp)
add_executable(frontier_exploration src/frontier_exploration.cpp)
add_executable(analysegrid src/analyseGrid.cpp)
add_executable(exploration src/fremenExploration.cpp)
add_executable(exploration_executioner src/exploration_executioner.cpp)
add_executable(exploration_planner src/exploration_planner.cpp)
add_executable(exploration_scheduler src/exploration_scheduler.cpp)
add_executable(exploration_schedule_creator src/exploration_schedule_creator.cpp)
#add_executable(load src/load.cpp)
add_executable(flat src/flat_simulation.cpp)
add_executable(flat_generate_grids src/flat_generate_grids.cpp)
add_executable(grid_planning src/GridPlanning.cpp)
add_executable(flat_diff src/flat_diff.cpp)

add_dependencies(fremengrid ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(fremenutil ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(frontier_exploration ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(exploration ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(exploration_executioner ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(exploration_planner ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(flat_generate_grids ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(exploration_scheduler ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(exploration_schedule_creator ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(grid_planning ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_library(frelement src/CFrelement.cpp)
add_library(fremen_timer src/CTimer.cpp)
add_library(fremengridlib src/CFremenGrid.cpp)
add_library(order src/order.cpp)
add_library(rawimagelib src/CRawImage.cpp)
add_library(guilib src/CGui.cpp)

target_link_libraries(frontier_exploration rawimagelib guilib)
target_link_libraries(frelement fremen_timer)
target_link_libraries(fremengridlib frelement)
target_link_libraries(fremengrid fremengridlib)
target_link_libraries(fremenutil fremengridlib)
target_link_libraries(analysegrid fremengridlib)
target_link_libraries(fremengrid ${catkin_LIBRARIES})
target_link_libraries(fremenutil ${catkin_LIBRARIES})
target_link_libraries(order ${catkin_LIBRARIES})


target_link_libraries(flat_diff fremengridlib)
target_link_libraries(exploration_planner order)


target_link_libraries(exploration ${catkin_LIBRARIES})
target_link_libraries(exploration_monitor ${catkin_LIBRARIES} curl)
target_link_libraries(exploration_executioner ${catkin_LIBRARIES})
target_link_libraries(exploration_planner ${catkin_LIBRARIES})
target_link_libraries(exploration_scheduler ${catkin_LIBRARIES})
target_link_libraries(exploration_schedule_creator ${catkin_LIBRARIES})
target_link_libraries(grid_planning ${catkin_LIBRARIES})
target_link_libraries(flat ${catkin_LIBRARIES})
target_link_libraries(flat_generate_grids ${catkin_LIBRARIES})
target_link_libraries(flat_diff ${catkin_LIBRARIES})
target_link_libraries(frontier_exploration ${catkin_LIBRARIES} SDL SDL_ttf)

