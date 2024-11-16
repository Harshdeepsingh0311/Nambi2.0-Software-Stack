cmake_minimum_required(VERSION 2.8.3)

project(inferno_bot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  
)

catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  
)
find_package(roslaunch)
find_package(map_server)
find_package(amcl)
foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

