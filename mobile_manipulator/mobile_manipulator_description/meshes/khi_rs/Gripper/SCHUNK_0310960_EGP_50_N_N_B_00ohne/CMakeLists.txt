cmake_minimum_required(VERSION 2.8.3)

project(SCHUNK_0310960_EGP_50_N_N_B_00ohne)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
