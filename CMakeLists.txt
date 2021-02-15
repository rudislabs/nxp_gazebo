cmake_minimum_required(VERSION 2.8.3)
project(nxp_gazebo)

add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        mavros_msgs
        geometry_msgs
        gazebo_msgs
        rqt_bag
        rviz
        octomap_ros
        rospy
        pcl_ros
        tf2_ros
        tf2_eigen
        tf2
        tf
        )

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)

add_message_files(FILES
   PixyVector.msg)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

link_libraries(
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

set(deps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${CATKIN_EXPORTED_TARGETS})
message(STATUS "deps: ${deps}")

# models
file(GLOB_RECURSE model_scripts
	${CMAKE_CURRENT_SOURCE_DIR}/world/*world.py
	${CMAKE_CURRENT_SOURCE_DIR}/models/*sdf.py
	)
set(model_files)
foreach(model_script ${model_scripts})
	get_filename_component(dir ${model_script} DIRECTORY)
	get_filename_component(filename ${model_script} NAME_WE)
	set(output_file ${dir}/${filename}.sdf)
	message(STATUS "creating generation rule for ${filename}") 
	set(deps ${model_script})
	set(jinja_file ${dir}/${filename}.jinja)
	if(EXISTS ${jinja_file})
		list(APPEND deps ${dir}/${filename}.jinja)
	endif()
	add_custom_command(OUTPUT ${output_file}
		COMMAND ${PYTHON_EXECUTABLE} ${model_script}
		WORKING_DIRECTORY ${dir}
		DEPENDS ${deps}
	)
	list(APPEND model_files ${output_file})
endforeach()
add_custom_target(model_generation ALL DEPENDS ${model_files})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
