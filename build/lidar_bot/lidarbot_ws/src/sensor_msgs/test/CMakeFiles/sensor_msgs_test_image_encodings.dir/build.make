# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/akshat/ProjectROS/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/akshat/ProjectROS/catkin_ws/build

# Include any dependencies generated for this target.
include lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/depend.make

# Include the progress variables for this target.
include lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/progress.make

# Include the compile flags for this target's objects.
include lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/flags.make

lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.o: lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/flags.make
lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.o: /home/akshat/ProjectROS/catkin_ws/src/lidar_bot/lidarbot_ws/src/sensor_msgs/test/test_image_encodings.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/akshat/ProjectROS/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.o"
	cd /home/akshat/ProjectROS/catkin_ws/build/lidar_bot/lidarbot_ws/src/sensor_msgs/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.o -c /home/akshat/ProjectROS/catkin_ws/src/lidar_bot/lidarbot_ws/src/sensor_msgs/test/test_image_encodings.cpp

lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.i"
	cd /home/akshat/ProjectROS/catkin_ws/build/lidar_bot/lidarbot_ws/src/sensor_msgs/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/akshat/ProjectROS/catkin_ws/src/lidar_bot/lidarbot_ws/src/sensor_msgs/test/test_image_encodings.cpp > CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.i

lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.s"
	cd /home/akshat/ProjectROS/catkin_ws/build/lidar_bot/lidarbot_ws/src/sensor_msgs/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/akshat/ProjectROS/catkin_ws/src/lidar_bot/lidarbot_ws/src/sensor_msgs/test/test_image_encodings.cpp -o CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.s

# Object files for target sensor_msgs_test_image_encodings
sensor_msgs_test_image_encodings_OBJECTS = \
"CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.o"

# External object files for target sensor_msgs_test_image_encodings
sensor_msgs_test_image_encodings_EXTERNAL_OBJECTS =

/home/akshat/ProjectROS/catkin_ws/devel/lib/sensor_msgs/sensor_msgs_test_image_encodings: lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/test_image_encodings.cpp.o
/home/akshat/ProjectROS/catkin_ws/devel/lib/sensor_msgs/sensor_msgs_test_image_encodings: lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/build.make
/home/akshat/ProjectROS/catkin_ws/devel/lib/sensor_msgs/sensor_msgs_test_image_encodings: gtest/lib/libgtest.so
/home/akshat/ProjectROS/catkin_ws/devel/lib/sensor_msgs/sensor_msgs_test_image_encodings: lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/akshat/ProjectROS/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/akshat/ProjectROS/catkin_ws/devel/lib/sensor_msgs/sensor_msgs_test_image_encodings"
	cd /home/akshat/ProjectROS/catkin_ws/build/lidar_bot/lidarbot_ws/src/sensor_msgs/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sensor_msgs_test_image_encodings.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/build: /home/akshat/ProjectROS/catkin_ws/devel/lib/sensor_msgs/sensor_msgs_test_image_encodings

.PHONY : lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/build

lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/clean:
	cd /home/akshat/ProjectROS/catkin_ws/build/lidar_bot/lidarbot_ws/src/sensor_msgs/test && $(CMAKE_COMMAND) -P CMakeFiles/sensor_msgs_test_image_encodings.dir/cmake_clean.cmake
.PHONY : lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/clean

lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/depend:
	cd /home/akshat/ProjectROS/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/akshat/ProjectROS/catkin_ws/src /home/akshat/ProjectROS/catkin_ws/src/lidar_bot/lidarbot_ws/src/sensor_msgs/test /home/akshat/ProjectROS/catkin_ws/build /home/akshat/ProjectROS/catkin_ws/build/lidar_bot/lidarbot_ws/src/sensor_msgs/test /home/akshat/ProjectROS/catkin_ws/build/lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lidar_bot/lidarbot_ws/src/sensor_msgs/test/CMakeFiles/sensor_msgs_test_image_encodings.dir/depend
