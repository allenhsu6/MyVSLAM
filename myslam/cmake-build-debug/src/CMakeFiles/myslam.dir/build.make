# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/xu/下载/clion-2019.3.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/xu/下载/clion-2019.3.1/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xu/MyVSLAM/myslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xu/MyVSLAM/myslam/cmake-build-debug

# Include any dependencies generated for this target.
include src/CMakeFiles/myslam.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/myslam.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/myslam.dir/flags.make

src/CMakeFiles/myslam.dir/Camera.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/Camera.cpp.o: ../src/Camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xu/MyVSLAM/myslam/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/myslam.dir/Camera.cpp.o"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/Camera.cpp.o -c /home/xu/MyVSLAM/myslam/src/Camera.cpp

src/CMakeFiles/myslam.dir/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/Camera.cpp.i"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xu/MyVSLAM/myslam/src/Camera.cpp > CMakeFiles/myslam.dir/Camera.cpp.i

src/CMakeFiles/myslam.dir/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/Camera.cpp.s"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xu/MyVSLAM/myslam/src/Camera.cpp -o CMakeFiles/myslam.dir/Camera.cpp.s

src/CMakeFiles/myslam.dir/Config.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/Config.cpp.o: ../src/Config.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xu/MyVSLAM/myslam/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/myslam.dir/Config.cpp.o"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/Config.cpp.o -c /home/xu/MyVSLAM/myslam/src/Config.cpp

src/CMakeFiles/myslam.dir/Config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/Config.cpp.i"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xu/MyVSLAM/myslam/src/Config.cpp > CMakeFiles/myslam.dir/Config.cpp.i

src/CMakeFiles/myslam.dir/Config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/Config.cpp.s"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xu/MyVSLAM/myslam/src/Config.cpp -o CMakeFiles/myslam.dir/Config.cpp.s

src/CMakeFiles/myslam.dir/Map.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/Map.cpp.o: ../src/Map.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xu/MyVSLAM/myslam/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/myslam.dir/Map.cpp.o"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/Map.cpp.o -c /home/xu/MyVSLAM/myslam/src/Map.cpp

src/CMakeFiles/myslam.dir/Map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/Map.cpp.i"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xu/MyVSLAM/myslam/src/Map.cpp > CMakeFiles/myslam.dir/Map.cpp.i

src/CMakeFiles/myslam.dir/Map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/Map.cpp.s"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xu/MyVSLAM/myslam/src/Map.cpp -o CMakeFiles/myslam.dir/Map.cpp.s

src/CMakeFiles/myslam.dir/MapPoint.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/MapPoint.cpp.o: ../src/MapPoint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xu/MyVSLAM/myslam/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/myslam.dir/MapPoint.cpp.o"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/MapPoint.cpp.o -c /home/xu/MyVSLAM/myslam/src/MapPoint.cpp

src/CMakeFiles/myslam.dir/MapPoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/MapPoint.cpp.i"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xu/MyVSLAM/myslam/src/MapPoint.cpp > CMakeFiles/myslam.dir/MapPoint.cpp.i

src/CMakeFiles/myslam.dir/MapPoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/MapPoint.cpp.s"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xu/MyVSLAM/myslam/src/MapPoint.cpp -o CMakeFiles/myslam.dir/MapPoint.cpp.s

src/CMakeFiles/myslam.dir/Frame.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/Frame.cpp.o: ../src/Frame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xu/MyVSLAM/myslam/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/myslam.dir/Frame.cpp.o"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/Frame.cpp.o -c /home/xu/MyVSLAM/myslam/src/Frame.cpp

src/CMakeFiles/myslam.dir/Frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/Frame.cpp.i"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xu/MyVSLAM/myslam/src/Frame.cpp > CMakeFiles/myslam.dir/Frame.cpp.i

src/CMakeFiles/myslam.dir/Frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/Frame.cpp.s"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xu/MyVSLAM/myslam/src/Frame.cpp -o CMakeFiles/myslam.dir/Frame.cpp.s

src/CMakeFiles/myslam.dir/visual_odometry.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/visual_odometry.cpp.o: ../src/visual_odometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xu/MyVSLAM/myslam/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/myslam.dir/visual_odometry.cpp.o"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/visual_odometry.cpp.o -c /home/xu/MyVSLAM/myslam/src/visual_odometry.cpp

src/CMakeFiles/myslam.dir/visual_odometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/visual_odometry.cpp.i"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xu/MyVSLAM/myslam/src/visual_odometry.cpp > CMakeFiles/myslam.dir/visual_odometry.cpp.i

src/CMakeFiles/myslam.dir/visual_odometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/visual_odometry.cpp.s"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xu/MyVSLAM/myslam/src/visual_odometry.cpp -o CMakeFiles/myslam.dir/visual_odometry.cpp.s

# Object files for target myslam
myslam_OBJECTS = \
"CMakeFiles/myslam.dir/Camera.cpp.o" \
"CMakeFiles/myslam.dir/Config.cpp.o" \
"CMakeFiles/myslam.dir/Map.cpp.o" \
"CMakeFiles/myslam.dir/MapPoint.cpp.o" \
"CMakeFiles/myslam.dir/Frame.cpp.o" \
"CMakeFiles/myslam.dir/visual_odometry.cpp.o"

# External object files for target myslam
myslam_EXTERNAL_OBJECTS =

../lib/libmyslam.so: src/CMakeFiles/myslam.dir/Camera.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/Config.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/Map.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/MapPoint.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/Frame.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/visual_odometry.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/build.make
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
../lib/libmyslam.so: /home/xu/slambook/3rdparty/Sophus/build/libSophus.so
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xu/MyVSLAM/myslam/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared library ../../lib/libmyslam.so"
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myslam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/myslam.dir/build: ../lib/libmyslam.so

.PHONY : src/CMakeFiles/myslam.dir/build

src/CMakeFiles/myslam.dir/clean:
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug/src && $(CMAKE_COMMAND) -P CMakeFiles/myslam.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/myslam.dir/clean

src/CMakeFiles/myslam.dir/depend:
	cd /home/xu/MyVSLAM/myslam/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xu/MyVSLAM/myslam /home/xu/MyVSLAM/myslam/src /home/xu/MyVSLAM/myslam/cmake-build-debug /home/xu/MyVSLAM/myslam/cmake-build-debug/src /home/xu/MyVSLAM/myslam/cmake-build-debug/src/CMakeFiles/myslam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/myslam.dir/depend

