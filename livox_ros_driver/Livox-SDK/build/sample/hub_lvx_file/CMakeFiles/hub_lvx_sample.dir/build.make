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
CMAKE_SOURCE_DIR = /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build

# Include any dependencies generated for this target.
include sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/depend.make

# Include the progress variables for this target.
include sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/progress.make

# Include the compile flags for this target's objects.
include sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/flags.make

sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/main.cpp.o: sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/flags.make
sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/main.cpp.o: ../sample/hub_lvx_file/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/main.cpp.o"
	cd /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/sample/hub_lvx_file && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hub_lvx_sample.dir/main.cpp.o -c /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/sample/hub_lvx_file/main.cpp

sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hub_lvx_sample.dir/main.cpp.i"
	cd /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/sample/hub_lvx_file && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/sample/hub_lvx_file/main.cpp > CMakeFiles/hub_lvx_sample.dir/main.cpp.i

sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hub_lvx_sample.dir/main.cpp.s"
	cd /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/sample/hub_lvx_file && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/sample/hub_lvx_file/main.cpp -o CMakeFiles/hub_lvx_sample.dir/main.cpp.s

sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.o: sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/flags.make
sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.o: ../sample/hub_lvx_file/lvx_file.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.o"
	cd /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/sample/hub_lvx_file && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.o -c /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/sample/hub_lvx_file/lvx_file.cpp

sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.i"
	cd /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/sample/hub_lvx_file && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/sample/hub_lvx_file/lvx_file.cpp > CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.i

sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.s"
	cd /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/sample/hub_lvx_file && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/sample/hub_lvx_file/lvx_file.cpp -o CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.s

# Object files for target hub_lvx_sample
hub_lvx_sample_OBJECTS = \
"CMakeFiles/hub_lvx_sample.dir/main.cpp.o" \
"CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.o"

# External object files for target hub_lvx_sample
hub_lvx_sample_EXTERNAL_OBJECTS =

sample/hub_lvx_file/hub_lvx_sample: sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/main.cpp.o
sample/hub_lvx_file/hub_lvx_sample: sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/lvx_file.cpp.o
sample/hub_lvx_file/hub_lvx_sample: sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/build.make
sample/hub_lvx_file/hub_lvx_sample: sdk_core/liblivox_sdk_static.a
sample/hub_lvx_file/hub_lvx_sample: sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable hub_lvx_sample"
	cd /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/sample/hub_lvx_file && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hub_lvx_sample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/build: sample/hub_lvx_file/hub_lvx_sample

.PHONY : sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/build

sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/clean:
	cd /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/sample/hub_lvx_file && $(CMAKE_COMMAND) -P CMakeFiles/hub_lvx_sample.dir/cmake_clean.cmake
.PHONY : sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/clean

sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/depend:
	cd /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/sample/hub_lvx_file /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/sample/hub_lvx_file /home/hmcl/offroad_ws/src/livox_ros_driver/Livox-SDK/build/sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sample/hub_lvx_file/CMakeFiles/hub_lvx_sample.dir/depend

