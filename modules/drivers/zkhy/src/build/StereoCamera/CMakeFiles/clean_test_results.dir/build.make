# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /apollo/modules/drivers/zkhy/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /apollo/modules/drivers/zkhy/src/build

# Utility rule file for clean_test_results.

# Include the progress variables for this target.
include StereoCamera/CMakeFiles/clean_test_results.dir/progress.make

StereoCamera/CMakeFiles/clean_test_results:
	cd /apollo/modules/drivers/zkhy/src/build/StereoCamera && /usr/bin/python /home/tmp/ros/share/catkin/cmake/test/remove_test_results.py /apollo/modules/drivers/zkhy/src/build/test_results

clean_test_results: StereoCamera/CMakeFiles/clean_test_results
clean_test_results: StereoCamera/CMakeFiles/clean_test_results.dir/build.make

.PHONY : clean_test_results

# Rule to build all files generated by this target.
StereoCamera/CMakeFiles/clean_test_results.dir/build: clean_test_results

.PHONY : StereoCamera/CMakeFiles/clean_test_results.dir/build

StereoCamera/CMakeFiles/clean_test_results.dir/clean:
	cd /apollo/modules/drivers/zkhy/src/build/StereoCamera && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results.dir/cmake_clean.cmake
.PHONY : StereoCamera/CMakeFiles/clean_test_results.dir/clean

StereoCamera/CMakeFiles/clean_test_results.dir/depend:
	cd /apollo/modules/drivers/zkhy/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /apollo/modules/drivers/zkhy/src /apollo/modules/drivers/zkhy/src/StereoCamera /apollo/modules/drivers/zkhy/src/build /apollo/modules/drivers/zkhy/src/build/StereoCamera /apollo/modules/drivers/zkhy/src/build/StereoCamera/CMakeFiles/clean_test_results.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : StereoCamera/CMakeFiles/clean_test_results.dir/depend

