# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/odroid/Desktop/work/src/motor_driver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/odroid/Desktop/work/src/motor_driver/build

# Include any dependencies generated for this target.
include CMakeFiles/motor_driver.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/motor_driver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/motor_driver.dir/flags.make

CMakeFiles/motor_driver.dir/src/motordriver.cpp.o: CMakeFiles/motor_driver.dir/flags.make
CMakeFiles/motor_driver.dir/src/motordriver.cpp.o: ../src/motordriver.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/odroid/Desktop/work/src/motor_driver/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/motor_driver.dir/src/motordriver.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/motor_driver.dir/src/motordriver.cpp.o -c /home/odroid/Desktop/work/src/motor_driver/src/motordriver.cpp

CMakeFiles/motor_driver.dir/src/motordriver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motor_driver.dir/src/motordriver.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/odroid/Desktop/work/src/motor_driver/src/motordriver.cpp > CMakeFiles/motor_driver.dir/src/motordriver.cpp.i

CMakeFiles/motor_driver.dir/src/motordriver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motor_driver.dir/src/motordriver.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/odroid/Desktop/work/src/motor_driver/src/motordriver.cpp -o CMakeFiles/motor_driver.dir/src/motordriver.cpp.s

CMakeFiles/motor_driver.dir/src/motordriver.cpp.o.requires:
.PHONY : CMakeFiles/motor_driver.dir/src/motordriver.cpp.o.requires

CMakeFiles/motor_driver.dir/src/motordriver.cpp.o.provides: CMakeFiles/motor_driver.dir/src/motordriver.cpp.o.requires
	$(MAKE) -f CMakeFiles/motor_driver.dir/build.make CMakeFiles/motor_driver.dir/src/motordriver.cpp.o.provides.build
.PHONY : CMakeFiles/motor_driver.dir/src/motordriver.cpp.o.provides

CMakeFiles/motor_driver.dir/src/motordriver.cpp.o.provides.build: CMakeFiles/motor_driver.dir/src/motordriver.cpp.o

# Object files for target motor_driver
motor_driver_OBJECTS = \
"CMakeFiles/motor_driver.dir/src/motordriver.cpp.o"

# External object files for target motor_driver
motor_driver_EXTERNAL_OBJECTS =

motor_driver: CMakeFiles/motor_driver.dir/src/motordriver.cpp.o
motor_driver: CMakeFiles/motor_driver.dir/build.make
motor_driver: CMakeFiles/motor_driver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable motor_driver"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motor_driver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/motor_driver.dir/build: motor_driver
.PHONY : CMakeFiles/motor_driver.dir/build

CMakeFiles/motor_driver.dir/requires: CMakeFiles/motor_driver.dir/src/motordriver.cpp.o.requires
.PHONY : CMakeFiles/motor_driver.dir/requires

CMakeFiles/motor_driver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/motor_driver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/motor_driver.dir/clean

CMakeFiles/motor_driver.dir/depend:
	cd /home/odroid/Desktop/work/src/motor_driver/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/odroid/Desktop/work/src/motor_driver /home/odroid/Desktop/work/src/motor_driver /home/odroid/Desktop/work/src/motor_driver/build /home/odroid/Desktop/work/src/motor_driver/build /home/odroid/Desktop/work/src/motor_driver/build/CMakeFiles/motor_driver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/motor_driver.dir/depend
