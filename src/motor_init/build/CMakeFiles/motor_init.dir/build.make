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
CMAKE_SOURCE_DIR = /home/odroid/Desktop/work/src/motor_init

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/odroid/Desktop/work/src/motor_init/build

# Include any dependencies generated for this target.
include CMakeFiles/motor_init.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/motor_init.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/motor_init.dir/flags.make

CMakeFiles/motor_init.dir/src/motor_init.cpp.o: CMakeFiles/motor_init.dir/flags.make
CMakeFiles/motor_init.dir/src/motor_init.cpp.o: ../src/motor_init.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/odroid/Desktop/work/src/motor_init/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/motor_init.dir/src/motor_init.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/motor_init.dir/src/motor_init.cpp.o -c /home/odroid/Desktop/work/src/motor_init/src/motor_init.cpp

CMakeFiles/motor_init.dir/src/motor_init.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motor_init.dir/src/motor_init.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/odroid/Desktop/work/src/motor_init/src/motor_init.cpp > CMakeFiles/motor_init.dir/src/motor_init.cpp.i

CMakeFiles/motor_init.dir/src/motor_init.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motor_init.dir/src/motor_init.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/odroid/Desktop/work/src/motor_init/src/motor_init.cpp -o CMakeFiles/motor_init.dir/src/motor_init.cpp.s

CMakeFiles/motor_init.dir/src/motor_init.cpp.o.requires:
.PHONY : CMakeFiles/motor_init.dir/src/motor_init.cpp.o.requires

CMakeFiles/motor_init.dir/src/motor_init.cpp.o.provides: CMakeFiles/motor_init.dir/src/motor_init.cpp.o.requires
	$(MAKE) -f CMakeFiles/motor_init.dir/build.make CMakeFiles/motor_init.dir/src/motor_init.cpp.o.provides.build
.PHONY : CMakeFiles/motor_init.dir/src/motor_init.cpp.o.provides

CMakeFiles/motor_init.dir/src/motor_init.cpp.o.provides.build: CMakeFiles/motor_init.dir/src/motor_init.cpp.o

# Object files for target motor_init
motor_init_OBJECTS = \
"CMakeFiles/motor_init.dir/src/motor_init.cpp.o"

# External object files for target motor_init
motor_init_EXTERNAL_OBJECTS =

motor_init: CMakeFiles/motor_init.dir/src/motor_init.cpp.o
motor_init: CMakeFiles/motor_init.dir/build.make
motor_init: CMakeFiles/motor_init.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable motor_init"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motor_init.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/motor_init.dir/build: motor_init
.PHONY : CMakeFiles/motor_init.dir/build

CMakeFiles/motor_init.dir/requires: CMakeFiles/motor_init.dir/src/motor_init.cpp.o.requires
.PHONY : CMakeFiles/motor_init.dir/requires

CMakeFiles/motor_init.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/motor_init.dir/cmake_clean.cmake
.PHONY : CMakeFiles/motor_init.dir/clean

CMakeFiles/motor_init.dir/depend:
	cd /home/odroid/Desktop/work/src/motor_init/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/odroid/Desktop/work/src/motor_init /home/odroid/Desktop/work/src/motor_init /home/odroid/Desktop/work/src/motor_init/build /home/odroid/Desktop/work/src/motor_init/build /home/odroid/Desktop/work/src/motor_init/build/CMakeFiles/motor_init.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/motor_init.dir/depend

