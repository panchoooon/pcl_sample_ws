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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pancho/pcl_sample_ws

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pancho/pcl_sample_ws

# Include any dependencies generated for this target.
include CMakeFiles/InOutPra.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/InOutPra.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/InOutPra.dir/flags.make

CMakeFiles/InOutPra.dir/InOutPra.cpp.o: CMakeFiles/InOutPra.dir/flags.make
CMakeFiles/InOutPra.dir/InOutPra.cpp.o: InOutPra.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pancho/pcl_sample_ws/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/InOutPra.dir/InOutPra.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/InOutPra.dir/InOutPra.cpp.o -c /home/pancho/pcl_sample_ws/InOutPra.cpp

CMakeFiles/InOutPra.dir/InOutPra.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/InOutPra.dir/InOutPra.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pancho/pcl_sample_ws/InOutPra.cpp > CMakeFiles/InOutPra.dir/InOutPra.cpp.i

CMakeFiles/InOutPra.dir/InOutPra.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/InOutPra.dir/InOutPra.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pancho/pcl_sample_ws/InOutPra.cpp -o CMakeFiles/InOutPra.dir/InOutPra.cpp.s

CMakeFiles/InOutPra.dir/InOutPra.cpp.o.requires:
.PHONY : CMakeFiles/InOutPra.dir/InOutPra.cpp.o.requires

CMakeFiles/InOutPra.dir/InOutPra.cpp.o.provides: CMakeFiles/InOutPra.dir/InOutPra.cpp.o.requires
	$(MAKE) -f CMakeFiles/InOutPra.dir/build.make CMakeFiles/InOutPra.dir/InOutPra.cpp.o.provides.build
.PHONY : CMakeFiles/InOutPra.dir/InOutPra.cpp.o.provides

CMakeFiles/InOutPra.dir/InOutPra.cpp.o.provides.build: CMakeFiles/InOutPra.dir/InOutPra.cpp.o

# Object files for target InOutPra
InOutPra_OBJECTS = \
"CMakeFiles/InOutPra.dir/InOutPra.cpp.o"

# External object files for target InOutPra
InOutPra_EXTERNAL_OBJECTS =

InOutPra: CMakeFiles/InOutPra.dir/InOutPra.cpp.o
InOutPra: CMakeFiles/InOutPra.dir/build.make
InOutPra: CMakeFiles/InOutPra.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable InOutPra"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/InOutPra.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/InOutPra.dir/build: InOutPra
.PHONY : CMakeFiles/InOutPra.dir/build

CMakeFiles/InOutPra.dir/requires: CMakeFiles/InOutPra.dir/InOutPra.cpp.o.requires
.PHONY : CMakeFiles/InOutPra.dir/requires

CMakeFiles/InOutPra.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/InOutPra.dir/cmake_clean.cmake
.PHONY : CMakeFiles/InOutPra.dir/clean

CMakeFiles/InOutPra.dir/depend:
	cd /home/pancho/pcl_sample_ws && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pancho/pcl_sample_ws /home/pancho/pcl_sample_ws /home/pancho/pcl_sample_ws /home/pancho/pcl_sample_ws /home/pancho/pcl_sample_ws/CMakeFiles/InOutPra.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/InOutPra.dir/depend
