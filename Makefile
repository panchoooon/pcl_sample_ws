# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

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

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running interactive CMake command-line interface..."
	/usr/bin/cmake -i .
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pancho/pcl_sample_ws/CMakeFiles /home/pancho/pcl_sample_ws/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pancho/pcl_sample_ws/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named cloud_viewer

# Build rule for target.
cloud_viewer: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 cloud_viewer
.PHONY : cloud_viewer

# fast build rule for target.
cloud_viewer/fast:
	$(MAKE) -f CMakeFiles/cloud_viewer.dir/build.make CMakeFiles/cloud_viewer.dir/build
.PHONY : cloud_viewer/fast

#=============================================================================
# Target rules for targets named create_window

# Build rule for target.
create_window: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 create_window
.PHONY : create_window

# fast build rule for target.
create_window/fast:
	$(MAKE) -f CMakeFiles/create_window.dir/build.make CMakeFiles/create_window.dir/build
.PHONY : create_window/fast

#=============================================================================
# Target rules for targets named openni_viewer_simple

# Build rule for target.
openni_viewer_simple: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 openni_viewer_simple
.PHONY : openni_viewer_simple

# fast build rule for target.
openni_viewer_simple/fast:
	$(MAKE) -f CMakeFiles/openni_viewer_simple.dir/build.make CMakeFiles/openni_viewer_simple.dir/build
.PHONY : openni_viewer_simple/fast

#=============================================================================
# Target rules for targets named pcd_read

# Build rule for target.
pcd_read: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 pcd_read
.PHONY : pcd_read

# fast build rule for target.
pcd_read/fast:
	$(MAKE) -f CMakeFiles/pcd_read.dir/build.make CMakeFiles/pcd_read.dir/build
.PHONY : pcd_read/fast

cloud_viewer.o: cloud_viewer.cpp.o
.PHONY : cloud_viewer.o

# target to build an object file
cloud_viewer.cpp.o:
	$(MAKE) -f CMakeFiles/cloud_viewer.dir/build.make CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o
.PHONY : cloud_viewer.cpp.o

cloud_viewer.i: cloud_viewer.cpp.i
.PHONY : cloud_viewer.i

# target to preprocess a source file
cloud_viewer.cpp.i:
	$(MAKE) -f CMakeFiles/cloud_viewer.dir/build.make CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.i
.PHONY : cloud_viewer.cpp.i

cloud_viewer.s: cloud_viewer.cpp.s
.PHONY : cloud_viewer.s

# target to generate assembly for a file
cloud_viewer.cpp.s:
	$(MAKE) -f CMakeFiles/cloud_viewer.dir/build.make CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.s
.PHONY : cloud_viewer.cpp.s

create_window.o: create_window.c.o
.PHONY : create_window.o

# target to build an object file
create_window.c.o:
	$(MAKE) -f CMakeFiles/create_window.dir/build.make CMakeFiles/create_window.dir/create_window.c.o
.PHONY : create_window.c.o

create_window.i: create_window.c.i
.PHONY : create_window.i

# target to preprocess a source file
create_window.c.i:
	$(MAKE) -f CMakeFiles/create_window.dir/build.make CMakeFiles/create_window.dir/create_window.c.i
.PHONY : create_window.c.i

create_window.s: create_window.c.s
.PHONY : create_window.s

# target to generate assembly for a file
create_window.c.s:
	$(MAKE) -f CMakeFiles/create_window.dir/build.make CMakeFiles/create_window.dir/create_window.c.s
.PHONY : create_window.c.s

openni_viewer_simple.o: openni_viewer_simple.cpp.o
.PHONY : openni_viewer_simple.o

# target to build an object file
openni_viewer_simple.cpp.o:
	$(MAKE) -f CMakeFiles/openni_viewer_simple.dir/build.make CMakeFiles/openni_viewer_simple.dir/openni_viewer_simple.cpp.o
.PHONY : openni_viewer_simple.cpp.o

openni_viewer_simple.i: openni_viewer_simple.cpp.i
.PHONY : openni_viewer_simple.i

# target to preprocess a source file
openni_viewer_simple.cpp.i:
	$(MAKE) -f CMakeFiles/openni_viewer_simple.dir/build.make CMakeFiles/openni_viewer_simple.dir/openni_viewer_simple.cpp.i
.PHONY : openni_viewer_simple.cpp.i

openni_viewer_simple.s: openni_viewer_simple.cpp.s
.PHONY : openni_viewer_simple.s

# target to generate assembly for a file
openni_viewer_simple.cpp.s:
	$(MAKE) -f CMakeFiles/openni_viewer_simple.dir/build.make CMakeFiles/openni_viewer_simple.dir/openni_viewer_simple.cpp.s
.PHONY : openni_viewer_simple.cpp.s

pcd_read.o: pcd_read.cpp.o
.PHONY : pcd_read.o

# target to build an object file
pcd_read.cpp.o:
	$(MAKE) -f CMakeFiles/pcd_read.dir/build.make CMakeFiles/pcd_read.dir/pcd_read.cpp.o
.PHONY : pcd_read.cpp.o

pcd_read.i: pcd_read.cpp.i
.PHONY : pcd_read.i

# target to preprocess a source file
pcd_read.cpp.i:
	$(MAKE) -f CMakeFiles/pcd_read.dir/build.make CMakeFiles/pcd_read.dir/pcd_read.cpp.i
.PHONY : pcd_read.cpp.i

pcd_read.s: pcd_read.cpp.s
.PHONY : pcd_read.s

# target to generate assembly for a file
pcd_read.cpp.s:
	$(MAKE) -f CMakeFiles/pcd_read.dir/build.make CMakeFiles/pcd_read.dir/pcd_read.cpp.s
.PHONY : pcd_read.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... cloud_viewer"
	@echo "... create_window"
	@echo "... edit_cache"
	@echo "... openni_viewer_simple"
	@echo "... pcd_read"
	@echo "... rebuild_cache"
	@echo "... cloud_viewer.o"
	@echo "... cloud_viewer.i"
	@echo "... cloud_viewer.s"
	@echo "... create_window.o"
	@echo "... create_window.i"
	@echo "... create_window.s"
	@echo "... openni_viewer_simple.o"
	@echo "... openni_viewer_simple.i"
	@echo "... openni_viewer_simple.s"
	@echo "... pcd_read.o"
	@echo "... pcd_read.i"
	@echo "... pcd_read.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
