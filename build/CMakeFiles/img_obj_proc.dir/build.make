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
CMAKE_SOURCE_DIR = /home/guest/usr/github/pcl_sample_ws

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/guest/usr/github/pcl_sample_ws/build

# Include any dependencies generated for this target.
include CMakeFiles/img_obj_proc.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/img_obj_proc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/img_obj_proc.dir/flags.make

CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o: CMakeFiles/img_obj_proc.dir/flags.make
CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o: ../img_obj_proc.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/guest/usr/github/pcl_sample_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o -c /home/guest/usr/github/pcl_sample_ws/img_obj_proc.cpp

CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/guest/usr/github/pcl_sample_ws/img_obj_proc.cpp > CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.i

CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/guest/usr/github/pcl_sample_ws/img_obj_proc.cpp -o CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.s

CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o.requires:
.PHONY : CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o.requires

CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o.provides: CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o.requires
	$(MAKE) -f CMakeFiles/img_obj_proc.dir/build.make CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o.provides.build
.PHONY : CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o.provides

CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o.provides.build: CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o

# Object files for target img_obj_proc
img_obj_proc_OBJECTS = \
"CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o"

# External object files for target img_obj_proc
img_obj_proc_EXTERNAL_OBJECTS =

img_obj_proc: CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o
img_obj_proc: CMakeFiles/img_obj_proc.dir/build.make
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_system.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_thread.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libpthread.so
img_obj_proc: /usr/lib/libpcl_common.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
img_obj_proc: /usr/lib/libpcl_kdtree.so
img_obj_proc: /usr/lib/libpcl_octree.so
img_obj_proc: /usr/lib/libpcl_search.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libqhull.so
img_obj_proc: /usr/lib/libpcl_surface.so
img_obj_proc: /usr/lib/libpcl_sample_consensus.so
img_obj_proc: /usr/lib/libOpenNI.so
img_obj_proc: /usr/lib/libOpenNI2.so
img_obj_proc: /usr/lib/libvtkCommon.so.5.8.0
img_obj_proc: /usr/lib/libvtkFiltering.so.5.8.0
img_obj_proc: /usr/lib/libvtkImaging.so.5.8.0
img_obj_proc: /usr/lib/libvtkGraphics.so.5.8.0
img_obj_proc: /usr/lib/libvtkGenericFiltering.so.5.8.0
img_obj_proc: /usr/lib/libvtkIO.so.5.8.0
img_obj_proc: /usr/lib/libvtkRendering.so.5.8.0
img_obj_proc: /usr/lib/libvtkVolumeRendering.so.5.8.0
img_obj_proc: /usr/lib/libvtkHybrid.so.5.8.0
img_obj_proc: /usr/lib/libvtkWidgets.so.5.8.0
img_obj_proc: /usr/lib/libvtkParallel.so.5.8.0
img_obj_proc: /usr/lib/libvtkInfovis.so.5.8.0
img_obj_proc: /usr/lib/libvtkGeovis.so.5.8.0
img_obj_proc: /usr/lib/libvtkViews.so.5.8.0
img_obj_proc: /usr/lib/libvtkCharts.so.5.8.0
img_obj_proc: /usr/lib/libpcl_io.so
img_obj_proc: /usr/lib/libpcl_filters.so
img_obj_proc: /usr/lib/libpcl_features.so
img_obj_proc: /usr/lib/libpcl_keypoints.so
img_obj_proc: /usr/lib/libpcl_registration.so
img_obj_proc: /usr/lib/libpcl_segmentation.so
img_obj_proc: /usr/lib/libpcl_recognition.so
img_obj_proc: /usr/lib/libpcl_visualization.so
img_obj_proc: /usr/lib/libpcl_people.so
img_obj_proc: /usr/lib/libpcl_outofcore.so
img_obj_proc: /usr/lib/libpcl_tracking.so
img_obj_proc: /usr/lib/libpcl_apps.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_system.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_thread.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libpthread.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libqhull.so
img_obj_proc: /usr/lib/libOpenNI.so
img_obj_proc: /usr/lib/libOpenNI2.so
img_obj_proc: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
img_obj_proc: /usr/lib/libvtkCommon.so.5.8.0
img_obj_proc: /usr/lib/libvtkFiltering.so.5.8.0
img_obj_proc: /usr/lib/libvtkImaging.so.5.8.0
img_obj_proc: /usr/lib/libvtkGraphics.so.5.8.0
img_obj_proc: /usr/lib/libvtkGenericFiltering.so.5.8.0
img_obj_proc: /usr/lib/libvtkIO.so.5.8.0
img_obj_proc: /usr/lib/libvtkRendering.so.5.8.0
img_obj_proc: /usr/lib/libvtkVolumeRendering.so.5.8.0
img_obj_proc: /usr/lib/libvtkHybrid.so.5.8.0
img_obj_proc: /usr/lib/libvtkWidgets.so.5.8.0
img_obj_proc: /usr/lib/libvtkParallel.so.5.8.0
img_obj_proc: /usr/lib/libvtkInfovis.so.5.8.0
img_obj_proc: /usr/lib/libvtkGeovis.so.5.8.0
img_obj_proc: /usr/lib/libvtkViews.so.5.8.0
img_obj_proc: /usr/lib/libvtkCharts.so.5.8.0
img_obj_proc: /usr/lib/libpcl_common.so
img_obj_proc: /usr/lib/libpcl_kdtree.so
img_obj_proc: /usr/lib/libpcl_octree.so
img_obj_proc: /usr/lib/libpcl_search.so
img_obj_proc: /usr/lib/libpcl_surface.so
img_obj_proc: /usr/lib/libpcl_sample_consensus.so
img_obj_proc: /usr/lib/libpcl_io.so
img_obj_proc: /usr/lib/libpcl_filters.so
img_obj_proc: /usr/lib/libpcl_features.so
img_obj_proc: /usr/lib/libpcl_keypoints.so
img_obj_proc: /usr/lib/libpcl_registration.so
img_obj_proc: /usr/lib/libpcl_segmentation.so
img_obj_proc: /usr/lib/libpcl_recognition.so
img_obj_proc: /usr/lib/libpcl_visualization.so
img_obj_proc: /usr/lib/libpcl_people.so
img_obj_proc: /usr/lib/libpcl_outofcore.so
img_obj_proc: /usr/lib/libpcl_tracking.so
img_obj_proc: /usr/lib/libpcl_apps.so
img_obj_proc: /usr/lib/libvtkViews.so.5.8.0
img_obj_proc: /usr/lib/libvtkInfovis.so.5.8.0
img_obj_proc: /usr/lib/libvtkWidgets.so.5.8.0
img_obj_proc: /usr/lib/libvtkVolumeRendering.so.5.8.0
img_obj_proc: /usr/lib/libvtkHybrid.so.5.8.0
img_obj_proc: /usr/lib/libvtkParallel.so.5.8.0
img_obj_proc: /usr/lib/libvtkRendering.so.5.8.0
img_obj_proc: /usr/lib/libvtkImaging.so.5.8.0
img_obj_proc: /usr/lib/libvtkGraphics.so.5.8.0
img_obj_proc: /usr/lib/libvtkIO.so.5.8.0
img_obj_proc: /usr/lib/libvtkFiltering.so.5.8.0
img_obj_proc: /usr/lib/libvtkCommon.so.5.8.0
img_obj_proc: /usr/lib/libvtksys.so.5.8.0
img_obj_proc: CMakeFiles/img_obj_proc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable img_obj_proc"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/img_obj_proc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/img_obj_proc.dir/build: img_obj_proc
.PHONY : CMakeFiles/img_obj_proc.dir/build

CMakeFiles/img_obj_proc.dir/requires: CMakeFiles/img_obj_proc.dir/img_obj_proc.cpp.o.requires
.PHONY : CMakeFiles/img_obj_proc.dir/requires

CMakeFiles/img_obj_proc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/img_obj_proc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/img_obj_proc.dir/clean

CMakeFiles/img_obj_proc.dir/depend:
	cd /home/guest/usr/github/pcl_sample_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guest/usr/github/pcl_sample_ws /home/guest/usr/github/pcl_sample_ws /home/guest/usr/github/pcl_sample_ws/build /home/guest/usr/github/pcl_sample_ws/build /home/guest/usr/github/pcl_sample_ws/build/CMakeFiles/img_obj_proc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/img_obj_proc.dir/depend

