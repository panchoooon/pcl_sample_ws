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
CMAKE_BINARY_DIR = /home/pancho/pcl_sample_ws/build

# Include any dependencies generated for this target.
include CMakeFiles/colorbase_segmentation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/colorbase_segmentation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/colorbase_segmentation.dir/flags.make

CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o: CMakeFiles/colorbase_segmentation.dir/flags.make
CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o: ../colorbase_segmentation.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pancho/pcl_sample_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o -c /home/pancho/pcl_sample_ws/colorbase_segmentation.cpp

CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pancho/pcl_sample_ws/colorbase_segmentation.cpp > CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.i

CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pancho/pcl_sample_ws/colorbase_segmentation.cpp -o CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.s

CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o.requires:
.PHONY : CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o.requires

CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o.provides: CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/colorbase_segmentation.dir/build.make CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o.provides.build
.PHONY : CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o.provides

CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o.provides.build: CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o

# Object files for target colorbase_segmentation
colorbase_segmentation_OBJECTS = \
"CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o"

# External object files for target colorbase_segmentation
colorbase_segmentation_EXTERNAL_OBJECTS =

colorbase_segmentation: CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o
colorbase_segmentation: CMakeFiles/colorbase_segmentation.dir/build.make
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_system.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
colorbase_segmentation: /usr/lib/libpcl_common.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
colorbase_segmentation: /usr/lib/libpcl_kdtree.so
colorbase_segmentation: /usr/lib/libpcl_octree.so
colorbase_segmentation: /usr/lib/libpcl_search.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libqhull.so
colorbase_segmentation: /usr/lib/libpcl_surface.so
colorbase_segmentation: /usr/lib/libpcl_sample_consensus.so
colorbase_segmentation: /usr/lib/libOpenNI.so
colorbase_segmentation: /usr/lib/libOpenNI2.so
colorbase_segmentation: /usr/lib/libvtkCommon.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkFiltering.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkImaging.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkGraphics.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkGenericFiltering.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkIO.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkRendering.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkVolumeRendering.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkHybrid.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkWidgets.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkParallel.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkInfovis.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkGeovis.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkViews.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkCharts.so.5.8.0
colorbase_segmentation: /usr/lib/libpcl_io.so
colorbase_segmentation: /usr/lib/libpcl_filters.so
colorbase_segmentation: /usr/lib/libpcl_features.so
colorbase_segmentation: /usr/lib/libpcl_keypoints.so
colorbase_segmentation: /usr/lib/libpcl_registration.so
colorbase_segmentation: /usr/lib/libpcl_segmentation.so
colorbase_segmentation: /usr/lib/libpcl_recognition.so
colorbase_segmentation: /usr/lib/libpcl_visualization.so
colorbase_segmentation: /usr/lib/libpcl_people.so
colorbase_segmentation: /usr/lib/libpcl_outofcore.so
colorbase_segmentation: /usr/lib/libpcl_tracking.so
colorbase_segmentation: /usr/lib/libpcl_apps.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_system.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libqhull.so
colorbase_segmentation: /usr/lib/libOpenNI.so
colorbase_segmentation: /usr/lib/libOpenNI2.so
colorbase_segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
colorbase_segmentation: /usr/lib/libvtkCommon.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkFiltering.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkImaging.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkGraphics.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkGenericFiltering.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkIO.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkRendering.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkVolumeRendering.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkHybrid.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkWidgets.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkParallel.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkInfovis.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkGeovis.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkViews.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkCharts.so.5.8.0
colorbase_segmentation: /usr/lib/libpcl_common.so
colorbase_segmentation: /usr/lib/libpcl_kdtree.so
colorbase_segmentation: /usr/lib/libpcl_octree.so
colorbase_segmentation: /usr/lib/libpcl_search.so
colorbase_segmentation: /usr/lib/libpcl_surface.so
colorbase_segmentation: /usr/lib/libpcl_sample_consensus.so
colorbase_segmentation: /usr/lib/libpcl_io.so
colorbase_segmentation: /usr/lib/libpcl_filters.so
colorbase_segmentation: /usr/lib/libpcl_features.so
colorbase_segmentation: /usr/lib/libpcl_keypoints.so
colorbase_segmentation: /usr/lib/libpcl_registration.so
colorbase_segmentation: /usr/lib/libpcl_segmentation.so
colorbase_segmentation: /usr/lib/libpcl_recognition.so
colorbase_segmentation: /usr/lib/libpcl_visualization.so
colorbase_segmentation: /usr/lib/libpcl_people.so
colorbase_segmentation: /usr/lib/libpcl_outofcore.so
colorbase_segmentation: /usr/lib/libpcl_tracking.so
colorbase_segmentation: /usr/lib/libpcl_apps.so
colorbase_segmentation: /usr/lib/libvtkViews.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkInfovis.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkWidgets.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkVolumeRendering.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkHybrid.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkParallel.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkRendering.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkImaging.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkGraphics.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkIO.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkFiltering.so.5.8.0
colorbase_segmentation: /usr/lib/libvtkCommon.so.5.8.0
colorbase_segmentation: /usr/lib/libvtksys.so.5.8.0
colorbase_segmentation: CMakeFiles/colorbase_segmentation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable colorbase_segmentation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/colorbase_segmentation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/colorbase_segmentation.dir/build: colorbase_segmentation
.PHONY : CMakeFiles/colorbase_segmentation.dir/build

CMakeFiles/colorbase_segmentation.dir/requires: CMakeFiles/colorbase_segmentation.dir/colorbase_segmentation.cpp.o.requires
.PHONY : CMakeFiles/colorbase_segmentation.dir/requires

CMakeFiles/colorbase_segmentation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/colorbase_segmentation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/colorbase_segmentation.dir/clean

CMakeFiles/colorbase_segmentation.dir/depend:
	cd /home/pancho/pcl_sample_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pancho/pcl_sample_ws /home/pancho/pcl_sample_ws /home/pancho/pcl_sample_ws/build /home/pancho/pcl_sample_ws/build /home/pancho/pcl_sample_ws/build/CMakeFiles/colorbase_segmentation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/colorbase_segmentation.dir/depend

