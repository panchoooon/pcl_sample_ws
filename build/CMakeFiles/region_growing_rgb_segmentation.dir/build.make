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
include CMakeFiles/region_growing_rgb_segmentation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/region_growing_rgb_segmentation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/region_growing_rgb_segmentation.dir/flags.make

CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o: CMakeFiles/region_growing_rgb_segmentation.dir/flags.make
CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o: ../region_growing_rgb_segmentation.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pancho/pcl_sample_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o -c /home/pancho/pcl_sample_ws/region_growing_rgb_segmentation.cpp

CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pancho/pcl_sample_ws/region_growing_rgb_segmentation.cpp > CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.i

CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pancho/pcl_sample_ws/region_growing_rgb_segmentation.cpp -o CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.s

CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o.requires:
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o.requires

CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o.provides: CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/region_growing_rgb_segmentation.dir/build.make CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o.provides.build
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o.provides

CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o.provides.build: CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o

# Object files for target region_growing_rgb_segmentation
region_growing_rgb_segmentation_OBJECTS = \
"CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o"

# External object files for target region_growing_rgb_segmentation
region_growing_rgb_segmentation_EXTERNAL_OBJECTS =

region_growing_rgb_segmentation: CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o
region_growing_rgb_segmentation: CMakeFiles/region_growing_rgb_segmentation.dir/build.make
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_system.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
region_growing_rgb_segmentation: /usr/lib/libpcl_common.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
region_growing_rgb_segmentation: /usr/lib/libpcl_kdtree.so
region_growing_rgb_segmentation: /usr/lib/libpcl_octree.so
region_growing_rgb_segmentation: /usr/lib/libpcl_search.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libqhull.so
region_growing_rgb_segmentation: /usr/lib/libpcl_surface.so
region_growing_rgb_segmentation: /usr/lib/libpcl_sample_consensus.so
region_growing_rgb_segmentation: /usr/lib/libOpenNI.so
region_growing_rgb_segmentation: /usr/lib/libOpenNI2.so
region_growing_rgb_segmentation: /usr/lib/libvtkCommon.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkFiltering.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkImaging.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkGraphics.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkGenericFiltering.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkIO.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkRendering.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkVolumeRendering.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkHybrid.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkWidgets.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkParallel.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkInfovis.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkGeovis.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkViews.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkCharts.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libpcl_io.so
region_growing_rgb_segmentation: /usr/lib/libpcl_filters.so
region_growing_rgb_segmentation: /usr/lib/libpcl_features.so
region_growing_rgb_segmentation: /usr/lib/libpcl_keypoints.so
region_growing_rgb_segmentation: /usr/lib/libpcl_registration.so
region_growing_rgb_segmentation: /usr/lib/libpcl_segmentation.so
region_growing_rgb_segmentation: /usr/lib/libpcl_recognition.so
region_growing_rgb_segmentation: /usr/lib/libpcl_visualization.so
region_growing_rgb_segmentation: /usr/lib/libpcl_people.so
region_growing_rgb_segmentation: /usr/lib/libpcl_outofcore.so
region_growing_rgb_segmentation: /usr/lib/libpcl_tracking.so
region_growing_rgb_segmentation: /usr/lib/libpcl_apps.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_system.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libqhull.so
region_growing_rgb_segmentation: /usr/lib/libOpenNI.so
region_growing_rgb_segmentation: /usr/lib/libOpenNI2.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
region_growing_rgb_segmentation: /usr/lib/libvtkCommon.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkFiltering.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkImaging.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkGraphics.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkGenericFiltering.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkIO.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkRendering.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkVolumeRendering.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkHybrid.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkWidgets.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkParallel.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkInfovis.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkGeovis.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkViews.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkCharts.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libpcl_common.so
region_growing_rgb_segmentation: /usr/lib/libpcl_kdtree.so
region_growing_rgb_segmentation: /usr/lib/libpcl_octree.so
region_growing_rgb_segmentation: /usr/lib/libpcl_search.so
region_growing_rgb_segmentation: /usr/lib/libpcl_surface.so
region_growing_rgb_segmentation: /usr/lib/libpcl_sample_consensus.so
region_growing_rgb_segmentation: /usr/lib/libpcl_io.so
region_growing_rgb_segmentation: /usr/lib/libpcl_filters.so
region_growing_rgb_segmentation: /usr/lib/libpcl_features.so
region_growing_rgb_segmentation: /usr/lib/libpcl_keypoints.so
region_growing_rgb_segmentation: /usr/lib/libpcl_registration.so
region_growing_rgb_segmentation: /usr/lib/libpcl_segmentation.so
region_growing_rgb_segmentation: /usr/lib/libpcl_recognition.so
region_growing_rgb_segmentation: /usr/lib/libpcl_visualization.so
region_growing_rgb_segmentation: /usr/lib/libpcl_people.so
region_growing_rgb_segmentation: /usr/lib/libpcl_outofcore.so
region_growing_rgb_segmentation: /usr/lib/libpcl_tracking.so
region_growing_rgb_segmentation: /usr/lib/libpcl_apps.so
region_growing_rgb_segmentation: /usr/lib/libvtkViews.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkInfovis.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkWidgets.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkVolumeRendering.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkHybrid.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkParallel.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkRendering.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkImaging.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkGraphics.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkIO.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkFiltering.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtkCommon.so.5.8.0
region_growing_rgb_segmentation: /usr/lib/libvtksys.so.5.8.0
region_growing_rgb_segmentation: CMakeFiles/region_growing_rgb_segmentation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable region_growing_rgb_segmentation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/region_growing_rgb_segmentation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/region_growing_rgb_segmentation.dir/build: region_growing_rgb_segmentation
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/build

CMakeFiles/region_growing_rgb_segmentation.dir/requires: CMakeFiles/region_growing_rgb_segmentation.dir/region_growing_rgb_segmentation.cpp.o.requires
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/requires

CMakeFiles/region_growing_rgb_segmentation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/region_growing_rgb_segmentation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/clean

CMakeFiles/region_growing_rgb_segmentation.dir/depend:
	cd /home/pancho/pcl_sample_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pancho/pcl_sample_ws /home/pancho/pcl_sample_ws /home/pancho/pcl_sample_ws/build /home/pancho/pcl_sample_ws/build /home/pancho/pcl_sample_ws/build/CMakeFiles/region_growing_rgb_segmentation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/depend

