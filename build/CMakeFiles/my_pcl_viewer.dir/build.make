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
include CMakeFiles/my_pcl_viewer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/my_pcl_viewer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my_pcl_viewer.dir/flags.make

CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o: CMakeFiles/my_pcl_viewer.dir/flags.make
CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o: ../my_pcl_viewer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pancho/pcl_sample_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o -c /home/pancho/pcl_sample_ws/my_pcl_viewer.cpp

CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pancho/pcl_sample_ws/my_pcl_viewer.cpp > CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.i

CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pancho/pcl_sample_ws/my_pcl_viewer.cpp -o CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.s

CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o.requires:
.PHONY : CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o.requires

CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o.provides: CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/my_pcl_viewer.dir/build.make CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o.provides.build
.PHONY : CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o.provides

CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o.provides.build: CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o

# Object files for target my_pcl_viewer
my_pcl_viewer_OBJECTS = \
"CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o"

# External object files for target my_pcl_viewer
my_pcl_viewer_EXTERNAL_OBJECTS =

my_pcl_viewer: CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o
my_pcl_viewer: CMakeFiles/my_pcl_viewer.dir/build.make
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_system.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libpthread.so
my_pcl_viewer: /usr/lib/libpcl_common.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
my_pcl_viewer: /usr/lib/libpcl_kdtree.so
my_pcl_viewer: /usr/lib/libpcl_octree.so
my_pcl_viewer: /usr/lib/libpcl_search.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libqhull.so
my_pcl_viewer: /usr/lib/libpcl_surface.so
my_pcl_viewer: /usr/lib/libpcl_sample_consensus.so
my_pcl_viewer: /usr/lib/libOpenNI.so
my_pcl_viewer: /usr/lib/libOpenNI2.so
my_pcl_viewer: /usr/lib/libvtkCommon.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkFiltering.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkImaging.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkGraphics.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkGenericFiltering.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkIO.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkRendering.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkVolumeRendering.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkHybrid.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkWidgets.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkParallel.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkInfovis.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkGeovis.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkViews.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkCharts.so.5.8.0
my_pcl_viewer: /usr/lib/libpcl_io.so
my_pcl_viewer: /usr/lib/libpcl_filters.so
my_pcl_viewer: /usr/lib/libpcl_features.so
my_pcl_viewer: /usr/lib/libpcl_keypoints.so
my_pcl_viewer: /usr/lib/libpcl_registration.so
my_pcl_viewer: /usr/lib/libpcl_segmentation.so
my_pcl_viewer: /usr/lib/libpcl_recognition.so
my_pcl_viewer: /usr/lib/libpcl_visualization.so
my_pcl_viewer: /usr/lib/libpcl_people.so
my_pcl_viewer: /usr/lib/libpcl_outofcore.so
my_pcl_viewer: /usr/lib/libpcl_tracking.so
my_pcl_viewer: /usr/lib/libpcl_apps.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_system.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libpthread.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libqhull.so
my_pcl_viewer: /usr/lib/libOpenNI.so
my_pcl_viewer: /usr/lib/libOpenNI2.so
my_pcl_viewer: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
my_pcl_viewer: /usr/lib/libvtkCommon.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkFiltering.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkImaging.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkGraphics.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkGenericFiltering.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkIO.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkRendering.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkVolumeRendering.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkHybrid.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkWidgets.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkParallel.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkInfovis.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkGeovis.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkViews.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkCharts.so.5.8.0
my_pcl_viewer: /usr/lib/libpcl_common.so
my_pcl_viewer: /usr/lib/libpcl_kdtree.so
my_pcl_viewer: /usr/lib/libpcl_octree.so
my_pcl_viewer: /usr/lib/libpcl_search.so
my_pcl_viewer: /usr/lib/libpcl_surface.so
my_pcl_viewer: /usr/lib/libpcl_sample_consensus.so
my_pcl_viewer: /usr/lib/libpcl_io.so
my_pcl_viewer: /usr/lib/libpcl_filters.so
my_pcl_viewer: /usr/lib/libpcl_features.so
my_pcl_viewer: /usr/lib/libpcl_keypoints.so
my_pcl_viewer: /usr/lib/libpcl_registration.so
my_pcl_viewer: /usr/lib/libpcl_segmentation.so
my_pcl_viewer: /usr/lib/libpcl_recognition.so
my_pcl_viewer: /usr/lib/libpcl_visualization.so
my_pcl_viewer: /usr/lib/libpcl_people.so
my_pcl_viewer: /usr/lib/libpcl_outofcore.so
my_pcl_viewer: /usr/lib/libpcl_tracking.so
my_pcl_viewer: /usr/lib/libpcl_apps.so
my_pcl_viewer: /usr/lib/libvtkViews.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkInfovis.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkWidgets.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkVolumeRendering.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkHybrid.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkParallel.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkRendering.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkImaging.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkGraphics.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkIO.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkFiltering.so.5.8.0
my_pcl_viewer: /usr/lib/libvtkCommon.so.5.8.0
my_pcl_viewer: /usr/lib/libvtksys.so.5.8.0
my_pcl_viewer: CMakeFiles/my_pcl_viewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable my_pcl_viewer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_pcl_viewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my_pcl_viewer.dir/build: my_pcl_viewer
.PHONY : CMakeFiles/my_pcl_viewer.dir/build

CMakeFiles/my_pcl_viewer.dir/requires: CMakeFiles/my_pcl_viewer.dir/my_pcl_viewer.cpp.o.requires
.PHONY : CMakeFiles/my_pcl_viewer.dir/requires

CMakeFiles/my_pcl_viewer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my_pcl_viewer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my_pcl_viewer.dir/clean

CMakeFiles/my_pcl_viewer.dir/depend:
	cd /home/pancho/pcl_sample_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pancho/pcl_sample_ws /home/pancho/pcl_sample_ws /home/pancho/pcl_sample_ws/build /home/pancho/pcl_sample_ws/build /home/pancho/pcl_sample_ws/build/CMakeFiles/my_pcl_viewer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/my_pcl_viewer.dir/depend

