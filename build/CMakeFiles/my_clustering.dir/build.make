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
include CMakeFiles/my_clustering.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/my_clustering.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my_clustering.dir/flags.make

CMakeFiles/my_clustering.dir/my_clustering.cpp.o: CMakeFiles/my_clustering.dir/flags.make
CMakeFiles/my_clustering.dir/my_clustering.cpp.o: ../my_clustering.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/guest/usr/github/pcl_sample_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/my_clustering.dir/my_clustering.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_clustering.dir/my_clustering.cpp.o -c /home/guest/usr/github/pcl_sample_ws/my_clustering.cpp

CMakeFiles/my_clustering.dir/my_clustering.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_clustering.dir/my_clustering.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/guest/usr/github/pcl_sample_ws/my_clustering.cpp > CMakeFiles/my_clustering.dir/my_clustering.cpp.i

CMakeFiles/my_clustering.dir/my_clustering.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_clustering.dir/my_clustering.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/guest/usr/github/pcl_sample_ws/my_clustering.cpp -o CMakeFiles/my_clustering.dir/my_clustering.cpp.s

CMakeFiles/my_clustering.dir/my_clustering.cpp.o.requires:
.PHONY : CMakeFiles/my_clustering.dir/my_clustering.cpp.o.requires

CMakeFiles/my_clustering.dir/my_clustering.cpp.o.provides: CMakeFiles/my_clustering.dir/my_clustering.cpp.o.requires
	$(MAKE) -f CMakeFiles/my_clustering.dir/build.make CMakeFiles/my_clustering.dir/my_clustering.cpp.o.provides.build
.PHONY : CMakeFiles/my_clustering.dir/my_clustering.cpp.o.provides

CMakeFiles/my_clustering.dir/my_clustering.cpp.o.provides.build: CMakeFiles/my_clustering.dir/my_clustering.cpp.o

# Object files for target my_clustering
my_clustering_OBJECTS = \
"CMakeFiles/my_clustering.dir/my_clustering.cpp.o"

# External object files for target my_clustering
my_clustering_EXTERNAL_OBJECTS =

my_clustering: CMakeFiles/my_clustering.dir/my_clustering.cpp.o
my_clustering: CMakeFiles/my_clustering.dir/build.make
my_clustering: /opt/ros/indigo/lib/libpcl_ros_filters.so
my_clustering: /opt/ros/indigo/lib/libpcl_ros_io.so
my_clustering: /opt/ros/indigo/lib/libpcl_ros_tf.so
my_clustering: /usr/lib/libpcl_common.so
my_clustering: /usr/lib/libpcl_octree.so
my_clustering: /usr/lib/libpcl_io.so
my_clustering: /usr/lib/libpcl_kdtree.so
my_clustering: /usr/lib/libpcl_search.so
my_clustering: /usr/lib/libpcl_sample_consensus.so
my_clustering: /usr/lib/libpcl_filters.so
my_clustering: /usr/lib/libpcl_features.so
my_clustering: /usr/lib/libpcl_keypoints.so
my_clustering: /usr/lib/libpcl_segmentation.so
my_clustering: /usr/lib/libpcl_visualization.so
my_clustering: /usr/lib/libpcl_outofcore.so
my_clustering: /usr/lib/libpcl_registration.so
my_clustering: /usr/lib/libpcl_recognition.so
my_clustering: /usr/lib/libpcl_surface.so
my_clustering: /usr/lib/libpcl_people.so
my_clustering: /usr/lib/libpcl_tracking.so
my_clustering: /usr/lib/libpcl_apps.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
my_clustering: /usr/lib/x86_64-linux-gnu/libqhull.so
my_clustering: /usr/lib/libOpenNI.so
my_clustering: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
my_clustering: /usr/lib/libvtkCommon.so.5.8.0
my_clustering: /usr/lib/libvtkRendering.so.5.8.0
my_clustering: /usr/lib/libvtkHybrid.so.5.8.0
my_clustering: /usr/lib/libvtkCharts.so.5.8.0
my_clustering: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
my_clustering: /opt/ros/indigo/lib/libnodeletlib.so
my_clustering: /opt/ros/indigo/lib/libbondcpp.so
my_clustering: /usr/lib/x86_64-linux-gnu/libuuid.so
my_clustering: /opt/ros/indigo/lib/librosbag.so
my_clustering: /opt/ros/indigo/lib/librosbag_storage.so
my_clustering: /opt/ros/indigo/lib/libroslz4.so
my_clustering: /usr/lib/x86_64-linux-gnu/liblz4.so
my_clustering: /opt/ros/indigo/lib/libtopic_tools.so
my_clustering: /opt/ros/indigo/lib/libtf.so
my_clustering: /opt/ros/indigo/lib/libtf2_ros.so
my_clustering: /opt/ros/indigo/lib/libactionlib.so
my_clustering: /opt/ros/indigo/lib/libtf2.so
my_clustering: /opt/ros/indigo/lib/libimage_transport.so
my_clustering: /opt/ros/indigo/lib/libmessage_filters.so
my_clustering: /opt/ros/indigo/lib/libclass_loader.so
my_clustering: /usr/lib/libPocoFoundation.so
my_clustering: /usr/lib/x86_64-linux-gnu/libdl.so
my_clustering: /opt/ros/indigo/lib/libroscpp.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_signals.so
my_clustering: /opt/ros/indigo/lib/libxmlrpcpp.so
my_clustering: /opt/ros/indigo/lib/libroslib.so
my_clustering: /opt/ros/indigo/lib/librospack.so
my_clustering: /usr/lib/x86_64-linux-gnu/libpython2.7.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
my_clustering: /usr/lib/x86_64-linux-gnu/libtinyxml.so
my_clustering: /opt/ros/indigo/lib/libcv_bridge.so
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
my_clustering: /opt/ros/indigo/lib/librosconsole.so
my_clustering: /opt/ros/indigo/lib/librosconsole_log4cxx.so
my_clustering: /opt/ros/indigo/lib/librosconsole_backend_interface.so
my_clustering: /usr/lib/liblog4cxx.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_regex.so
my_clustering: /opt/ros/indigo/lib/libopencv_apps.so
my_clustering: /opt/ros/indigo/lib/libroscpp_serialization.so
my_clustering: /opt/ros/indigo/lib/librostime.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
my_clustering: /opt/ros/indigo/lib/libcpp_common.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_system.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_thread.so
my_clustering: /usr/lib/x86_64-linux-gnu/libpthread.so
my_clustering: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_system.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_thread.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
my_clustering: /usr/lib/x86_64-linux-gnu/libpthread.so
my_clustering: /usr/lib/libpcl_common.so
my_clustering: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
my_clustering: /usr/lib/libpcl_kdtree.so
my_clustering: /usr/lib/libpcl_octree.so
my_clustering: /usr/lib/libpcl_search.so
my_clustering: /usr/lib/x86_64-linux-gnu/libqhull.so
my_clustering: /usr/lib/libpcl_surface.so
my_clustering: /usr/lib/libpcl_sample_consensus.so
my_clustering: /usr/lib/libOpenNI.so
my_clustering: /usr/lib/libOpenNI2.so
my_clustering: /usr/lib/libvtkCommon.so.5.8.0
my_clustering: /usr/lib/libvtkFiltering.so.5.8.0
my_clustering: /usr/lib/libvtkImaging.so.5.8.0
my_clustering: /usr/lib/libvtkGraphics.so.5.8.0
my_clustering: /usr/lib/libvtkGenericFiltering.so.5.8.0
my_clustering: /usr/lib/libvtkIO.so.5.8.0
my_clustering: /usr/lib/libvtkRendering.so.5.8.0
my_clustering: /usr/lib/libvtkVolumeRendering.so.5.8.0
my_clustering: /usr/lib/libvtkHybrid.so.5.8.0
my_clustering: /usr/lib/libvtkWidgets.so.5.8.0
my_clustering: /usr/lib/libvtkParallel.so.5.8.0
my_clustering: /usr/lib/libvtkInfovis.so.5.8.0
my_clustering: /usr/lib/libvtkGeovis.so.5.8.0
my_clustering: /usr/lib/libvtkViews.so.5.8.0
my_clustering: /usr/lib/libvtkCharts.so.5.8.0
my_clustering: /usr/lib/libpcl_io.so
my_clustering: /usr/lib/libpcl_filters.so
my_clustering: /usr/lib/libpcl_features.so
my_clustering: /usr/lib/libpcl_keypoints.so
my_clustering: /usr/lib/libpcl_registration.so
my_clustering: /usr/lib/libpcl_segmentation.so
my_clustering: /usr/lib/libpcl_recognition.so
my_clustering: /usr/lib/libpcl_visualization.so
my_clustering: /usr/lib/libpcl_people.so
my_clustering: /usr/lib/libpcl_outofcore.so
my_clustering: /usr/lib/libpcl_tracking.so
my_clustering: /usr/lib/libpcl_apps.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_system.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_thread.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
my_clustering: /usr/lib/x86_64-linux-gnu/libpthread.so
my_clustering: /usr/lib/x86_64-linux-gnu/libqhull.so
my_clustering: /usr/lib/libOpenNI.so
my_clustering: /usr/lib/libOpenNI2.so
my_clustering: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
my_clustering: /usr/lib/libvtkCommon.so.5.8.0
my_clustering: /usr/lib/libvtkFiltering.so.5.8.0
my_clustering: /usr/lib/libvtkImaging.so.5.8.0
my_clustering: /usr/lib/libvtkGraphics.so.5.8.0
my_clustering: /usr/lib/libvtkGenericFiltering.so.5.8.0
my_clustering: /usr/lib/libvtkIO.so.5.8.0
my_clustering: /usr/lib/libvtkRendering.so.5.8.0
my_clustering: /usr/lib/libvtkVolumeRendering.so.5.8.0
my_clustering: /usr/lib/libvtkHybrid.so.5.8.0
my_clustering: /usr/lib/libvtkWidgets.so.5.8.0
my_clustering: /usr/lib/libvtkParallel.so.5.8.0
my_clustering: /usr/lib/libvtkInfovis.so.5.8.0
my_clustering: /usr/lib/libvtkGeovis.so.5.8.0
my_clustering: /usr/lib/libvtkViews.so.5.8.0
my_clustering: /usr/lib/libvtkCharts.so.5.8.0
my_clustering: /usr/local/lib/libopencv_viz.so.2.4.11
my_clustering: /usr/local/lib/libopencv_videostab.so.2.4.11
my_clustering: /usr/local/lib/libopencv_video.so.2.4.11
my_clustering: /usr/local/lib/libopencv_ts.a
my_clustering: /usr/local/lib/libopencv_superres.so.2.4.11
my_clustering: /usr/local/lib/libopencv_stitching.so.2.4.11
my_clustering: /usr/local/lib/libopencv_photo.so.2.4.11
my_clustering: /usr/local/lib/libopencv_ocl.so.2.4.11
my_clustering: /usr/local/lib/libopencv_objdetect.so.2.4.11
my_clustering: /usr/local/lib/libopencv_nonfree.so.2.4.11
my_clustering: /usr/local/lib/libopencv_ml.so.2.4.11
my_clustering: /usr/local/lib/libopencv_legacy.so.2.4.11
my_clustering: /usr/local/lib/libopencv_imgproc.so.2.4.11
my_clustering: /usr/local/lib/libopencv_highgui.so.2.4.11
my_clustering: /usr/local/lib/libopencv_gpu.so.2.4.11
my_clustering: /usr/local/lib/libopencv_flann.so.2.4.11
my_clustering: /usr/local/lib/libopencv_features2d.so.2.4.11
my_clustering: /usr/local/lib/libopencv_core.so.2.4.11
my_clustering: /usr/local/lib/libopencv_contrib.so.2.4.11
my_clustering: /usr/local/lib/libopencv_calib3d.so.2.4.11
my_clustering: /usr/lib/libpcl_common.so
my_clustering: /usr/lib/libpcl_octree.so
my_clustering: /usr/lib/libpcl_kdtree.so
my_clustering: /usr/lib/libpcl_search.so
my_clustering: /usr/lib/libpcl_sample_consensus.so
my_clustering: /usr/lib/libpcl_surface.so
my_clustering: /usr/lib/libvtkCommon.so.5.8.0
my_clustering: /usr/lib/libvtkRendering.so.5.8.0
my_clustering: /usr/lib/libvtkHybrid.so.5.8.0
my_clustering: /usr/lib/libvtkCharts.so.5.8.0
my_clustering: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
my_clustering: /opt/ros/indigo/lib/libnodeletlib.so
my_clustering: /opt/ros/indigo/lib/libbondcpp.so
my_clustering: /usr/lib/x86_64-linux-gnu/libuuid.so
my_clustering: /opt/ros/indigo/lib/librosbag.so
my_clustering: /opt/ros/indigo/lib/librosbag_storage.so
my_clustering: /opt/ros/indigo/lib/libroslz4.so
my_clustering: /usr/lib/x86_64-linux-gnu/liblz4.so
my_clustering: /opt/ros/indigo/lib/libtopic_tools.so
my_clustering: /opt/ros/indigo/lib/libtf.so
my_clustering: /opt/ros/indigo/lib/libtf2_ros.so
my_clustering: /opt/ros/indigo/lib/libactionlib.so
my_clustering: /opt/ros/indigo/lib/libtf2.so
my_clustering: /opt/ros/indigo/lib/libimage_transport.so
my_clustering: /opt/ros/indigo/lib/libmessage_filters.so
my_clustering: /opt/ros/indigo/lib/libclass_loader.so
my_clustering: /usr/lib/libPocoFoundation.so
my_clustering: /usr/lib/x86_64-linux-gnu/libdl.so
my_clustering: /opt/ros/indigo/lib/libroscpp.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_signals.so
my_clustering: /opt/ros/indigo/lib/libxmlrpcpp.so
my_clustering: /opt/ros/indigo/lib/libroslib.so
my_clustering: /opt/ros/indigo/lib/librospack.so
my_clustering: /usr/lib/x86_64-linux-gnu/libpython2.7.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
my_clustering: /usr/lib/x86_64-linux-gnu/libtinyxml.so
my_clustering: /opt/ros/indigo/lib/libcv_bridge.so
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
my_clustering: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
my_clustering: /opt/ros/indigo/lib/librosconsole.so
my_clustering: /opt/ros/indigo/lib/librosconsole_log4cxx.so
my_clustering: /opt/ros/indigo/lib/librosconsole_backend_interface.so
my_clustering: /usr/lib/liblog4cxx.so
my_clustering: /usr/lib/x86_64-linux-gnu/libboost_regex.so
my_clustering: /opt/ros/indigo/lib/libopencv_apps.so
my_clustering: /opt/ros/indigo/lib/libroscpp_serialization.so
my_clustering: /opt/ros/indigo/lib/librostime.so
my_clustering: /opt/ros/indigo/lib/libcpp_common.so
my_clustering: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
my_clustering: /usr/lib/libvtkGenericFiltering.so.5.8.0
my_clustering: /usr/lib/libvtkGeovis.so.5.8.0
my_clustering: /usr/lib/libvtkCharts.so.5.8.0
my_clustering: /usr/lib/libvtkViews.so.5.8.0
my_clustering: /usr/lib/libvtkInfovis.so.5.8.0
my_clustering: /usr/lib/libvtkWidgets.so.5.8.0
my_clustering: /usr/lib/libvtkVolumeRendering.so.5.8.0
my_clustering: /usr/lib/libvtkHybrid.so.5.8.0
my_clustering: /usr/lib/libvtkParallel.so.5.8.0
my_clustering: /usr/lib/libvtkRendering.so.5.8.0
my_clustering: /usr/lib/libvtkImaging.so.5.8.0
my_clustering: /usr/lib/libvtkGraphics.so.5.8.0
my_clustering: /usr/lib/libvtkIO.so.5.8.0
my_clustering: /usr/lib/libvtkFiltering.so.5.8.0
my_clustering: /usr/lib/libvtkCommon.so.5.8.0
my_clustering: /usr/lib/libvtksys.so.5.8.0
my_clustering: /usr/lib/x86_64-linux-gnu/libGLU.so
my_clustering: /usr/lib/x86_64-linux-gnu/libGL.so
my_clustering: /usr/lib/x86_64-linux-gnu/libSM.so
my_clustering: /usr/lib/x86_64-linux-gnu/libICE.so
my_clustering: /usr/lib/x86_64-linux-gnu/libX11.so
my_clustering: /usr/lib/x86_64-linux-gnu/libXext.so
my_clustering: /usr/local/lib/libopencv_nonfree.so.2.4.11
my_clustering: /usr/local/lib/libopencv_ocl.so.2.4.11
my_clustering: /usr/local/lib/libopencv_gpu.so.2.4.11
my_clustering: /usr/local/lib/libopencv_photo.so.2.4.11
my_clustering: /usr/local/lib/libopencv_objdetect.so.2.4.11
my_clustering: /usr/local/lib/libopencv_legacy.so.2.4.11
my_clustering: /usr/local/lib/libopencv_video.so.2.4.11
my_clustering: /usr/local/lib/libopencv_ml.so.2.4.11
my_clustering: /usr/local/lib/libopencv_calib3d.so.2.4.11
my_clustering: /usr/local/lib/libopencv_features2d.so.2.4.11
my_clustering: /usr/local/lib/libopencv_highgui.so.2.4.11
my_clustering: /usr/local/lib/libopencv_imgproc.so.2.4.11
my_clustering: /usr/local/lib/libopencv_flann.so.2.4.11
my_clustering: /usr/local/lib/libopencv_core.so.2.4.11
my_clustering: CMakeFiles/my_clustering.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable my_clustering"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_clustering.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my_clustering.dir/build: my_clustering
.PHONY : CMakeFiles/my_clustering.dir/build

CMakeFiles/my_clustering.dir/requires: CMakeFiles/my_clustering.dir/my_clustering.cpp.o.requires
.PHONY : CMakeFiles/my_clustering.dir/requires

CMakeFiles/my_clustering.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my_clustering.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my_clustering.dir/clean

CMakeFiles/my_clustering.dir/depend:
	cd /home/guest/usr/github/pcl_sample_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guest/usr/github/pcl_sample_ws /home/guest/usr/github/pcl_sample_ws /home/guest/usr/github/pcl_sample_ws/build /home/guest/usr/github/pcl_sample_ws/build /home/guest/usr/github/pcl_sample_ws/build/CMakeFiles/my_clustering.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/my_clustering.dir/depend

