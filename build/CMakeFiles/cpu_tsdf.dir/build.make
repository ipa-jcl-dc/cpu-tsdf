# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jcl-stud/decai/hello-world

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jcl-stud/decai/hello-world/build

# Include any dependencies generated for this target.
include CMakeFiles/cpu_tsdf.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cpu_tsdf.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cpu_tsdf.dir/flags.make

CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o: CMakeFiles/cpu_tsdf.dir/flags.make
CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o: ../src/lib/octree.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jcl-stud/decai/hello-world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o -c /home/jcl-stud/decai/hello-world/src/lib/octree.cpp

CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jcl-stud/decai/hello-world/src/lib/octree.cpp > CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.i

CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jcl-stud/decai/hello-world/src/lib/octree.cpp -o CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.s

CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o.requires:

.PHONY : CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o.requires

CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o.provides: CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o.requires
	$(MAKE) -f CMakeFiles/cpu_tsdf.dir/build.make CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o.provides.build
.PHONY : CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o.provides

CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o.provides.build: CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o


CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o: CMakeFiles/cpu_tsdf.dir/flags.make
CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o: ../src/lib/tsdf_volume_octree.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jcl-stud/decai/hello-world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o -c /home/jcl-stud/decai/hello-world/src/lib/tsdf_volume_octree.cpp

CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jcl-stud/decai/hello-world/src/lib/tsdf_volume_octree.cpp > CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.i

CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jcl-stud/decai/hello-world/src/lib/tsdf_volume_octree.cpp -o CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.s

CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o.requires:

.PHONY : CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o.requires

CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o.provides: CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o.requires
	$(MAKE) -f CMakeFiles/cpu_tsdf.dir/build.make CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o.provides.build
.PHONY : CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o.provides

CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o.provides.build: CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o


CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o: CMakeFiles/cpu_tsdf.dir/flags.make
CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o: ../src/lib/tsdf_interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jcl-stud/decai/hello-world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o -c /home/jcl-stud/decai/hello-world/src/lib/tsdf_interface.cpp

CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jcl-stud/decai/hello-world/src/lib/tsdf_interface.cpp > CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.i

CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jcl-stud/decai/hello-world/src/lib/tsdf_interface.cpp -o CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.s

CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o.requires:

.PHONY : CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o.requires

CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o.provides: CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o.requires
	$(MAKE) -f CMakeFiles/cpu_tsdf.dir/build.make CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o.provides.build
.PHONY : CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o.provides

CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o.provides.build: CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o


CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o: CMakeFiles/cpu_tsdf.dir/flags.make
CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o: ../src/lib/marching_cubes_tsdf_octree.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jcl-stud/decai/hello-world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o -c /home/jcl-stud/decai/hello-world/src/lib/marching_cubes_tsdf_octree.cpp

CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jcl-stud/decai/hello-world/src/lib/marching_cubes_tsdf_octree.cpp > CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.i

CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jcl-stud/decai/hello-world/src/lib/marching_cubes_tsdf_octree.cpp -o CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.s

CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o.requires:

.PHONY : CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o.requires

CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o.provides: CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o.requires
	$(MAKE) -f CMakeFiles/cpu_tsdf.dir/build.make CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o.provides.build
.PHONY : CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o.provides

CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o.provides.build: CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o


# Object files for target cpu_tsdf
cpu_tsdf_OBJECTS = \
"CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o" \
"CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o" \
"CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o" \
"CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o"

# External object files for target cpu_tsdf
cpu_tsdf_EXTERNAL_OBJECTS =

libcpu_tsdf.so: CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o
libcpu_tsdf.so: CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o
libcpu_tsdf.so: CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o
libcpu_tsdf.so: CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o
libcpu_tsdf.so: CMakeFiles/cpu_tsdf.dir/build.make
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libcpu_tsdf.so: /usr/lib/libpcl_common.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
libcpu_tsdf.so: /usr/lib/libpcl_kdtree.so
libcpu_tsdf.so: /usr/lib/libpcl_octree.so
libcpu_tsdf.so: /usr/lib/libpcl_search.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libqhull.so
libcpu_tsdf.so: /usr/lib/libpcl_surface.so
libcpu_tsdf.so: /usr/lib/libpcl_sample_consensus.so
libcpu_tsdf.so: /usr/lib/libOpenNI.so
libcpu_tsdf.so: /usr/lib/libOpenNI2.so
libcpu_tsdf.so: /usr/lib/libpcl_io.so
libcpu_tsdf.so: /usr/lib/libpcl_filters.so
libcpu_tsdf.so: /usr/lib/libpcl_features.so
libcpu_tsdf.so: /usr/lib/libpcl_keypoints.so
libcpu_tsdf.so: /usr/lib/libpcl_registration.so
libcpu_tsdf.so: /usr/lib/libpcl_segmentation.so
libcpu_tsdf.so: /usr/lib/libpcl_recognition.so
libcpu_tsdf.so: /usr/lib/libpcl_visualization.so
libcpu_tsdf.so: /usr/lib/libpcl_people.so
libcpu_tsdf.so: /usr/lib/libpcl_outofcore.so
libcpu_tsdf.so: /usr/lib/libpcl_tracking.so
libcpu_tsdf.so: /usr/lib/libpcl_apps.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libqhull.so
libcpu_tsdf.so: /usr/lib/libOpenNI.so
libcpu_tsdf.so: /usr/lib/libOpenNI2.so
libcpu_tsdf.so: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
libcpu_tsdf.so: /usr/lib/libvtkGenericFiltering.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkGeovis.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkCharts.so.5.8.0
libcpu_tsdf.so: /usr/lib/libpcl_common.so
libcpu_tsdf.so: /usr/lib/libpcl_kdtree.so
libcpu_tsdf.so: /usr/lib/libpcl_octree.so
libcpu_tsdf.so: /usr/lib/libpcl_search.so
libcpu_tsdf.so: /usr/lib/libpcl_surface.so
libcpu_tsdf.so: /usr/lib/libpcl_sample_consensus.so
libcpu_tsdf.so: /usr/lib/libpcl_io.so
libcpu_tsdf.so: /usr/lib/libpcl_filters.so
libcpu_tsdf.so: /usr/lib/libpcl_features.so
libcpu_tsdf.so: /usr/lib/libpcl_keypoints.so
libcpu_tsdf.so: /usr/lib/libpcl_registration.so
libcpu_tsdf.so: /usr/lib/libpcl_segmentation.so
libcpu_tsdf.so: /usr/lib/libpcl_recognition.so
libcpu_tsdf.so: /usr/lib/libpcl_visualization.so
libcpu_tsdf.so: /usr/lib/libpcl_people.so
libcpu_tsdf.so: /usr/lib/libpcl_outofcore.so
libcpu_tsdf.so: /usr/lib/libpcl_tracking.so
libcpu_tsdf.so: /usr/lib/libpcl_apps.so
libcpu_tsdf.so: /usr/lib/libvtkViews.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkInfovis.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkWidgets.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkVolumeRendering.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkHybrid.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkParallel.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkRendering.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkImaging.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkGraphics.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkIO.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkFiltering.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtkCommon.so.5.8.0
libcpu_tsdf.so: /usr/lib/libvtksys.so.5.8.0
libcpu_tsdf.so: CMakeFiles/cpu_tsdf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jcl-stud/decai/hello-world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX shared library libcpu_tsdf.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cpu_tsdf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cpu_tsdf.dir/build: libcpu_tsdf.so

.PHONY : CMakeFiles/cpu_tsdf.dir/build

CMakeFiles/cpu_tsdf.dir/requires: CMakeFiles/cpu_tsdf.dir/src/lib/octree.cpp.o.requires
CMakeFiles/cpu_tsdf.dir/requires: CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_volume_octree.cpp.o.requires
CMakeFiles/cpu_tsdf.dir/requires: CMakeFiles/cpu_tsdf.dir/src/lib/tsdf_interface.cpp.o.requires
CMakeFiles/cpu_tsdf.dir/requires: CMakeFiles/cpu_tsdf.dir/src/lib/marching_cubes_tsdf_octree.cpp.o.requires

.PHONY : CMakeFiles/cpu_tsdf.dir/requires

CMakeFiles/cpu_tsdf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cpu_tsdf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cpu_tsdf.dir/clean

CMakeFiles/cpu_tsdf.dir/depend:
	cd /home/jcl-stud/decai/hello-world/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jcl-stud/decai/hello-world /home/jcl-stud/decai/hello-world /home/jcl-stud/decai/hello-world/build /home/jcl-stud/decai/hello-world/build /home/jcl-stud/decai/hello-world/build/CMakeFiles/cpu_tsdf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cpu_tsdf.dir/depend

