# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /opt/cmake-3.15.2-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.15.2-Linux-x86_64/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape

# Include any dependencies generated for this target.
include CMakeFiles/clear_shape.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/clear_shape.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/clear_shape.dir/flags.make

CMakeFiles/clear_shape.dir/clear_shape.cpp.o: CMakeFiles/clear_shape.dir/flags.make
CMakeFiles/clear_shape.dir/clear_shape.cpp.o: clear_shape.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/clear_shape.dir/clear_shape.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/clear_shape.dir/clear_shape.cpp.o -c /home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape/clear_shape.cpp

CMakeFiles/clear_shape.dir/clear_shape.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clear_shape.dir/clear_shape.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape/clear_shape.cpp > CMakeFiles/clear_shape.dir/clear_shape.cpp.i

CMakeFiles/clear_shape.dir/clear_shape.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clear_shape.dir/clear_shape.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape/clear_shape.cpp -o CMakeFiles/clear_shape.dir/clear_shape.cpp.s

# Object files for target clear_shape
clear_shape_OBJECTS = \
"CMakeFiles/clear_shape.dir/clear_shape.cpp.o"

# External object files for target clear_shape
clear_shape_EXTERNAL_OBJECTS =

clear_shape: CMakeFiles/clear_shape.dir/clear_shape.cpp.o
clear_shape: CMakeFiles/clear_shape.dir/build.make
clear_shape: /usr/local/lib/libmpfr.so
clear_shape: /usr/lib/x86_64-linux-gnu/libgmp.so
clear_shape: /home/ilaria/CGAL-4.13.1/lib/libCGAL.so.13.0.2
clear_shape: /usr/local/lib/libmpfr.so
clear_shape: /usr/lib/x86_64-linux-gnu/libgmp.so
clear_shape: /usr/lib/x86_64-linux-gnu/libboost_thread.so
clear_shape: /usr/lib/x86_64-linux-gnu/libboost_system.so
clear_shape: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
clear_shape: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
clear_shape: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
clear_shape: /usr/lib/x86_64-linux-gnu/libpthread.so
clear_shape: /usr/lib/x86_64-linux-gnu/libboost_thread.so
clear_shape: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
clear_shape: /usr/lib/x86_64-linux-gnu/libboost_system.so
clear_shape: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
clear_shape: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
clear_shape: CMakeFiles/clear_shape.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable clear_shape"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/clear_shape.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/clear_shape.dir/build: clear_shape

.PHONY : CMakeFiles/clear_shape.dir/build

CMakeFiles/clear_shape.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clear_shape.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clear_shape.dir/clean

CMakeFiles/clear_shape.dir/depend:
	cd /home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape /home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape /home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape /home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape /home/ilaria/progetto/progetto_offset_assile/cgal/Clear_shape/CMakeFiles/clear_shape.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/clear_shape.dir/depend

