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
CMAKE_SOURCE_DIR = /home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply

# Include any dependencies generated for this target.
include CMakeFiles/read_ply_with_properties.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/read_ply_with_properties.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/read_ply_with_properties.dir/flags.make

CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.o: CMakeFiles/read_ply_with_properties.dir/flags.make
CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.o: read_ply_with_properties.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.o -c /home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply/read_ply_with_properties.cpp

CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply/read_ply_with_properties.cpp > CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.i

CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply/read_ply_with_properties.cpp -o CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.s

# Object files for target read_ply_with_properties
read_ply_with_properties_OBJECTS = \
"CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.o"

# External object files for target read_ply_with_properties
read_ply_with_properties_EXTERNAL_OBJECTS =

read_ply_with_properties: CMakeFiles/read_ply_with_properties.dir/read_ply_with_properties.cpp.o
read_ply_with_properties: CMakeFiles/read_ply_with_properties.dir/build.make
read_ply_with_properties: /usr/local/lib/libmpfr.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libgmp.so
read_ply_with_properties: /home/ilaria/CGAL-4.13.1/lib/libCGAL.so.13.0.2
read_ply_with_properties: /usr/local/lib/libmpfr.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libgmp.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libboost_thread.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libboost_system.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libpthread.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libboost_thread.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libboost_system.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
read_ply_with_properties: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
read_ply_with_properties: CMakeFiles/read_ply_with_properties.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable read_ply_with_properties"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/read_ply_with_properties.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/read_ply_with_properties.dir/build: read_ply_with_properties

.PHONY : CMakeFiles/read_ply_with_properties.dir/build

CMakeFiles/read_ply_with_properties.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/read_ply_with_properties.dir/cmake_clean.cmake
.PHONY : CMakeFiles/read_ply_with_properties.dir/clean

CMakeFiles/read_ply_with_properties.dir/depend:
	cd /home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply /home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply /home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply /home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply /home/ilaria/progetto/progetto_offset_assile/cgal/Read_ply/CMakeFiles/read_ply_with_properties.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/read_ply_with_properties.dir/depend

