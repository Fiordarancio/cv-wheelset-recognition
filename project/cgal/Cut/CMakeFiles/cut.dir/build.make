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
CMAKE_SOURCE_DIR = /home/ilaria/progetto/progetto_offset_assile/cgal/Cut

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ilaria/progetto/progetto_offset_assile/cgal/Cut

# Include any dependencies generated for this target.
include CMakeFiles/cut.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cut.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cut.dir/flags.make

CMakeFiles/cut.dir/cut.cpp.o: CMakeFiles/cut.dir/flags.make
CMakeFiles/cut.dir/cut.cpp.o: cut.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ilaria/progetto/progetto_offset_assile/cgal/Cut/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cut.dir/cut.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cut.dir/cut.cpp.o -c /home/ilaria/progetto/progetto_offset_assile/cgal/Cut/cut.cpp

CMakeFiles/cut.dir/cut.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cut.dir/cut.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ilaria/progetto/progetto_offset_assile/cgal/Cut/cut.cpp > CMakeFiles/cut.dir/cut.cpp.i

CMakeFiles/cut.dir/cut.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cut.dir/cut.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ilaria/progetto/progetto_offset_assile/cgal/Cut/cut.cpp -o CMakeFiles/cut.dir/cut.cpp.s

# Object files for target cut
cut_OBJECTS = \
"CMakeFiles/cut.dir/cut.cpp.o"

# External object files for target cut
cut_EXTERNAL_OBJECTS =

cut: CMakeFiles/cut.dir/cut.cpp.o
cut: CMakeFiles/cut.dir/build.make
cut: /usr/local/lib/libmpfr.so
cut: /usr/lib/x86_64-linux-gnu/libgmp.so
cut: /home/ilaria/CGAL-4.13.1/lib/libCGAL.so.13.0.2
cut: /usr/local/lib/libmpfr.so
cut: /usr/lib/x86_64-linux-gnu/libgmp.so
cut: /usr/lib/x86_64-linux-gnu/libboost_thread.so
cut: /usr/lib/x86_64-linux-gnu/libboost_system.so
cut: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
cut: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
cut: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
cut: /usr/lib/x86_64-linux-gnu/libpthread.so
cut: /usr/lib/x86_64-linux-gnu/libboost_thread.so
cut: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
cut: /usr/lib/x86_64-linux-gnu/libboost_system.so
cut: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
cut: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
cut: CMakeFiles/cut.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ilaria/progetto/progetto_offset_assile/cgal/Cut/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cut"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cut.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cut.dir/build: cut

.PHONY : CMakeFiles/cut.dir/build

CMakeFiles/cut.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cut.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cut.dir/clean

CMakeFiles/cut.dir/depend:
	cd /home/ilaria/progetto/progetto_offset_assile/cgal/Cut && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ilaria/progetto/progetto_offset_assile/cgal/Cut /home/ilaria/progetto/progetto_offset_assile/cgal/Cut /home/ilaria/progetto/progetto_offset_assile/cgal/Cut /home/ilaria/progetto/progetto_offset_assile/cgal/Cut /home/ilaria/progetto/progetto_offset_assile/cgal/Cut/CMakeFiles/cut.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cut.dir/depend

