# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/liwei/dlib/dlib-19.4/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liwei/dlib/dlib-19.4/examples/build

# Include any dependencies generated for this target.
include CMakeFiles/max_cost_assignment_ex.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/max_cost_assignment_ex.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/max_cost_assignment_ex.dir/flags.make

CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o: CMakeFiles/max_cost_assignment_ex.dir/flags.make
CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o: ../max_cost_assignment_ex.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liwei/dlib/dlib-19.4/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o -c /home/liwei/dlib/dlib-19.4/examples/max_cost_assignment_ex.cpp

CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liwei/dlib/dlib-19.4/examples/max_cost_assignment_ex.cpp > CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.i

CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liwei/dlib/dlib-19.4/examples/max_cost_assignment_ex.cpp -o CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.s

CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o.requires:

.PHONY : CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o.requires

CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o.provides: CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o.requires
	$(MAKE) -f CMakeFiles/max_cost_assignment_ex.dir/build.make CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o.provides.build
.PHONY : CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o.provides

CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o.provides.build: CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o


# Object files for target max_cost_assignment_ex
max_cost_assignment_ex_OBJECTS = \
"CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o"

# External object files for target max_cost_assignment_ex
max_cost_assignment_ex_EXTERNAL_OBJECTS =

max_cost_assignment_ex: CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o
max_cost_assignment_ex: CMakeFiles/max_cost_assignment_ex.dir/build.make
max_cost_assignment_ex: dlib_build/libdlib.a
max_cost_assignment_ex: /usr/lib/x86_64-linux-gnu/libnsl.so
max_cost_assignment_ex: /usr/lib/x86_64-linux-gnu/libSM.so
max_cost_assignment_ex: /usr/lib/x86_64-linux-gnu/libICE.so
max_cost_assignment_ex: /usr/lib/x86_64-linux-gnu/libX11.so
max_cost_assignment_ex: /usr/lib/x86_64-linux-gnu/libXext.so
max_cost_assignment_ex: /usr/lib/x86_64-linux-gnu/libgif.so
max_cost_assignment_ex: /usr/lib/x86_64-linux-gnu/libpng.so
max_cost_assignment_ex: /usr/lib/x86_64-linux-gnu/libjpeg.so
max_cost_assignment_ex: /usr/lib/libblas.so
max_cost_assignment_ex: /usr/lib/liblapack.so
max_cost_assignment_ex: /usr/lib/x86_64-linux-gnu/libsqlite3.so
max_cost_assignment_ex: CMakeFiles/max_cost_assignment_ex.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/liwei/dlib/dlib-19.4/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable max_cost_assignment_ex"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/max_cost_assignment_ex.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/max_cost_assignment_ex.dir/build: max_cost_assignment_ex

.PHONY : CMakeFiles/max_cost_assignment_ex.dir/build

CMakeFiles/max_cost_assignment_ex.dir/requires: CMakeFiles/max_cost_assignment_ex.dir/max_cost_assignment_ex.cpp.o.requires

.PHONY : CMakeFiles/max_cost_assignment_ex.dir/requires

CMakeFiles/max_cost_assignment_ex.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/max_cost_assignment_ex.dir/cmake_clean.cmake
.PHONY : CMakeFiles/max_cost_assignment_ex.dir/clean

CMakeFiles/max_cost_assignment_ex.dir/depend:
	cd /home/liwei/dlib/dlib-19.4/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liwei/dlib/dlib-19.4/examples /home/liwei/dlib/dlib-19.4/examples /home/liwei/dlib/dlib-19.4/examples/build /home/liwei/dlib/dlib-19.4/examples/build /home/liwei/dlib/dlib-19.4/examples/build/CMakeFiles/max_cost_assignment_ex.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/max_cost_assignment_ex.dir/depend

