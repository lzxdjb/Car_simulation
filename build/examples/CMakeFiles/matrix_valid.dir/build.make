# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/lzx/anaconda3/lib/python3.11/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/lzx/anaconda3/lib/python3.11/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /media/lzx/lzx/lzx/car_simulation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/lzx/lzx/lzx/car_simulation/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/matrix_valid.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/matrix_valid.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/matrix_valid.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/matrix_valid.dir/flags.make

examples/CMakeFiles/matrix_valid.dir/matrix_valid.cpp.o: examples/CMakeFiles/matrix_valid.dir/flags.make
examples/CMakeFiles/matrix_valid.dir/matrix_valid.cpp.o: /media/lzx/lzx/lzx/car_simulation/examples/matrix_valid.cpp
examples/CMakeFiles/matrix_valid.dir/matrix_valid.cpp.o: examples/CMakeFiles/matrix_valid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/lzx/lzx/lzx/car_simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/matrix_valid.dir/matrix_valid.cpp.o"
	cd /media/lzx/lzx/lzx/car_simulation/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/matrix_valid.dir/matrix_valid.cpp.o -MF CMakeFiles/matrix_valid.dir/matrix_valid.cpp.o.d -o CMakeFiles/matrix_valid.dir/matrix_valid.cpp.o -c /media/lzx/lzx/lzx/car_simulation/examples/matrix_valid.cpp

examples/CMakeFiles/matrix_valid.dir/matrix_valid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix_valid.dir/matrix_valid.cpp.i"
	cd /media/lzx/lzx/lzx/car_simulation/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/lzx/lzx/lzx/car_simulation/examples/matrix_valid.cpp > CMakeFiles/matrix_valid.dir/matrix_valid.cpp.i

examples/CMakeFiles/matrix_valid.dir/matrix_valid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix_valid.dir/matrix_valid.cpp.s"
	cd /media/lzx/lzx/lzx/car_simulation/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/lzx/lzx/lzx/car_simulation/examples/matrix_valid.cpp -o CMakeFiles/matrix_valid.dir/matrix_valid.cpp.s

# Object files for target matrix_valid
matrix_valid_OBJECTS = \
"CMakeFiles/matrix_valid.dir/matrix_valid.cpp.o"

# External object files for target matrix_valid
matrix_valid_EXTERNAL_OBJECTS =

examples/matrix_valid: examples/CMakeFiles/matrix_valid.dir/matrix_valid.cpp.o
examples/matrix_valid: examples/CMakeFiles/matrix_valid.dir/build.make
examples/matrix_valid: src/tinympc/libtinympc.a
examples/matrix_valid: examples/CMakeFiles/matrix_valid.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/lzx/lzx/lzx/car_simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable matrix_valid"
	cd /media/lzx/lzx/lzx/car_simulation/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/matrix_valid.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/matrix_valid.dir/build: examples/matrix_valid
.PHONY : examples/CMakeFiles/matrix_valid.dir/build

examples/CMakeFiles/matrix_valid.dir/clean:
	cd /media/lzx/lzx/lzx/car_simulation/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/matrix_valid.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/matrix_valid.dir/clean

examples/CMakeFiles/matrix_valid.dir/depend:
	cd /media/lzx/lzx/lzx/car_simulation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/lzx/lzx/lzx/car_simulation /media/lzx/lzx/lzx/car_simulation/examples /media/lzx/lzx/lzx/car_simulation/build /media/lzx/lzx/lzx/car_simulation/build/examples /media/lzx/lzx/lzx/car_simulation/build/examples/CMakeFiles/matrix_valid.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/matrix_valid.dir/depend

