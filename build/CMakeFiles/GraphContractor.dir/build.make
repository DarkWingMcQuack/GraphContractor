# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_SOURCE_DIR = /home/lukas/Projects/GraphContractor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lukas/Projects/GraphContractor/build

# Include any dependencies generated for this target.
include CMakeFiles/GraphContractor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/GraphContractor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/GraphContractor.dir/flags.make

CMakeFiles/GraphContractor.dir/src/main.cpp.o: CMakeFiles/GraphContractor.dir/flags.make
CMakeFiles/GraphContractor.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lukas/Projects/GraphContractor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/GraphContractor.dir/src/main.cpp.o"
	ccache /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/GraphContractor.dir/src/main.cpp.o -c /home/lukas/Projects/GraphContractor/src/main.cpp

CMakeFiles/GraphContractor.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GraphContractor.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lukas/Projects/GraphContractor/src/main.cpp > CMakeFiles/GraphContractor.dir/src/main.cpp.i

CMakeFiles/GraphContractor.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GraphContractor.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lukas/Projects/GraphContractor/src/main.cpp -o CMakeFiles/GraphContractor.dir/src/main.cpp.s

CMakeFiles/GraphContractor.dir/src/Graph.cpp.o: CMakeFiles/GraphContractor.dir/flags.make
CMakeFiles/GraphContractor.dir/src/Graph.cpp.o: ../src/Graph.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lukas/Projects/GraphContractor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/GraphContractor.dir/src/Graph.cpp.o"
	ccache /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/GraphContractor.dir/src/Graph.cpp.o -c /home/lukas/Projects/GraphContractor/src/Graph.cpp

CMakeFiles/GraphContractor.dir/src/Graph.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GraphContractor.dir/src/Graph.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lukas/Projects/GraphContractor/src/Graph.cpp > CMakeFiles/GraphContractor.dir/src/Graph.cpp.i

CMakeFiles/GraphContractor.dir/src/Graph.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GraphContractor.dir/src/Graph.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lukas/Projects/GraphContractor/src/Graph.cpp -o CMakeFiles/GraphContractor.dir/src/Graph.cpp.s

# Object files for target GraphContractor
GraphContractor_OBJECTS = \
"CMakeFiles/GraphContractor.dir/src/main.cpp.o" \
"CMakeFiles/GraphContractor.dir/src/Graph.cpp.o"

# External object files for target GraphContractor
GraphContractor_EXTERNAL_OBJECTS =

GraphContractor: CMakeFiles/GraphContractor.dir/src/main.cpp.o
GraphContractor: CMakeFiles/GraphContractor.dir/src/Graph.cpp.o
GraphContractor: CMakeFiles/GraphContractor.dir/build.make
GraphContractor: deps/fmt/lib64/libfmt.a
GraphContractor: CMakeFiles/GraphContractor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lukas/Projects/GraphContractor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable GraphContractor"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/GraphContractor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/GraphContractor.dir/build: GraphContractor

.PHONY : CMakeFiles/GraphContractor.dir/build

CMakeFiles/GraphContractor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/GraphContractor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/GraphContractor.dir/clean

CMakeFiles/GraphContractor.dir/depend:
	cd /home/lukas/Projects/GraphContractor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lukas/Projects/GraphContractor /home/lukas/Projects/GraphContractor /home/lukas/Projects/GraphContractor/build /home/lukas/Projects/GraphContractor/build /home/lukas/Projects/GraphContractor/build/CMakeFiles/GraphContractor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/GraphContractor.dir/depend

