# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mot/Documents/Robot_example

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mot/Documents/Robot_example/build

# Include any dependencies generated for this target.
include CMakeFiles/pnd_mujoco.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/pnd_mujoco.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/pnd_mujoco.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pnd_mujoco.dir/flags.make

CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.o: CMakeFiles/pnd_mujoco.dir/flags.make
CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.o: ../src/src/glfw_adapter.cc
CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.o: CMakeFiles/pnd_mujoco.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mot/Documents/Robot_example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.o -MF CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.o.d -o CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.o -c /home/mot/Documents/Robot_example/src/src/glfw_adapter.cc

CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mot/Documents/Robot_example/src/src/glfw_adapter.cc > CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.i

CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mot/Documents/Robot_example/src/src/glfw_adapter.cc -o CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.s

CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.o: CMakeFiles/pnd_mujoco.dir/flags.make
CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.o: ../src/src/glfw_dispatch.cc
CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.o: CMakeFiles/pnd_mujoco.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mot/Documents/Robot_example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.o -MF CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.o.d -o CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.o -c /home/mot/Documents/Robot_example/src/src/glfw_dispatch.cc

CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mot/Documents/Robot_example/src/src/glfw_dispatch.cc > CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.i

CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mot/Documents/Robot_example/src/src/glfw_dispatch.cc -o CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.s

CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.o: CMakeFiles/pnd_mujoco.dir/flags.make
CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.o: ../src/src/platform_ui_adapter.cc
CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.o: CMakeFiles/pnd_mujoco.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mot/Documents/Robot_example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.o -MF CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.o.d -o CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.o -c /home/mot/Documents/Robot_example/src/src/platform_ui_adapter.cc

CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mot/Documents/Robot_example/src/src/platform_ui_adapter.cc > CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.i

CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mot/Documents/Robot_example/src/src/platform_ui_adapter.cc -o CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.s

CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.o: CMakeFiles/pnd_mujoco.dir/flags.make
CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.o: ../src/src/simulate.cc
CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.o: CMakeFiles/pnd_mujoco.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mot/Documents/Robot_example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.o -MF CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.o.d -o CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.o -c /home/mot/Documents/Robot_example/src/src/simulate.cc

CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mot/Documents/Robot_example/src/src/simulate.cc > CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.i

CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mot/Documents/Robot_example/src/src/simulate.cc -o CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.s

CMakeFiles/pnd_mujoco.dir/src/main.cc.o: CMakeFiles/pnd_mujoco.dir/flags.make
CMakeFiles/pnd_mujoco.dir/src/main.cc.o: ../src/main.cc
CMakeFiles/pnd_mujoco.dir/src/main.cc.o: CMakeFiles/pnd_mujoco.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mot/Documents/Robot_example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/pnd_mujoco.dir/src/main.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pnd_mujoco.dir/src/main.cc.o -MF CMakeFiles/pnd_mujoco.dir/src/main.cc.o.d -o CMakeFiles/pnd_mujoco.dir/src/main.cc.o -c /home/mot/Documents/Robot_example/src/main.cc

CMakeFiles/pnd_mujoco.dir/src/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pnd_mujoco.dir/src/main.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mot/Documents/Robot_example/src/main.cc > CMakeFiles/pnd_mujoco.dir/src/main.cc.i

CMakeFiles/pnd_mujoco.dir/src/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pnd_mujoco.dir/src/main.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mot/Documents/Robot_example/src/main.cc -o CMakeFiles/pnd_mujoco.dir/src/main.cc.s

CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.o: CMakeFiles/pnd_mujoco.dir/flags.make
CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.o: ../src/mujoco_robot.cpp
CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.o: CMakeFiles/pnd_mujoco.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mot/Documents/Robot_example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.o -MF CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.o.d -o CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.o -c /home/mot/Documents/Robot_example/src/mujoco_robot.cpp

CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mot/Documents/Robot_example/src/mujoco_robot.cpp > CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.i

CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mot/Documents/Robot_example/src/mujoco_robot.cpp -o CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.s

# Object files for target pnd_mujoco
pnd_mujoco_OBJECTS = \
"CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.o" \
"CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.o" \
"CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.o" \
"CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.o" \
"CMakeFiles/pnd_mujoco.dir/src/main.cc.o" \
"CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.o"

# External object files for target pnd_mujoco
pnd_mujoco_EXTERNAL_OBJECTS =

pnd_mujoco: CMakeFiles/pnd_mujoco.dir/src/src/glfw_adapter.cc.o
pnd_mujoco: CMakeFiles/pnd_mujoco.dir/src/src/glfw_dispatch.cc.o
pnd_mujoco: CMakeFiles/pnd_mujoco.dir/src/src/platform_ui_adapter.cc.o
pnd_mujoco: CMakeFiles/pnd_mujoco.dir/src/src/simulate.cc.o
pnd_mujoco: CMakeFiles/pnd_mujoco.dir/src/main.cc.o
pnd_mujoco: CMakeFiles/pnd_mujoco.dir/src/mujoco_robot.cpp.o
pnd_mujoco: CMakeFiles/pnd_mujoco.dir/build.make
pnd_mujoco: /usr/local/lib/libmujoco.so.3.2.3
pnd_mujoco: CMakeFiles/pnd_mujoco.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mot/Documents/Robot_example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable pnd_mujoco"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pnd_mujoco.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pnd_mujoco.dir/build: pnd_mujoco
.PHONY : CMakeFiles/pnd_mujoco.dir/build

CMakeFiles/pnd_mujoco.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pnd_mujoco.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pnd_mujoco.dir/clean

CMakeFiles/pnd_mujoco.dir/depend:
	cd /home/mot/Documents/Robot_example/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mot/Documents/Robot_example /home/mot/Documents/Robot_example /home/mot/Documents/Robot_example/build /home/mot/Documents/Robot_example/build /home/mot/Documents/Robot_example/build/CMakeFiles/pnd_mujoco.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pnd_mujoco.dir/depend

