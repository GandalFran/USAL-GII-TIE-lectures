# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /opt/fslc-x11/2.5.3/release/sysroots/x86_64-fslcsdk-linux/usr/bin/cmake

# The command to remove a file.
RM = /opt/fslc-x11/2.5.3/release/sysroots/x86_64-fslcsdk-linux/usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/deepsy/Desktop/examples/example_2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/deepsy/Desktop/examples/example_2/buildarm

# Include any dependencies generated for this target.
include CMakeFiles/app.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/app.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/app.dir/flags.make

CMakeFiles/app.dir/src/main.cpp.o: CMakeFiles/app.dir/flags.make
CMakeFiles/app.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/deepsy/Desktop/examples/example_2/buildarm/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/app.dir/src/main.cpp.o"
	/opt/fslc-x11/2.5.3/release/sysroots/x86_64-fslcsdk-linux/usr/bin/arm-fslc-linux-gnueabi/arm-fslc-linux-gnueabi-g++   -march=armv7-a -mthumb -mfpu=neon -mfloat-abi=hard --sysroot=/opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/app.dir/src/main.cpp.o -c /home/deepsy/Desktop/examples/example_2/src/main.cpp

CMakeFiles/app.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app.dir/src/main.cpp.i"
	/opt/fslc-x11/2.5.3/release/sysroots/x86_64-fslcsdk-linux/usr/bin/arm-fslc-linux-gnueabi/arm-fslc-linux-gnueabi-g++   -march=armv7-a -mthumb -mfpu=neon -mfloat-abi=hard --sysroot=/opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/deepsy/Desktop/examples/example_2/src/main.cpp > CMakeFiles/app.dir/src/main.cpp.i

CMakeFiles/app.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app.dir/src/main.cpp.s"
	/opt/fslc-x11/2.5.3/release/sysroots/x86_64-fslcsdk-linux/usr/bin/arm-fslc-linux-gnueabi/arm-fslc-linux-gnueabi-g++   -march=armv7-a -mthumb -mfpu=neon -mfloat-abi=hard --sysroot=/opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/deepsy/Desktop/examples/example_2/src/main.cpp -o CMakeFiles/app.dir/src/main.cpp.s

CMakeFiles/app.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/app.dir/src/main.cpp.o.requires

CMakeFiles/app.dir/src/main.cpp.o.provides: CMakeFiles/app.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/app.dir/src/main.cpp.o.provides

CMakeFiles/app.dir/src/main.cpp.o.provides.build: CMakeFiles/app.dir/src/main.cpp.o


# Object files for target app
app_OBJECTS = \
"CMakeFiles/app.dir/src/main.cpp.o"

# External object files for target app
app_EXTERNAL_OBJECTS =

app: CMakeFiles/app.dir/src/main.cpp.o
app: CMakeFiles/app.dir/build.make
app: /opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi/usr/lib/libboost_system-mt.so
app: /opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi/usr/lib/libboost_thread-mt.so
app: /opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi/usr/lib/libboost_chrono-mt.so
app: /opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi/usr/lib/libboost_filesystem-mt.so
app: /opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi/usr/lib/libboost_program_options-mt.so
app: /opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi/usr/lib/libboost_date_time-mt.so
app: /opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi/usr/lib/libboost_timer-mt.so
app: /opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi/usr/lib/libboost_regex-mt.so
app: /opt/fslc-x11/2.5.3/release/sysroots/armv7at2hf-neon-fslc-linux-gnueabi/usr/lib/libboost_atomic-mt.so
app: CMakeFiles/app.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/deepsy/Desktop/examples/example_2/buildarm/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable app"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/app.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/app.dir/build: app

.PHONY : CMakeFiles/app.dir/build

CMakeFiles/app.dir/requires: CMakeFiles/app.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/app.dir/requires

CMakeFiles/app.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/app.dir/cmake_clean.cmake
.PHONY : CMakeFiles/app.dir/clean

CMakeFiles/app.dir/depend:
	cd /home/deepsy/Desktop/examples/example_2/buildarm && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/deepsy/Desktop/examples/example_2 /home/deepsy/Desktop/examples/example_2 /home/deepsy/Desktop/examples/example_2/buildarm /home/deepsy/Desktop/examples/example_2/buildarm /home/deepsy/Desktop/examples/example_2/buildarm/CMakeFiles/app.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/app.dir/depend

