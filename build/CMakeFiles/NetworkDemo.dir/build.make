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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/eman/3D-Random-Walk-CUDA

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/eman/3D-Random-Walk-CUDA/build

# Include any dependencies generated for this target.
include CMakeFiles/NetworkDemo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/NetworkDemo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/NetworkDemo.dir/flags.make

CMakeFiles/NetworkDemo.dir/NetworkDemo_generated_main.cu.o: CMakeFiles/NetworkDemo.dir/NetworkDemo_generated_main.cu.o.depend
CMakeFiles/NetworkDemo.dir/NetworkDemo_generated_main.cu.o: CMakeFiles/NetworkDemo.dir/NetworkDemo_generated_main.cu.o.cmake
CMakeFiles/NetworkDemo.dir/NetworkDemo_generated_main.cu.o: ../main.cu
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/eman/3D-Random-Walk-CUDA/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building NVCC (Device) object CMakeFiles/NetworkDemo.dir/NetworkDemo_generated_main.cu.o"
	cd /home/eman/3D-Random-Walk-CUDA/build/CMakeFiles/NetworkDemo.dir && /usr/bin/cmake -E make_directory /home/eman/3D-Random-Walk-CUDA/build/CMakeFiles/NetworkDemo.dir//.
	cd /home/eman/3D-Random-Walk-CUDA/build/CMakeFiles/NetworkDemo.dir && /usr/bin/cmake -D verbose:BOOL=$(VERBOSE) -D build_configuration:STRING= -D generated_file:STRING=/home/eman/3D-Random-Walk-CUDA/build/CMakeFiles/NetworkDemo.dir//./NetworkDemo_generated_main.cu.o -D generated_cubin_file:STRING=/home/eman/3D-Random-Walk-CUDA/build/CMakeFiles/NetworkDemo.dir//./NetworkDemo_generated_main.cu.o.cubin.txt -P /home/eman/3D-Random-Walk-CUDA/build/CMakeFiles/NetworkDemo.dir//NetworkDemo_generated_main.cu.o.cmake

CMakeFiles/NetworkDemo.dir/NetworkDemo_intermediate_link.o: CMakeFiles/NetworkDemo.dir/NetworkDemo_generated_main.cu.o
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/eman/3D-Random-Walk-CUDA/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building NVCC intermediate link file CMakeFiles/NetworkDemo.dir/NetworkDemo_intermediate_link.o"
	/usr/bin/nvcc -m64 -ccbin /usr/bin/gcc -dlink /home/eman/3D-Random-Walk-CUDA/build/CMakeFiles/NetworkDemo.dir//./NetworkDemo_generated_main.cu.o -o /home/eman/3D-Random-Walk-CUDA/build/CMakeFiles/NetworkDemo.dir/./NetworkDemo_intermediate_link.o

CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o: CMakeFiles/NetworkDemo.dir/flags.make
CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o: ../Network/Sources/socket.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eman/3D-Random-Walk-CUDA/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o -c /home/eman/3D-Random-Walk-CUDA/Network/Sources/socket.cpp

CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/eman/3D-Random-Walk-CUDA/Network/Sources/socket.cpp > CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.i

CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/eman/3D-Random-Walk-CUDA/Network/Sources/socket.cpp -o CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.s

CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o.requires:

.PHONY : CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o.requires

CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o.provides: CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o.requires
	$(MAKE) -f CMakeFiles/NetworkDemo.dir/build.make CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o.provides.build
.PHONY : CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o.provides

CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o.provides.build: CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o


CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o: CMakeFiles/NetworkDemo.dir/flags.make
CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o: NetworkDemo_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eman/3D-Random-Walk-CUDA/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o -c /home/eman/3D-Random-Walk-CUDA/build/NetworkDemo_autogen/mocs_compilation.cpp

CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/eman/3D-Random-Walk-CUDA/build/NetworkDemo_autogen/mocs_compilation.cpp > CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.i

CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/eman/3D-Random-Walk-CUDA/build/NetworkDemo_autogen/mocs_compilation.cpp -o CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.s

CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o.requires:

.PHONY : CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o.requires

CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o.provides: CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o.requires
	$(MAKE) -f CMakeFiles/NetworkDemo.dir/build.make CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o.provides.build
.PHONY : CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o.provides

CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o.provides.build: CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o


# Object files for target NetworkDemo
NetworkDemo_OBJECTS = \
"CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o" \
"CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o"

# External object files for target NetworkDemo
NetworkDemo_EXTERNAL_OBJECTS = \
"/home/eman/3D-Random-Walk-CUDA/build/CMakeFiles/NetworkDemo.dir/NetworkDemo_generated_main.cu.o" \
"/home/eman/3D-Random-Walk-CUDA/build/CMakeFiles/NetworkDemo.dir/NetworkDemo_intermediate_link.o"

NetworkDemo: CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o
NetworkDemo: CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o
NetworkDemo: CMakeFiles/NetworkDemo.dir/NetworkDemo_generated_main.cu.o
NetworkDemo: CMakeFiles/NetworkDemo.dir/NetworkDemo_intermediate_link.o
NetworkDemo: CMakeFiles/NetworkDemo.dir/build.make
NetworkDemo: /usr/lib/x86_64-linux-gnu/libcudart_static.a
NetworkDemo: /usr/lib/x86_64-linux-gnu/librt.so
NetworkDemo: CMakeFiles/NetworkDemo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/eman/3D-Random-Walk-CUDA/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable NetworkDemo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/NetworkDemo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/NetworkDemo.dir/build: NetworkDemo

.PHONY : CMakeFiles/NetworkDemo.dir/build

CMakeFiles/NetworkDemo.dir/requires: CMakeFiles/NetworkDemo.dir/Network/Sources/socket.cpp.o.requires
CMakeFiles/NetworkDemo.dir/requires: CMakeFiles/NetworkDemo.dir/NetworkDemo_autogen/mocs_compilation.cpp.o.requires

.PHONY : CMakeFiles/NetworkDemo.dir/requires

CMakeFiles/NetworkDemo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/NetworkDemo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/NetworkDemo.dir/clean

CMakeFiles/NetworkDemo.dir/depend: CMakeFiles/NetworkDemo.dir/NetworkDemo_generated_main.cu.o
CMakeFiles/NetworkDemo.dir/depend: CMakeFiles/NetworkDemo.dir/NetworkDemo_intermediate_link.o
	cd /home/eman/3D-Random-Walk-CUDA/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eman/3D-Random-Walk-CUDA /home/eman/3D-Random-Walk-CUDA /home/eman/3D-Random-Walk-CUDA/build /home/eman/3D-Random-Walk-CUDA/build /home/eman/3D-Random-Walk-CUDA/build/CMakeFiles/NetworkDemo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/NetworkDemo.dir/depend

