# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:

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
CMAKE_SOURCE_DIR = /home/feixiao/workspace/ivoldual/isodual

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/feixiao/workspace/ivoldual/isodual

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/bin/ccmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/feixiao/workspace/ivoldual/isodual/CMakeFiles /home/feixiao/workspace/ivoldual/isodual/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/feixiao/workspace/ivoldual/isodual/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named doc

# Build rule for target.
doc: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 doc
.PHONY : doc

# fast build rule for target.
doc/fast:
	$(MAKE) -f CMakeFiles/doc.dir/build.make CMakeFiles/doc.dir/build
.PHONY : doc/fast

#=============================================================================
# Target rules for targets named isodual

# Build rule for target.
isodual: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 isodual
.PHONY : isodual

# fast build rule for target.
isodual/fast:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/build
.PHONY : isodual/fast

#=============================================================================
# Target rules for targets named tar

# Build rule for target.
tar: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 tar
.PHONY : tar

# fast build rule for target.
tar/fast:
	$(MAKE) -f CMakeFiles/tar.dir/build.make CMakeFiles/tar.dir/build
.PHONY : tar/fast

ijkdual_datastruct.o: ijkdual_datastruct.cxx.o
.PHONY : ijkdual_datastruct.o

# target to build an object file
ijkdual_datastruct.cxx.o:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/ijkdual_datastruct.cxx.o
.PHONY : ijkdual_datastruct.cxx.o

ijkdual_datastruct.i: ijkdual_datastruct.cxx.i
.PHONY : ijkdual_datastruct.i

# target to preprocess a source file
ijkdual_datastruct.cxx.i:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/ijkdual_datastruct.cxx.i
.PHONY : ijkdual_datastruct.cxx.i

ijkdual_datastruct.s: ijkdual_datastruct.cxx.s
.PHONY : ijkdual_datastruct.s

# target to generate assembly for a file
ijkdual_datastruct.cxx.s:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/ijkdual_datastruct.cxx.s
.PHONY : ijkdual_datastruct.cxx.s

ijkdualtable.o: ijkdualtable.cxx.o
.PHONY : ijkdualtable.o

# target to build an object file
ijkdualtable.cxx.o:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/ijkdualtable.cxx.o
.PHONY : ijkdualtable.cxx.o

ijkdualtable.i: ijkdualtable.cxx.i
.PHONY : ijkdualtable.i

# target to preprocess a source file
ijkdualtable.cxx.i:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/ijkdualtable.cxx.i
.PHONY : ijkdualtable.cxx.i

ijkdualtable.s: ijkdualtable.cxx.s
.PHONY : ijkdualtable.s

# target to generate assembly for a file
ijkdualtable.cxx.s:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/ijkdualtable.cxx.s
.PHONY : ijkdualtable.cxx.s

isodual.o: isodual.cxx.o
.PHONY : isodual.o

# target to build an object file
isodual.cxx.o:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/isodual.cxx.o
.PHONY : isodual.cxx.o

isodual.i: isodual.cxx.i
.PHONY : isodual.i

# target to preprocess a source file
isodual.cxx.i:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/isodual.cxx.i
.PHONY : isodual.cxx.i

isodual.s: isodual.cxx.s
.PHONY : isodual.s

# target to generate assembly for a file
isodual.cxx.s:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/isodual.cxx.s
.PHONY : isodual.cxx.s

isodualIO.o: isodualIO.cxx.o
.PHONY : isodualIO.o

# target to build an object file
isodualIO.cxx.o:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/isodualIO.cxx.o
.PHONY : isodualIO.cxx.o

isodualIO.i: isodualIO.cxx.i
.PHONY : isodualIO.i

# target to preprocess a source file
isodualIO.cxx.i:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/isodualIO.cxx.i
.PHONY : isodualIO.cxx.i

isodualIO.s: isodualIO.cxx.s
.PHONY : isodualIO.s

# target to generate assembly for a file
isodualIO.cxx.s:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/isodualIO.cxx.s
.PHONY : isodualIO.cxx.s

isodual_main.o: isodual_main.cxx.o
.PHONY : isodual_main.o

# target to build an object file
isodual_main.cxx.o:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/isodual_main.cxx.o
.PHONY : isodual_main.cxx.o

isodual_main.i: isodual_main.cxx.i
.PHONY : isodual_main.i

# target to preprocess a source file
isodual_main.cxx.i:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/isodual_main.cxx.i
.PHONY : isodual_main.cxx.i

isodual_main.s: isodual_main.cxx.s
.PHONY : isodual_main.s

# target to generate assembly for a file
isodual_main.cxx.s:
	$(MAKE) -f CMakeFiles/isodual.dir/build.make CMakeFiles/isodual.dir/isodual_main.cxx.s
.PHONY : isodual_main.cxx.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... tar"
	@echo "... isodual"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... doc"
	@echo "... ijkdual_datastruct.o"
	@echo "... ijkdual_datastruct.i"
	@echo "... ijkdual_datastruct.s"
	@echo "... ijkdualtable.o"
	@echo "... ijkdualtable.i"
	@echo "... ijkdualtable.s"
	@echo "... isodual.o"
	@echo "... isodual.i"
	@echo "... isodual.s"
	@echo "... isodualIO.o"
	@echo "... isodualIO.i"
	@echo "... isodualIO.s"
	@echo "... isodual_main.o"
	@echo "... isodual_main.i"
	@echo "... isodual_main.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

