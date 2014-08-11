# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/huagc/SIFTICGM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/huagc/SIFTICGM

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/bin/cmake-gui -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
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
	$(CMAKE_COMMAND) -E cmake_progress_start /home/huagc/SIFTICGM/CMakeFiles /home/huagc/SIFTICGM/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/huagc/SIFTICGM/CMakeFiles 0
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
# Target rules for targets named SIFTICGM

# Build rule for target.
SIFTICGM: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 SIFTICGM
.PHONY : SIFTICGM

# fast build rule for target.
SIFTICGM/fast:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/build
.PHONY : SIFTICGM/fast

imgfeatures.o: imgfeatures.c.o
.PHONY : imgfeatures.o

# target to build an object file
imgfeatures.c.o:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/imgfeatures.c.o
.PHONY : imgfeatures.c.o

imgfeatures.i: imgfeatures.c.i
.PHONY : imgfeatures.i

# target to preprocess a source file
imgfeatures.c.i:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/imgfeatures.c.i
.PHONY : imgfeatures.c.i

imgfeatures.s: imgfeatures.c.s
.PHONY : imgfeatures.s

# target to generate assembly for a file
imgfeatures.c.s:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/imgfeatures.c.s
.PHONY : imgfeatures.c.s

kdtree.o: kdtree.c.o
.PHONY : kdtree.o

# target to build an object file
kdtree.c.o:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/kdtree.c.o
.PHONY : kdtree.c.o

kdtree.i: kdtree.c.i
.PHONY : kdtree.i

# target to preprocess a source file
kdtree.c.i:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/kdtree.c.i
.PHONY : kdtree.c.i

kdtree.s: kdtree.c.s
.PHONY : kdtree.s

# target to generate assembly for a file
kdtree.c.s:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/kdtree.c.s
.PHONY : kdtree.c.s

main.o: main.cpp.o
.PHONY : main.o

# target to build an object file
main.cpp.o:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/main.cpp.o
.PHONY : main.cpp.o

main.i: main.cpp.i
.PHONY : main.i

# target to preprocess a source file
main.cpp.i:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/main.cpp.i
.PHONY : main.cpp.i

main.s: main.cpp.s
.PHONY : main.s

# target to generate assembly for a file
main.cpp.s:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/main.cpp.s
.PHONY : main.cpp.s

minpq.o: minpq.c.o
.PHONY : minpq.o

# target to build an object file
minpq.c.o:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/minpq.c.o
.PHONY : minpq.c.o

minpq.i: minpq.c.i
.PHONY : minpq.i

# target to preprocess a source file
minpq.c.i:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/minpq.c.i
.PHONY : minpq.c.i

minpq.s: minpq.c.s
.PHONY : minpq.s

# target to generate assembly for a file
minpq.c.s:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/minpq.c.s
.PHONY : minpq.c.s

sift.o: sift.c.o
.PHONY : sift.o

# target to build an object file
sift.c.o:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/sift.c.o
.PHONY : sift.c.o

sift.i: sift.c.i
.PHONY : sift.i

# target to preprocess a source file
sift.c.i:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/sift.c.i
.PHONY : sift.c.i

sift.s: sift.c.s
.PHONY : sift.s

# target to generate assembly for a file
sift.c.s:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/sift.c.s
.PHONY : sift.c.s

utils.o: utils.c.o
.PHONY : utils.o

# target to build an object file
utils.c.o:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/utils.c.o
.PHONY : utils.c.o

utils.i: utils.c.i
.PHONY : utils.i

# target to preprocess a source file
utils.c.i:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/utils.c.i
.PHONY : utils.c.i

utils.s: utils.c.s
.PHONY : utils.s

# target to generate assembly for a file
utils.c.s:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/utils.c.s
.PHONY : utils.c.s

xform.o: xform.c.o
.PHONY : xform.o

# target to build an object file
xform.c.o:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/xform.c.o
.PHONY : xform.c.o

xform.i: xform.c.i
.PHONY : xform.i

# target to preprocess a source file
xform.c.i:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/xform.c.i
.PHONY : xform.c.i

xform.s: xform.c.s
.PHONY : xform.s

# target to generate assembly for a file
xform.c.s:
	$(MAKE) -f CMakeFiles/SIFTICGM.dir/build.make CMakeFiles/SIFTICGM.dir/xform.c.s
.PHONY : xform.c.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... SIFTICGM"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... imgfeatures.o"
	@echo "... imgfeatures.i"
	@echo "... imgfeatures.s"
	@echo "... kdtree.o"
	@echo "... kdtree.i"
	@echo "... kdtree.s"
	@echo "... main.o"
	@echo "... main.i"
	@echo "... main.s"
	@echo "... minpq.o"
	@echo "... minpq.i"
	@echo "... minpq.s"
	@echo "... sift.o"
	@echo "... sift.i"
	@echo "... sift.s"
	@echo "... utils.o"
	@echo "... utils.i"
	@echo "... utils.s"
	@echo "... xform.o"
	@echo "... xform.i"
	@echo "... xform.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

