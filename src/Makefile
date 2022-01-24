# Configuration Options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Set platform, debug mode, and address sanitizer if not already set.
platform ?= Ubuntu
debug    ?= 1
asan     ?= 0

#! Important note about ASAN: you will also need to set these environment
#  variables to get useful output:
#  export ASAN_SYMBOLIZER_PATH="/usr/bin/llvm-symbolizer"
#  export ASAN_OPTIONS="symbolize=1"
#! You may instead want to set this option since the lab machines' version of
#  ASAN has at least one malloc which is later deleted instead of freed (thereby
#  terminating your program with a bogus error):
#  export ASAN_OPTIONS="symbolize=1:alloc_dealloc_mismatch=0"


# Temporary setting for making vizmo-compatible maps for nonholonomic robots.
vizmo_map := -DVIZMO_MAP

# Temporary setting for making vizmo-compatible maps for robot groups.
group_map := #-DGROUP_MAP
ifeq ($(group_map), -DGROUP_MAP)
  vizmo_map ?= -DVIZMO_MAP
endif

# Define robot type if it isn't already set.
ROBOT_DEF ?= PMPCfg

# Build support for iCreate hardware?
icreate ?= 0

# Build support for aruco detector hardware?
aruco ?= 0

# Compile with matlab support?
matlab ?= 0

# Compile the parallel version of PMPL? This is very old and has not been
# maintained during the simulator refactoring (too many things to manage at
# once). At this point it likely needs to be rebuilt entirely using the latest
# work from Matthew Bulluck's parallel branch.
parallel ?= 0

# Directory Layout ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

PMPL_DIR  := .
UTILS_DIR := ../pmpl_utils

# The OBJ_DIR is the landing zone for our object files. Create it with a
# directory structure to match our source tree.
OBJ_DIR := build
MOC_DIR := $(OBJ_DIR)/moc

# Treat all sub-directories except the build tree as possible source locations.
SRC_DIRS  := $(patsubst ./%,%, $(shell find . -type d \
               ! \( -path "./$(OBJ_DIR)*" -o -name '.' \) ))

# Always try to rebuild the build directory to catch newly added dirs.
#.PHONY: $(OBJ_DIR)
$(OBJ_DIR):
	@mkdir -p $@ $(patsubst %,$(OBJ_DIR)/%,$(SRC_DIRS))
$(MOC_DIR): | $(OBJ_DIR)
	@mkdir $@

# Look for files in the source directories.
vpath %.h       $(SRC_DIRS)
vpath %.cpp     $(SRC_DIRS)
vpath moc_%.cpp $(MOC_DIR)

# External Makefiles ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

include $(UTILS_DIR)/makefile_includes/Makefile.defaults
include $(PMPL_DIR)/Makefile.PMPLdefaults

# Compiler Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

DEFS = $(PMPL_DEF) $(UTILS_DEF) $(SIM_DEF) $(PLANNER_DEF) $(TASK_DEF) $(vizmo_map) $(group_map) $(TEST_DEF)
INCL = $(PMPL_INCL) $(UTILS_INCL) $(STAPL_INCL) $(BOOST_INCL) $(SIM_INCL) $(TEST_INCL)
LIBS = $(PMPL_LIB) $(UTILS_LIB) $(STAPL_LIB) $(BOOST_LIB) $(SIM_LIBS) $(TEST_LIBS)

# Object File Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Pre-pend the build directory to the object file names.
PMPL_OBJS     := $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(PMPL_SRCS))
PPL_OBJS  		:= $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(PPL_SRCS))
SIM_OBJS      := $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(SIM_SRCS))
COMM_OBJS     := $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(COMM_SRCS))
PLANNER_OBJS  := $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(PLANNER_SRCS))
TASK_OBJS     := $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(TASK_SRCS))
CLIENT_OBJS   := $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(CLIENT_SRCS))
MASTER_OBJS   := $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(MASTER_SRCS))
HARDWARE_OBJS := $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(HARDWARE_SRCS))
TEST_OBJS     := $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(TEST_SRCS))

# Make the object files depend on updated utility libraries so the libs are
# built first.
$(PMPL_OBJS):     $(UTILS_LIBFILE) $(STAPL_LIBFILE)
$(TASK_OBJS):     $(UTILS_LIBFILE) $(STAPL_LIBFILE)
$(SIM_OBJS):      $(UTILS_LIBFILE) $(STAPL_LIBFILE)
$(COMM_OBJS):     $(UTILS_LIBFILE) $(STAPL_LIBFILE)
$(PLANNER_OBJS):  $(UTILS_LIBFILE) $(STAPL_LIBFILE)
$(CLIENT_OBJS):   $(UTILS_LIBFILE) $(STAPL_LIBFILE)
$(HARDWARE_OBJS): $(UTILS_LIBFILE) $(STAPL_LIBFILE)
$(TEST_OBJS):     $(UTILS_LIBFILE) $(STAPL_LIBFILE)

# Set which object defines the main function.
ifeq ($(parallel), 1)
  MAIN := $(OBJ_DIR)/parallel_main.o
else
  MAIN := $(OBJ_DIR)/main.o
endif

# Dependency Tracking ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

DEPS_OUT = $(OBJ_DIR)/$*.d
DEPS     = -MMD -MT $(OBJ_DIR)/$*.o -MF $(DEPS_OUT)

DEPENDENCY_FILE := $(OBJ_DIR)/Dependencies
-include $(DEPENDENCY_FILE)

# Library Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Default is to build the library object.
.DEFAULT_GOAL := $(PMPL_LIBFILE)

$(PMPL_LIBFILE): $(PMPL_OBJS)
	@echo Linking pmpl library...
	@$(AR) $@ $^

# Executable Recipes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Build the traditional pmpl executable.
pmpl: $(MAIN) $(HARDWARE_OBJS) $(PMPL_LIBFILE)
	@echo Linking $@...
	@$(CXX) $(CXXFLAGS) $(OPTS) $(MAIN) $(HARDWARE_OBJS) $(LIBS) -o $@

# Build the tmp executable.
ppl: $(PPL_OBJS) $(TASK_OBJS) $(COMM_OBJS) $(HARDWARE_OBJS) $(PMPL_LIBFILE) $(MOC_OBJS) \
     | $(patsubst %,$(MOC_DIR)/%,$(MOC_SRCS))
	@echo Linking $@...
	@$(CXX) $(CXXFLAGS) $(OPTS) $(PPL_OBJS) $(TASK_OBJS) $(COMM_OBJS) $(MOC_OBJS) $(HARDWARE_OBJS) $(LIBS) -o $@

# Build the simulator.
sim: $(PMPL_LIBFILE) $(SIM_OBJS) $(TASK_OBJS) $(COMM_OBJS) $(MOC_OBJS) $(HARDWARE_OBJS) \
     | $(patsubst %,$(MOC_DIR)/%,$(MOC_SRCS))
	@echo Linking $@...
	@$(CXX) $(CXXFLAGS) $(OPTS) $(SIM_OBJS) $(TASK_OBJS) $(COMM_OBJS) $(MOC_OBJS) $(HARDWARE_OBJS) \
	  $(LIBS) -o $@

# Build the server.
planner: $(PLANNER_OBJS) $(TASK_OBJS) $(COMM_OBJS) $(HARDWARE_OBJS) $(PMPL_LIBFILE) $(MOC_OBJS) \
     | $(patsubst %,$(MOC_DIR)/%,$(MOC_SRCS))

	@echo Linking $@...
	@$(CXX) $(CXXFLAGS) $(OPTS) $(PLANNER_OBJS) $(TASK_OBJS) $(COMM_OBJS) $(MOC_OBJS) $(HARDWARE_OBJS) $(LIBS) -o $@

client: $(CLIENT_OBJS) $(COMM_OBJS) $(HARDWARE_OBJS) $(PMPL_LIBFILE)
	@echo Linking $@...
	@$(CXX) $(CXXFLAGS) $(OPTS) $(CLIENT_OBJS) $(COMM_OBJS) $(HARDWARE_OBJS) $(LIBS) -o $@

master: $(MASTER_OBJS) $(COMM_OBJS) $(PMPL_LIBFILE)
	@echo Linking $@...
	@$(CXX) $(CXXFLAGS) $(OPTS) $(MASTER_OBJS) $(COMM_OBJS) $(LIBS) -o $@

tests: $(TEST_OBJS) $(TASK_OBJS) $(COMM_OBJS) $(HARDWARE_OBJS) $(PMPL_LIBFILE) $(MOC_OBJS) \
     | $(patsubst %,$(MOC_DIR)/%,$(MOC_SRCS))
	@echo Linking $@...
	@$(CXX) $(CXXFLAGS) $(OPTS) $(TEST_OBJS) $(TASK_OBJS) $(COMM_OBJS) $(MOC_OBJS) $(HARDWARE_OBJS) $(LIBS) -o $@


# Object File Recipes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# This recipe runs headers with Q_OBJECT through the qt MOC.
#@mkdir -p $(MOC_DIR)/moc_$(shell dirname $<) #needed for MACOS support
$(MOC_DIR)/moc_%.cpp: %.h | $(MOC_DIR)
	@echo Moc\'ing $(shell echo $< | sed 's|.*/||' )...
	@$(QT_MOC) $< -o $@

# This recipe is for MOC sources.
$(MOC_DIR)/moc_%.o: $(MOC_DIR)/moc_%.cpp %.h | $(MOC_DIR)
	@echo Compiling Moc\'d source $(shell echo $< | sed 's|.*/||' )...
	@$(CXX) -c $(CXXFLAGS) $(OPTS) $(DEFS) $(INCL) $(DEPS) $< -o $@
	@cat $(DEPS_OUT) >> $(DEPENDENCY_FILE)
	@rm -f $(DEPS_OUT)

# This recipe is for regular objects.
$(OBJ_DIR)/%.o: %.cpp | $(OBJ_DIR)
	@$(MAKE) -s check_platform
	@echo Compiling $<...
	@$(CXX) -c $(CXXFLAGS) $(OPTS) $(DEFS) $(INCL) $(DEPS) $< -o $@
	@cat $(DEPS_OUT) >> $(DEPENDENCY_FILE)
	@rm -f $(DEPS_OUT)

# Cleanup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

.PHONY: clean
clean:
	@echo Cleaning pmpl library and executables...
	@rm -f $(PMPL_LIBFILE) $(MAIN) pmpl sim planner ppl master tests

.PHONY: reallyclean
reallyclean: clean
	@echo Cleaning all pmpl objects...
	@rm -rf $(OBJ_DIR)

.PHONY: reallyreallyclean
reallyreallyclean: reallyclean
	@echo Cleaning all utilities...
	@cd $(CD_DIR) && $(MAKE) clean
	@cd $(TINYXML_DIR) && $(MAKE) clean
	@cd $(MPNN_SRC) && $(MAKE) clean
	@cd $(MODELLOADER_DIR) && $(MAKE) clean
	@cd $(KMEANS_DIR) && $(MAKE) clean
	@cd $(TETGEN_DIR) && $(MAKE) clean
	@cd $(SANDBOX_DIR) && $(MAKE) clean
	@cd $(BULLET_DIR) && $(MAKE) clean
	@cd $(PHYSICAL_ROBOTS_DIR) && $(MAKE) clean
	@cd $(LKH_DIR) && $(MAKE) clean
ifneq ("$(CGAL_DIR)","")
	@cd $(CGAL_DIR) && $(MAKE) clean
endif
ifeq ($(parallel), 1)
	@cd $(STAPL_DIR) && $(MAKE) platform=$(platform) stl=$(STL_LIB) clean
endif

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
