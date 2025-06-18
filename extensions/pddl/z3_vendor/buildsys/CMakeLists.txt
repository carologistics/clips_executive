cmake_minimum_required(VERSION 3.8)
project(up_nextflap)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -Wextra -pedantic -fPIC")

# Check for pybind11
find_package(pybind11 CONFIG REQUIRED)
if(NOT pybind11_FOUND)
  message(
    FATAL_ERROR "pybind11 not found. Install it using 'pip install pybind11'.")
endif()

# Prompt user for Z3 library path
prompt_user(
  Z3_DIR
  "Please specify the path to Z3 (e.g., the directory containing include/ and lib/):"
)
set(Z3_INCLUDE_DIR "${Z3_DIR}/include")
set(Z3_LIB_DIR "${Z3_DIR}/lib")

if(NOT EXISTS "${Z3_INCLUDE_DIR}/z3++.h")
  message(
    FATAL_ERROR
      "z3++.h not found in ${Z3_INCLUDE_DIR}. Check your Z3 installation.")
endif()

# Detect Python include and library directories
find_package(Python3 REQUIRED COMPONENTS Development)
if(NOT Python3_FOUND)
  message(
    FATAL_ERROR
      "Python3 development package not found. Make sure Python headers and libraries are installed."
  )
endif()

# Source files
set(FILES
    nextflap.cpp
    parser/parser.cpp
    parser/syntaxAnalyzer.cpp
    parser/parsedTask.cpp
    preprocess/preprocess.cpp
    preprocess/preprocessedTask.cpp
    grounder/grounder.cpp
    grounder/groundedTask.cpp
    heuristics/evaluator.cpp
    heuristics/hFF.cpp
    heuristics/hLand.cpp
    heuristics/landmarks.cpp
    heuristics/numericRPG.cpp
    heuristics/rpg.cpp
    heuristics/temporalRPG.cpp
    planner/intervalCalculations.cpp
    planner/linearizer.cpp
    planner/plan.cpp
    planner/planBuilder.cpp
    planner/planComponents.cpp
    planner/planEffects.cpp
    planner/planner.cpp
    planner/plannerSetting.cpp
    planner/printPlan.cpp
    planner/selector.cpp
    planner/state.cpp
    planner/successors.cpp
    planner/z3Checker.cpp
    sas/mutexGraph.cpp
    sas/sasTask.cpp
    sas/sasTranslator.cpp
    utils/utils.cpp)

# Add the Python module target
pybind11_add_module(nextflap MODULE ${FILES})

# Include directories
include_directories(${Z3_INCLUDE_DIR} ${Python3_INCLUDE_DIRS}
                    ${pybind11_INCLUDE_DIRS})

# Link directories and libraries
target_link_directories(nextflap PRIVATE ${Z3_LIB_DIR})
target_link_libraries(nextflap PRIVATE ${Python3_LIBRARIES} pybind11::module)

if(UNIX AND NOT APPLE)
  # RPATH settings for Linux
  set_target_properties(nextflap PROPERTIES INSTALL_RPATH "$ORIGIN")
elseif(APPLE)
  # RPATH settings for macOS
  set_target_properties(nextflap PROPERTIES INSTALL_RPATH "@loader_path")
endif()

# Output
if(WIN32)
  set_target_properties(nextflap PROPERTIES SUFFIX ".pyd")
else()
  set_target_properties(nextflap PROPERTIES SUFFIX ".so")
endif()

message(STATUS "NextFLAP module configured. Run 'make' to build.")
