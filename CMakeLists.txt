cmake_minimum_required(VERSION 3.16)

project(HandTracker
  VERSION 2.0.0
  DESCRIPTION "Real-time hand gesture control for the desktop"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(HT_BUILD_TESTS "Build the unit tests" ON)
option(HT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# ---------------------------------------------------------------------------
# Shared compiler warnings
# ---------------------------------------------------------------------------
add_library(ht_warnings INTERFACE)
if(MSVC)
  target_compile_options(ht_warnings INTERFACE /W4 /permissive- /utf-8)
  if(HT_WARNINGS_AS_ERRORS)
    target_compile_options(ht_warnings INTERFACE /WX)
  endif()
else()
  target_compile_options(ht_warnings INTERFACE -Wall -Wextra -Wpedantic -Wshadow)
  if(HT_WARNINGS_AS_ERRORS)
    target_compile_options(ht_warnings INTERFACE -Werror)
  endif()
endif()

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# ht_core: gesture logic, temporal detection, modes, config. No third-party deps.
# ---------------------------------------------------------------------------
add_library(ht_core STATIC
  src/core/actions.cpp
  src/core/async_system_control.cpp
  src/core/config.cpp
  src/core/controller.cpp
  src/core/filters.cpp
  src/core/gesture.cpp
  src/core/landmarks.cpp
  src/core/mapping.cpp
  src/core/mouse_controller.cpp
  src/core/orientation.cpp
  src/core/system_control.cpp
  src/core/temporal.cpp)
target_include_directories(ht_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ht_core PUBLIC Threads::Threads PRIVATE ht_warnings)
target_compile_definitions(ht_core PUBLIC HT_VERSION="${PROJECT_VERSION}")

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(HT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
