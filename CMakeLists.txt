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

option(HT_BUILD_APP   "Build the camera application and vision pipeline (requires OpenCV)" ON)
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
# ht_platform: OS integration (volume, brightness, input injection, processes)
# ---------------------------------------------------------------------------
if(WIN32)
  add_library(ht_platform STATIC src/platform/win32_system_control.cpp)
  target_compile_definitions(ht_platform PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
  target_link_libraries(ht_platform PRIVATE ole32 oleaut32 dxva2 user32 shell32)
else()
  add_library(ht_platform STATIC src/platform/null_system_control.cpp)
endif()
target_link_libraries(ht_platform PUBLIC ht_core PRIVATE ht_warnings)

# ---------------------------------------------------------------------------
# Vision pipeline + application (OpenCV)
# ---------------------------------------------------------------------------
if(HT_BUILD_APP)
  find_package(OpenCV 4.7 REQUIRED COMPONENTS core imgproc imgcodecs highgui videoio dnn)
  message(STATUS "Using OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")

  add_library(ht_vision STATIC
    src/vision/hand_landmarker.cpp
    src/vision/hand_tracker.cpp
    src/vision/palm_detector.cpp
    src/ui/overlay.cpp)
  # SYSTEM: don't let warnings inside OpenCV's headers fail -Werror builds.
  target_include_directories(ht_vision SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(ht_vision PUBLIC ht_core ${OpenCV_LIBS} PRIVATE ht_warnings)

  add_executable(handtracker
    apps/main.cpp
    apps/app_runner.cpp
    apps/control_app.cpp
    apps/mouse_app.cpp
    apps/orientation_app.cpp)
  target_link_libraries(handtracker PRIVATE ht_vision ht_platform ht_warnings)

  # Ship the default config next to the binary so it runs from the build tree.
  add_custom_command(TARGET handtracker POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:handtracker>/config
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/config/handtracker.ini
            $<TARGET_FILE_DIR:handtracker>/config/handtracker.ini)

  install(TARGETS handtracker RUNTIME DESTINATION bin)
  install(FILES config/handtracker.ini DESTINATION bin/config)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(HT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
