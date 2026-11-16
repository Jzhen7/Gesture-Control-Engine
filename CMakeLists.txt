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
  src/core/hand_space.cpp
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
# ht_arm: robot-arm front-end logic (hand -> workspace target, gripper rule).
# No third-party deps, so it is always built and tested.
# ---------------------------------------------------------------------------
add_library(ht_arm STATIC
  src/arm/arm_controller.cpp
  src/arm/workspace.cpp)
target_link_libraries(ht_arm PUBLIC ht_core PRIVATE ht_warnings)

# ---------------------------------------------------------------------------
# ht_arm_mujoco: simulated SO-101 arm. Built automatically when MuJoCo is
# present (scripts/download_arm.* puts it in third_party/mujoco).
# ---------------------------------------------------------------------------
set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco" CACHE PATH
    "MuJoCo release folder (contains include/ and lib/)")
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS "${MUJOCO_DIR}/include" NO_DEFAULT_PATH)
find_library(MUJOCO_LIBRARY NAMES mujoco PATHS "${MUJOCO_DIR}/lib" NO_DEFAULT_PATH)
set(HT_HAS_MUJOCO OFF)
if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  set(HT_HAS_MUJOCO ON)
  add_library(mujoco::mujoco SHARED IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
  if(WIN32)
    set(MUJOCO_DLL "${MUJOCO_DIR}/bin/mujoco.dll")
    set_target_properties(mujoco::mujoco PROPERTIES
      IMPORTED_IMPLIB "${MUJOCO_LIBRARY}" IMPORTED_LOCATION "${MUJOCO_DLL}")
  else()
    set_target_properties(mujoco::mujoco PROPERTIES IMPORTED_LOCATION "${MUJOCO_LIBRARY}")
  endif()

  add_library(ht_arm_mujoco STATIC src/arm/mujoco_arm.cpp)
  target_link_libraries(ht_arm_mujoco PUBLIC ht_arm mujoco::mujoco PRIVATE ht_warnings)
  message(STATUS "MuJoCo found in ${MUJOCO_DIR}: building the simulated arm")
else()
  message(STATUS "MuJoCo not found in ${MUJOCO_DIR}: skipping the simulated arm "
                 "(run scripts/download_arm.ps1 or .sh to enable it)")
endif()

# Windows finds DLLs next to the .exe; copy mujoco.dll there.
function(ht_copy_mujoco_dll target)
  if(HT_HAS_MUJOCO AND WIN32)
    add_custom_command(TARGET ${target} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${MUJOCO_DLL}" $<TARGET_FILE_DIR:${target}>)
  endif()
endfunction()

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
  if(HT_HAS_MUJOCO)
    target_sources(handtracker PRIVATE apps/arm_app.cpp apps/arm_view.cpp)
    target_link_libraries(handtracker PRIVATE ht_arm_mujoco)
    target_compile_definitions(handtracker PRIVATE HT_HAS_ARM=1)
    ht_copy_mujoco_dll(handtracker)
  endif()

  # Ship the default configs next to the binary so it runs from the build tree.
  add_custom_command(TARGET handtracker POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:handtracker>/config
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/config/handtracker.ini
            ${CMAKE_CURRENT_SOURCE_DIR}/config/arm.ini
            $<TARGET_FILE_DIR:handtracker>/config/)

  # Windows: also copy OpenCV's DLLs next to the .exe, so a prebuilt OpenCV
  # works without editing PATH. The `if` keeps an empty list (e.g. a static
  # OpenCV) from becoming a copy command with no files.
  if(WIN32 AND NOT CMAKE_VERSION VERSION_LESS 3.21)
    add_custom_command(TARGET handtracker POST_BUILD
      COMMAND "$<$<BOOL:$<TARGET_RUNTIME_DLLS:handtracker>>:${CMAKE_COMMAND};-E;copy_if_different;$<TARGET_RUNTIME_DLLS:handtracker>;$<TARGET_FILE_DIR:handtracker>>"
      COMMAND_EXPAND_LISTS)
  endif()

  install(TARGETS handtracker RUNTIME DESTINATION bin)
  install(FILES config/handtracker.ini config/arm.ini DESTINATION bin/config)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(HT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
