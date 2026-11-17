set(HT_TEST_SOURCES
  test_main.cpp
  test_actions_config.cpp
  test_arm.cpp
  test_controller.cpp
  test_gesture.cpp
  test_misc.cpp
  test_temporal.cpp)

if(HT_BUILD_APP)
  list(APPEND HT_TEST_SOURCES test_vision.cpp)
endif()

add_executable(ht_tests ${HT_TEST_SOURCES})
target_link_libraries(ht_tests PRIVATE ht_core ht_arm ht_warnings)
if(HT_BUILD_APP)
  target_link_libraries(ht_tests PRIVATE ht_vision)
endif()
target_compile_definitions(ht_tests PRIVATE HT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
add_test(NAME ht_tests COMMAND ht_tests)

# Headless MuJoCo simulation tests (only when MuJoCo and the arm model are present).
set(HT_ARM_MODEL "${PROJECT_SOURCE_DIR}/models/so101/scene.xml")
if(HT_HAS_MUJOCO AND EXISTS "${HT_ARM_MODEL}")
  add_executable(ht_arm_sim_tests test_main.cpp test_arm_sim.cpp)
  target_link_libraries(ht_arm_sim_tests PRIVATE ht_arm_mujoco ht_warnings)
  target_compile_definitions(ht_arm_sim_tests PRIVATE HT_TEST_ARM_MODEL="${HT_ARM_MODEL}")
  ht_copy_mujoco_dll(ht_arm_sim_tests)
  add_test(NAME ht_arm_sim_tests COMMAND ht_arm_sim_tests)
elseif(HT_HAS_MUJOCO)
  message(STATUS "Arm model not found at ${HT_ARM_MODEL}: skipping simulation tests")
endif()
