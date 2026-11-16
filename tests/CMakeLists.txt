set(HT_TEST_SOURCES
  test_main.cpp
  test_actions_config.cpp
  test_controller.cpp
  test_gesture.cpp
  test_misc.cpp
  test_temporal.cpp)

if(HT_BUILD_APP)
  list(APPEND HT_TEST_SOURCES test_vision.cpp)
endif()

add_executable(ht_tests ${HT_TEST_SOURCES})
target_link_libraries(ht_tests PRIVATE ht_core ht_warnings)
if(HT_BUILD_APP)
  target_link_libraries(ht_tests PRIVATE ht_vision)
endif()

add_test(NAME ht_tests COMMAND ht_tests)
