#include "test_framework.hpp"

// Usage: ht_tests [substring-filter]
int main(int argc, char** argv) { return httest::runAll(argc > 1 ? argv[1] : nullptr); }
