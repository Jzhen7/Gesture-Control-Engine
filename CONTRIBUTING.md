# Contributing

## Workflow

1. Branch from `main`: `feature/<topic>`, `fix/<topic>` or `docs/<topic>`.
2. Keep commits focused. Messages follow
   [Conventional Commits](https://www.conventionalcommits.org/):
   `feat(core): add scroll gesture`, `fix(win32): release mouse on exit`,
   `docs: clarify volume dial`. The body explains *why*.
3. Open a pull request. CI must pass (unit tests on Windows/Linux/macOS, plus
   full OpenCV builds and an end-to-end image smoke test).

## Code style

- C++17, formatted with the repo's `.clang-format` (Google-based, 100 columns).
- `ht_core` must not depend on OpenCV or OS headers. If logic can be tested
  without a camera, it belongs in core.
- Anything that touches the OS goes through `ht::SystemControl`.
- New gestures, events, actions and config keys need unit tests
  (`tests/hand_builder.hpp` generates synthetic hands) and a docs update.

## Running tests

```sh
cmake -S . -B build -DHT_BUILD_APP=OFF   # no OpenCV needed
cmake --build build
ctest --test-dir build --output-on-failure
```
