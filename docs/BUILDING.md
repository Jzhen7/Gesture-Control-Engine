# Building

## Requirements

| | Version | Notes |
|---|---|---|
| C++ compiler | C++17 | MSVC 2019+, GCC 9+, Clang 10+ |
| CMake | 3.16+ | |
| OpenCV | 4.7+ | modules: core, imgproc, imgcodecs, highgui, videoio, **dnn**. Only needed for the app; the core library and tests build without it |

## CMake options

| Option | Default | |
|---|---|---|
| `HT_BUILD_APP` | `ON` | Build the vision pipeline and `handtracker` executable (requires OpenCV) |
| `HT_BUILD_TESTS` | `ON` | Build `ht_tests` |
| `HT_WARNINGS_AS_ERRORS` | `OFF` | `-Werror` / `/WX` (on in CI) |

## Windows

1. Install Visual Studio 2022 with **Desktop development with C++**, and CMake
   (`winget install Kitware.CMake`).
2. Install OpenCV. Either:
   - **Prebuilt** (fastest): `choco install opencv`, or download the Windows
     release from <https://opencv.org/releases/> and extract it. The CMake
     config lives in `<opencv>\build`.
   - **vcpkg**: `vcpkg install opencv4[dnn]:x64-windows`, then add
     `-DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake` below.
3. Configure and build:
   ```powershell
   cmake -S . -B build -DOpenCV_DIR=C:/tools/opencv/build
   cmake --build build --config Release
   ```
4. Make OpenCV's DLLs findable. Either add `<opencv>\build\x64\vc16\bin` to
   `PATH` or copy `opencv_world*.dll` next to `build\Release\handtracker.exe`.
5. Download the models:
   ```powershell
   powershell -ExecutionPolicy Bypass -File scripts\download_models.ps1
   ```
6. Run `build\Release\handtracker.exe --dry-run`.

## Linux

```sh
sudo apt install build-essential cmake libopencv-dev   # needs OpenCV >= 4.7 (see below)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
scripts/download_models.sh
./build/handtracker --dry-run
```

> **Ubuntu 24.04 and older ship OpenCV 4.6 or earlier**, which can't import the
> models (an ONNX `Clip` node error). On those releases, build a minimal OpenCV
> from source and point CMake at it. CI does exactly this:
>
> ```sh
> curl -fsSL https://github.com/opencv/opencv/archive/refs/tags/4.10.0.tar.gz | tar xz
> cmake -S opencv-4.10.0 -B opencv-build -DCMAKE_BUILD_TYPE=Release \
>   -DCMAKE_INSTALL_PREFIX=$HOME/opencv-install \
>   -DBUILD_LIST=core,imgproc,imgcodecs,highgui,videoio,dnn \
>   -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF
> cmake --build opencv-build -j && cmake --install opencv-build
> cmake -S . -B build -DOpenCV_DIR=$HOME/opencv-install/lib/cmake/opencv4
> ```
>
> Install `libgtk-3-dev` first if you want the preview window.

Hand tracking, gestures and the UI work, but OS actions are logged rather than
performed until a native Linux backend lands (see the roadmap).

## macOS

```sh
brew install cmake opencv
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
scripts/download_models.sh
./build/handtracker --dry-run
```

Grant the terminal camera access when prompted. OS actions are logged only.

## Robot-arm front-end (MuJoCo)

The arm needs MuJoCo and the SO-101 model. Download both, then re-run CMake:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\download_arm.ps1   # Windows
```
```sh
scripts/download_arm.sh                                              # Linux
```

This puts MuJoCo 3.17.0 in `third_party/mujoco` and the model in
`models/so101` (both git-ignored). CMake detects MuJoCo automatically, builds
`handtracker arm`, copies `mujoco.dll` next to the executable on Windows, and
adds the headless simulation tests (`ht_arm_sim_tests`). Without MuJoCo,
everything else still builds. To use a MuJoCo installed elsewhere, pass
`-DMUJOCO_DIR=<path>`.

macOS: MuJoCo ships as a `.dmg` framework, which the script doesn't handle yet.

## Core library and tests only (no OpenCV)

```sh
cmake -S . -B build -DHT_BUILD_APP=OFF
cmake --build build
ctest --test-dir build --output-on-failure
```

## Tests

```sh
ctest --test-dir build -C Release --output-on-failure
./build/tests/ht_tests            # all tests
./build/tests/ht_tests temporal   # tests whose name contains "temporal"
```

With the app built, `handtracker --image <photo> --output out.png` runs the full
detection pipeline on a still image and prints the recognised gestures. CI uses
this as an end-to-end smoke test.
