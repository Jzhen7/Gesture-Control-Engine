# Models

The `.onnx` files are not committed. Download them with:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\download_models.ps1   # Windows
```
```sh
scripts/download_models.sh                                              # Linux / macOS
```

| File | Purpose | Input | Source |
|---|---|---|---|
| `palm_detection_mediapipe_2023feb.onnx` | Finds palms in the full frame (SSD, 2016 anchors) | 1×192×192×3 RGB | [OpenCV Zoo](https://github.com/opencv/opencv_zoo/tree/main/models/palm_detection_mediapipe) |
| `handpose_estimation_mediapipe_2023feb.onnx` | 21 hand landmarks, handedness, confidence | 1×224×224×3 RGB | [OpenCV Zoo](https://github.com/opencv/opencv_zoo/tree/main/models/handpose_estimation_mediapipe) |

Both models are Google's MediaPipe hand models converted to ONNX by the
OpenCV Model Zoo and distributed under the Apache 2.0 license.
