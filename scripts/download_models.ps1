# Downloads the MediaPipe hand models (ONNX exports from the OpenCV Model Zoo).
# Usage:  powershell -ExecutionPolicy Bypass -File scripts\download_models.ps1 [-Dest models]
param(
    [string]$Dest = (Join-Path $PSScriptRoot "..\models")
)

$ErrorActionPreference = "Stop"
$base = "https://github.com/opencv/opencv_zoo/raw/main/models"
$models = @(
    @{ Path = "palm_detection_mediapipe/palm_detection_mediapipe_2023feb.onnx"; MinBytes = 3000000 },
    @{ Path = "handpose_estimation_mediapipe/handpose_estimation_mediapipe_2023feb.onnx"; MinBytes = 3000000 }
)

New-Item -ItemType Directory -Force -Path $Dest | Out-Null
foreach ($m in $models) {
    $file = Join-Path $Dest (Split-Path $m.Path -Leaf)
    if ((Test-Path $file) -and ((Get-Item $file).Length -ge $m.MinBytes)) {
        Write-Host "ok        $file"
        continue
    }
    Write-Host "download  $($m.Path)"
    Invoke-WebRequest -Uri "$base/$($m.Path)" -OutFile $file -UseBasicParsing
    if ((Get-Item $file).Length -lt $m.MinBytes) {
        throw "Downloaded $file is too small - got a Git LFS pointer or an error page?"
    }
}
Write-Host "Models are in $(Resolve-Path $Dest)"
