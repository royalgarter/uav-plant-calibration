# ARCHITECTURE.md

## Overview

This project is a UAV plant-image calibration tool. The core binary (`calib`) is a C++17 CLI that processes multispectral drone imagery through a multi-stage pipeline. It ships in three forms: a Linux CLI, a Windows CLI/GUI, and a Node.js web UI that spawns the CLI as a subprocess.

## Build System

All builds are driven by `./run.sh`. There is no Make or CMake. Run `./run.sh` with no args to list all targets.

### Build Targets

| Target | Output | Description |
|---|---|---|
| `build:calib` | `./calib` | Linux CLI — main tool |
| `build:calib-win` | `window_build/calib.exe` | Windows CLI (cross-compiled with MinGW) |
| `build:calib-win-gui` | `window_build/calib-wingui.exe` | Windows GUI (raylib/raygui) |
| `build:fisheye` | `./fisheye` | Fisheye calibration tool (Linux) |
| `build:fisheye-win` | `window_build/fisheye` | Fisheye calibration tool (Windows) |

### Compiler Flags

- C++17, OpenMP (`-fopenmp`).
- Linux: OpenCV 5 via `pkg-config`, libtiff from homebrew (`/home/linuxbrew/.linuxbrew`).
- Windows: hardcoded OpenCV 4.5.5 MinGW libs at `c:/opencv`.
- GUI builds add `-DWINGUI`, raylib, raygui, and Windows system libs (`-lopengl32 -lgdi32 -lwinmm`, etc.).

**Critical:** Linux and Windows use different OpenCV majors. Flags are not interchangeable; always build via `run.sh`.

## Source Layout

```
src/
  calib.cc            Main entrypoint (main), arg parsing, orchestration
  calib-func.cc       Core implementation (all processing functions)
  calib.h             Shared structs, prototypes, globals (gLog, gConfig)
  calib-raygui.cc     raylib/raygui GUI (Windows only, #include under WINGUI)
  calib-gui.cc        Deprecated Win32 GUI (native controls, not built by run.sh)
  wingui.cc           Fisheye tool Windows GUI (included by cli.cc under WINGUI)
  cli.cc              Fisheye tool entrypoint (not used by calib)
  calib-web.cc        Deprecated native C web server (mongoose)
  mongoose.c/.h       Vendored mongoose (deprecated, reference only)
  cJSON.c/.h          Vendored cJSON (deprecated, reference only)
  raygui.h            Vendored raygui header

webui/
  server.js           Node.js HTTP server, spawns ./calib as subprocess
  sea-config.json     Node SEA (Single Executable App) config
```

## Build Matrix

Each `run.sh` target compiles specific source combinations:

| Target | Sources | Defines |
|---|---|---|
| `build:calib` | `calib.cc` + `calib-func.cc` | — |
| `build:calib-win` | `calib.cc` + `calib-func.cc` | — |
| `build:calib-win-gui` | `calib.cc` + `calib-func.cc` | `-DWINGUI` |
| `build:fisheye` | `cli.cc` | — |
| `build:fisheye-win` | `cli.cc` | — |

## Processing Pipeline

The `calib` binary processes a directory of multispectral drone images (TIFF/JPG) through these stages:

### Stage 1: Metadata Parse
- `parseMetadata()` reads XMP from TIFF (tag 700) or JPEG (APP1 marker).
- Extracts DJI drone metadata: distortion coefficients (`DewarpData`), alignment homography (`DewarpHMatrix`), relative optical center.
- Groups images by filename prefix (e.g., `DJI_0223` → bands `DJI_02230.tif` through `DJI_02235.tif`).

### Stage 2: Radiometric Calibration (optional, `--radio`)
- Loads reference values from CSV (`radiometric_reference.csv`).
- Detects the radiometric board in the reference image (template match or 2-point click or auto-detect with `--auto`).
- Collects DN (digital number) values from all images aligned to reference geometry.
- Fits per-channel linear calibration coefficients (a, b) via least-squares.
- Exports calibration data to CSV.

### Stage 3: Per-Image Alignment
Each image in every group is processed in parallel (`#pragma omp parallel for`):

- **Step A — Dewarp:** `undistortImg()` applies the DJI radial distortion model (fx, fy, cx, cy, k1, k2, p1, p2, k3).
- **Step B — Metadata Alignment:** Applies the DJI homography matrix or relative translation from XMP.
- **Step C — Fine Alignment (optional):**
  - Primary: ECC (Enhanced Correlation Coefficient) optimization with optional downscaling.
  - Fallback: SIFT feature matching + RANSAC homography.
  - Composes: `H_total = H_meta_ref * H_ecc`.
- Saves to `output/alignment/`.

### Stage 4: Vegetation Index Calculation
- Builds aligned band map (band 0=RGB, 1-5=multispectral).
- Computes green mask via HSV filtering (default) or custom color spots (`--color-mask`).
- Green mask refinement pipeline: median blur → morphological open/close → area filtering → solidity filtering.
- Optional: spectral K-Means refinement (`--kmeans`), spatial cluster filtering (`--spatial`).
- Calculates requested vegetation index (default: NDVI; 28+ indices supported).
- Saves raw float, contrast-stretched, and colorized index images.
- Exports per-group averages to CSV.

## GUI Architecture

### Windows GUI (`--gui` flag, requires `-DWINGUI`)

Two implementations exist in the tree, but only the raygui version is built by `run.sh`:

1. **`calib-raygui.cc`** (active) — Immediate-mode GUI using raylib + raygui. Pulled into `calib.cc` via `#ifdef WINGUI`.
2. **`calib-gui.cc`** (deprecated) — Native Win32 controls. Not referenced by any build target.

**GUI flow:**
1. `main()` detects `--gui` or `WINGUI` build and calls `runCalibGui()`.
2. GUI collects parameters via raylib window (folder/file browsers, checkboxes, numeric input).
3. "2-Point Click" and "Auto-Detect" checkboxes are mutually exclusive.
4. GUI closes, parameters returned to `main()`, pipeline proceeds normally.

### Web UI (`webui/server.js`)

- Node.js HTTP server on port 11918.
- `POST /run` receives JSON config, spawns `./calib` (Linux) or `window_build\calib.exe` (Windows) as child process.
- Streams stdout/stderr as NDJSON to the browser.
- `GET /list?dir=` lists output directory contents.
- `GET /firstimg?dir=` returns first displayable image (for ROI picker).
- `POST /save-template` saves a cropped board template PNG.
- **Warning:** Deletes the output directory before each run.
- Packaged via Node SEA: `npm run build:blob` then `npm run build:exe`.

## Key Data Structures (`calib.h`)

- `ImageInfo` — Per-image metadata (path, distortion params, alignment homography, dimensions).
- `RadioCoeffs` — Radiometric calibration coefficients (per-channel linear: a, b) + DN collection data.
- `GroupData` — Groups images by prefix; holds reference image, radio coefficients, collected DNs.
- `GreenMaskParams` — All tunable parameters for the green mask pipeline (centroid, thresholds, morphology, K-Means, color spots).
- `GreenMaskResults` — Output of green mask: binary mask, centroid, convex hull, ellipse, area.
- `Config` / `ECCOptimization` — Runtime config for ECC alignment (enabled, downscale, iterations, epsilon).
- `Logger` — Thread-safe logger (writes to stdout + `.logs/` file).

## Directories

| Path | Purpose |
|---|---|
| `.input/` | Default input directory (drone images) |
| `.output/` | Default output directory |
| `.output/alignment/` | Dewarped + aligned images |
| `.output/radiometric/` | Radiometrically calibrated images |
| `.output/vegetation_index/` | Vegetation index outputs (raw, color, mask) |
| `.input_ref/` | Radiometric reference CSV + board template |
| `.logs/` | Timestamped log files |
| `.deprecated/` | Old node-opencv binding (ignore) |
| `window_build/` | Windows build output (gitignored) |

## Vendored Third-Party Code

- `mongoose.c/.h` — Deprecated C web server. Do not use.
- `cJSON.c/.h` — Deprecated JSON parser. Do not use.
- `raygui.h` — Active, used by `calib-raygui.cc`.
- `libtiff/` — Extracted from `libtiff.zip` before Windows builds (gitignored).
- `raylib/` — Extracted from `raylib.zip` before Windows builds (gitignored).

## CLI Reference

```
./calib <src_dir> <dest_dir> [options]

Options:
  --radio [interval]         Enable radiometric calibration
  --auto [thickness]         Auto-detect radiometric board (with --radio)
  --template <path>          Board template image (default: .input_ref/radiometric_board.jpg)
  --ref <path>               Reference CSV (default: .input_ref/radiometric_reference.csv)
  --optimize [down,iters,eps]  ECC optimization (default: 2,50,1e-4)
  --veg-idx=<csv>            Vegetation indices (default: ndvi)
  --green-centroid-radius/-gcr <r>  Centroid focus radius
  --kmeans                   Spectral K-Means blob refinement
  --spatial                  Spatial cluster filtering
  --cluster                  Both kmeans + spatial
  --gentle                   Stressed-plant mode (soft rejection)
  --color-mask H,S,V[;...]   Custom HSV leaf selection
  --color-tol H,S,V          HSV tolerance (default: 15,40,40)
  --gui                      Launch Windows GUI (WINGUI builds only)
```
