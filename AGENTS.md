# AGENTS.md

UAV plant-image calibration tool. C++17 CLI + optional raylib GUI + web UI. All builds are driven by `./run.sh`, not Make/CMake.

## Build & verify

- Every target is a `./run.sh` subcommand. Run `./run.sh` with no args to list all targets.
- Linux CLI (main tool): `./run.sh build:calib`
- Windows cross-build: `./run.sh build:calib-win`, `./run.sh build:calib-win-gui`, `./run.sh build:calib-webui-win`
- Fisheye tool: `./run.sh build:fisheye`
- Smoke test: `./run.sh example:calib` (also `example:calib-radio`, `example:calib-auto`). There is no unit-test framework; use these on `.input`/`.output`.
- Release: `./run.sh release:zip` (zip) or `release:win` (PowerShell Compress-Archive).

## Toolchain quirks (critical)

- Linux and Windows use **different OpenCV majors**. Linux: `opencv5` resolved via `pkg-config` (homebrew at `/home/linuxbrew/.linuxbrew`, TIFF at `/home/linuxbrew/.linuxbrew/lib`). Windows: hardcoded `opencv455` MinGW libs at `c:/opencv`. Always build both via `run.sh`; the flags are not interchangeable.
- `src/calib.cc` has `#define WINGUI` **commented out** on purpose. GUI is enabled only by `-DWINGUI` passed in `run.sh build:calib-win-gui`. Do not uncomment it.
- Vendored third-party code lives in the tree: `mongoose.c/.h`, `cJSON.c/.h`, `raygui.h` in `src/`, plus `libtiff/` and `raylib/` (gitignored; must be extracted from `libtiff.zip` / `raylib.zip` per README before Windows builds).
- `src/fisheye.cc` is legacy/unused; the fisheye binary is built from `src/cli.cc`.

## Layout & entrypoints

- `src/calib.cc` — CLI entrypoint (main) and arg parsing; `src/calib-func.cc` — core implementation (undistort, ECC/SIFT align, radiometric, vegetation indices); `src/calib.h` — shared structs/prototypes/globals (`gLog`, `gConfig`).
- `src/calib-raygui.cc` — raylib/raygui Windows GUI, pulled in by `#include` under `WINGUI`.
- `webui/server.js` — Node web server that spawns `./calib` as a subprocess (streams NDJSON on `/run`). Chosen binary: `window_build\calib.exe` on Windows, `./calib` otherwise.
- `src/calib-web.cc` + `src/mongoose.{c,h}` + `src/cJSON.{c,h}` — **deprecated** native C web server (mongoose). Not built by `build:all` anymore; the NodeJS webui is used instead. Keep only as reference, do not extend.
- CLI positional args are `<src_dir>` `<dest_dir>`; options: `--radio`, `--auto [thickness]`, `--template`, `--ref`, `--optimize [downscale,iters,eps]`, `--veg-idx=<csv>`, `--green-centroid-radius/-gcr`, `--gui`.
- Default dirs are `.input/` → `.output/`; radiometric refs default to `.input_ref/radiometric_reference.csv` + `radiometric_board.jpg`. Logs go to `.logs/<yymmdd-HHMM>.log`.

## Node packaging (webui)

- `package.json` uses Node SEA (Single Executable App): `npm run build:blob` then `npm run build:exe`. Requires Node >= 22. `webui/server.js` alone runs via `npm start`.
- `webui/server.js` deletes the output dir (`outDir`) before running calibration — be aware when testing.

## Conventions & gotchas

- Code style: tabs, C-style types (`uint32_t`), `using namespace` at top of `.cc` files. No comments policy — match existing file conventions.
- Windows GUI state is passed through globals and a message loop; "2-Point Click" and "Auto-Detect" are mutually exclusive (GUI enforces).
- Do not commit build artifacts: `release_*.zip`, `calib.exe`, `calib`, `*.log`, `.data/`, `libtiff/`, `raylib/` are gitignored.
- `.deprecated/` holds an old node-opencv binding — ignore it.
