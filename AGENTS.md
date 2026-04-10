# Repository Guidelines

## Project Structure & Module Organization
`src/` contains the runtime binaries: `main.cpp` drives the 500 Hz control loop, `communication.cpp` handles UDP I/O, `trt_inference.cpp` wraps TensorRT, and `gamepad_input.cpp` handles joystick input. Public headers live in `include/`. Manual test and diagnostic programs live in `test/` and are built as standalone executables such as `test_udp`, `test_trt_engine`, and `gamepad_calibration`. Utility conversion code is in `tools/` and `scripts/`. Runtime assets sit at the repo root or under `model/`, for example `robot.yaml` and TensorRT engine files.

## Build, Test, and Development Commands
Build from a separate directory:

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

Run the main binary with an engine and config:

```bash
./JetsonRLDeploy ../model.engine --config ../robot.yaml --ip 192.168.5.159 --port 10000
```

Useful checks:
- `./test_trt_engine ../model.engine`: verify TensorRT engine loading and inference.
- `./test_trt_engine_csv ../model.engine ../sim_inputs_outputs_100steps.csv`: compare engine output against recorded data.
- `./test_udp [ip] [port]`: validate UDP connectivity.
- `python3 scripts/convert_to_trt.py ../model.pt ../model.engine --full`: build an engine on the target Jetson.

## Coding Style & Naming Conventions
Use C++14 and keep new code consistent with the existing style: 4-space indentation, opening braces on the same line, and descriptive comments only where control logic is non-obvious. Prefer `snake_case` for files and local variables, `PascalCase` for types, and `UPPER_SNAKE_CASE` for constants. Keep headers in `include/` aligned with implementation files in `src/`.

## Testing Guidelines
There is no unit-test framework in this repository; `test/` contains hardware-facing executable checks. Name new test programs `test_<feature>.cpp` and register them in [CMakeLists.txt](/home/zomnk/Documents/Jetson_GYM/CMakeLists.txt). Before opening a PR, build cleanly and run the relevant executables for any changed area, especially TensorRT, UDP, or joystick paths.

## Commit & Pull Request Guidelines
Recent commits use short, task-focused Chinese summaries such as `添加手柄测试相关代码` and `同步Jetson上的代码`. Keep commit messages concise, imperative, and scoped to one change. PRs should state the runtime impact, list the commands you ran, reference related issues, and include logs or screenshots when a change affects calibration tools, CSV output, or operator-facing diagnostics.

## Configuration & Runtime Notes
TensorRT engines are GPU-architecture specific; generate them on the deployment Jetson. Do not commit build outputs from `build/`, but keep tracked calibration/config assets such as `robot.yaml` current with code changes.
