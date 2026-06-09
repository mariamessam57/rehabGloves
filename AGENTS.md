# Agents guidance for rehabGloves repository

Purpose
- Provide concise, actionable instructions for automated coding agents (Copilot/CI bots) working on this repository.

How agents should behave
- Run deterministic builds before proposing code changes: use PlatformIO (see "Build & Test").
- Prefer small, focused edits. Link to existing docs rather than copying them.
- Avoid changing hardware pin mappings or board settings in `include/config.h` unless the change is requested and hardware validated.
- Do not modify `platformio.ini` environment versions without confirming compatibility.
- When touching safety or motor code, add or update unit tests (if applicable) and include a safety rationale in the PR description.

Build & Test
- Build (local) for the default environment:

```
pio run -e esp32dev
```

- Upload to device (specify correct serial port via environment or CLI):

```
pio run -e esp32dev -t upload
```

- Monitor serial output (uses `monitor_speed` from `platformio.ini`):

```
pio device monitor -b 115200
```

- If tests exist, run them with:

```
pio test
```

Key files (reference)
- platformio.ini: project environments, libraries, build flags — [platformio.ini](platformio.ini#L1)
- Entrypoint: [src/main.cpp](src/main.cpp#L1)
- Hardware configuration: [include/config.h](include/config.h#L1)
- Shared system state: [include/systemstate/System_State.h](include/systemstate/System_State.h#L1)
- Libraries folder: [lib/](lib/)
- Lib docs: [lib/README](lib/README)
- Include docs: [include/README](include/README)
- Tests: [test/README](test/README)

Project conventions & notes for agents
- Architecture: FreeRTOS tasks pinned to cores; `SharedState` mediates cross-task data. Prefer task-safe changes and respect mutex/event group usage.
- Motor safety: Motor ramping and safety checks live in `lib/MotorDrive` and `lib/Safety`; do not remove safety checks or reduce configured safety margins without explicit review.
- Calibration persists to NVS via `Preferences` (see `lib/Calibration`). Be cautious when changing storage keys or formats.
- Build flags include `-Iinclude` and several `lib` include paths (see `platformio.ini`).

Common pitfalls
- Hardware-specific code (I2C, ADC pins, PWM timers) may not be testable on CI — prefer small unit-testable refactors and mock hardware accesses when adding tests.
- Changing `PWM_DUTY_MAX`, `PWM_RAMP_STEP`, or timing constants can affect safety; require review and runtime validation.
- NVS/persistent storage changes can corrupt calibration state for users — provide migration code when necessary.

Suggested next customizations (optional)
- Add `.github/copilot-instructions.md` to provide repository-scoped instructions for PRs and commit messages.
- Create a small `skill` to run `pio run` + static checks and collect build logs for agents.

Feedback
- If you want different behavior (e.g., allow automated pin changes for a specific board), say so and specify the board/environment to target.
