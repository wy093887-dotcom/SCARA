# SCARA_F103 Version Log

## 2026-06-03 v0.23.1

- Changed trajectory limit ownership to the host side:
  - Added `APP_HOST_OWNS_LIMIT_CHECKS = 1`.
  - Disabled MCU software pulse range and joint angle range checks for formal streamed trajectory limits.
  - Kept MCU-side G-code syntax checks, geometric IK feasibility, enable/ESTOP checks, homing input and status reporting.
- Updated `HOSTCAP` to report `host_limit=1 mcu_soft_limit=0`.
- Improved homing restart behavior:
  - `$H` can be sent again after `HS:Done` or `HS:Error`.
  - `$H` still returns `error:5` while homing/search/backoff is active, because the first `ok` only means the homing state machine started.
- Updated `Control.md` and `Work.md` with host-side limit responsibilities and the `error:5` explanation.

## 2026-06-03 v0.23.0

### Done

- Re-scoped the project for course-design goals:
  - reliable serial receive/ACK
  - high-frequency host-planned G-code stream testing
  - complete queue drain and status feedback
- Disabled the communication watchdog by default:
  - `APP_COMM_WATCHDOG_DEFAULT_MS = 0`
  - this prevents confusing `err=4` during control-board-only testing
  - `WATCHDOG ON timeout_ms` remains available for later experiments
- Removed old or inactive test scripts and launchers:
  - legacy pulse protocol test
  - old COM9 debug scripts
  - older pointcloud/line-arc/short-segment split tests
  - old Python/PowerShell smoke scripts
- Kept the focused course-design test path:
  - `tools/serial_link_check.ps1`
  - `tools/gcode_stream_check.ps1`
  - `tools/host_planned_stream_stress.ps1`
  - `Run_COM13_HostPlanned_3000.bat`
- Added Chinese comments to the maintained `UserApp` modules and test scripts.
- Rewrote `Control.md` as a concise Chinese course-design build/flash/serial-test guide.
- Added protocol character/field explanations for direct serial-terminal debugging.
- Added PyQt upper-computer simulator under `tools/robot_upper_sim`:
  - host-side interpolation
  - GRBL-style forward/reverse feed planning
  - XY trajectory preview
  - feed curve plotting
  - optional serial G-code streaming and ACK observation

### Notes

- `err=4` means `STEPPER_ERR_COMM_TIMEOUT`, not a required trajectory error. With default watchdog off, it should not appear unless `WATCHDOG ON ...` is enabled manually.

### Verification

- `cmake --build --preset Debug` passed.
- `tools/verify_project.ps1` passed.
- Flashed v0.23.0 to the connected STM32F103 board through CMSIS-DAP, OpenOCD verified OK.
- `tools/gcode_stream_check.ps1 -Port COM13` passed:
  - `VERSION` returned `0.23.0`
  - `HOSTCAP` returned `role=pulse_executor host_plan=1 host_limit=1`
  - status showed `E:0`
- `tools/host_planned_stream_stress.ps1 -Port COM13 -Count 20 -FeedMin 500 -FeedMax 1800 -EnableMotion` passed with live TX/RX/MATCH output.
- `tools/host_planned_stream_stress.ps1 -Port COM13 -Count 3000 -FeedMin 500 -FeedMax 1800 -EnableMotion -QuietLines` passed:
  - `ok=3000`
  - final status reached `Idle`
  - planner queue reached `Q:0`
  - error field stayed `E:0`

## 2026-06-03 v0.22.2

### Done

- Re-scoped the firmware as a host-planned pulse executor:
  - upper computer owns trajectory generation, speed planning, and formal limit judgment
  - MCU owns receive, ACK echo, buffering, pulse execution, status feedback, and safety backstop
- Removed MCU corner-speed scaling from the G-code enqueue path.
- Kept only thin adjacent-block handoff:
  - if the host sends continuous G1 blocks with planned F values, MCU can use the lower adjacent pps as execution exit speed
  - MCU no longer decides corner slowdown from XY vector angle
- Added `HOSTCAP` command:
  - reports `role=pulse_executor`
  - reports `host_plan=1 host_limit=1`
  - documents that comments are ignored by motion parsing but echoed in ACK
- Added host-planned high-frequency stream test:
  - `tools/host_planned_stream_stress.ps1`
  - script-side simulates host workspace/feed checks
  - sends `G1 X... Y... F... ;ID=xxxx LIM=1`
  - verifies `ok seq/cs/line` for every point
- Rewrote `Work.md` around the new division of responsibility.

### Notes

- The MCU soft limit remains as a safety backstop only. Formal trajectory validity must be decided by the upper computer before sending.
- The test comments after `;` are a reserved host metadata interface. They are echoed for verification but do not affect motion.

## 2026-06-03 v0.22.1

### Done

- Reviewed the v0.22 continuous-trajectory path and fixed the main velocity-planning gap:
  - previous code still treated each block as a zero-exit-speed move
  - short G1 blocks therefore braked toward zero at every block boundary
- Added blended absolute moves:
  - `MotionPlanner_MoveAbsBlend()`
  - `Stepper_MoveAbsBlend()`
  - each queued G1 block can carry per-axis exit pps calculated from the next block
- Added a small G1 blend start delay:
  - first G1 waits until at least two planner blocks are buffered, or until `APP_GCODE_BLEND_START_DELAY_MS`
  - this lets the MCU calculate the first block exit speed before motion starts
- Fixed idle pulse accounting:
  - idle axes no longer keep outputting PWM pulses while `current_pps` ramps down
  - this avoids extra software pulse counts outside an active move block
- Updated line+arc stress tooling:
  - `tools/gcode_line_arc_stress.ps1` now supports `-VaryFeed -FeedMin ... -FeedMax ...`
  - straight-line feed ramps across the line
  - arc feed varies periodically across the circle
- Added `Work.md` for upper-computer integration responsibilities and protocol details.

### Notes

- This is still a lightweight C8T6 planner, not a full GRBL segment executor. It now avoids intentional per-segment zero-speed braking, but true industrial continuous motion still depends on keeping the planner buffered and later adding a deeper precomputed step-segment queue if required.

## 2026-06-03 v0.22.0

### Done

- Increased queue depths for G-code streaming:
  - planner blocks: 32
  - RX line queue: 16
  - TX response queue: 8
- Added lightweight continuous segment handoff:
  - G-code can load the next move when the previous segment is complete even if axis speed has not fully decelerated to zero
  - segment completion no longer forces an immediate `axis_stop_now()`
  - simple corner speed scaling is applied from adjacent XY vectors
- Added automatic MCU homing controller for `$H`:
  - axis 1 searches first, then axis 2
  - default search direction is `-1` for both axes
  - home state appears in status as `HS:<state>`
- Added `HEARTBEAT seq` for upper-computer polling.
- Expanded status push with planner queue, RX free, axis enable/run/speed, home state, and switch bits.
- Added short-segment 3000-point stress tooling:
  - `tools/gcode_short_segment_stress.ps1`
  - `Run_COM13_Gcode_ShortSegment_3000.bat`

### Verification

- `cmake --build --preset Debug` passed.
- `tools/verify_project.ps1` passed.
- Current memory:
  - RAM: 8176 B / 20 KB, 39.92%
  - FLASH: 50784 B / 63 KB, 78.72%
- Flashed via CMSIS-DAP and verified with OpenOCD.
- `tools/gcode_stream_check.ps1 -Port COM13` passed, including `HEARTBEAT 42`.
- `tools/gcode_pointcloud_stress.ps1 -Port COM13 -Count 3000 -Feed 1200 -EnableMotion` passed in 32.6 s.
- `tools/gcode_line_arc_stress.ps1 -Port COM13 -LinePoints 1000 -ArcPoints 2000 -Feed 1200 -EnableMotion` passed in 28.4 s.
- `tools/gcode_short_segment_stress.ps1 -Port COM13 -Count 3000 -Feed 900 -EnableMotion` passed in 28.9 s and drained to `Idle/Q:0`.

## 2026-06-03 v0.21.0

### Done

- Trimmed F103 firmware build for the G-code path:
  - removed old pulse protocol from the build
  - removed old teach module from the build
  - removed old queued trajectory module from the build
  - reduced legacy ASCII protocol to essential debug and safety commands
- G-code acceptance responses now echo verification data:
  - `ok seq=<n> cs=<hex> line=<received line>`
- Added high-volume point-cloud stream test:
  - `tools/gcode_pointcloud_stress.ps1`
  - `Run_COM13_Gcode_Pointcloud.bat`
- The stress script validates that each acknowledged G-code line has the expected checksum and exact line echo.
- Removed retired `pulse_protocol`, `trajectory`, and `teach` source/header files from `UserApp`.
- Reclaimed the unused teach Flash page:
  - application Flash region is back to 63 KB
  - the final 1 KB parameter page remains reserved at `0x0800F800`
- `$G` now sends a normal stream `ok seq=<n> cs=<hex> line=$G` after the modal report.
- Fixed a short-segment MOVE stall where deceleration could stop at zero speed with `remaining_pulse > 0`, leaving the planner buffer full forever.
- Verified on hardware over COM13 at 115200:
  - `gcode_stream_check.ps1` passed
  - `gcode_pointcloud_stress.ps1 -Count 3000 -Feed 1200 -EnableMotion` passed with 3000 echoed ACKs in 37.4 s
- Optimized joint/pulse conversion to integer arithmetic to reduce F103 soft-float work in common kinematic conversions.
- Added disabled-motion protection:
  - real motion while an axis is disabled returns an error instead of entering a non-draining MOVE state
  - `ERRORS` now reports `disabled=1` when this occurs
- Added long straight + full-circle arc 3000-point stress tooling:
  - `tools/gcode_line_arc_stress.ps1`
  - `Run_COM13_Gcode_LineArc_3000.bat`
- Verified the new line + arc stress on hardware over COM13 at 115200:
  - `gcode_line_arc_stress.ps1 -LinePoints 1000 -ArcPoints 2000 -Feed 1200 -EnableMotion`
  - passed with 3000 echoed ACKs in 29.0 s
  - final status reported planner free and `Err:0`

### Notes

- Default point-cloud script keeps motors disabled and verifies receive/parse/ack behavior. Add `-EnableMotion` only when the machine is physically safe.

## 2026-06-03 v0.20.0

### Done

- Added a local GRBL-style G-code stream layer without directly importing the full GRBL AVR codebase.
- Added send-response flow control:
  - accepted G-code returns `ok`
  - invalid G-code returns `error:<code>`
  - planner-full input is retained and acknowledged later when space is available
- Added an 8-block G-code planner queue for `G0/G1 X/Y/F`.
- Added modal support for:
  - `G90/G91`
  - `G20/G21`
  - `G4 P`
  - `M0/M2/M30`
  - `$G`
  - `$X`
  - `$H`
- Added real-time character handling for:
  - `?`
  - `!`
  - `~`
  - `Ctrl-X`
- Added automatic 5 Hz GRBL-style status push messages:
  - `<Idle|MPos:x,y|Pulses:p1,p2|Bf:planner_free,rx_free|Err:n|Home:h1,h2>`
- Added G-code stream test helper:
  - `tools/gcode_stream_check.ps1`
  - `Run_COM9_Gcode_Check.bat`

### Notes

- This version still uses the existing stepper output backend. It adds the GRBL-style protocol and planner shell first; a fuller GRBL segment/Bresenham stepper backend should be the next optimization if continuous short-segment motion still start-stops.
- Flash is near the C8T6 application limit. If more GRBL features are added, old MOVL/TEACH/legacy pulse helpers should be trimmed.

## 2026-06-02 v0.19.0

### Done

- Pulse protocol signed-field parsing is now explicit and supports negative `p1/p2` absolute pulse targets.
- Added home microswitch inputs:
  - `HOME1`: PB0
  - `HOME2`: PB1
  - default wiring is active-low with internal pull-up
- `STATUS` and `QSTAT` now report `home=h1,h2`.
- Added `HOME_SENSOR` command to read switch state.
- `HOME` now performs sensor-confirmed zeroing only when both home switches are active.

### Notes

- The microswitch datasheet in `SOURCE/C231409_BEBDE6DE725C7F41F42AEF4FAD962318.pdf` describes V-series miniature basic switches with SPDT/SPST variants. For the current firmware default, wire switch COM to GND and NO to PB0/PB1.
- If only one shared home switch is installed, connect the unused input to GND during homing or adjust `HomeSensor_AllActive()` for single-switch behavior.

## 2026-06-02 v0.18.0

### Done

- Changed `STATUS` to a shorter debug-safe response using 32-bit pulse text output.
- Added `QSTAT` as a compact status alias for scripts and upper-computer polling.
- Updated the COM9 motion script so high-frequency motion testing does not depend on the long `STATUS` path.
- `Run_COM9_Motion_Debug.bat` now sends 40 continuous tiny absolute pulse frames at 5000 pps after confirmation.

### Notes

- If `VERSION` stops responding after an old `STATUS` test, reset the board and flash v0.18.0 before running motion validation again.

## 2026-06-02 v0.17.0

### Done

- Pulse protocol success responses now echo command and include motor debug feedback:
  - current pulse position
  - target pulse position
  - busy flag
  - combined error bits
- Added automatic COM9 serial debug script:
  - `tools/auto_com9_debug.ps1`
  - `Run_COM9_Auto_Debug.bat`
- Added opt-in COM9 motion debug launcher:
  - `Run_COM9_Motion_Debug.bat`
- The automatic COM9 test validates:
  - `VERSION`
  - `PING`
  - `STATUS`
  - bad checksum rejection
  - software zero ACK
  - pulse status fields
- `tools/auto_com9_debug.ps1` can now run:
  - burst `PING/STATUS` communication testing
  - high-frequency tiny absolute pulse trajectory-frame testing
  - ACK target-feedback validation for each motion point
  - optional `ERR BUSY` retry for continuous point streaming

### Notes

- The automatic COM9 debug path does not send motion unless `-MotionTest` or `-HighFreqMotionTest` is explicitly provided to the PowerShell script.
- The double-click motion launcher asks for confirmation before enabling motors or sending trajectory points.

## 2026-06-02 v0.16.0

### Done

- Changed USART1 and all serial test scripts back to `115200` baud.
- Added progress messages to `serial_link_check.ps1` after port open:
  - waiting for banner
  - sending `VERSION`
  - sending repeated `PING`
  - sending `STATUS`
- Kept the double-click launcher:
  - `Run_Serial_Test.bat`

### Debug Notes

- If the launcher appears to stop after `PASS port opened`, wait for the following progress lines. The script waits briefly for boot/banner text before sending commands.
- A pulse move with `speed=1` pps and a target such as `1512` pulses can keep the motor busy for about 1512 seconds. This does not mean the serial link is stuck, but `STATUS` will report the controller as busy.
- After flashing v0.16.0, use `115200` baud. Older v0.15.0 firmware still uses the previous configured baud until reflashed.

## 2026-06-02 v0.15.0

### Done

- Slimmed the TIM2 interrupt trajectory path:
  - `Trajectory_Tick1kHz()` now only sets a next-segment pending flag.
  - `Trajectory_Loop()` runs in the main loop and calls `MotionPlanner_MoveAbs()`.
- Added the formal pulse-controller text protocol:
  - `<mode,p1,p2,speed,checksum>`
  - `mode=0`: software zero/calibration
  - `mode=1`: absolute pulse target move
  - `mode=9`: emergency stop
- The pulse protocol uses an 8-bit ASCII payload sum, represented as two uppercase hex digits.
- `STATUS` now reports:
  - `pulse_proto=1`
  - `pulse_mode`
  - `pulse_err`
- Removed the v0.14 binary protocol source and 256-sample binary trajectory buffer to recover Flash/RAM.
- Added pulse protocol serial helper:
  - `tools/pulse_protocol_check.ps1`

### Verification

- `cmake --build --preset Debug` passed.
- Current firmware binary size:
  - `SCARA_F103.bin`: 57060 bytes
  - application limit: 63488 bytes
- Current memory usage:
  - RAM: 7120 B / 20 KB, 34.77%
  - FLASH: 57060 B / 62 KB, 89.88%

### Notes

- `Trajectory_Tick1kHz()` no longer contains `MotionPlanner_MoveAbs()`.
- v0.15 treats the board primarily as a pulse controller; the upper computer owns trajectory planning.

## 2026-06-01 v0.14.0

### Done

- Added the first formal binary trajectory protocol beside the ASCII debug protocol.
- Binary frames use:
  - header `A5 5A`
  - protocol version
  - frame type
  - sequence
  - payload length
  - payload
  - CRC16
- Added binary commands:
  - `HELLO`
  - `STATUS`
  - `STOP`
  - `ESTOP`
  - `CLEAR_ERROR`
  - `TRAJ_BEGIN`
  - `TRAJ_CHUNK`
  - `TRAJ_VALIDATE`
  - `TRAJ_COMMIT`
  - `TRAJ_RUN`
  - `TRAJ_ABORT`
- Added binary ACK/NACK responses with standardized error codes.
- Added 256-sample binary trajectory buffer using host-generated absolute pulse samples:
  - `int32 p1_abs`
  - `int32 p2_abs`
  - `uint16 dt_ms`
  - `uint16 flags`
- Added binary trajectory state machine:
  - `IDLE`
  - `UPLOADING`
  - `VALIDATED`
  - `READY`
  - `RUNNING`
  - `DONE`
  - `ERROR`
  - `ESTOP`
- `STATUS` now reports binary trajectory state and upload progress.
- USART1 default baud rate is now `460800` for practical trajectory upload speed.
- Added PC-side binary smoke-test tool:
  - `tools/binary_protocol_smoke.py`

### Performance Notes

- ASCII `<mode, p1, p2, speed, checksum>\n` style packets remain suitable for manual debug, calibration, and emergency commands.
- Formal multi-vector trajectories use binary frames because text parsing and decimal conversion are expensive on STM32F103C8T6 and waste serial bandwidth.
- Current binary trajectory execution still uses the existing stepper output backend. The protocol and preload/CRC/state-machine path are ready; hardware-exact pulse accounting remains the next control-layer upgrade before production trajectory claims.

### Verification

- `cmake --build --preset Debug` passed.
- Current firmware binary size:
  - `SCARA_F103.bin`: 60584 bytes
  - application limit: 63488 bytes
- Current memory usage:
  - RAM: 10696 B / 20 KB, 52.23%
  - FLASH: 60584 B / 62 KB, 95.43%

## 2026-06-01 v0.13.0

### Done

- Reworked serial RX from a single ready-line buffer to an 8-line queue.
- Reworked serial TX from a single DMA buffer to a 4-message transmit queue.
- Main loop now drains all queued RX lines each pass, so back-to-back commands are not overwritten by later lines.
- Added serial queue counters in `STATUS`:
  - `rx_ov`
  - `tx_drop`
  - `tx_q`
- `PARAM_SAVE` and `TEACH_SAVE` now reject writes while motion or trajectory replay is active:
  - `ERR PARAM_SAVE_BUSY`
  - `ERR TEACH_SAVE_BUSY`
- Added `Stepper_GetStateSnapshot()` and moved protocol/teach 64-bit position reads to interrupt-protected snapshots.
- Updated movement command setup to use a snapshot of the current 64-bit position before preparing new absolute targets.

### Formal-Control Boundaries

- PWM position tracking is still software-estimated from commanded pulse rate; before formal trajectory control it should be upgraded to exact pulse accounting from timer update events, compare callbacks, or a hardware counter path.
- `MOVL` is still segmented point-to-point Cartesian motion; it is not continuous interpolation with blended velocity yet.
- `HOME` still moves to software pulse zero; true homing requires real origin/limit switch inputs and a switch-seeking state machine.

### Verification

- `cmake --build --preset Debug` passed.
- Current firmware binary size:
  - `SCARA_F103.bin`: 55256 bytes
  - application limit: 63488 bytes
- Current memory usage:
  - RAM: 7120 B / 20 KB, 34.77%
  - FLASH: 55256 B / 62 KB, 87.03%

### Notes

- Project verification now checks that `APP_FW_VERSION` is `0.13.0`.
- RX/TX queue depths are compile-time settings:
  - `APP_SERIAL_LINE_QUEUE_DEPTH`
  - `APP_SERIAL_TX_QUEUE_DEPTH`

## 2026-06-01 v0.12.0

### Done

- Added firmware identity macros:
  - `APP_FW_NAME`
  - `APP_FW_VERSION`
- Added serial command:
  - `VERSION`
- Boot banner now includes firmware name and version.
- Serial smoke-test scripts now query `VERSION` first.
- Project verification now checks that `APP_FW_VERSION` is `0.12.0`.

### Verification

- `tools/serial_smoke.ps1` parsed successfully with PowerShell.
- `tools/serial_smoke.py` passed `py_compile`.
- `powershell -NoProfile -ExecutionPolicy Bypass -File tools\verify_project.ps1` passed.
- Current firmware binary size:
  - `SCARA_F103.bin`: 54088 bytes
  - application limit: 63488 bytes
- Current memory usage:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 54088 B / 62 KB, 85.19%

### Notes

- Use `VERSION` after flashing to confirm the board is running the expected firmware.

## 2026-06-01 v0.11.0

### Done

- Added project verification script:
  - `tools/verify_project.ps1`
- Added VS Code task:
  - `Project: verify`
- Verification checks:
  - CMake Debug build passes
  - `.elf`, `.hex`, `.bin`, and `.map` artifacts exist
  - `.bin` size fits inside the 62 KB application region
  - linker reserves the final 2 KB Flash
  - parameter page remains at `0x0800F800`
  - teach page remains at `0x0800FC00`
  - CMake still generates `.hex` and `.bin`
  - VS Code task/debug/settings files exist
  - `Version.md` and `Control.md` exist

### Verification

- `powershell -NoProfile -ExecutionPolicy Bypass -File tools\verify_project.ps1` passed.
- Current firmware binary size:
  - `SCARA_F103.bin`: 53908 bytes
  - application limit: 63488 bytes

### Notes

- This script is the preferred local regression check after future edits.

## 2026-06-01 v0.10.0

### Done

- Added serial smoke-test tools:
  - `tools/serial_smoke.ps1`
  - `tools/serial_smoke.py`
- Added VS Code serial test tasks:
  - `Serial: smoke safe`
  - `Serial: smoke with tiny motion`
- The default smoke test sends only safe read/query commands.
- The tiny motion smoke test is opt-in and sends a 100-pulse move after disabling the watchdog.

### Verification

- `tools/serial_smoke.ps1` parsed successfully with PowerShell.
- `tools/serial_smoke.py` passed `py_compile` using the bundled Python runtime.
- `cmake --build --preset Debug` passed.
- Firmware memory usage unchanged from v0.9.0:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 53908 B / 62 KB, 84.91%

### Notes

- The PowerShell script uses .NET serial APIs and needs no extra Python package.
- The Python script requires `pyserial` on user machines.

## 2026-06-01 v0.9.0

### Done

- Added post-build firmware artifacts:
  - `SCARA_F103.hex`
  - `SCARA_F103.bin`
- Added VS Code workspace configuration:
  - `.vscode/tasks.json`
  - `.vscode/launch.json`
  - `.vscode/settings.json`
- Added VS Code tasks:
  - `CMake: configure Debug`
  - `CMake: build Debug`
  - `OpenOCD: flash ST-Link`
  - `OpenOCD: flash CMSIS-DAP`
- Added Cortex-Debug launch configurations:
  - `Debug SCARA_F103 ST-Link`
  - `Debug SCARA_F103 CMSIS-DAP`

### Build Verification

- `cmake --build --preset Debug` passed.
- Generated artifacts:
  - `build/Debug/SCARA_F103.elf`
  - `build/Debug/SCARA_F103.hex`
  - `build/Debug/SCARA_F103.bin`
  - `build/Debug/SCARA_F103.map`
- Current memory usage with two 1 KB Flash storage pages reserved:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 53908 B / 62 KB, 84.91%

## 2026-06-01 v0.8.0

### Done

- Added joint-space convenience commands:
  - `JSTATUS`
  - `JREL dtheta1_mrad dtheta2_mrad vmax accel`
  - `HOME`
  - `HOME vmax accel`
- Refactored `JOINT` execution through a shared joint-to-pulse helper.
- Updated `HELP 3` to include the new joint and home commands.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage with two 1 KB Flash storage pages reserved:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 53908 B / 62 KB, 84.91%

### Notes

- `HOME` is a software return to pulse position `0,0`; it is not sensor homing.
- Without limit switches or encoders, establish a reliable mechanical zero before using `ZERO` and `HOME`.

## 2026-06-01 v0.7.0

### Done

- Split `HELP` into paged responses:
  - `HELP`
  - `HELP 1`
  - `HELP 2`
  - `HELP 3`
  - `HELP 4`
- Reduced longest help response from 376 bytes to 150 bytes.
- Added trajectory validation:
  - `TRAJ_VALIDATE`
- Added teach point validation:
  - `TEACH_VALIDATE`
- `Trajectory_Run()` now rejects queues containing pulse targets outside current runtime motor soft limits.
- `TEACH_RUN` now validates converted trajectory before starting.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage with two 1 KB Flash storage pages reserved:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 53188 B / 62 KB, 83.78%

### Notes

- The paged `HELP` design leaves TX buffer headroom for future status fields.
- `TEACH_VALIDATE` does not modify the current trajectory queue.

## 2026-06-01 v0.6.0

### Done

- Added Flash-backed teach point persistence:
  - `TEACH_SAVE`
  - `TEACH_LOAD`
- Teach points are loaded automatically during `Teach_Init()` if the saved record is valid.
- Teach Flash record includes:
  - magic
  - version
  - count
  - CRC
  - up to `APP_TEACH_MAX_POINTS` pulse points
- Split the final 2 KB of Flash into two independent pages:
  - parameter page: `0x0800F800`
  - teach page: `0x0800FC00`
- Reduced application Flash region from 63 KB to 62 KB to reserve both pages.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage with two 1 KB Flash storage pages reserved:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 52432 B / 62 KB, 82.59%

### Notes

- `HELP` is now 376 bytes, under the 384-byte TX buffer. Add future commands carefully or split help into multiple responses.
- Full-chip erase clears both saved parameters and saved teach points.

## 2026-06-01 v0.5.0

### Done

- Added communication watchdog safety stop:
  - default timeout: 3000 ms
  - if no command is received while motion is active, trajectory execution is stopped and axes decelerate to stop
  - watchdog trip sets `STEPPER_ERR_COMM_TIMEOUT`
- Added watchdog commands:
  - `WATCHDOG`
  - `WATCHDOG ON timeout_ms`
  - `WATCHDOG OFF`
- Extended `STATUS` with watchdog fields:
  - `wd`
  - `wd_ms`
  - `idle_ms`
- Added readable error query:
  - `ERRORS`
- Exposed stepper error bit definitions in `stepper_driver.h`:
  - `STEPPER_ERR_SOFT_LIMIT`
  - `STEPPER_ERR_ESTOP`
  - `STEPPER_ERR_COMM_TIMEOUT`
- Added `Stepper_SetErrorAll()` for system-level fault reporting.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage with the 1 KB parameter page reserved:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 51608 B / 63 KB, 80.00%

### Notes

- `CLEAR_ERROR` / `RESET` clears the watchdog trip and stepper error bits.
- Continuous `SPEED` mode now requires periodic commands, such as `PING`, `STATUS`, or any valid command, unless `WATCHDOG OFF` is used.

## 2026-06-01 v0.4.0

### Done

- Added Flash-backed parameter persistence on the last 1 KB flash page:
  - parameter page address: `0x0800FC00`
  - page size: 1024 bytes
  - magic/version/size/CRC validation
- Reserved the last Flash page by changing linker Flash length from 64 KB to 63 KB.
- Startup now loads saved parameters automatically after applying defaults.
- Added parameter persistence commands:
  - `PARAM_SAVE`
  - `PARAM_LOAD`
  - `PARAM_DEFAULTS`
- Increased serial TX buffer to 384 bytes so the expanded `HELP` response is not truncated.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage with the 1 KB parameter page reserved:
  - RAM: 5264 B / 20 KB, 25.70%
  - FLASH: 50676 B / 63 KB, 78.55%

### Notes

- If an external flashing tool performs a full-chip erase, saved parameters will be erased.
- After changing calibration over serial, run `PARAM_SAVE` to persist it.
- `PARAM_DEFAULTS` restores compile-time defaults in RAM; run `PARAM_SAVE` after it if defaults should replace saved Flash parameters.

## 2026-06-01 v0.3.0

### Done

- Added RAM runtime parameter layer:
  - `app_params.h`
  - `app_params.c`
- Kinematics now uses runtime parameters instead of only compile-time macros.
- Motor pulse soft limits now use runtime parameters.
- Added parameter query/configuration commands:
  - `PARAMS`
  - `GET_PARAMS`
  - `SET_PULSE ppr1 ppr2 rr1 rr2`
  - `SET_GEOM base_um active1_um active2_um passive1_um passive2_um`
  - `SET_LIMIT theta1_min theta1_max theta2_min theta2_max`
  - `SET_MOTOR_LIMIT min_pulse max_pulse`
  - `SET_ZERO zero1_mrad zero2_mrad`
  - `SET_DIR dir1 dir2`
  - `SET_IK left_sign right_sign`
  - `SET_MOVL_SEG segment_um`
- Added point inspection commands:
  - `TRAJ_GET index`
  - `TEACH_GET index`
- Increased serial TX buffer to 320 bytes so long `HELP` and `PARAMS` responses fit.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage:
  - RAM: 5168 B / 20 KB, 25.23%
  - FLASH: 48804 B / 64 KB, 74.47%

### Current Limits

- Runtime parameters are RAM-only. Power cycling restores values from `app_config.h`.
- Flash-backed `PARAM_SAVE` / `PARAM_LOAD` is still pending.
- Flash usage is now about 74%, so future additions should stay compact.

## 2026-06-01 v0.2.0

### Done

- Added configurable five-bar SCARA geometry in `UserApp/app_config.h`:
  - motor pulse/rev
  - motor direction sign
  - motor zero offset
  - base distance
  - active/passive arm lengths
  - joint soft limits
  - inverse-kinematics branch signs
- Extended `scara_kinematics`:
  - joint angle to motor pulse
  - motor pulse to joint angle
  - five-bar inverse kinematics
  - five-bar forward kinematics
  - workspace and joint-limit checks
- Added XY and trajectory protocol commands:
  - `WHERE`
  - `GOTOXY x_um y_um vmax accel`
  - `MOVL x_um y_um vmax accel`
  - `TRAJ_CLEAR`
  - `TRAJ_BEGIN`
  - `TRAJ_POINT p1 p2 v1 v2`
  - `TRAJ_XY x_um y_um vmax`
  - `TRAJ_RUN`
  - `TRAJ_END`
  - `TRAJ_STOP`
  - `TRAJ_STATUS`
- Added RAM trajectory queue with 48 points.
- Added basic teach replay:
  - `TEACH_RUN`
- Added motor pulse soft-limit rejection for `MOVE_REL`, `MOVE_ABS`, `JOINT`, `GOTOXY`, trajectory execution, and teach replay.
- Improved realtime boundaries:
  - TIM2 ISR no longer transmits serial status directly.
  - `STREAM ON` only sets a pending flag in ISR; DMA TX is started from the main loop.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage:
  - RAM: 4952 B / 20 KB, 24.18%
  - FLASH: 45464 B / 64 KB, 69.37%

### Current Limits

- SCARA dimensions are safe placeholder values. Measure the real mechanism and update `app_config.h`.
- `MOVL` uses fixed spatial segmentation, not a full continuous-velocity Cartesian planner yet.
- Trajectory and teach points are RAM-only.
- Without encoders or limit switches, `WHERE`, `MOVL`, and teach replay depend on software-estimated position.

## 2026-06-01 v0.1.0

### Done

- Added `SCARA_F103/UserApp` application layer:
  - `app_config`
  - `board_pins`
  - `stepper_driver`
  - `motion_planner`
  - `serial_dma`
  - `protocol`
  - `scara_kinematics`
  - `trajectory`
  - `teach`
  - `app_main`
- Integrated `App_Init()` and `App_Loop()` into `Core/Src/main.c`.
- Integrated all user application sources into CMake.
- Implemented TIM1_CH1 / TIM4_CH1 dual stepper PWM basics:
  - enable/disable
  - direction
  - pps speed
  - stop / emergency stop
  - 1 kHz software position estimate
- Implemented TIM2 1 kHz motion scheduler:
  - velocity ramp
  - MOVE-mode deceleration
  - PWM ARR/CCR update
- Implemented USART1 DMA circular line protocol:
  - DMA circular RX
  - LF/CRLF line framing
  - DMA non-blocking TX
- Supported initial commands:
  - `PING`
  - `HELP`
  - `STATUS`
  - `STREAM ON`
  - `STREAM OFF`
  - `ENABLE 1/0`
  - `SPEED pps1 pps2`
  - `STOP`
  - `ESTOP`
  - `CLEAR_ERROR` / `RESET`
  - `ZERO`
  - `ACCEL a1 a2`
  - `MOVE_REL dp1 dp2 v1 v2`
  - `MOVE_ABS p1 p2 v1 v2`
  - `JOINT theta1_mrad theta2_mrad vmax accel`
  - `TEACH_CLEAR`
  - `TEACH_ADD`
  - `TEACH_LIST`

### Build Verification

- `cmake --preset Debug` passed.
- `cmake --build --preset Debug` passed.
- Memory usage:
  - RAM: 3784 B / 20 KB, 18.48%
  - FLASH: 30420 B / 64 KB, 46.42%

### Hardware Assumptions

- MCU: STM32F103C8T6
- Clock: 72 MHz
- PWM timer counter clock: 1 MHz
- M1:
  - PUL: PA8 / TIM1_CH1 / AF Open-Drain
  - DIR: PB12 / Open-Drain
  - ENA: PB13 / Open-Drain
- M2:
  - PUL: PB6 / TIM4_CH1 / AF Open-Drain
  - DIR: PB7 / Open-Drain
  - ENA: PB8 / Open-Drain
- Recommended DM556 common-anode wiring:
  - PUL+/DIR+/ENA+ to +5V
  - STM32 pins to PUL-/DIR-/ENA-
- Placeholder pulse settings:
  - M1: 1600 pulse/rev
  - M2: 1600 pulse/rev
