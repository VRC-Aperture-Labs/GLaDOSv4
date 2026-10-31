# GLaDOSv4 — PROS C++

Port of GLaDOSv3 for the existing PROS 4.2.2 project. Open this directory
(the one containing `project.pros`) in VS Code / PROS. Build with `pros make`
or `make` using a complete PROS ARM C++ toolchain, then upload through PROS.

## Brain screen and autonomous

The single touch button starts **Red**. Each press alternates **Blue** / **Red**
and updates the atomic `selectedTeam` value in `src/main.cpp`. Holding the
button does not repeat. The selector remains available in every competition
mode. The team is sampled at autonomous start; changes during a run apply to
the next run. Restarting the program resets the selection to Red.

Autonomous advances the auxiliary motor 120 degrees once for Blue, twice for
Red, then drives forward at 25% for 1.5 seconds, as in v3.

## Hardware and controls

The arm uses a red 36:1 (100 RPM) cartridge. All other motors use green
18:1 (200 RPM) cartridges. All encoders use degrees. Velocity percentages
are scaled by each motor's configured cartridge for both velocity and position
commands: manual arm control is +/-20 RPM (20%), and arm pickup is 35 RPM
(35%). Wrist pickup remains 70 RPM (35%).
Negative ports mean reversed motors.

| Device | Port |
| --- | --- |
| Front left / right drive | 2 / -3 |
| Back left / right drive | 1 / -4 |
| Intake | 10 |
| Arm | -12 |
| Wrist | 11 |
| Auxiliary | -5 |
| Claw solenoid | Brain ADI H |

| Control | Action |
| --- | --- |
| Left stick Y / X | Forward / strafe |
| Right stick X | Rotate |
| Y | Drive multiplier 25% / 75% |
| A | Intake on / off |
| B | Reverse enabled intake |
| L1 / L2 | Arm up / down at 20% |
| Up / Down | Wrist at +/-25%; cancels leveling |
| R1 | Toggle claw |
| R2 | Advance auxiliary 120 degrees |
| Right | Toggle wrist leveling |
| Left | Pickup from intake |

Start with the **arm straight down and claw horizontal**: startup zeroes both
encoders. Joint ratios, pickup targets, and leveling gain/speed match v3.

Pickup runs without blocking driver control. Manual arm/wrist input cancels
pickup; any pickup step exceeding five seconds also cancels and holds the
joints. Autonomous auxiliary movement has a three-second timeout. Disabling
stops motors and clears transient pickup/intake/leveling state. PROS competition
callbacks replace VEX event callbacks. Joystick values are converted from
PROS's +/-127 range before applying the original exponential drive curve, and
motor velocity percentages are converted to RPM.

## Toolchain and validation

Build with `make -j4`. A clean ARM firmware build and link passed using Arm
GNU 15.3.rel2, producing `bin/cold.package.bin` and `bin/hot.package.bin`.
The bundled PROS headers emit LLEMU deprecation warnings; this project's GUI
uses `pros::screen` and does not require LLEMU or LVGL.
Physical motor direction, joint calibration, touchscreen response, and
competition operation still require checking on the robot.
