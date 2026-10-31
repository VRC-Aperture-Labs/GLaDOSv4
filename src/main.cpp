#include "main.h"
#include <algorithm>
#include <atomic>
#include <cmath>

namespace {
enum class Team { Red, Blue };
std::atomic<Team> selectedTeam{Team::Red};

// Signed ports preserve the VEX project's reversals. Arm: red; all others: green.
pros::Motor frontLeft(2, pros::MotorGears::green, pros::MotorUnits::degrees);
pros::Motor frontRight(-3, pros::MotorGears::green, pros::MotorUnits::degrees);
pros::Motor backLeft(1, pros::MotorGears::green, pros::MotorUnits::degrees);
pros::Motor backRight(-4, pros::MotorGears::green, pros::MotorUnits::degrees);
pros::Motor intake(10, pros::MotorGears::green, pros::MotorUnits::degrees);
pros::Motor arm(-12, pros::MotorGears::red, pros::MotorUnits::degrees);
pros::Motor wrist(11, pros::MotorGears::green, pros::MotorUnits::degrees);
pros::Motor auxiliary(-5, pros::MotorGears::green, pros::MotorUnits::degrees);
pros::adi::DigitalOut claw('H', false);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

constexpr double armRatio = 84.0 * 36.0 / 36.0;
constexpr double wristRatio = 60.0 * 18.0 / 12.0;
bool intakeEnabled = false, intakeReversed = false;
bool clawClosed = false, wristLeveling = false;
double driveMultiplier = 0.25;

// Both velocity and position commands take RPM. Scale by the configured cartridge.
std::int32_t percentToRpm(const pros::Motor& motor, double percent) {
    double maxRpm;
    switch (motor.get_gearing()) {
        case pros::MotorGears::red: maxRpm = 100.0; break;
        case pros::MotorGears::green: maxRpm = 200.0; break;
        case pros::MotorGears::blue: maxRpm = 600.0; break;
        default: return 0; // Do not command motion if gearing cannot be read.
    }
    return std::lround(std::clamp(percent, -100.0, 100.0) * maxRpm / 100.0);
}
void velocity(const pros::Motor& motor, double percent) {
    motor.move_velocity(percentToRpm(motor, percent));
}
void stopDrive() {
    frontLeft.brake(); frontRight.brake(); backLeft.brake(); backRight.brake();
}
void stopAll() {
    stopDrive(); arm.brake(); wrist.brake(); intake.brake(); auxiliary.brake();
}
void drive(double forward, double strafe, double turn, double multiplier) {
    velocity(frontLeft, (forward + strafe + turn) * multiplier);
    velocity(frontRight, (forward - strafe - turn) * multiplier);
    velocity(backLeft, (forward - strafe + turn) * multiplier);
    velocity(backRight, (forward + strafe - turn) * multiplier);
}
double curve(pros::controller_analog_e_t axis) {
    const double x = controller.get_analog(axis) * 100.0 / 127.0;
    return std::abs(x) < 2.5 ? 0.0 : std::copysign(std::pow(10.0, std::abs(x) / 50.0), x);
}
bool pressed(pros::controller_digital_e_t button) { return controller.get_digital(button); }
bool tapped(pros::controller_digital_e_t button) { return controller.get_digital_new_press(button); }
void setClaw(bool closed) { clawClosed = closed; claw.set_value(closed); }
bool manualJointInput() {
    return pressed(pros::E_CONTROLLER_DIGITAL_L1) || pressed(pros::E_CONTROLLER_DIGITAL_L2)
        || pressed(pros::E_CONTROLLER_DIGITAL_UP) || pressed(pros::E_CONTROLLER_DIGITAL_DOWN);
}
bool atTarget(const pros::Motor& motor, double target) {
    const double position = motor.get_position();
    return std::isfinite(position) && std::abs(position - target) < 2.0;
}

// Pickup runs as a state machine so driving and cancellation remain responsive.
struct PickupStep { double armTarget, wristTarget; int pauseMs; int clawState; };
constexpr PickupStep pickupSteps[] = {
    {0, 12, 125, 1}, // First step holds the arm at its current position.
    {-300, 12, 0, -1},
    {-200, 12, 0, 0},
    {-200, -275, 125, -1},
    {-200, -275, 125, 1},
    {-200, 56.8, 125, -1},
    {54.4, 56.8, 0, -1}
};
int pickupStep = -1;
std::uint32_t pickupStarted = 0, pickupSettled = 0;
bool pickupAtTarget = false;
double pickupArmTarget = 0;
void finishPickup() {
    arm.brake(); wrist.brake(); pickupStep = -1;
}
void beginPickupStep(int index) {
    pickupStep = index;
    const auto& step = pickupSteps[index];
    pickupArmTarget = index == 0 ? arm.get_position() : step.armTarget;
    if (step.clawState >= 0) setClaw(step.clawState != 0);
    arm.move_absolute(pickupArmTarget, percentToRpm(arm, 35));
    wrist.move_absolute(step.wristTarget, percentToRpm(wrist, 35));
    pickupStarted = pros::millis();
    pickupAtTarget = false;
}
void updatePickup() {
    if (pickupStep < 0) return;
    const auto now = pros::millis();
    if (manualJointInput() || now - pickupStarted >= 5000) { finishPickup(); return; }
    const auto& step = pickupSteps[pickupStep];
    if (!atTarget(arm, pickupArmTarget) || !atTarget(wrist, step.wristTarget)) {
        pickupAtTarget = false;
        return;
    }
    if (!pickupAtTarget) { pickupSettled = now; pickupAtTarget = true; }
    if (now - pickupSettled < static_cast<std::uint32_t>(step.pauseMs)) return;
    if (pickupStep == 6) finishPickup();
    else beginPickupStep(pickupStep + 1);
}
void updateJoints() {
    if (pickupStep >= 0) return;
    const int armDirection = int(pressed(pros::E_CONTROLLER_DIGITAL_L1)) - int(pressed(pros::E_CONTROLLER_DIGITAL_L2));
    if (armDirection) velocity(arm, armDirection * 20.0); else arm.brake();
    const bool up = pressed(pros::E_CONTROLLER_DIGITAL_UP), down = pressed(pros::E_CONTROLLER_DIGITAL_DOWN);
    if (up || down) wristLeveling = false;
    if (wristLeveling) {
        const double error = -arm.get_position() / armRatio * wristRatio - wrist.get_position();
        if (!std::isfinite(error) || std::abs(error) < 0.5) wrist.brake();
        else velocity(wrist, std::clamp(error, -35.0, 35.0));
    } else if (up != down) velocity(wrist, up ? 25.0 : -25.0);
    else wrist.brake();
}

void drawTeamButton() {
    const bool red = selectedTeam.load() == Team::Red;
    pros::screen::set_pen(red ? 0xFF0000u : 0x0000FFu);
    pros::screen::fill_rect(140, 80, 340, 180);
    pros::screen::set_pen(0xFFFFFFu);
    pros::screen::print(pros::E_TEXT_LARGE, red ? 214 : 205, 116, "%s", red ? "Red" : "Blue");
}
void screenTask() {
    pros::screen::set_eraser(0x000000u);
    pros::screen::erase();
    drawTeamButton();
    bool wasDown = false;
    unsigned refresh = 0;
    while (true) {
        const auto touch = pros::screen::touch_status();
        const bool down = touch.touch_status == pros::E_TOUCH_PRESSED || touch.touch_status == pros::E_TOUCH_HELD;
        if (down && !wasDown && touch.x >= 140 && touch.x <= 340 && touch.y >= 80 && touch.y <= 180) {
            selectedTeam.store(selectedTeam.load() == Team::Red ? Team::Blue : Team::Red);
            drawTeamButton();
        }
        wasDown = down;
        if (refresh++ % 5 == 0) {
            pros::screen::set_pen(0x000000u);
            pros::screen::fill_rect(0, 190, 479, 235);
            pros::screen::set_pen(0xFFFFFFu);
            pros::screen::print(pros::E_TEXT_MEDIUM, 10, 195, "Arm: %.1f  Wrist: %.1f", arm.get_position(), wrist.get_position());
        }
        pros::delay(20);
    }
}
bool autonomousActive() { return !pros::competition::is_disabled() && pros::competition::is_autonomous(); }
bool runToggleAutonomous() {
    if (!autonomousActive()) return false;
    const double target = auxiliary.get_position() + 120.0;
    auxiliary.move_absolute(target, percentToRpm(auxiliary, 100));
    const auto start = pros::millis();
    while (!atTarget(auxiliary, target)) {
        if (!autonomousActive() || pros::millis() - start >= 3000) { auxiliary.brake(); return false; }
        pros::delay(10);
    }
    auxiliary.brake();
    return true;
}
} // namespace

void initialize() {
    for (auto motor : {&frontLeft, &frontRight, &backLeft, &backRight}) motor->set_brake_mode(pros::MotorBrake::brake);
    arm.set_brake_mode(pros::MotorBrake::hold);
    wrist.set_brake_mode(pros::MotorBrake::hold);
    intake.set_brake_mode(pros::MotorBrake::coast);
    auxiliary.set_brake_mode(pros::MotorBrake::coast);
    arm.set_current_limit(2500); wrist.set_current_limit(2500);
    // Start with the arm straight down and the claw horizontal.
    arm.tare_position(); wrist.tare_position();
    stopAll();
    static pros::Task gui(screenTask, "Team selector");
}
void disabled() {
    stopAll();
    pickupStep = -1;
    intakeEnabled = false;
    wristLeveling = false;
}
void competition_initialize() {}

// automomnus (this is correct spelling do not change)
void autonomous() {
    stopAll();
    pickupStep = -1;
    // Snapshot the selection so a screen tap cannot change a running routine.
    const Team team = selectedTeam.load();
    if (!runToggleAutonomous()) return;
    if (team == Team::Red && !runToggleAutonomous()) return;
    if (!autonomousActive()) return;
    drive(25, 0, 0, 1.0);
    for (int elapsed = 0; elapsed < 1500 && autonomousActive(); elapsed += 10) pros::delay(10);
    stopDrive();
}
void opcontrol() {
    stopAll();
    pickupStep = -1;
    intakeEnabled = false;
    intakeReversed = false;
    wristLeveling = false;
    while (!pros::competition::is_disabled() && !pros::competition::is_autonomous()) {
        if (tapped(pros::E_CONTROLLER_DIGITAL_A)) { intakeEnabled = !intakeEnabled; intakeReversed = false; }
        if (tapped(pros::E_CONTROLLER_DIGITAL_B) && intakeEnabled) intakeReversed = !intakeReversed;
        if (intakeEnabled) velocity(intake, intakeReversed ? -100 : 100); else intake.brake();
        if (tapped(pros::E_CONTROLLER_DIGITAL_Y)) driveMultiplier = driveMultiplier == 0.25 ? 0.75 : 0.25;
        if (tapped(pros::E_CONTROLLER_DIGITAL_R2)) auxiliary.move_relative(120, percentToRpm(auxiliary, 100));
        if (tapped(pros::E_CONTROLLER_DIGITAL_R1) && pickupStep < 0) setClaw(!clawClosed);
        if (tapped(pros::E_CONTROLLER_DIGITAL_RIGHT) && pickupStep < 0) wristLeveling = !wristLeveling;
        if (tapped(pros::E_CONTROLLER_DIGITAL_LEFT) && pickupStep < 0 && !manualJointInput()) {
            wristLeveling = false;
            beginPickupStep(0);
        }
        updatePickup();
        updateJoints();
        drive(curve(pros::E_CONTROLLER_ANALOG_LEFT_Y), curve(pros::E_CONTROLLER_ANALOG_LEFT_X),
              curve(pros::E_CONTROLLER_ANALOG_RIGHT_X), driveMultiplier);
        pros::delay(10);
    }
    stopAll();
}
