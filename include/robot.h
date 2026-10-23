/**
 * \file robot.h
 *
 * Shared handles for every device and subsystem on the robot, plus the entry
 * point into the autonomous routes.
 *
 * The objects themselves are DEFINED once in src/main.cpp. This header only
 * declares them so other translation units (src/autons.cpp, etc.) can use them.
 *
 * Routes live in src/autons.cpp and are selected inside runAuton(), so adding
 * a new one never requires touching this header.
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

extern pros::Distance frontSensor;
extern pros::Distance leftSensor;
extern pros::Distance rightSensor;
extern pros::Distance backBottomSensor;
extern pros::Distance backTopSensor;
extern pros::Distance claw_sensor;


// The V5 IMU under-reads turns slightly. This scales its rotation so LemLib's
// odom (which reads the IMU through a pros::Imu*) gets true degrees. Measured
// with spinTest(): 10 turns each way read ~3577 instead of 3600.
class ScaledImu : public pros::Imu {
  public:
    ScaledImu(std::uint8_t port, double scale) : pros::Imu(port), scale(scale) {}
    double get_rotation() const override { return pros::Imu::get_rotation() * scale; }
    double get_heading() const override {
        double heading = std::fmod(get_rotation(), 360.0);
        return heading < 0 ? heading + 360.0 : heading;
    }
  private:
    double scale;
};
extern ScaledImu imu;
extern pros::Rotation vertical_rotation_sensor;
extern pros::Rotation horizontal_rotation_sensor;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;

extern pros::MotorGroup cascade;
extern pros::Rotation cascade_sensor;
extern lemlib::PID cascade_pid;
//void moveCascadeTo(double target);
void moveCascadeTo(double target, int timeoutMs = 2000);

extern pros::MotorGroup arm_motor;
extern pros::Rotation arm_sensor;
extern lemlib::PID arm_pid;
extern lemlib::PID arm_hold_pid;
void moveArmTo(double target, int timeoutMs = 2000, float maxSpeed = 127.0f);
void moveArmToHold(double target, int timeoutMs = 2000, float maxSpeed = 127.0f);
inline constexpr double downArmDegPinAndCup = 65;
inline constexpr double downArmDegPinAndCupClamping = 20;
inline constexpr double downArmDegJustPin = 0;
inline constexpr double allianceGoalWithNothing = 5;
inline constexpr double normalAllianceGoal = 5;

constexpr float HS_AXLE_DIAMETER = 0.25f; 
// cascade stages multiply travel; calibrated so brain-screen inches match a ruler (8.5 in real move)
constexpr float CASCADE_TRAVEL_SCALE = 2.355f;
constexpr float INCHES_PER_DEGREE = (HS_AXLE_DIAMETER * 3.14159f) / 360.0f * CASCADE_TRAVEL_SCALE;

// cascade heights are in inches
inline constexpr double cascadeRestHeight = 1.917;  //2.517   // CHANGE THIS: The higher hover height (was 490 deg)
inline constexpr double cascadeClampingHeight = 0.0;   
inline constexpr double cascadeLiftOffset = 5.137;     // extra lift before moving the arm (was 1000 deg)

float getCascadeInches();

extern pros::adi::DigitalOut clawShut;
extern bool flipBool;
extern bool clawBool;
extern bool armNeedsDown;
extern bool armPosition;
extern pros::adi::DigitalOut leftToggle;
extern pros::adi::DigitalOut rightToggle;


// ---------------------------------------------------------------- controller
extern pros::Controller controller;

// -------------------------------------------------------- autonomous (autons.cpp)
/// Runs the route picked with autonSelector(). Routes are listed in src/autons.cpp.
void runAuton();
/// True while runAuton() is running. The driver button task in initialize() waits on it.
extern bool autonRunning;
/// Controller auton picker: Left/Right to choose, X to lock in. initialize() waits on it.
void autonSelector();
extern bool pickingAuton;

// Tuning routines, callable from opcontrol() for testing.
void darwin_test();
void frictiontest();
void torquetest();
void tuningPID();
void spinTest();
/// Thread-safe brain screen print for lines 5-7 (the screen task draws it).
/// Use this instead of pros::lcd::print anywhere outside the screen task.
void screenPrint(int line, const char* fmt, ...);

// Read by the screen task in main.cpp to show the spin test readout.
extern bool spinTestActive;
extern double spinTestRotation;
extern double spinTestPeakRate;

void moveArmToFront();
void moveArmToBack(bool async = false);
void primeArm();
void scoreArm();

void L1Button();
void L2Button();
void R1Button();
void toggleMech();
void toggleMechUp();
void toggleMechDown();


void rotationChassisPID();
void linearChassisPID();

void closeClaw(int timeoutMs = 3000);
void openClaw();