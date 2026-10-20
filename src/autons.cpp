#include "main.h"
#include "robot.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"


// ---------------------------------------------------------------------------
// Autonomous entry point -- pick the route to run here.
// xxx
// Everything below is local to this file, so adding a route means editing
// this file only: write the function, then call it from runAuton().
// ---------------------------------------------------------------------------

bool armPosition = true;

// ---------------------------------------------------------------------------
// Subsystem helpers used by the routes below
// ---------------------------------------------------------------------------

//for scoring only
void openClaw() {
    clawShut.set_value(false);
    clawBool=false;
};

void openClawAndLiftThenPutBackFront(){
    clawShut.set_value(false);
    clawBool=false;
    moveCascadeTo(getCascadeInches() + cascadeLiftOffset, 500);
    moveArmToHold(downArmDegPinAndCup);
    moveCascadeTo(cascadeRestHeight);
}

// Waits in the background for an object, then grabs it. Gives up after
// timeoutMs so the task can't outlive auton and fire during driver control.
// Same sequence as the L1 auto-clamp in main.cpp.
void closeClaw(int timeoutMs) {
     uint32_t start = pros::millis();
     pros::Task armTask([=]() {
        while (pros::millis() - start < (uint32_t)timeoutMs) {
            if (claw_sensor.get_distance() < 50) {

                // 1. Lower both the cascade and the arm
                moveCascadeTo(cascadeClampingHeight, 200);
                moveArmTo(downArmDegPinAndCupClamping, 300);

                // 2. Clamp the object
                clawBool = true;
                clawShut.set_value(true);
                pros::delay(150);

                // 3. Lift both back to their resting heights
                moveCascadeTo(cascadeRestHeight, 600);
                moveArmTo(downArmDegPinAndCup, 600);
                armNeedsDown = true;
                break;
            }
            pros::delay(50);
            }
        });

}

// First R1 press in driver control: swing the arm up to 450 and hold it there.
// Safe to call early (e.g. while driving to the goal) to save time later.
void primeArm() {
    armNeedsDown = false;
    moveArmToHold(450);
}

// Second R1 press in driver control: swing over to score and drop.
// Follow with openClawAndLiftThenPutBackFront() or moveArmToFront().
void scoreArm() {
    cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    cascade.move_velocity(0);
    moveArmTo(800, 2000, 90.0f);
    openClaw();
    armNeedsDown = true;
    armPosition = true;
}

// Prime and score in one call.
void moveArmToBack() {
    primeArm();
    scoreArm();
};

void moveArmToFront() {
    pros::delay(100);
    cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    moveCascadeTo(getCascadeInches() + cascadeLiftOffset, 500);
    moveArmToHold(downArmDegPinAndCup);
    moveCascadeTo(cascadeRestHeight);
    armPosition=false;
}

// ---------------------------------------------------------------------------
// Autonomous routes
// ---------------------------------------------------------------------------

void EastWestAuton() {
    // Toggle flip
    toggleMechDown();
    chassis.moveToPoint(55, 0, 1000, {.forwards = false});//for connor, since i don't have a wall i don't want to change this but maybe add a minimum voltage/speed to make sure it goes
    
    //Go score the preloaded pin
    chassis.turnToHeading(345, 1000);
    moveArmToHold(downArmDegPinAndCup);
    moveCascadeTo(2.569,200); // was 500 deg, changed to inches. to connor, i think this maybe lower, idk figure out with air
    chassis.moveToPoint(49, 20, 2000, {}, false);
    chassis.turnToHeading(345, 1000);
    moveArmToHold(0);
    moveCascadeTo(0,1000);
    openClaw();

    // // Get out of the way and face towards the first cup/pin
    chassis.moveToPoint(50, 10, 1000, {.forwards = false});
    chassis.turnToHeading(300, 1000);
    moveArmToHold(downArmDegPinAndCup);
    moveCascadeTo(2.569,1000);

    // Get the first cup/pin and get it
    chassis.moveToPoint(25, 25, 2000, {});
    closeClaw();
    pros::delay(1000); 
    
    // Move to goal, move arm to back, and score
    chassis.turnToHeading(230, 1000);
    chassis.moveToPoint(44, 27, 2000, {.forwards = false});
    moveArmToBack();
    openClawAndLiftThenPutBackFront();

    // Swing to the second cup/pin and get it
    chassis.swingToPoint(47, 30, lemlib::DriveSide::RIGHT, 500, {.forwards = false});
    chassis.moveToPoint(47, 42, 2000);
    closeClaw();

    // // Move back to goal and score
    // chassis.moveToPoint(47, 30, 2000, {.forwards = false});
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

    // // Swing to the third cup/pin and get it
    // chassis.swingToPoint(54, 23, lemlib::DriveSide::RIGHT, 5000);
    // chassis.moveToPoint(62, 23, 2000);
    // closeClaw();

    // // Rotate to face neutral goal and go to it backwards, move arm to back and then score it.
    // chassis.turnToHeading(20, 1000);
    // chassis.moveToPoint(50, -15, 2000, {.forwards = false}, false);
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

    // // Swing to the fourth cup/pin and get it
    // chassis.swingToPoint(54, -23, lemlib::DriveSide::RIGHT, 5000);
    // chassis.moveToPoint(62, -23, 2000);
    // closeClaw();

    // // Move back to goal and score
    // chassis.moveToPoint(54, -23, 2000, {.forwards = false});
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

}

void SkillsAuton() {
    toggleMechDown();
    chassis.moveToPoint(55, 0, 1000, {.forwards = false});
    // chassis.moveToPoint(63, 0, 1000, {}, false);
    // chassis.moveToPoint(59, 0, 1000, {.forwards = false});
    // chassis.moveToPoint(63, 0, 1000, {}, false);

    
    chassis.turnToHeading(340, 1000);
    moveCascadeTo(2.569,200); // was 500 deg
    chassis.moveToPoint(50, 20, 1000, {}, false);

    // openClaw();
    // chassis.moveToPose(15, -55, 0, 5000);
    // closeClaw();
    // chassis.moveToPoint(23.5, -56, 5000);
    // moveArmToFront();
    // chassis.waitUntil(2);
    // openClaw();
    // chassis.swingToPoint(-17.5, -29.5, lemlib::DriveSide::LEFT, 5000, {.forwards = false});
    // moveArmToBack();
    // chassis.moveToPoint(-17.5, -29.5, 5000);
}
// ---------------------------------------------------------------------------
// Test / tuning routines
// ---------------------------------------------------------------------------

void darwin_test() {
    int counter = 0;
    chassis.setPose(0, 0, 0);
    int secondVar = 0;
    while (frontSensor.get() > 500) {
        chassis.arcade(127, 0);
        counter++;
        screenPrint(5, "%d sec: %f", counter, chassis.getPose().y);
        pros::delay(100);
    }
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.arcade(0, 0);
}

void frictiontest() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.arcade(127, 0);
    pros::delay(500);
    chassis.arcade(0, 0);
    screenPrint(6, "Final Velocity: %.4f", vertical_rotation_sensor.get_velocity());
    int count = 0;
    while (vertical_rotation_sensor.get_velocity() > 1) {
        count++;
        pros::delay(50);
    }
    screenPrint(7, "Time till stop: %.4f seconds",
                     vertical_rotation_sensor.get_velocity() * 20); // 20 = 1000/50 or 1 second / milli
}

void torquetest() {
    int torqueCount = 47;
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            torqueCount += 10;
            if (torqueCount > 127) torqueCount = 127;
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            torqueCount -= 10;
            if (torqueCount < 0) torqueCount = 0;
        }
        screenPrint(5, "Current voltage: %d", torqueCount);
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            left_motors.move(torqueCount);
        } else {
            left_motors.move(0);
        }
    }
}

void tuningPID(){
    // chassis.turnToHeading(90, 2000);
    // pros::delay(2000);
    // chassis.turnToHeading(180, 2000);
    // pros::delay(2000);
    // chassis.turnToHeading(270, 2000);
    // pros::delay(2000);
    // chassis.turnToHeading(0, 2000);

    chassis.moveToPoint(0,12, 2000);
    pros::delay(4000);
    chassis.moveToPoint(0,24, 2000);
    pros::delay(4000);
    chassis.moveToPoint(0,48, 2000, {.forwards=false});
    pros::delay(4000);
    chassis.moveToPoint(0,0, 2000);


}

// IMU scale test. Blocks opcontrol until B is pressed: only right-stick
// turning works while it runs. Line the robot up against a wall/tile edge,
// press A to zero, spin in place with the right stick (or by hand), and line
// it back up after some whole number of turns. The screen task in main.cpp
// shows the rotation and the scale factor (true degrees / IMU degrees).
// This never prints to the brain screen itself: LVGL isn't thread-safe, and
// printing from here and the screen task at once crashes the brain.
bool spinTestActive = false;
double spinTestRotation = 0;
double spinTestPeakRate = 0; // fastest turn rate seen since the last zero, deg/s

void spinTest() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // Use an offset instead of tare_rotation() so LemLib's odom isn't disturbed.
    double start = imu.get_rotation();
    spinTestRotation = 0;
    spinTestPeakRate = 0;
    spinTestActive = true;
    while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            start = imu.get_rotation();
            spinTestPeakRate = 0;
        }
        chassis.arcade(0, controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        spinTestRotation = imu.get_rotation() - start;
        double rate = std::fabs(imu.get_gyro_rate().z);
        if (std::isfinite(rate) && rate > spinTestPeakRate) spinTestPeakRate = rate;
        pros::delay(20);
    }
    chassis.arcade(0, 0);
    spinTestActive = false;
}

void testing(){
    moveArmTo(downArmDegPinAndCup);
    closeClaw(10000);
    pros::delay(4000);
    // tuningPID();
    moveArmToBack();
    moveArmToFront();
}

// ---------------------------------------------------------------------------
// Auton selector -- shown on controller line 2. Left/Right to choose.
//   1st X: locks in the route and sets its start pose. Odometry keeps
//          tracking from there, so the robot can be pushed into place.
//   2nd X: starts the route (only without a competition switch/field; on a
//          switch/field, autonomous() starts it instead).
// To add a route, write the function above and add a line here with its start
// pose (keep Testing last).
// ---------------------------------------------------------------------------

struct AutonRoute {
    const char* name;
    void (*run)(); // nullptr = no auton, straight to driver control
    float x, y, theta; // start pose set on the first X press
};

AutonRoute autons[] = {
    {"Driver", nullptr, 0, 0, 0},
    {"EastWest Auton", EastWestAuton, 69, 0, 90},
    {"Skills Auton", SkillsAuton, 69, 0, 90},
    {"Testing", testing, 0, 0, 0},
};
constexpr int autonCount = sizeof(autons) / sizeof(autons[0]);

int selectedAuton = 0; // starts on Driver
bool pickingAuton = true; // true from boot so nothing runs before the picker is done

// the field has enabled the robot, so stop waiting on the controller
static bool matchStarted() {
    return pros::competition::is_connected() && !pros::competition::is_disabled();
}

void autonSelector() {
    pickingAuton = true; // screen task stops printing to controller line 2
    pros::delay(100); 

    // step 1: pick the route, first X locks it in
    int shown = -1;
    uint32_t lastPrint = 0;
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) && selectedAuton > 0) {
            selectedAuton--;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) && selectedAuton < autonCount - 1) {
            selectedAuton++;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            break;
        }
        // safety net on the field: if the match starts before X is pressed,
        // lock in whatever is showing so initialize() can't block the match
        if (matchStarted()) {
            break;
        }

        // the controller drops prints sent faster than ~50ms apart, so only
        // print on a change, plus a slow refresh in case one got dropped
        if (selectedAuton != shown || pros::millis() - lastPrint > 250) {
            controller.print(2, 0, "Auton: %-12s", autons[selectedAuton].name);
            shown = selectedAuton;
            lastPrint = pros::millis();
        }
        pros::delay(20);
    }

    AutonRoute& route = autons[selectedAuton];
    screenPrint(6, "Auton: %s", route.name);

    // step 2: set the start pose, then wait for the second X to start.
    if (route.run) {
        chassis.setPose(route.x, route.y, route.theta);
        if (!pros::competition::is_connected()) {
            pros::delay(60);
            controller.print(2, 0, "X to start    ");
            uint32_t lastPrint = pros::millis();
            while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                if (pros::millis() - lastPrint > 250) {
                    controller.print(2, 0, "X to start    ");
                    lastPrint = pros::millis();
                }
                pros::delay(20);
            }
        }
    }

    pickingAuton = false;
}

bool autonRunning = false;

void runAuton() { 
    autonRunning = true; // keeps the driver button task off the arm/claw
    if (autons[selectedAuton].run) autons[selectedAuton].run();
    autonRunning = false;
}
