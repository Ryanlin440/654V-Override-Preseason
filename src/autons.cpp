#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"

bool armPosition = true;


//for scoring only
void openClaw() {
    clawShut.set_value(false);
    clawBool=false;
    // pros::delay(500);
};

bool armBusy = false;

void openClawAndLiftThenPutBackFront(bool async = false){
    if (async) {
        pros::Task([]() {
            while (armBusy) pros::delay(10);
            openClawAndLiftThenPutBackFront();
        });
        return;
    }
    clawShut.set_value(false);
    clawBool=false;
    pros::delay(200);
    moveCascadeTo(getCascadeInches() + cascadeLiftOffset+5, 500);
    moveArmToHold(downArmDegPinAndCup);
    moveCascadeTo(cascadeRestHeight,1000);
}


void closeClaw(int timeoutMs) {
     uint32_t start = pros::millis();
     pros::Task armTask([=]() {
        while (pros::millis() - start < (uint32_t)timeoutMs) {
            if (clawBool == false && claw_sensor.get_distance() < 50) {

                // 1. Lower cascade, lower arm VERY gently against gravity (30.0f)
                moveCascadeTo(cascadeClampingHeight, 200);
                moveArmTo(downArmDegPinAndCupClamping, 300, 30.0f);

                // 2. Clamp the object
                clawBool = true;
                clawShut.set_value(true);
                pros::delay(150);

                // 3. Lift both back to their resting heights
                moveCascadeTo(cascadeRestHeight, 600);
                moveArmTo(downArmDegPinAndCup, 600, 127.0f);
                armNeedsDown = true;
                break;
            }
            pros::delay(50);
            }
        });

}

void primeArm() {
    armNeedsDown = false;
    moveArmToHold(450);
}

void scoreArm() {
    cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    cascade.move_velocity(0);
    moveArmTo(750, 2000, 100.0f);
    openClaw();
    armNeedsDown = true;
    armPosition = true;
    
}

void moveArmToBack(bool async) {
    // wait for any background lift (e.g. from searchForStack) to finish
    while (armBusy) pros::delay(10);
    if (async) {
        armBusy = true;
        pros::Task([]() {
            // primeArm();
            scoreArm();
            armBusy = false;
        });
        return;
    }
    // primeArm();
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

void searchForStack(int timeoutMs, double cascadeHeight = cascadeRestHeight, int detectRange = 150, int approachSpeed = 55) {
    float center = chassis.getPose().theta;
    bool clawClosed = false;
    bool goLeft = false;
    uint32_t start = pros::millis();

    while (!clawClosed && pros::millis() - start < (uint32_t)timeoutMs) {
        float target = goLeft ? center - 20 : center + 20;
        chassis.turnToHeading(target, 1000, {.maxSpeed = 20});
        chassis.arcade(approachSpeed, 0);
        while (chassis.isInMotion()) {
            // Same auto-clamp as L1Button() in main.cpp
            if (clawBool == false && claw_sensor.get_distance() < detectRange) {
                chassis.cancelMotion();

                // creep straight at the stack until it's right in the claw
                uint32_t approachStart = pros::millis();
                while (claw_sensor.get_distance() >= 60 && pros::millis() - approachStart < 1500) {
                    chassis.arcade(approachSpeed, 0);
                    pros::delay(10);
                }

                // Lower cascade, lower arm VERY gently against gravity (30.0f)
                moveCascadeTo(0, 1000);
                moveArmTo(-10, 500, 60.0f);

                //Clamp the object
                clawBool = true;
                clawShut.set_value(true);
                chassis.arcade(approachSpeed, 0);
                pros::delay(150);
                chassis.arcade(0, 0);

                // Lift both back to their resting heights in the background so
                // the next chassis movement can start right away
                armBusy = true;
                pros::Task([=]() {
                    moveCascadeTo(cascadeHeight, 600);
                    primeArm();
                    armNeedsDown = true;
                    armBusy = false;
                });
                clawClosed = true;
                break;
            }
            pros::delay(10);
        }

        if (!clawClosed) goLeft = !goLeft;
    }
}


bool turningAgainstGoal = false;

void turnAgainstGoal(lemlib::AngularDirection direction, float targetHeading, int timeoutMs = 3500, bool async = false, int turnSpeed = 90, int backSpeed = 60) {
    if (async) {
        turningAgainstGoal = true;
        pros::Task([=]() {
            turnAgainstGoal(direction, targetHeading, timeoutMs, false, turnSpeed, backSpeed);
            turningAgainstGoal = false;
        });
        return;
    }

    // +1 if the heading should go up during this turn (clockwise), -1 if down
    int dir;
    if (direction == lemlib::AngularDirection::CW_CLOCKWISE) dir = 1;
    else if (direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE) dir = -1;
    else dir = std::remainder(targetHeading - chassis.getPose().theta, 360.0f) > 0 ? 1 : -1;

    // seat firmly against the goal first: full speed straight back for 500ms
    chassis.arcade(-127, 0);
    pros::delay(500);

    uint32_t start = pros::millis();
    while (pros::millis() - start < (uint32_t)timeoutMs) {
        float error = std::remainder(targetHeading - chassis.getPose().theta, 360.0f);
        if (error * dir <= 1) break; // reached (or just passed) the target

        chassis.arcade(-backSpeed, turnSpeed * dir);
        pros::delay(10);
    }
    chassis.arcade(0, 0);
}

void goBackwards(int time = 500, bool async = true) {
    if (async) {
        pros::Task([=]() {
            goBackwards(time, false);
        });
        return;
    }
    chassis.arcade(-127, 0);
    pros::delay(time);
    chassis.arcade(0, 0);
}

void waitForTurnAgainstGoal() {
    while (turningAgainstGoal) pros::delay(10);
}

static void seperatePinBodyHelper() {
    // lower cascade to 0
    moveCascadeTo(cascadeClampingHeight, 600);
    moveArmTo(downArmDegJustPin, 600);

    // open claw for a small interval to release cup
    clawShut.set_value(false);
    pros::delay(75);
    clawShut.set_value(true);
    pros::delay(150);
    
    // return arm to resting height
    moveCascadeTo(cascadeRestHeight, 600);
    moveArmTo(downArmDegPinAndCup, 600);
    armNeedsDown = true;
}

void seperatePin(bool async = true){
    // wait for any background arm move to finish
    while (armBusy) pros::delay(10);
    armBusy = true;
    if (async) {
        pros::Task([]() {
            seperatePinBodyHelper();
            armBusy = false;
        });
        return;
    }
    seperatePinBodyHelper();
    armBusy = false;
}
// ---------------------------------------------------------------------------
// Autonomous routes
// ---------------------------------------------------------------------------

void EastWestAuton() {
    // Toggle flip
    //move this back when there is wall - testing without wall
    chassis.setPose(61.5,0,90);
    distanceReset(90);
    toggleMechDown();
    chassis.moveToPoint(58, 0, 400, {.forwards = false});
    chassis.moveToPoint(72, 0, 300);
    chassis.moveToPoint(58, 0, 400, {.forwards = false});
    chassis.moveToPoint(72, 0, 300);

    chassis.moveToPoint(47, 0, 1000, {.forwards = false});
    moveArmToHold(downArmDegPinAndCup, 1000, 60.0f);
    moveCascadeTo(3,200);
    //Go score the preloaded pin

    chassis.turnToPoint(47, 18, 500, {});//uncommented to make faster

    chassis.moveToPoint(47, 18, 1000);
    pros::delay(500);
    moveArmToHold(0);
    moveCascadeTo(0,1000);
    openClaw();
    pros::delay(250);

    // Get out of the way and face towards the first cup/pin
    chassis.turnToPoint(47, 10, 200, {.forwards = false});
    chassis.moveToPoint(47,10, 400, {.forwards = false});
    chassis.turnToPoint(32, 17, 700);
    moveArmToHold(downArmDegPinAndCup);
    moveCascadeTo(cascadeRestHeight,1000);

    // Get the first cup/pin and get it
    chassis.moveToPoint(32, 18, 2000);
    searchForStack(1000, cascadeRestHeight);
    pros::delay(250);
    
    // Move to goal, move arm to back, and score
    chassis.turnToPoint(35, 30, 500, {.forwards = false});
    chassis.moveToPoint(35, 30, 500, {.forwards = false});

    
    chassis.moveToPoint(44, 25, 800, {.forwards = false}, false);
    moveArmToBack();
    openClawAndLiftThenPutBackFront(true);
    pros::delay(250);
    

    // second cup/pin and get it
    chassis.turnToPoint(47, 38, 2000);
    chassis.moveToPoint(47, 38, 2000);
    pros::delay(1000);
    searchForStack(1000, 10);

    // // chassis.turnToHeading(315, 1000, {.maxSpeed = 30});
    chassis.turnToPoint(47, 30, 1000, {.forwards=false});
    chassis.moveToPoint(47, 30, 2000, {.forwards=false});
    pros::delay(500);
    moveArmToBack();
    //make cascade go higher here
    openClawAndLiftThenPutBackFront(true);


    // // Move back to goal and score
    // // chassis.moveToPoint(56, 45, 1000, {.forwards = false});
    // chassis.moveToPoint(55, 31, 1000, {.forwards = false});
    // chassis.moveToPoint(51, 25, 1000, {.forwards = false});
    // turnAgainstGoal(lemlib::AngularDirection::CW_CLOCKWISE, 90, 3500, true);
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

    // // Swing to the third cup/pin and get it
    // chassis.moveToPoint(65, 29, 2000);
    // searchForStack(2000);

    // // Rotate to face neutral goal and go to it backwards, move arm to back and then score it.
    // chassis.turnToHeading(20, 1000);
    // chassis.moveToPoint(53, -20, 2000, {.forwards = false}, false);
    // turnAgainstGoal(lemlib::AngularDirection::CW_CLOCKWISE, 90, 3500, true);
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

    // // Swing to the fourth cup/pin and get it
    // chassis.moveToPoint(68, -30, 2000);
    // closeClaw();

    // // Move back to goal and score
    // chassis.moveToPoint(54, -30, 2000, {.forwards = false});
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

}

void NorthSouthAuton() {
    // Toggle flip
    //move this back when there is wall - testing without wall

    chassis.moveToPoint(0, -53, 200, {.forwards = false});
    toggleMechDown();
    moveArmToHold(350);
    chassis.moveToPoint(0, -72, 400);
    chassis.moveToPoint(0, -55, 400, {.forwards = false});
    chassis.moveToPoint(0, -72, 400);

    // chassis.setPose(63, 0, 90);
    chassis.moveToPoint(0, -46, 2000, {.forwards = false});

    
    

    //Go score the preloaded pin
    chassis.turnToPoint(-20, -46, 2000, {});
    moveArmToHold(downArmDegPinAndCup, 1000, 60.0f);
    moveCascadeTo(2.569,200); // change height HEREREARASERASEFASFE
    chassis.moveToPoint(-20, -46, 2000, {}, false);
    moveArmToHold(0);
    moveCascadeTo(0,1000);
    openClaw();
    pros::delay(200);

    // Get out of the way and face towards the first cup/pin
    chassis.turnToPoint(-10, -46, 200, {.forwards = false});
    chassis.moveToPoint(-10, -46, 800, {.forwards = false});
    // chassis.turnToHeading(300, 1000);
    // pros::delay(5000);
    chassis.turnToPoint(-19, -59, 500);
    moveArmToHold(downArmDegPinAndCup);
    moveCascadeTo(cascadeRestHeight,1000);
    // Get the first cup/pin and get it
    chassis.moveToPoint(-15, -59, 2000);
    
    // Move to goal, move arm to back, and score
    // chassis.turnToHeading(200, 1000, {.maxSpeed = 60});
    chassis.turnToPoint(-27, -50, 500, {.forwards = false});
    chassis.moveToPoint(-27, -50, 400, {.forwards = false});
    chassis.moveToPoint(-24, 45,250, {}, false);

    chassis.moveToPoint(-24, -66, 1000, {}, false);
    searchForStack(1000);
    chassis.moveToPoint(-24, -50, 1000, {.forwards = false});

    pros::delay(250);
    moveArmToBack();
    openClawAndLiftThenPutBackFront(true);
    distanceReset();

    // // Move back to goal and score
    // // chassis.moveToPoint(56, 45, 1000, {.forwards = false});
    // chassis.moveToPoint(55, 31, 1000, {.forwards = false});
    // chassis.moveToPoint(51, 25, 1000, {.forwards = false});
    // turnAgainstGoal(lemlib::AngularDirection::CW_CLOCKWISE, 90, 3500, true);
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

    // // Swing to the third cup/pin and get it
    // chassis.moveToPoint(65, 29, 2000);
    // searchForStack(2000);

    // // Rotate to face neutral goal and go to it backwards, move arm to back and then score it.
    // chassis.turnToHeading(20, 1000);
    // chassis.moveToPoint(53, -20, 2000, {.forwards = false}, false);
    // turnAgainstGoal(lemlib::AngularDirection::CW_CLOCKWISE, 90, 3500, true);
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

    // // Swing to the fourth cup/pin and get it
    // chassis.moveToPoint(68, -30, 2000);
    // closeClaw();

    // // Move back to goal and score
    // chassis.moveToPoint(54, -30, 2000, {.forwards = false});
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

}


void SkillsAuton() {

    EastWestAuton();
    pros::delay(1000);

    chassis.turnToPoint(47, 58.5, 1000);
    chassis.moveToPoint(47, 58.5, 1000);

    chassis.turnToPoint(65, 58.5, 1000);
    chassis.moveToPoint(65, 58.5, 1000);

    searchForStack(1000, cascadeRestHeight);

    chassis.turnToPoint(24, 58.5, 1000,{.forwards=false});
    chassis.moveToPoint(24, 58.5, 1000, {.forwards=false});

    seperatePin();

    moveArmToHold(downArmDegPinAndCup, 1000, 60.0f);
    moveCascadeTo(3,200);

    chassis.turnToPoint(24, 52, 1000);
    chassis.moveToPoint(24, 52, 1000);
    pros::delay(500);
    moveArmToHold(0);
    moveCascadeTo(0,1000);
    openClaw();

    // // Move in front of matchloader
    // chassis.turnToHeading(70, 1000);
    // chassis.moveToPoint(48, 80, 2000);
    // chassis.turnToHeading(90, 1000);
    // chassis.moveToPoint(65, 80, 2000);

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
    chassis.turnToHeading(90, 2000);
    pros::delay(4000);
    chassis.turnToHeading(180, 2000);
    pros::delay(4000);
    chassis.turnToHeading(270, 2000);
    pros::delay(4000);
    chassis.turnToHeading(0, 2000);

    // chassis.moveToPoint(0,12, 2000);
    // pros::delay(4000);
    // chassis.moveToPoint(0,24, 2000);
    // pros::delay(4000);
    // chassis.moveToPoint(0,48, 2000);
    // pros::delay(4000);
    // chassis.moveToPoint(0,0, 2000, {.forwards=false});


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
    {"EastWest Auton", EastWestAuton, 61.5, 0, 90},//in the future change this value, cause this is not the correct starting position
    {"NorthSouth Auton", NorthSouthAuton, 0, -61.5, 180},//in the future change this value, cause this is not the correct starting position
    {"Skills Auton", SkillsAuton, 63, 0, 90},
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
