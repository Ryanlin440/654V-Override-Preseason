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
    moveCascadeTo((cascade_sensor.get_position() / 100.0)+1000, 500);
    moveArmToHold(downArmDegPinAndCup);
    moveCascadeTo(cascadeRestHeight);
}

// Waits in the background for an object, then grabs it. Gives up after
// timeoutMs so the task can't outlive auton and fire during driver control.
void closeClaw(int timeoutMs) {
     uint32_t start = pros::millis();
     pros::Task armTask([=]() {
        while (pros::millis() - start < (uint32_t)timeoutMs) {
            if (claw_sensor.get_distance() < 70) {
        
                // go down
                moveArmTo(downArmDegPinAndCupClamping, 300); 
                
                // clamp
                clawBool = true;
                clawShut.set_value(true);
                pros::delay(150);
                
                // lift back up
                moveArmTo(downArmDegPinAndCup+20, 600); 
                armNeedsDown = true; 
                break;
            }
            pros::delay(50);
            }
        });
    
}

void moveArmToBack() {
    
    if(armPosition == false){
        // arm_hold_pid.reset();
        armNeedsDown = false;
        armPosition = true;
        moveArmToHold(450);
    }else if (armPosition==true){
        cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        cascade.move_velocity(0);
        moveArmTo(650);
        
        
        openClaw();
    }
     if(armNeedsDown) {
        arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        arm_motor.move_velocity(0);
     }
};

void moveArmToFront() {
    pros::delay(100);
    cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    moveCascadeTo((cascade_sensor.get_position() / 100.0)+1000, 500);
    moveArmToHold(downArmDegPinAndCup);
    moveCascadeTo(cascadeClampingHeight);
    armPosition=false;
}

// ---------------------------------------------------------------------------
// Autonomous routes
// ---------------------------------------------------------------------------

void EastWestAuton() {
    // Toggle flip
    toggleMechDown();
    chassis.moveToPoint(55, 0, 1000, {.forwards = false});
    
    //Go score the preloaded pin
    chassis.turnToHeading(340, 1000);
    moveCascadeTo(500,200);
    chassis.moveToPoint(50, 15, 1000, {}, false);
    openClaw();

    // Get out of the way and face towards the first cup/pin
    chassis.moveToPose(52, 10, 300, 1000, {.forwards = false});

    // Get the first cup/pin and get it
    chassis.moveToPoint(30, 20, 2000);
    closeClaw();
    
    // Move to goal, move arm to back, and score
    chassis.moveToPoint(40, 23, 1000, {.forwards = false});
    moveArmToBack();
    openClawAndLiftThenPutBackFront();

    // Swing to the second cup/pin and get it
    chassis.swingToPoint(47, 30, lemlib::DriveSide::RIGHT, 5000);
    chassis.moveToPoint(47, 42, 2000);
    closeClaw();

    // Move back to goal and score
    chassis.moveToPoint(47, 30, 2000, {.forwards = false});
    moveArmToBack();
    openClawAndLiftThenPutBackFront();

    // Swing to the third cup/pin and get it
    chassis.swingToPoint(54, 23, lemlib::DriveSide::RIGHT, 5000);
    chassis.moveToPoint(62, 23, 2000);
    closeClaw();

    // Rotate to face neutral goal and go to it backwards, move arm to back and then score it.
    chassis.turnToHeading(20, 1000);
    chassis.moveToPoint(50, -15, 2000, {.forwards = false}, false);
    moveArmToBack();
    openClawAndLiftThenPutBackFront();

    // Swing to the fourth cup/pin and get it
    chassis.swingToPoint(54, -23, lemlib::DriveSide::RIGHT, 5000);
    chassis.moveToPoint(62, -23, 2000);
    closeClaw();

    // Move back to goal and score
    chassis.moveToPoint(54, -23, 2000, {.forwards = false});
    moveArmToBack();
    openClawAndLiftThenPutBackFront();

}

void SkillsAuton() {
    toggleMechDown();
    chassis.moveToPoint(55, 0, 1000, {.forwards = false});
    // chassis.moveToPoint(63, 0, 1000, {}, false);
    // chassis.moveToPoint(59, 0, 1000, {.forwards = false});
    // chassis.moveToPoint(63, 0, 1000, {}, false);

    
    chassis.turnToHeading(340, 1000);
    moveCascadeTo(500,200);
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
    int line = 0;
    chassis.setPose(0, 0, 0);
    int secondVar = 0;
    while (frontSensor.get() > 500) {
        chassis.arcade(127, 0);
        counter++;
        pros::lcd::print(line++, "%d sec: %f", counter, chassis.getPose().y);
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
    pros::lcd::print(7, "Final Velocity: .4%f", vertical_rotation_sensor.get_velocity());
    int count = 0;
    while (vertical_rotation_sensor.get_velocity() > 1) {
        count++;
        pros::delay(50);
    }
    pros::lcd::print(8, "Time till stop: .4%f seconds",
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
        pros::lcd::print(1, "Current voltage: ", torqueCount);
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            left_motors.move(torqueCount);
        } else {
            left_motors.move(0);
        }
    }
}

void tuningPID(){
    chassis.turnToHeading(90, 2000);
    chassis.turnToHeading(180, 2000);
    chassis.turnToHeading(270, 2000);
    chassis.turnToHeading(0, 2000);
}

void testing(){
    moveArmTo(downArmDegPinAndCup);
    closeClaw(10000);
    pros::delay(10000);
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
    {"EastWest Auton", EastWestAuton, 62.5, 0, 90},
    {"Skills Auton", SkillsAuton, 62.5, 0, 90},
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
    pros::lcd::print(6, "Auton: %s", route.name);

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

void runAuton() { 
    if (autons[selectedAuton].run) autons[selectedAuton].run();
}
