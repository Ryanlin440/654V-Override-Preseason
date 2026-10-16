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

void openClaw() {//for scoring only
    clawShut.set_value(false);
    clawBool=false;
    armNeedsDown = true;
};



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

void redRightAuton() {
    chassis.setPose(0, -62.5, 0);
    //manually closing the claw
    clawBool = true;
    clawShut.set_value(true);
    moveArmTo(downArmDegPinAndCup+30, 600); 
    armNeedsDown = true; 

    chassis.moveToPose(28, -50, 75, 2000, {}, false);
    moveCascadeTo(0);
    openClaw();
    // moveArmToBack();
    // chassis.moveToPose(23.5, -60, 0, 5000);
    // chassis.waitUntil(2);
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
// Auton selector -- shown on controller line 2. Left/Right to choose, X to
// lock it in. initialize() waits here until X is pressed. To add a route,
// write the function above and add a line here (keep Testing last).
// ---------------------------------------------------------------------------

struct AutonRoute {
    const char* name;
    void (*run)(); // nullptr = no auton, straight to driver control
};

AutonRoute autons[] = {
    {"Driver", nullptr},
    {"Red Right", redRightAuton},
    {"Testing", testing},
};
constexpr int autonCount = sizeof(autons) / sizeof(autons[0]);

int selectedAuton = 0; // starts on Driver
bool pickingAuton = true; // true from boot so nothing runs before the picker is done

void autonSelector() {
    pickingAuton = true; // screen task stops printing to controller line 2
    pros::delay(100);    // let its last print go through first

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
        if (pros::competition::is_connected() && !pros::competition::is_disabled()) {
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

    pros::lcd::print(6, "Auton: %s", autons[selectedAuton].name);
    pickingAuton = false;
}

void runAuton() { 
    if (autons[selectedAuton].run) autons[selectedAuton].run();
}
