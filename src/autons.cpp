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



void closeClaw() {
     pros::Task armTask([&]() {
        while (true) {
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
    moveCascadeTo(0);
    armPosition=false;
}

// ---------------------------------------------------------------------------
// Autonomous routes
// ---------------------------------------------------------------------------

void redRightAuton() {
    chassis.setPose(0, -62.5, 0);
    chassis.moveToPose(16, -52.5, 56, 5000);
    chassis.waitUntil(14);
    openClaw();
    moveArmToBack();
    chassis.moveToPose(23.5, -60, 0, 5000);
    chassis.waitUntil(2);
    closeClaw();
    chassis.moveToPoint(23.5, -56, 5000);
    moveArmToFront();
    chassis.waitUntil(2);
    openClaw();
    chassis.swingToPoint(-17.5, -29.5, lemlib::DriveSide::LEFT, 5000, {.forwards = false});
    moveArmToBack();
    chassis.moveToPoint(-17.5, -29.5, 5000);
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
    closeClaw();
    pros::delay(10000);
    // tuningPID();
    moveArmToBack();
    moveArmToFront();
}

void runAuton() { 
    testing(); 
}
