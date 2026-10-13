#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "robot.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
pros::MotorGroup right_motors({11, 12, -13}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup left_motors({-20, -19, 18}, pros::MotorGearset::blue); // right motors use 200 RPM cartridges
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.6, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// pros::Distance backSensor(20);
pros::Distance frontSensor(15);
pros::Distance leftSensor(17);
pros::Distance rightSensor(14);

pros::Imu imu(5);
pros::Rotation vertical_rotation_sensor(-21);
pros::Rotation horizontal_rotation_sensor(4);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation_sensor, lemlib::Omniwheel::NEW_275, -1);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation_sensor, lemlib::Omniwheel::NEW_275, -1.5);

pros::MotorGroup cascade ({1, -2}, pros::v5::MotorGears::blue);//2 11w
pros::Rotation cascade_sensor(-3);
lemlib::PID cascade_pid(5,0,15,0);

void moveCascadeTo(double target, int timeoutMs) {
    cascade_pid.reset();
    uint32_t start = pros::millis();

    while (true) {
        float current_pos = cascade_sensor.get_position() / 100.0; 
        float error = target - current_pos;
        float output = cascade_pid.update(error);

        output = std::clamp(output, -127.0f, 127.0f);
        cascade.move(output);

        // stop moving if the target reached or if timer run out
        if (std::abs(error) < 5.0 || pros::millis() - start > (uint32_t)timeoutMs) {
            cascade.move_velocity(0);
            break;
        }
        pros::delay(10);
    }
}

// TODO: placeholder port/gearset -- update to the real toggles port(s)
bool clawBool = false;
pros::adi::DigitalOut clawShut(7, clawBool);
// TODO: placeholder ADI ports -- update to the real toggle ports
bool flipBool = false;
pros::adi::DigitalOut toggles(8, false);

pros::Distance claw_sensor(9);
pros::Distance backTopSensor(8);

pros::MotorGroup arm_motor ({6, -7}, pros::v5::MotorGears::green);//2 5.5
pros::Rotation arm_sensor(10);
lemlib::PID arm_pid(8,0,70,0);     
lemlib::PID arm_hold_pid(7,0,50,0); 
void moveArmTo(double target, int timeoutMs){
    arm_pid.reset();
    uint32_t start = pros::millis();
    while (true) {
        float current_pos = arm_sensor.get_position() / 100.0; 
        float error = target - current_pos;
        float output = arm_pid.update(error);
        output = std::clamp(output, -127.0f, 127.0f);
        arm_motor.move(output);
        if (std::abs(error) < 3.0 || pros::millis() - start > (uint32_t)timeoutMs) {
            arm_motor.move(0);
            break;
        }
        pros::delay(10);
    }
}
void moveArmToHold(double target, int timeoutMs){

    arm_hold_pid.reset();
    uint32_t start = pros::millis();
    while (true) {
        float current_pos = arm_sensor.get_position() / 100.0; 
        float error = target - current_pos;
        float output = arm_hold_pid.update(error);
        output = std::clamp(output, -127.0f, 127.0f); // cap arm power (move() range is -127 to 127)

        arm_motor.move(output);

        // give up after timeoutMs so the arm task can't get stuck here forever
        if (std::abs(error) < 1.0 || pros::millis() - start > (uint32_t)timeoutMs) {
            arm_motor.set_brake_mode(pros::MotorBrake::hold);
            //arm_motor.move_voltage(0);
            arm_motor.move_velocity(0);
            break;
        }
        pros::delay(10);
    }

}


lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(20, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              .1, // integral gain (kI)
                                              11, // derivative gain (kD)
                                              6, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.025 // expo curve gain
);
//both previously 1.019
// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.025 // expo curve gain
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, 
                        &steer_curve
);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
int pressNum = 0;
// Cleared by turnTunerAuton() so the screen task stops overwriting its readout.
bool printingDistances = true;
bool armNeedsDown = true;
bool isScoring = false;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    cascade_sensor.reset_position();
    arm_sensor.reset_position();
    arm_motor.set_brake_mode(pros::MotorBrake::hold);
    // arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // while (true) {
    //         // print robot location to the brain screen
    //         // pros::lcd::print(3, "Front sensor: %.2f", (double)(frontSensor.get() / 25.4));
    //         pros::lcd::print(4, "asdl %d", count);
    //         // delay to save resources
    //         count+=5;
    //         pros::delay(200);
    //     }
    pros::Task screenTask([&]() {
        // pros::lcd::print(0, "TASK RUNNING");
        // count = 20;
        while (true) {
            // print robot location to the brain screen
            if (not printingDistances) {
                pros::delay(40);
                continue;
            }
            controller.print(2, 0, "%.2f", chassis.getPose().theta);
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Front sensor: %.2f", (double)(frontSensor.get() / 25.4));
            pros::lcd::print(4, "Cascade Sensor: %.2f", (double)(cascade_sensor.get_position()/100.0));
            // pros::lcd::print(4, "asdl %d", count);
            // count+=2;
            // delay to save resources
            pros::delay(40);
            //
        }
    });
    pros::Task armTask([&]() {
        while (true) {
            L1Button();

            L2Button();

            R1Button();
            

            pros::delay(50);
            }
        });

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() { 
    tuningPID(); 
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void rotationChassisPID(){
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 1000);
    pros::delay(2000);
    chassis.turnToHeading(180, 1000);
    pros::delay(2000);
    chassis.turnToHeading(360, 1000);
    pros::delay(2000);
}

void linearChassisPID(){
    chassis.setPose(0, 0, 0);
    chassis.turnToPoint(24, 0, 2000);
    pros::delay(2000);
    chassis.turnToPoint(48, 0, 2000);
    pros::delay(2000);
    chassis.turnToPoint(0, 0, 2000);
    pros::delay(2000);
}

void opcontrol() {
    // loop forever
    // autonomous();

    // rotationChassisPID();
    // linearChassisPID();
    
    // return;
    // while(true){
    //     arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //     arm_motor.move_velocity(0);
    // }

    moveArmTo(downArmDegPinAndCup);

    while (true) {

        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        if (!isScoring){
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){//cascade movement
                cascade.move(127);
            }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
                cascade.move(-127);
                std::cout << "Testing" << std::endl;
            }else{
                cascade.move(0);
            }
        }
        
        toggleMech();
        // L1Button();
        // L2Button();
        // R1Button();
        //cascade winch toggle
        // delay to save resources
        pros::delay(25);
    }
}

void L1Button() {
    static bool l1WasPressed = false;
    static bool autoClampedThisPress = false;

    bool l1CurrentlyPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    // button pressed
    if (l1CurrentlyPressed && !l1WasPressed) {
        autoClampedThisPress = false; 
    }
    
    // if the button is held down
    if (l1CurrentlyPressed) {
        if (clawBool == false && !autoClampedThisPress) {
            if (claw_sensor.get_distance() < 70) {
                autoClampedThisPress = true;
                
                // go down
                moveArmTo(downArmDegPinAndCupClamping, 300); 
                
                // clamp
                clawBool = true;
                clawShut.set_value(true);
                pros::delay(50);
                
                // lift back up
                moveArmTo(downArmDegPinAndCup, 600); 
                armNeedsDown = true; 
            }
        }
    }

    // button released (manual clamp)
    if (!l1CurrentlyPressed && l1WasPressed) {
        if (!autoClampedThisPress) {
            
            // if open, then close the claw
            if (clawBool == false) {
                // go down
                moveArmTo(downArmDegPinAndCupClamping, 600); 
                
                // clamp
                clawBool = true;
                clawShut.set_value(true);
                pros::delay(150); 
                
                // go back up
                moveArmTo(downArmDegPinAndCup, 600); 
                armNeedsDown = true;
            } 
            // if closed, then open the claw and lower arm to score pin
            else {
                moveArmTo(0, 600);
                clawBool = false;
                clawShut.set_value(false);

                // get starting motor encoder position
                double startPosition = left_motors.get_position(); 
                uint32_t waitStart = pros::millis();
                
                // loop until back away
                while (true) {
                    double currentPosition = left_motors.get_position();
                    double degreesMoved = std::abs(currentPosition - startPosition);
                    
                    // move arm back up if moved back or timeout
                    if (degreesMoved > 150.0 || pros::millis() - waitStart > 2000) {
                        break;
                    }
                    
                    pros::delay(10); 
                }
                armNeedsDown = true;
                
                moveArmTo(downArmDegPinAndCup, 600);
            }
        }
    }

    // Save button state for next loop
    l1WasPressed = l1CurrentlyPressed;
}

void L2Button() {
    if (clawBool == true && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        moveArmTo(downArmDegJustPin, 600);

        clawShut.set_value(false);
        pros::delay(75);
        
        clawShut.set_value(true);
        pros::delay(150);
        
        moveArmTo(downArmDegPinAndCup, 600);
        armNeedsDown = true; 
    }
}

void R1Button(){
    if(pressNum==1){
        // moveArmToHold(300);
    }
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){//ARM movement

        if(pressNum == 0){
            // arm_hold_pid.reset();
            armNeedsDown = false;
            pressNum = 1;
            moveArmToHold(450);
            pros::lcd::print(5, "priming...");
        }else if (pressNum==1){
            isScoring = true;
            pros::lcd::print(5, "scoring...");
            cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            //moveCascadeTo(660);
            cascade.move_velocity(0);
            moveArmTo(650);
            
            pressNum=0;
            clawShut.set_value(false);
            clawBool=false;
            armNeedsDown = true;
            pros::delay(100);
            cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            moveCascadeTo((cascade_sensor.get_position() / 100.0)+1000, 500);
            moveArmToHold(downArmDegPinAndCup);
            moveCascadeTo(0);
            isScoring = false;
        }
        //if(armNeedsDown) moveArmTo(downArmDegPinAndCup+50);
    }
     if(armNeedsDown) {
        arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        arm_motor.move_velocity(0);
     }
};
void toggleMech(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        toggles.set_value(true);
        // pros::delay(1000);
    }else{
        toggles.set_value(false);
    }
};


