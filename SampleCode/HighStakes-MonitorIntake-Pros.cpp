#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

/**
 * @file intake_control.cpp
 * @brief Implements intake monitoring and control logic for a robot using PROS.
 */

/** 
 * @brief Motor used for the intake mechanism.
 * @details Replace the port number (1) with the actual port your motor is connected to.
 */
pros::Motor intake_motor(1);

/** 
 * @brief Desired velocity for the intake motor in RPM.
 */
const int desired_velocity = 200;

/** 
 * @brief Threshold velocity below which the motor is considered stuck.
 * @details If the motor's velocity falls below this value, corrective action will be taken.
 */
const int velocity_threshold = 50;

/** 
 * @brief Degrees to reverse the intake motor when it is stuck.
 */
const int reverse_degrees = 90;

/** 
 * @brief Speed for reversing the intake motor.
 */
const int reverse_speed = -100;

/**
 * @brief Task function that monitors the intake motor for stalls and takes corrective action.
 *
 * This task continuously checks the velocity of the intake motor. If the velocity falls below
 * the defined threshold while the motor is supposed to be running, it assumes the motor is stuck.
 * To resolve this, the motor is reversed for a certain number of degrees before resuming
 * normal operation.
 *
 * @param param Pointer to additional data passed to the task (not used here, can be nullptr).
 */
void intake_monitor_task(void* param) {
    while (true) {
        // Get the current velocity of the intake motor
        double current_velocity = intake_motor.get_actual_velocity();

        // Check if the intake motor is stuck
        if (abs(current_velocity) < velocity_threshold && intake_motor.get_target_velocity() != 0) {
            // Log a message to the LCD for debugging purposes
            pros::lcd::print(0, "Intake stuck! Reversing...");

            // Reverse the intake motor to resolve the stall
            intake_motor.move_relative(-reverse_degrees, reverse_speed);

            // Wait for the reverse motion to complete
            while (!intake_motor.is_stopped()) {
                pros::delay(10);
            }

            // Resume normal intake operation by setting the desired velocity
            intake_motor.move_velocity(desired_velocity);
        }

        // Delay to reduce CPU usage of the task
        pros::delay(20);
    }
}

/**
 * @brief Main operator control function for the robot.
 *
 * This function starts the intake monitoring task and handles manual control of the intake motor
 * based on controller input. The intake motor runs at the desired velocity when the R1 button is
 * pressed, and stops when the button is released.
 */
void opcontrol() {
    // Start the intake monitoring task
    pros::Task intake_monitor(intake_monitor_task, nullptr, "Intake Monitor Task");

    // Main operator control loop
    while (true) {
        // Check if the R1 button on the controller is pressed
        if (pros::controller_get_digital(E_CONTROLLER_DIGITAL_R1)) {
            // Run the intake motor at the desired velocity
            intake_motor.move_velocity(desired_velocity);
        } else {
            // Stop the intake motor
            intake_motor.move_velocity(0);
        }

        // Delay to prevent excessive CPU usage
        pros::delay(20);
    }
}
