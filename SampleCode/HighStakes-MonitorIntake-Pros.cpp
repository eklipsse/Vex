#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

/**
 * @file intake_control.cpp
 * @brief Implements intake monitoring and control logic for a robot using PROS.
 */

/**
 * Global pointer to the intake monitoring task.
 * Initialized to nullptr, ensuring no task is running by default.
 */
pros::Task *intake_monitor_task = nullptr;

/**
 * @brief Motor used for the intake mechanism.
 * @details Replace the port number (1) with the actual port your motor is connected to.
 */
pros::Motor intake_motor(1);

/**
 * @brief Enum to track the intake motor direction.
 */
enum IntakeDirection
{
    FORWARD,
    BACKWARD,
    STOPPED
};
IntakeDirection intake_direction = STOPPED; // Initialize to STOPPED

/**
 * @brief Spin-up grace period in milliseconds.
 * @details Allows the motor time to reach operating speed before checking for stalls.
 */
const int SPIN_UP_GRACE_MS_MS = 1000;

/**
 * @brief Desired velocity for the intake motor in RPM.
 */
const int DESIRED_VELOCITY = 600;

/**
 * @brief Threshold velocity below which the motor is considered stuck.
 * @details If the motor's velocity falls below this value, corrective action will be taken.
 */
const int VELOCITY_THRESHOLD = 50;

/**
 * @brief Degrees to reverse the intake motor when it is stuck.
 */
const int REVERSE_DEGREES = 90;

/**
 * @brief Speed for reversing the intake motor.
 */
const int REVERSE_SPEED = -100;

/**
 * @brief Starts the intake monitoring task.
 */
void startMonitoringTask()
{
    if (intake_monitor_task == nullptr)
    {
        intake_monitor_task = new pros::Task(intake_monitor_task_function, nullptr, "Intake Monitor Task");
    }
}

/**
 * @brief Stops the intake monitoring task.
 */
void stopMonitoringTask()
{
    if (intake_monitor_task != nullptr)
    {
        intake_monitor_task->remove(); // Stop the task
        delete intake_monitor_task;    // Free the allocated memory
        intake_monitor_task = nullptr;
    }
}

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
void intake_monitor_task_function(void *param)
{
    bool reversing = false;    // Track if the motor is currently reversing
    bool spin_up_grace = true; // Grace period flag to allow for "spooling", when the motor starts spinning the first time

    while (true)
    {
        // Get the current velocity of the intake motor
        double current_velocity = intake_motor.get_actual_velocity();

        // Allow a grace period for spin-up after the motor starts
        if (spin_up_grace)
        {
            pros::delay(SPIN_UP_GRACE); // Use configurable grace period for spin-up
            spin_up_grace = false;      // Disable grace period after initial delay
            continue;                   // Skip the stuck check during grace period
        }

        // Check if the intake motor is stuck
        if (!reversing && abs(current_velocity) < VELOCITY_THRESHOLD && intake_motor.get_target_velocity() != 0)
        {
            // Log a message to the LCD for debugging purposes
            pros::lcd::print(0, "Intake stuck! Reversing...");
            pros::lcd::print(2, "Current velocity: %.2f", current_velocity);

            // Reverse the intake motor to resolve the stall
            reversing = true; // Set reversing flag to avoid repeated reversals
            intake_motor.move_relative(-REVERSE_DEGREES, REVERSE_SPEED);

            // Wait for the reverse motion to complete
            while (abs(intake_motor.get_actual_velocity()) > 1)
            {
                pros::delay(10);
            }

            // Resume normal intake operation
            intake_motor.move_velocity(DESIRED_VELOCITY);
            reversing = false; // Reset the reversing flag
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
void opcontrol()
{
    while (true)
    {
        // Toggle forward intake with R1
        if (pros::controller_get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
        {
            if (intake_direction == FORWARD)
            {
                // If currently forward, stop the intake
                intake_motor.move_velocity(0);
                intake_direction = STOPPED;
                stopMonitoringTask(); // Stop monitoring
            }
            else
            {
                // Start forward intake
                intake_motor.move_velocity(DESIRED_VELOCITY); // Forward intake speed
                intake_direction = FORWARD;
                startMonitoringTask(); // Start monitoring
            }
        }

        // Toggle reverse intake with A
        if (pros::controller_get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            if (intake_direction == BACKWARD)
            {
                // If currently backward, stop the intake
                intake_motor.move_velocity(0);
                intake_direction = STOPPED;
                stopMonitoringTask(); // Stop monitoring
            }
            else
            {
                // Start reverse intake
                intake_motor.move_velocity(-DESIRED_VELOCITY); // Reverse intake speed
                intake_direction = BACKWARD;
                stopMonitoringTask(); // Stop monitoring (monitoring only for forward intake)
            }
        }

        // Delay to prevent excessive CPU usage
        pros::delay(20);
    }
}
