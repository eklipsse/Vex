// Your path to the pros instalation may vary, but this is the default path for a regular installation
#include "pros/apix.h"
#include "RobotConfig.hpp"

// The assumption is that you have defined the required objects in some other file, for example RobotConfig.hpp
// So colorSortSensor, intake and mainController are defined
// if you have different names, change them accordingly in the code below

/**
 * @brief Enumeration of alliance colors.
 * Used to determine the robot's team color and apply related logic.
 */
enum AllianceColor
{
    RED,
    BLUE,
    UNKNOWN
};

/**
 * @brief Global variable representing the current alliance color.
 * Initialized to red by default, but can be changed to blue if needed.`
 * Suggest having your autonomous routines automatically set the ALLIANCE_COLOR
 */
AllianceColor ALLIANCE_COLOR = RED;

/**
 * @brief Detects the color using the colorSortSensor.
 *
 * Uses the hue reading to determine if the detected color is RED, BLUE, or UNKNOWN.
 *
 * @return AllianceColor The detected color as an AllianceColor enum value.
 */
AllianceColor detectColor()
{
    int hue = colorSortSensor.get_hue();

    if (hue >= 330 || hue <= 30)
    {
        return RED;
    }
    else if (hue >= 210 && hue <= 270)
    {
        return BLUE;
    }
    else
    {
        return UNKNOWN; // Define UNKNOWN in your enum if needed
    }
}

/**
 * @brief Task function to handle color sorting logic.
 *
 * This function continuously monitors objects detected by the color sorting sensor.
 * It determines whether the detected object matches the ALLIANCE_COLOR and
 * controls the intake motor to either allow or reject the object.
 * This function should be run as a separate task to avoid blocking the main loop.
 */
void colorSortTask()
{
    // Constants for motor behavior during color sorting
    constexpr int TRAVEL_DELAY = 100; // Delay (ms) before stopping to eject
    constexpr int STOP_DELAY = 200;   // Delay (ms) to ensure ejection
    constexpr int INTAKE_SPEED = 100; // Default motor speed for intake

    while (true)
    {
        // Detect the current color of the object using the sensor
        AllianceColor detectedColor = detectColor();

        // Scenario 1: Detected color matches the alliance color
        if (detectedColor == ALLIANCE_COLOR)
        {
            // Display a message indicating a color match
            mainController.set_text(2, 0, "Color Match!");
            // Intake motor continues to operate normally
        }
        // Scenario 2: Detected color does not match the alliance color
        else if (detectedColor != UNKNOWN)
        {
            // Brief delay to allow the object to reach the eject position
            pros::delay(TRAVEL_DELAY);

            // Stop the intake motor momentarily to eject the object
            intake.move_velocity(0);

            // Display a message indicating a color mismatch
            mainController.set_text(2, 0, "Color Mismatch!");

            // Allow time for the object to be ejected via inertia
            pros::delay(STOP_DELAY);

            // Resume normal intake motor operation
            intake.move_velocity(INTAKE_SPEED);
        }
        // Scenario 3: No object is detected by the sensor
        else
        {
            // Display a message indicating that no object is detected
            mainController.set_text(2, 0, "No Ring!");
            // No changes to the intake motor state; remains under external control
        }

        // Add a small delay to prevent excessive sensor polling or spamming messages
        pros::delay(50);
    }
}

/**
 * @brief Starts the color sorting task if not already running.
 */
void startColorSortTask()
{
    if (colorSortTaskHandle == nullptr)
    {
        colorSortTaskHandle = new pros::Task(colorSortTask);
    }
}

/**
 * @brief Stops the color sorting task if it's running.
 */
void stopColorSortTask()
{
    if (colorSortTaskHandle != nullptr)
    {
        colorSortTaskHandle->remove();
        delete colorSortTaskHandle;
        colorSortTaskHandle = nullptr;
    }
}

/**
 * @brief Main operator control function.
 */
void opcontrol()
{
    // Start the color sorting task
    startColorSortTask();
    intake.move_velocity(600);

    while (true)
    {
        // Driver control logic goes here
        pros::delay(20);
    }

    // Stop the task when transitioning out of opcontrol
    stopColorSortTask();
}
