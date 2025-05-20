#include "joystick_system.h"
#include "Debug.h"
#include "robot_system.h"
#include "stepper_system.h"
#include "display_system.h"
#include <EEPROM.h>

// ========================================================
// Joystick class implementation - from joystick.cpp
// ========================================================

#define JOYSTICK_EEPROM_ADDR 300
#define JOYSTICK_MAGIC 0x4A4F59 // "JOY" in ASCII

// Default values for joystick calibration
#define DEFAULT_MIN 0
#define DEFAULT_CENTER 512
#define DEFAULT_MAX 1023

namespace JoystickSystem {
    // Constructor
    Joystick::Joystick(int xPin, int yPin, int btnPin) 
        : _xPin(xPin), _yPin(yPin), _btnPin(btnPin),
          _xFilter(0.2f, 4.0f, 1.0f, 0.0f),  // Conservative filter settings
          _yFilter(0.2f, 4.0f, 1.0f, 0.0f)   // Same settings for Y
    {
        // Default calibration values
        _xMin = DEFAULT_MIN;
        _xCenter = DEFAULT_CENTER;
        _xMax = DEFAULT_MAX;
        _yMin = DEFAULT_MIN;
        _yCenter = DEFAULT_CENTER;
        _yMax = DEFAULT_MAX;
    }

    void Joystick::begin() {
        // Configure pins
        pinMode(_xPin, INPUT);
        pinMode(_yPin, INPUT);
        pinMode(_btnPin, INPUT_PULLUP); // Button uses internal pull-up
        
        // Try to load calibration from EEPROM
        struct {
            uint32_t magic;
            int xMin, xCenter, xMax;
            int yMin, yCenter, yMax;
        } calibData;
        
        EEPROM.get(JOYSTICK_EEPROM_ADDR, calibData);
        
        // Check if valid calibration data is stored
        if (calibData.magic == JOYSTICK_MAGIC) {
            _xMin = calibData.xMin;
            _xCenter = calibData.xCenter;
            _xMax = calibData.xMax;
            _yMin = calibData.yMin;
            _yCenter = calibData.yCenter;
            _yMax = calibData.yMax;
            
            DEBUG_PRINTLN("Loaded joystick calibration from EEPROM");
        } else {
            DEBUG_PRINTLN("Using default joystick calibration");
        }
        
        // Initial read
        read();
    }

    void Joystick::calibrate() {
        DEBUG_PRINTLN("Starting joystick calibration...");
        
        // Find center position (no input)
        DEBUG_PRINTLN("Center the joystick and wait...");
        delay(1000);
        
        // Take average of 10 readings for center position
        int xSum = 0, ySum = 0;
        for (int i = 0; i < 10; i++) {
            xSum += analogRead(_xPin);
            ySum += analogRead(_yPin);
            delay(50);
        }
        _xCenter = xSum / 10;
        _yCenter = ySum / 10;
        
        DEBUG_PRINTF("Center: X=%d, Y=%d\n", _xCenter, _yCenter);
        
        // Find min/max by asking user to move joystick
        DEBUG_PRINTLN("Move joystick to all extremes...");
        
        // Initialize min/max with center values
        _xMin = _xCenter;
        _xMax = _xCenter;
        _yMin = _yCenter;
        _yMax = _yCenter;
        
        // Monitor for 5 seconds to catch extremes
        unsigned long startTime = millis();
        while (millis() - startTime < 5000) {
            int x = analogRead(_xPin);
            int y = analogRead(_yPin);
            
            // Update min/max
            if (x < _xMin) _xMin = x;
            if (x > _xMax) _xMax = x;
            if (y < _yMin) _yMin = y;
            if (y > _yMax) _yMax = y;
            
            delay(10);
        }
        
        DEBUG_PRINTF("X: Min=%d, Center=%d, Max=%d\n", _xMin, _xCenter, _xMax);
        DEBUG_PRINTF("Y: Min=%d, Center=%d, Max=%d\n", _yMin, _yCenter, _yMax);
        
        // Add a little margin to min/max
        _xMin -= 5;
        _xMax += 5;
        _yMin -= 5;
        _yMax += 5;
        
        // Save calibration to EEPROM
        struct {
            uint32_t magic;
            int xMin, xCenter, xMax;
            int yMin, yCenter, yMax;
        } calibData;
        
        calibData.magic = JOYSTICK_MAGIC;
        calibData.xMin = _xMin;
        calibData.xCenter = _xCenter;
        calibData.xMax = _xMax;
        calibData.yMin = _yMin;
        calibData.yCenter = _yCenter;
        calibData.yMax = _yMax;
        
        EEPROM.put(JOYSTICK_EEPROM_ADDR, calibData);
        
        DEBUG_PRINTLN("Calibration saved to EEPROM");
    }

    void Joystick::read() {
        // Read raw values
        int xRaw = analogRead(_xPin);
        int yRaw = analogRead(_yPin);
        
        // Apply Kalman filter
        _xValue = _xFilter.updateEstimate(xRaw);
        _yValue = _yFilter.updateEstimate(yRaw);
    }

    int Joystick::getX() {
        return _xValue;
    }

    int Joystick::getY() {
        return _yValue;
    }

    bool Joystick::isPressed() {
        return (digitalRead(_btnPin) == LOW); // Active LOW with pull-up
    }

    float Joystick::getNormalizedX() {
        // Map to range -1.0 to 1.0
        if (_xValue < _xCenter) {
            // Left half
            return -map(_xValue, _xMin, _xCenter, 1000, 0) / 1000.0f;
        } else {
            // Right half
            return map(_xValue, _xCenter, _xMax, 0, 1000) / 1000.0f;
        }
    }

    float Joystick::getNormalizedY() {
        // Map to range -1.0 to 1.0
        if (_yValue < _yCenter) {
            // Top half (inverted)
            return map(_yValue, _yMin, _yCenter, 1000, 0) / 1000.0f;
        } else {
            // Bottom half (inverted)
            return -map(_yValue, _yCenter, _yMax, 0, 1000) / 1000.0f;
        }
    }

    float Joystick::getXWithDeadband(float deadbandPercentage) {
        float normalizedX = getNormalizedX();
        float deadband = deadbandPercentage / 100.0f;
        
        // Apply deadband
        if (abs(normalizedX) < deadband) {
            return 0.0f;
        } else if (normalizedX > 0) {
            // Rescale positive value to go from 0 to 1.0
            return (normalizedX - deadband) / (1.0f - deadband);
        } else {
            // Rescale negative value to go from -1.0 to 0
            return (normalizedX + deadband) / (1.0f - deadband);
        }
    }

    float Joystick::getYWithDeadband(float deadbandPercentage) {
        float normalizedY = getNormalizedY();
        float deadband = deadbandPercentage / 100.0f;
        
        // Apply deadband
        if (abs(normalizedY) < deadband) {
            return 0.0f;
        } else if (normalizedY > 0) {
            // Rescale positive value to go from 0 to 1.0
            return (normalizedY - deadband) / (1.0f - deadband);
        } else {
            // Rescale negative value to go from -1.0 to 0
            return (normalizedY + deadband) / (1.0f - deadband);
        }
    }

    void Joystick::startCalibration() {
        // This is a more explicit version of calibrate() that can be
        // controlled by an external state machine
        calibrate();
    }

    void Joystick::saveMinMax(int xMin, int xMax, int yMin, int yMax) {
        _xMin = xMin;
        _xMax = xMax;
        _yMin = yMin;
        _yMax = yMax;
        
        // Save to EEPROM
        struct {
            uint32_t magic;
            int xMin, xCenter, xMax;
            int yMin, yCenter, yMax;
        } calibData;
        
        calibData.magic = JOYSTICK_MAGIC;
        calibData.xMin = _xMin;
        calibData.xCenter = _xCenter;
        calibData.xMax = _xMax;
        calibData.yMin = _yMin;
        calibData.yCenter = _yCenter;
        calibData.yMax = _yMax;
        
        EEPROM.put(JOYSTICK_EEPROM_ADDR, calibData);
        
        DEBUG_PRINTLN("Custom calibration saved to EEPROM");
    }

    int Joystick::mapJoystickValue(int value, int minVal, int center, int maxVal) {
        // Map values to 0-1000 range with center at 500
        if (value < center) {
            return map(value, minVal, center, 0, 500);
        } else {
            return map(value, center, maxVal, 500, 1000);
        }
    }
    
        // ========================================================
    // JoystickSystem implementation - from joystick_system.cpp
    // ========================================================
    
    // Private variables for the JoystickSystem namespace
    static Joystick* leftJoystick = nullptr;
    static Joystick* rightJoystick = nullptr;
    
    // Button state tracking
    static ButtonState leftButtonState = BUTTON_IDLE;
    static ButtonState rightButtonState = BUTTON_IDLE;
    static unsigned long leftButtonPressTime = 0;
    static unsigned long rightButtonPressTime = 0;
    static bool lastLeftPressed = false;
    static bool lastRightPressed = false;
    
    // Double-click detection
    static unsigned long leftButtonLastReleaseTime = 0;
    static unsigned long rightButtonLastReleaseTime = 0;
    static bool leftButtonDoubleClick = false;
    static bool rightButtonDoubleClick = false;
    
    // Configuration
    static int joystickDeadband = 10; // 10% deadband default
    static float joystickSensitivity = 1.0f;
    
    // Operational variables
    static bool calibrationMode = false;
    static int calibrationStep = 0;
    static unsigned long calibrationStartTime = 0;
    static int selectedJoint = 0;
    static float jointSpeed = 1.0f;
    
    // Initialize the joystick system
    void init() {
        DEBUG_PRINTLN(F("Initializing joystick system..."));
        
        // Create joystick objects
        leftJoystick = new Joystick(_pinConfig.leftXPin, _pinConfig.leftYPin, _pinConfig.leftBtnPin);
        rightJoystick = new Joystick(_pinConfig.rightXPin, _pinConfig.rightYPin, _pinConfig.rightBtnPin);
        
        // Initialize the joysticks
        leftJoystick->begin();
        rightJoystick->begin();
        
        // Load configurations from EEPROM
        joystickDeadband = _joystickConfig.deadband;
        joystickSensitivity = _joystickConfig.sensitivity;
        
        DEBUG_PRINTLN(F("Joystick system initialized"));
    }
    
    // Update joystick readings
    void update() {
        // Read joystick values
        leftJoystick->read();
        rightJoystick->read();
        
        // Track button state changes
        bool leftPressed = leftJoystick->isPressed();
        bool rightPressed = rightJoystick->isPressed();
        
        // Left button state machine
        switch (leftButtonState) {
            case BUTTON_IDLE:
                if (leftPressed && !lastLeftPressed) {
                    leftButtonState = BUTTON_PRESSED;
                    leftButtonPressTime = millis();
                }
                break;
                
            case BUTTON_PRESSED:
                if (!leftPressed) {
                    leftButtonState = BUTTON_RELEASED;
                    
                    // Check for double-click
                    if (millis() - leftButtonLastReleaseTime < DOUBLE_CLICK_TIME) {
                        leftButtonDoubleClick = true;
                    }
                    leftButtonLastReleaseTime = millis();
                } else if (millis() - leftButtonPressTime > BUTTON_HOLD_TIME) {
                    leftButtonState = BUTTON_HELD;
                }
                break;
                
            case BUTTON_HELD:
                if (!leftPressed) {
                    leftButtonState = BUTTON_RELEASED;
                }
                break;
                
            case BUTTON_RELEASED:
                leftButtonState = BUTTON_IDLE;
                break;
        }
        
        // Right button state machine (similar to left)
        switch (rightButtonState) {
            case BUTTON_IDLE:
                if (rightPressed && !lastRightPressed) {
                    rightButtonState = BUTTON_PRESSED;
                    rightButtonPressTime = millis();
                }
                break;
                
            case BUTTON_PRESSED:
                if (!rightPressed) {
                    rightButtonState = BUTTON_RELEASED;
                    
                    // Check for double-click
                    if (millis() - rightButtonLastReleaseTime < DOUBLE_CLICK_TIME) {
                        rightButtonDoubleClick = true;
                    }
                    rightButtonLastReleaseTime = millis();
                } else if (millis() - rightButtonPressTime > BUTTON_HOLD_TIME) {
                    rightButtonState = BUTTON_HELD;
                }
                break;
                
            case BUTTON_HELD:
                if (!rightPressed) {
                    rightButtonState = BUTTON_RELEASED;
                }
                break;
                
            case BUTTON_RELEASED:
                rightButtonState = BUTTON_IDLE;
                break;
        }
        
        // Process calibration if in calibration mode
        if (calibrationMode) {
            processCalibration();
        }
        
        // Update last states
        lastLeftPressed = leftPressed;
        lastRightPressed = rightPressed;
    }
    
    // Process joysticks in Joint Mode
    void processJointModeJoysticks() {
        // Left joystick controls joint selection
        float leftX = leftJoystick->getXWithDeadband(joystickDeadband);
        float leftY = leftJoystick->getYWithDeadband(joystickDeadband);
        
        // Right joystick controls joint movement
        float rightY = rightJoystick->getYWithDeadband(joystickDeadband);
        
        // Detect when to change joint selection
        static unsigned long lastJointChange = 0;
        const unsigned long JOINT_CHANGE_DELAY = 300; // 300ms delay between joint changes
        
        if (millis() - lastJointChange > JOINT_CHANGE_DELAY) {
            if (leftX > 0.7f) {
                selectedJoint = (selectedJoint + 1) % 6;
                lastJointChange = millis();
                DEBUG_PRINT(F("Selected joint: "));
                DEBUG_PRINTLN(selectedJoint);
            } else if (leftX < -0.7f) {
                selectedJoint = (selectedJoint + 5) % 6; // +5 is same as -1 modulo 6
                lastJointChange = millis();
                DEBUG_PRINT(F("Selected joint: "));
                DEBUG_PRINTLN(selectedJoint);
            }
        }
        
        // Control joint speed
        static unsigned long lastSpeedChange = 0;
        const unsigned long SPEED_CHANGE_DELAY = 200; // 200ms delay between speed changes
        
        if (millis() - lastSpeedChange > SPEED_CHANGE_DELAY) {
            if (leftY > 0.7f) {
                jointSpeed = min(jointSpeed + 0.1f, 1.0f);
                lastSpeedChange = millis();
                DEBUG_PRINT(F("Joint speed: "));
                DEBUG_PRINTLN(jointSpeed);
            } else if (leftY < -0.7f) {
                jointSpeed = max(jointSpeed - 0.1f, 0.1f);
                lastSpeedChange = millis();
                DEBUG_PRINT(F("Joint speed: "));
                DEBUG_PRINTLN(jointSpeed);
            }
        }
        
        // Move the selected joint
        if (abs(rightY) > 0.1f) {
            // Scale movement with joint speed
            float scaledSpeed = rightY * jointSpeed;
            
            // Move the joint
            StepperSystem::moveJoint(selectedJoint, scaledSpeed);
            
            // Update the display
            DisplaySystem::displayJointMode(RobotSystem::getKinematics(), selectedJoint);
        } else {
            // Stop movement if joystick is centered
            StepperSystem::moveJoint(selectedJoint, 0);
        }
    }
    
    // Process joysticks in Kinematic Mode
    void processKinematicModeJoysticks() {
        // Get normalized joystick values with deadband
        float leftX = leftJoystick->getXWithDeadband(joystickDeadband);
        float leftY = leftJoystick->getYWithDeadband(joystickDeadband);
        float rightX = rightJoystick->getXWithDeadband(joystickDeadband);
        float rightY = rightJoystick->getYWithDeadband(joystickDeadband);
        
        // Get current pose
        CartesianPose currentPose = RobotSystem::getKinematics()->getCurrentPose();
        
        // Scale movements by sensitivity and a base speed
        const float BASE_LINEAR_SPEED = 2.0f; // 2mm per update
        const float BASE_ANGULAR_SPEED = 0.05f; // ~3 degrees per update
        
        // Apply joystick values to pose
        currentPose.x += leftX * BASE_LINEAR_SPEED * joystickSensitivity;
        currentPose.y += rightX * BASE_LINEAR_SPEED * joystickSensitivity;
        currentPose.z += rightY * BASE_LINEAR_SPEED * joystickSensitivity;
        
        // Rotation control with left joystick Y and button combinations
        if (leftJoystick->isPressed()) {
            // Yaw control (Z-axis rotation)
            currentPose.yaw += leftY * BASE_ANGULAR_SPEED * joystickSensitivity;
        } else if (rightJoystick->isPressed()) {
            // Pitch control (Y-axis rotation)
            currentPose.pitch += leftY * BASE_ANGULAR_SPEED * joystickSensitivity;
        } else {
            // Roll control (X-axis rotation)
            currentPose.roll += leftY * BASE_ANGULAR_SPEED * joystickSensitivity;
        }
        
        // Ensure angles stay within valid range
        while (currentPose.yaw > M_PI) currentPose.yaw -= 2*M_PI;
        while (currentPose.yaw < -M_PI) currentPose.yaw += 2*M_PI;
        while (currentPose.pitch > M_PI) currentPose.pitch -= 2*M_PI;
        while (currentPose.pitch < -M_PI) currentPose.pitch += 2*M_PI;
        while (currentPose.roll > M_PI) currentPose.roll -= 2*M_PI;
        while (currentPose.roll < -M_PI) currentPose.roll += 2*M_PI;
        
        // Check if pose is within workspace
        if (RobotSystem::isWithinWorkspace(currentPose)) {
            // Update robot with new pose
            RobotSystem::moveToPose(currentPose, false);
            
            // Update the display
            DisplaySystem::displayPositionMode(RobotSystem::getKinematics());
        } else {
            // Pose is outside workspace - show warning
            DEBUG_PRINTLN(F("Target pose outside workspace!"));
            DisplaySystem::showError("Outside workspace!");
        }
    }
    
    // Start calibration mode
    void startCalibration() {
        DEBUG_PRINTLN(F("Starting joystick calibration..."));
        calibrationMode = true;
        calibrationStep = 0;
        calibrationStartTime = millis();
        
        // Show calibration information
        DisplaySystem::showMessage("Calibrating...", "Center joysticks");
    }
    
        // Process calibration steps
    void processCalibration() {
        const int STEP_DELAY = 2000; // 2 seconds per step
        
        switch (calibrationStep) {
            case 0: // Center calibration
                if (millis() - calibrationStartTime > STEP_DELAY) {
                    // Save center positions
                    int leftXCenter = leftJoystick->getX();
                    int leftYCenter = leftJoystick->getY();
                    int rightXCenter = rightJoystick->getX();
                    int rightYCenter = rightJoystick->getY();
                    
                    DEBUG_PRINTF("Center: L(%d,%d) R(%d,%d)\n", 
                        leftXCenter, leftYCenter, rightXCenter, rightYCenter);
                    
                    // Move to next step
                    calibrationStep = 1;
                    calibrationStartTime = millis();
                    
                    // Show instruction
                    DisplaySystem::showMessage("Move joysticks", "to all extremes");
                }
                break;
                
            case 1: // Find extremes
                if (millis() - calibrationStartTime > 5000) { // 5 seconds to move joysticks
                    // Complete calibration
                    DisplaySystem::showMessage("Calibration", "completed!");
                    
                    // Save calibration to EEPROM
                    saveCalibration();
                    
                    // Exit calibration mode
                    calibrationMode = false;
                    calibrationStep = 0;
                    delay(1000); // Show completion message
                }
                break;
        }
    }
    
    // Save calibration data
    void saveCalibration() {
        // This is just a placeholder
        // The actual calibration is saved in the Joystick class already
        DEBUG_PRINTLN(F("Saving joystick calibration"));
    }
    
    // Load calibration data
    void loadCalibration() {
        // This is just a placeholder
        // The actual calibration is loaded in the Joystick class constructor
        DEBUG_PRINTLN(F("Loading joystick calibration"));
    }
    
    // Check if joysticks are calibrated
    bool isJoystickCalibrated() {
        // You could implement a check to see if valid calibration data exists
        // For now, just return true
        return true;
    }
    
    // Get normalized left X value
    float getLeftX() {
        return leftJoystick->getXWithDeadband(joystickDeadband);
    }
    
    // Get normalized left Y value
    float getLeftY() {
        return leftJoystick->getYWithDeadband(joystickDeadband);
    }
    
    // Get normalized right X value
    float getRightX() {
        return rightJoystick->getXWithDeadband(joystickDeadband);
    }
    
    // Get normalized right Y value
    float getRightY() {
        return rightJoystick->getYWithDeadband(joystickDeadband);
    }
    
    // Get raw left X value
    int getRawLeftX() {
        return leftJoystick->getX();
    }
    
    // Get raw left Y value
    int getRawLeftY() {
        return leftJoystick->getY();
    }
    
    // Get raw right X value
    int getRawRightX() {
        return rightJoystick->getX();
    }
    
    // Get raw right Y value
    int getRawRightY() {
        return rightJoystick->getY();
    }
    
    // Check if left button is currently pressed
    bool isLeftButtonPressed() {
        return leftJoystick->isPressed();
    }
    
    // Check if right button is currently pressed
    bool isRightButtonPressed() {
        return rightJoystick->isPressed();
    }
    
    // Check if left button was just pressed
    bool isLeftButtonJustPressed() {
        return (leftButtonState == BUTTON_PRESSED);
    }
    
    // Check if right button was just pressed
    bool isRightButtonJustPressed() {
        return (rightButtonState == BUTTON_PRESSED);
    }
    
    // Check if left button was just released
    bool isLeftButtonReleased() {
        return (leftButtonState == BUTTON_RELEASED);
    }
    
    // Check if right button was just released
    bool isRightButtonReleased() {
        return (rightButtonState == BUTTON_RELEASED);
    }
    
    // Check if left button has been held
    bool hasLeftButtonLongPress() {
        return (leftButtonState == BUTTON_HELD);
    }
    
    // Check if right button has been held
    bool hasRightButtonLongPress() {
        return (rightButtonState == BUTTON_HELD);
    }
    
    // Check if left button was double-clicked
    bool hasLeftButtonDoubleClick() {
        bool result = leftButtonDoubleClick;
        leftButtonDoubleClick = false; // Reset after reading
        return result;
    }
    
    // Check if right button was double-clicked
    bool hasRightButtonDoubleClick() {
        bool result = rightButtonDoubleClick;
        rightButtonDoubleClick = false; // Reset after reading
        return result;
    }
    
    // Get left joystick object
    Joystick* getLeftJoystick() {
        return leftJoystick;
    }
    
    // Get right joystick object
    Joystick* getRightJoystick() {
        return rightJoystick;
    }
    
    // Get all joystick values at once
    JoystickValues getJoystickValues() {
        JoystickValues values;
        
        values.leftX = getLeftX();
        values.leftY = getLeftY();
        values.rightX = getRightX();
        values.rightY = getRightY();
        values.leftButton = isLeftButtonPressed();
        values.rightButton = isRightButtonPressed();
        
        return values;
    }

} // namespace JoystickSystem


