#ifndef JOYSTICK_SYSTEM_H
#define JOYSTICK_SYSTEM_H

#include <Arduino.h>
#include "config.h"
#include "kalmanfilter.h"

// Button state constants
#define BUTTON_HOLD_TIME 800    // Time in ms for a button to be considered "held"
#define DOUBLE_CLICK_TIME 300   // Time in ms to detect double clicks

// Button states
enum ButtonState {
    BUTTON_IDLE,
    BUTTON_PRESSED,
    BUTTON_HELD,
    BUTTON_RELEASED
};

// Joystick values structure
struct JoystickValues {
    float leftX;
    float leftY;
    float rightX;
    float rightY;
    bool leftButton;
    bool rightButton;
};

namespace JoystickSystem {
    // Joystick class definition
    class Joystick {
    public:
        Joystick(int xPin, int yPin, int btnPin);
        void begin();
        void calibrate();
        void read();
        int getX();
        int getY();
        bool isPressed();
        float getNormalizedX();
        float getNormalizedY();
        float getXWithDeadband(float deadbandPercentage);
        float getYWithDeadband(float deadbandPercentage);
        
        // Extended calibration
        void startCalibration();
        void saveMinMax(int xMin, int xMax, int yMin, int yMax);
        
    private:
        int _xPin, _yPin, _btnPin;
        int _xCenter, _yCenter;
        int _xValue, _yValue;
        int _xMin, _xMax, _yMin, _yMax; // Min/Max values for mapping
        KalmanFilter _xFilter, _yFilter;

        // Helper function for mapping to 0-1000
        int mapJoystickValue(int value, int minVal, int center, int maxVal);
    };
    
    // Initialization
    void init();
    
    // Update function - call regularly in the main loop
    void update();
    
    // Joystick read functions
    float getLeftX();
    float getLeftY();
    float getRightX();
    float getRightY();
    
    // Raw joystick values
    int getRawLeftX();
    int getRawLeftY();
    int getRawRightX();
    int getRawRightY();
    
    // Button state functions
    bool isLeftButtonPressed();
    bool isRightButtonPressed();
    bool isLeftButtonJustPressed();
    bool isRightButtonJustPressed();
    bool isLeftButtonReleased();
    bool isRightButtonReleased();
    bool hasLeftButtonLongPress();
    bool hasRightButtonLongPress();
    bool hasLeftButtonDoubleClick();
    bool hasRightButtonDoubleClick();
    
    // Calibration functions
    void startCalibration();
    void processCalibration();
    void saveCalibration();
    void loadCalibration();
    bool isJoystickCalibrated();
    
    // Joint and kinematic control functions
    void processJointModeJoysticks();
    void processKinematicModeJoysticks();
    
    // Access to joystick objects
    Joystick* getLeftJoystick();
    Joystick* getRightJoystick();
    
    // Get all joystick values at once
    JoystickValues getJoystickValues();
}

#endif // JOYSTICK_SYSTEM_H
