#ifndef STEPPER_SYSTEM_H
#define STEPPER_SYSTEM_H

#include <Arduino.h>
#include <AccelStepper.h>

// Define the number of joints/steppers
#ifndef NUM_JOINTS
#define NUM_JOINTS 6
#endif

// Motor driver constants
#define MOTOR_STEPS_PER_REV 200        // 200 steps per revolution (1.8° step angle)
#define DEFAULT_STEPS_PER_DEGREE 5.56  // 200 steps / 360° = 0.56 steps/degree × microstepping
#define MAX_SPEED 1000.0               // Maximum speed in steps/second
#define DEFAULT_ACCEL 500.0            // Default acceleration in steps/second²
#define DEFAULT_HOMING_SPEED 400.0     // Default homing speed in steps/second

namespace StepperSystem {
    // Structure for enhanced stepper data
    struct EnhancedStepper {
        float stepsPerDegree;
        float maxSpeed;
        float acceleration;
        float homingSpeed;
        int limitSwitchPin;
        bool invertLimit;
        bool enabled;
    };

    // External declarations for stepper arrays
    extern AccelStepper* steppers[NUM_JOINTS];
    extern EnhancedStepper enhancedSteppers[NUM_JOINTS];
    
    // Pin arrays
    extern const int STEP_PINS[NUM_JOINTS];
    extern const int DIR_PINS[NUM_JOINTS];
    extern const int ENABLE_PINS[NUM_JOINTS]; 
    extern const int LIMIT_PINS[NUM_JOINTS];
    
    // Initialization
    void init();
    
    // Main update function - call frequently
    void update();
    
    // === Stepper movement functions ===
    
    // Move to absolute position
    void moveTo(int joint, long position);
    
    // Move relative distance
    void move(int joint, long distance);
    
    // Stop motor (decelerate to stop)
    void stop(int joint);
    
    // Set speed for constant velocity movement
    void setSpeed(int joint, float speed);
    
    // Run at constant speed
    void runSpeed(int joint);
    
    // Run using acceleration profile
    void run(int joint);
    
    // Move joint at specified speed (-1.0 to 1.0)
    void moveJoint(int joint, float speed);
    
    // === Position queries ===
    
    // Get current position
    long getCurrentPosition(int joint);
    
    // Set current position
    void setCurrentPosition(int joint, long position);
    
    // Get target position
    long getTargetPosition(int joint);
    
    // Get distance to go
    long getDistanceToGo(int joint);
    
    // Is motor running?
    bool isMoving(int joint);
    
    // Are any motors running?
    bool areAnyMoving();
    
    // === Configuration ===
    
    // Get steps per degree
    float getStepsPerDegree(int joint);
    
    // Set steps per degree
    void setStepsPerDegree(int joint, float stepsPerDegree);
    
    // Get homing speed
    float getHomingSpeed(int joint);
    
    // Read limit switch
    int readLimitSwitch(int joint);
    
    // Is limit switch triggered?
    bool isLimitSwitchTriggered(int joint);
    
    // === Motor control ===
    
    // Enable motor
    void enableMotor(int joint);
    
    // Disable motor
    void disableMotor(int joint);
    
    // Enable all motors
    void enableAllMotors();
    
    // Disable all motors
    void disableAllMotors();
    
    // Emergency stop - immediately disable all motors
    void emergencyStop();
    
    // === Advanced functions ===
    
    // Set up joint configuration
    void setupJoint(int joint, int stepPin, int dirPin, int enablePin, int limitPin);
    
    // Set motor current (for drivers that support it)
    void setMotorCurrent(int joint, int current);
    
    // Get motor current setting
    int getMotorCurrent(int joint);
    
    // Get limit switch pin
    int getLimitSwitchPin(int joint);
}

#endif // STEPPER_SYSTEM_H
