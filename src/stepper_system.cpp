#include "stepper_system.h"
#include "Debug.h"
#include "config.h"
#include <EEPROM.h>

namespace StepperSystem {
    // Define the arrays that were declared as external in the header
    AccelStepper* steppers[NUM_JOINTS];
    EnhancedStepper enhancedSteppers[NUM_JOINTS];
    
    // Define the pin arrays
    const int STEP_PINS[NUM_JOINTS] = {2, 5, 8, 11, 28, 31};   // From STEPPER_PINS[][0]
    const int DIR_PINS[NUM_JOINTS] = {3, 6, 9, 12, 29, 32};    // From STEPPER_PINS[][1]
    const int ENABLE_PINS[NUM_JOINTS] = {4, 7, 10, 13, 30, 33}; // From STEPPER_PINS[][2]
    const int LIMIT_PINS[NUM_JOINTS] = {22, 23, 24, 25, 26, 27}; // From STEPPER_PINS[][3]
    
    // Initialization
    void init() {
        Debug::println(F("Initializing stepper system..."));
        
        // Create AccelStepper instances for each joint
        for (int i = 0; i < NUM_JOINTS; i++) {
            steppers[i] = new AccelStepper(
                AccelStepper::DRIVER,
                STEP_PINS[i],  // STEP pin
                DIR_PINS[i]    // DIR pin
            );
            
            // Load stepper configuration from user-defined variables
            enhancedSteppers[i].stepsPerDegree = _stepperConfig[i].stepsPerDegree;
            enhancedSteppers[i].maxSpeed = _stepperConfig[i].maxSpeed;
            enhancedSteppers[i].acceleration = _stepperConfig[i].acceleration;
            enhancedSteppers[i].homingSpeed = _stepperConfig[i].homingSpeed;
            enhancedSteppers[i].limitSwitchPin = LIMIT_PINS[i];
            enhancedSteppers[i].invertLimit = false;
            enhancedSteppers[i].enabled = false;
            
            // Set default parameters
            steppers[i]->setMaxSpeed(enhancedSteppers[i].maxSpeed);
            steppers[i]->setAcceleration(enhancedSteppers[i].acceleration);
            
            // Configure pins
            pinMode(ENABLE_PINS[i], OUTPUT); // ENABLE pin
            pinMode(LIMIT_PINS[i], INPUT_PULLUP); // LIMIT pin
            
            // Disable motors by default (HIGH = disabled for most drivers)
            digitalWrite(ENABLE_PINS[i], HIGH);
        }
        
        Debug::println(F("Stepper system initialized"));
    }
    
    // Update function - should be called frequently
    void update() {
        // Run each stepper
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (enhancedSteppers[i].enabled) {
                steppers[i]->run();
            }
        }
    }
    
    // Move joint at a specified speed (-1.0 to 1.0)
    void moveJoint(int joint, float speed) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        // Map speed to steps/second
        float mappedSpeed = speed * enhancedSteppers[joint].maxSpeed;
        
        // Enable motor if it's not already enabled
        if (!enhancedSteppers[joint].enabled) {
            enableMotor(joint);
        }
        
        // Set the speed
        steppers[joint]->setSpeed(mappedSpeed);
        
        // In constant speed mode, we need to call runSpeed repeatedly
        // This just primes the motor, the actual movement is done in update()
    }
    
    // Move to absolute position
    void moveTo(int joint, long position) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        // Enable motor
        enableMotor(joint);
        
        // Set target position
        steppers[joint]->moveTo(position);
    }
    
    // Move relative distance
    void move(int joint, long distance) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        // Enable motor
        enableMotor(joint);
        
        // Move relative distance
        steppers[joint]->move(distance);
    }
    
    // Stop motor (decelerate to stop)
    void stop(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        steppers[joint]->stop();
    }
    
    // Run motor using acceleration profile
    void run(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        steppers[joint]->run();
    }
    
    // Run at constant speed
    void runSpeed(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        steppers[joint]->runSpeed();
    }
    
    // Set constant speed
    void setSpeed(int joint, float speed) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        steppers[joint]->setSpeed(speed);
    }
    
    // Get current position
    long getCurrentPosition(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return 0;
        
        return steppers[joint]->currentPosition();
    }
    
    // Set current position
    void setCurrentPosition(int joint, long position) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        steppers[joint]->setCurrentPosition(position);
    }
    
    // Get distance to go
    long getDistanceToGo(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return 0;
        
        return steppers[joint]->distanceToGo();
    }
    
    // Get target position
    long getTargetPosition(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return 0;
        
        return steppers[joint]->targetPosition();
    }
    
    // Get steps per degree
    float getStepsPerDegree(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return DEFAULT_STEPS_PER_DEGREE;
        
        return enhancedSteppers[joint].stepsPerDegree;
    }
    
    // Set steps per degree
    void setStepsPerDegree(int joint, float stepsPerDegree) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        enhancedSteppers[joint].stepsPerDegree = stepsPerDegree;
    }
    
    // Get homing speed
    float getHomingSpeed(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return DEFAULT_HOMING_SPEED;
        
        return enhancedSteppers[joint].homingSpeed;
    }
    
    // Enable motor
    void enableMotor(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        digitalWrite(ENABLE_PINS[joint], LOW); // LOW = enabled for most drivers
        enhancedSteppers[joint].enabled = true;
    }
    
    // Disable motor
    void disableMotor(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        digitalWrite(ENABLE_PINS[joint], HIGH); // HIGH = disabled for most drivers
        enhancedSteppers[joint].enabled = false;
    }
    
    // Enable all motors
    void enableAllMotors() {
        for (int i = 0; i < NUM_JOINTS; i++) {
            enableMotor(i);
        }
    }
    
    // Disable all motors
    void disableAllMotors() {
        for (int i = 0; i < NUM_JOINTS; i++) {
            disableMotor(i);
        }
    }
    
    // Read limit switch state
    int readLimitSwitch(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return HIGH;
        
        return digitalRead(LIMIT_PINS[joint]);
    }
    
    // Get limit switch pin
    int getLimitSwitchPin(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return -1;
        
        return enhancedSteppers[joint].limitSwitchPin;
    }
    
    // Check if limit switch is triggered
    bool isLimitSwitchTriggered(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return false;
        
        int pin = enhancedSteppers[joint].limitSwitchPin;
        bool state = digitalRead(pin) == LOW; // Assuming active LOW
        
        return enhancedSteppers[joint].invertLimit ? !state : state;
    }
    
    // Set up joint configuration
    void setupJoint(int joint, int stepPin, int dirPin, int enablePin, int limitPin) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        // Update pin configuration - Here we'd update the individual pins
        // This is just a placeholder - you would need to implement this based on your configuration
        
        // Update pin modes
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        pinMode(enablePin, OUTPUT);
        pinMode(limitPin, INPUT_PULLUP);
        
        // Update limit switch pin in enhanced stepper
        enhancedSteppers[joint].limitSwitchPin = limitPin;
    }
    
        // Check if motor is moving
    bool isMoving(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return false;
        
        return steppers[joint]->distanceToGo() != 0;
    }
    
    // Check if any motors are moving
    bool areAnyMoving() {
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (isMoving(i)) return true;
        }
        return false;
    }
    
    // Emergency stop - immediately disable all motors
    void emergencyStop() {
        // First stop all steppers
        for (int i = 0; i < NUM_JOINTS; i++) {
            steppers[i]->setSpeed(0);
            steppers[i]->stop();
        }
        
        // Then disable all motors
        disableAllMotors();
        
        Debug::println(F("EMERGENCY STOP ACTIVATED!"));
    }
    
    // Set motor current (for drivers that support it)
    void setMotorCurrent(int joint, int current) {
        if (joint < 0 || joint >= NUM_JOINTS) return;
        
        // Implementation depends on your specific motor driver
        // This is a placeholder - implement based on your hardware
        
        // For example, if using a Trinamic driver with SPI:
        // TMC2130_setRunCurrent(joint, current);
        
        Debug::print(F("Set current for joint "));
        Debug::print(joint);
        Debug::print(F(" to "));
        Debug::println(current);
    }
    
    // Get motor current
    int getMotorCurrent(int joint) {
        if (joint < 0 || joint >= NUM_JOINTS) return 0;
        
        // Implementation depends on your specific motor driver
        // This is a placeholder - implement based on your hardware
        
        // For example, if using a Trinamic driver with SPI:
        // return TMC2130_getRunCurrent(joint);
        
        return 0; // Default placeholder
    }
} // namespace StepperSystem

