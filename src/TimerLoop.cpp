#include "TimerLoop.h"
#include "Debug.h"

namespace TimerLoop {
    // Timing statistics
    volatile unsigned long controlLoopStartTime = 0;
    volatile unsigned long controlLoopEndTime = 0;
    volatile unsigned long controlLoopMaxTime = 0;
    volatile unsigned long controlLoopCount = 0;
    volatile bool controlLoopOverrun = false;
    
    // Manual timing variables
    unsigned long lastJoystickTime = 0;
    unsigned long lastDisplayTime = 0;
    unsigned long joystickLoopTime = 0;
    unsigned long displayLoopTime = 0;
    
    // Timer callback storage
    TimerCallback controlCB = nullptr;
    
    // Hardware timer (Teensy specific)
    #if defined(__IMXRT1062__) // Teensy 4.0/4.1
    IntervalTimer controlTimer;
    #else
    // For other platforms, we'll do software timing
    unsigned long lastControlTime = 0;
    #endif
    
    // Control loop callback function (ISR)
    void controlLoopISR() {
        // Start timing the execution
        controlLoopStartTime = micros();
        
        // Check for overruns
        if (controlLoopOverrun) {
            // Previous iteration took too long, just count it
            DEBUG_PRINTLN("Control loop overrun!");
        }
        
        // Mark that we've started processing
        controlLoopOverrun = true;
        
        // Call the control callback
        if (controlCB) {
            controlCB();
        }
        
        // End timing and statistics
        controlLoopEndTime = micros();
        unsigned long executionTime = controlLoopEndTime - controlLoopStartTime;
        
        // Update max execution time
        if (executionTime > controlLoopMaxTime) {
            controlLoopMaxTime = executionTime;
        }
        
        // Increment counter and clear overrun flag
        controlLoopCount++;
        controlLoopOverrun = false;
    }
    
    void begin(TimerCallback controlCallback) {
        // Store callback
        controlCB = controlCallback;
        
        // Initialize timing variables
        controlLoopCount = 0;
        controlLoopMaxTime = 0;
        controlLoopOverrun = false;
        lastJoystickTime = millis();
        lastDisplayTime = millis();
        
        // Start the control timer
        #if defined(__IMXRT1062__) // Teensy 4.0/4.1
        // Start the hardware timer
        if (controlTimer.begin(controlLoopISR, CONTROL_LOOP_INTERVAL)) {
            DEBUG_PRINTLN("Control loop timer started successfully");
        } else {
            DEBUG_PRINTLN("Failed to start control loop timer!");
        }
        #else
        // For non-Teensy platforms, we'll use the loop() function
        lastControlTime = micros();
        DEBUG_PRINTLN("Software timing for control loop enabled");
        #endif
        
        DEBUG_PRINTLN("TimerLoop initialized");
    }
    
    void loop(TimerCallback joystickCallback, TimerCallback displayCallback) {
        // Check if we need to run the control loop on non-Teensy platforms
        #if !defined(__IMXRT1062__)
        unsigned long currentTime = micros();
        if (currentTime - lastControlTime >= CONTROL_LOOP_INTERVAL) {
            lastControlTime = currentTime;
            controlLoopISR();
        }
        #endif
        
        // Check if it's time to run the joystick loop
        if (checkJoystickTimer()) {
            unsigned long startTime = micros();
            
            if (joystickCallback) {
                joystickCallback();
            }
            
            joystickLoopTime = micros() - startTime;
        }
        
        // Check if it's time to run the display loop
        if (checkDisplayTimer()) {
            unsigned long startTime = micros();
            
            if (displayCallback) {
                displayCallback();
            }
            
            displayLoopTime = micros() - startTime;
        }
    }
    
    void end() {
        #if defined(__IMXRT1062__) // Teensy 4.0/4.1
        // Stop the hardware timer
        controlTimer.end();
        #endif
        
        DEBUG_PRINTLN("TimerLoop stopped");
    }
    
    unsigned long getControlLoopTime() {
        return controlLoopEndTime - controlLoopStartTime;
    }
    
    unsigned long getJoystickLoopTime() {
        return joystickLoopTime;
    }
    
    unsigned long getDisplayLoopTime() {
        return displayLoopTime;
    }
    
    unsigned long getControlLoopCount() {
        return controlLoopCount;
    }
    
    bool checkJoystickTimer() {
        unsigned long currentTime = millis();
        if (currentTime - lastJoystickTime >= (1000 / JOYSTICK_LOOP_FREQ)) {
            lastJoystickTime = currentTime;
            return true;
        }
        return false;
    }
    
    bool checkDisplayTimer() {
        unsigned long currentTime = millis();
        if (currentTime - lastDisplayTime >= (1000 / DISPLAY_LOOP_FREQ)) {
            lastDisplayTime = currentTime;
            return true;
        }
        return false;
    }
}