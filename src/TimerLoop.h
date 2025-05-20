#ifndef TIMER_LOOP_H
#define TIMER_LOOP_H

#include <Arduino.h>

// Setup IntervalTimer for Teensy or generic interval timing for other boards
#if defined(__IMXRT1062__) // Teensy 4.0/4.1
#include <IntervalTimer.h>
#endif

namespace TimerLoop {
    // Timer frequencies
    const unsigned long CONTROL_LOOP_FREQ = 1000;  // 1kHz control loop (1ms)
    const unsigned long JOYSTICK_LOOP_FREQ = 100;  // 100Hz joystick loop (10ms)
    const unsigned long DISPLAY_LOOP_FREQ = 50;    // 50Hz display loop (20ms)
    
    // Timer intervals in microseconds
    const unsigned long CONTROL_LOOP_INTERVAL = 1000000 / CONTROL_LOOP_FREQ;
    const unsigned long JOYSTICK_LOOP_INTERVAL = 1000000 / JOYSTICK_LOOP_FREQ;
    const unsigned long DISPLAY_LOOP_INTERVAL = 1000000 / DISPLAY_LOOP_FREQ;
    
    // Timer callback types
    typedef void (*TimerCallback)(void);
    
    // Initialize timer system
    void begin(TimerCallback controlCallback);
    
    // Main loop (handles joystick and display loops)
    void loop(TimerCallback joystickCallback, TimerCallback displayCallback);
    
    // Stop timers
    void end();
    
    // Get timing statistics
    unsigned long getControlLoopTime();
    unsigned long getJoystickLoopTime();
    unsigned long getDisplayLoopTime();
    unsigned long getControlLoopCount();
    
    // Check if a timer has elapsed (for manual timing)
    bool checkJoystickTimer();
    bool checkDisplayTimer();
}

#endif // TIMER_LOOP_H