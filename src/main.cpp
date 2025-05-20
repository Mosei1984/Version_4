#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include "config.h"
#include "Debug.h"
#include "TimerLoop.h"
#include "stepper_system.h"  // Using stepper_system.h instead of stepper_control.h
#include "display_system.h"
#include "joystick_system.h"
#include "robot_system.h"
#include "menu_system.h"

// SD Card pin definition
#define SD_CS_PIN 10  // Set this to your actual SD CS pin

// External configurations
extern PinConfig _pinConfig;
extern JoystickConfig _joystickConfig;
extern StepperConfig _stepperConfig[6];

// Function prototypes
void controlCallback();
void joystickCallback();
void displayCallback();

// Menu action function prototypes
void menuSystemInfo();
void menuStartHoming();
void menuMoveToCenter();
void menuJointMode();
void menuKinematicMode();
void menuCalibration();

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println(F("6DOF Robot Controller v4 Initializing..."));
  
  // Initialize debug output
  Debug::enabled = true;
  Debug::println(F("Debug output enabled"));
  
  // Initialize configuration
  ConfigSystem::init();
  
  // Initialize hardware
  pinMode(_pinConfig.errorLedPin, OUTPUT);
  digitalWrite(_pinConfig.errorLedPin, LOW);  // No error initially
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize subsystems
  DisplaySystem::init();
  JoystickSystem::init();
  StepperSystem::init();
  RobotSystem::init();
  MenuSystem::init();
  
  // Initialize SD card if available
  if (!SD.begin(SD_CS_PIN)) {
    Debug::println(F("SD card initialization failed. Continuing without SD."));
  } else {
    Debug::println(F("SD card initialized successfully."));
  }
  
  // Setup menu system with proper action functions
  MenuSystem::addActionItem("System Info", menuSystemInfo);
  MenuSystem::addActionItem("Start Homing", menuStartHoming);
  MenuSystem::addActionItem("Move to Center", menuMoveToCenter);
  MenuSystem::addActionItem("Joint Mode", menuJointMode);
  MenuSystem::addActionItem("Kinematic Mode", menuKinematicMode);
  MenuSystem::addActionItem("Calibration", menuCalibration);
  
  // Start timer for real-time control
  TimerLoop::begin(controlCallback);
  
  Debug::println(F("Initialization complete, starting main loop."));
}

void loop() {
  // Main timer loop handles everything
  TimerLoop::loop(joystickCallback, displayCallback);
  
  // Emergency stop monitoring
  if (Serial.available() && Serial.read() == '!') {
    // Emergency stop - disable all motors
    StepperSystem::disableAllMotors();
    Debug::println(F("EMERGENCY STOP triggered!"));
    DisplaySystem::showMessage("EMERGENCY STOP", "System halted!", 0);
    
    // Wait for reset
    while(true) {
      // Keep updating display
      DisplaySystem::update();
      delay(100);
    }
  }
}

// Control callback - runs at 1kHz from timer
void controlCallback() {
  // Update stepper motors
  StepperSystem::update();
}

// Joystick callback - runs at 100Hz
void joystickCallback() {
  // Update joystick values
  JoystickSystem::update();
  
  // Process robot state
  RobotSystem::update();
  
  // Update menu system
  MenuSystem::update();
}

// Display callback - runs at 50Hz
void displayCallback() {
  // Update display information
  DisplaySystem::update();
}

// Menu action function implementations
void menuSystemInfo() {
  DisplaySystem::showMessage("Robot Control", "Version 4.0", 2000);
  Debug::println(F("System info menu option selected"));
}

void menuStartHoming() {
  Debug::println(F("Starting homing sequence"));
  RobotSystem::setState(STATE_HOMING_MODE);
  RobotSystem::setHomingStarted(true);
  RobotSystem::setHomingJointIndex(0);
}

void menuMoveToCenter() {
  Debug::println(F("Moving to center position"));
  // Use the current system state's move to center functionality
  if (RobotSystem::getState() == STATE_HOMING_MODE) {
    // This will trigger the _moveToCenter function through menu selection
    RobotSystem::setHomingMenuSelection(HOMING_MENU_TO_CENTER);
    RobotSystem::processHomingMenuSelection();
  } else {
    // Change to homing mode first, then move to center
    RobotSystem::changeState(STATE_HOMING_MODE);
    RobotSystem::setHomingMenuSelection(HOMING_MENU_TO_CENTER);
    RobotSystem::processHomingMenuSelection();
  }
}

void menuJointMode() {
  Debug::println(F("Switching to joint mode"));
  RobotSystem::changeState(STATE_JOINT_MODE);
}

void menuKinematicMode() {
  Debug::println(F("Switching to kinematic mode"));
  RobotSystem::changeState(STATE_KINEMATIC_MODE);
}

void menuCalibration() {
  Debug::println(F("Entering calibration mode"));
  RobotSystem::changeState(STATE_CALIBRATION_MODE);
}
