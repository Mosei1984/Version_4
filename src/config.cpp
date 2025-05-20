#include "config.h"
#include "debug.h"

// Globale Konfigurationsvariablen (für extern-Verweise)
PinConfig _pinConfig;
JoystickConfig _joystickConfig;
StepperConfig _stepperConfig[6];

// Default-Ladefunktionen
void loadDefaultPinConfig() {
  _pinConfig.leftXPin    = 40;
  _pinConfig.leftYPin    = 41;
  _pinConfig.leftBtnPin  = 27;
  _pinConfig.rightXPin   = 38;
  _pinConfig.rightYPin   = 39;
  _pinConfig.rightBtnPin = 26;
  
  // Stepper-Pins
  _pinConfig.stepperPins[0][0] = 2;   // Base - STEP
  _pinConfig.stepperPins[0][1] = 3;   // Base - DIR
  _pinConfig.stepperPins[0][2] = 4;   // Base - ENABLE
  _pinConfig.stepperPins[0][3] = 22;  // Base - LIMIT
  
  _pinConfig.stepperPins[1][0] = 5;   // Shoulder - STEP
  _pinConfig.stepperPins[1][1] = 6;   // Shoulder - DIR
  _pinConfig.stepperPins[1][2] = 7;   // Shoulder - ENABLE
  _pinConfig.stepperPins[1][3] = 23;  // Shoulder - LIMIT
  
  _pinConfig.stepperPins[2][0] = 8;   // Elbow - STEP
  _pinConfig.stepperPins[2][1] = 9;   // Elbow - DIR
  _pinConfig.stepperPins[2][2] = 10;  // Elbow - ENABLE
  _pinConfig.stepperPins[2][3] = 24;  // Elbow - LIMIT
  
  _pinConfig.stepperPins[3][0] = 11;  // Wrist Pitch - STEP
  _pinConfig.stepperPins[3][1] = 12;  // Wrist Pitch - DIR
  _pinConfig.stepperPins[3][2] = 41;  // Wrist Pitch - ENABLE
  _pinConfig.stepperPins[3][3] = 25;  // Wrist Pitch - LIMIT
  
  _pinConfig.stepperPins[4][0] = 14;  // Wrist Roll - STEP
  _pinConfig.stepperPins[4][1] = 15;  // Wrist Roll - DIR
  _pinConfig.stepperPins[4][2] = 16;  // Wrist Roll - ENABLE
  _pinConfig.stepperPins[4][3] = 29;  // Wrist Roll - LIMIT
  
  _pinConfig.stepperPins[5][0] = 17;  // Gripper - STEP
  _pinConfig.stepperPins[5][1] = 20;  // Gripper - DIR
  _pinConfig.stepperPins[5][2] = 21;  // Gripper - ENABLE
  _pinConfig.stepperPins[5][3] = 0;   // Gripper hat keinen Limit-Switch
  
  _pinConfig.errorLedPin = 13;
  _pinConfig.oledSdaPin  = 18;
  _pinConfig.oledSclPin  = 19;
  
  DEBUG_PRINTLN("Default pin configuration loaded");
}

void loadDefaultJoystickConfig() {
  _joystickConfig.deadband    = 50;    // 10-Bit ADC
  _joystickConfig.sensitivity = 0.03f;
  DEBUG_PRINTLN("Default joystick configuration loaded");
}

void loadDefaultStepperConfig() {
  for (int i = 0; i < 6; i++) {
    _stepperConfig[i].maxSpeed       = 1000;
    _stepperConfig[i].acceleration   = 500;
    _stepperConfig[i].stepsPerDegree = 20;
    _stepperConfig[i].homingSpeed    = 200;
  }
  // Grenzen pro Achse
  _stepperConfig[0].minPosition = -9000; _stepperConfig[0].maxPosition =  9000;
  _stepperConfig[1].minPosition = -4500; _stepperConfig[1].maxPosition =  9000;
  _stepperConfig[2].minPosition = -9000; _stepperConfig[2].maxPosition =  9000;
  _stepperConfig[3].minPosition = -4500; _stepperConfig[3].maxPosition =  4500;
  _stepperConfig[4].minPosition = -9000; _stepperConfig[4].maxPosition =  9000;
  _stepperConfig[5].minPosition =     0; _stepperConfig[5].maxPosition =  3000;
  
  DEBUG_PRINTLN("Default stepper configuration loaded");
}

// Namespace für die Init-Funktion, die in main.cpp aufgerufen wird
namespace ConfigSystem {
  void init() {
    loadDefaultPinConfig();
    loadDefaultJoystickConfig();
    loadDefaultStepperConfig();
  }
}
