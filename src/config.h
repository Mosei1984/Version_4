#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ======================== Pin Definitions ========================
// (Diese Konstanten sind optional, da Du über _pinConfig arbeitest,
//  können aber für kurze Aufrufe weiterhin genutzt werden.)
const int LEFT_X_PIN    = 14;  // A0 auf Teensy 4.1
const int LEFT_Y_PIN    = 15;  // A1
const int LEFT_BTN_PIN  = 26;  // Linker Joystick-Knopf
const int RIGHT_X_PIN   = 16;  // A2
const int RIGHT_Y_PIN   = 17;  // A3
const int RIGHT_BTN_PIN = 27;  // Rechter Joystick-Knopf

// Optional, da Du Default-Pin-Konfig im Code lädst:
const int STEPPER_PINS[6][4] = {
  {2, 3, 4, 22},
  {5, 6, 7, 23},
  {8, 9, 10,24},
  {11,12,13,25},
  {28,29,30,26},
  {31,32,33,27}
};

// ======================== System States ========================
enum SystemState {
  IDLE = 0,
  HOMING,
  NORMAL_OPERATION,
  RECORD,
  PLAY,
  GCODE,
  CALIBRATION,
  CONFIG,
  STATE_STARTUP,
  STATE_JOINT_MODE,
  STATE_KINEMATIC_MODE,
  STATE_HOMING_MODE,
  STATE_CALIBRATION_MODE,
  STATE_GEAR_MENU,
  STATE_CONFIG_MODE,
  STATE_ERROR
};

extern SystemState currentState;
void setSystemState(SystemState newState);

// ======================== Display Modes ========================
enum DisplayMode {
  DISPLAY_SPLASH,
  DISPLAY_JOINTS,
  DISPLAY_GCODE,
  DISPLAY_RECORD,
  DISPLAY_MENU,
  DISPLAY_HOMING,
  DISPLAY_ERROR,
  DISPLAY_CONFIG
};

extern DisplayMode currentDisplayMode;
void setDisplayMode(DisplayMode mode);

// ======================== EEPROM Storage ========================
#define EEPROM_MAGIC_VALUE  0x42
#define EEPROM_ADDR_MAGIC   0
#define EEPROM_ADDR_CALIB   10
#define EEPROM_ADDR_HOMING 100
#define ROBOT_HOME_MAGIC   0x42ABCDEF

// ======================== Constants for Stepper System ========================
#define NUM_JOINTS 6

// ======================== Configuration Management ========================
namespace ConfigSystem {
    void init();
    bool saveConfig(const char* filename);
    bool loadConfig(const char* filename);
    void loadDefaultPinConfig();
    void loadDefaultJoystickConfig();
    void loadDefaultStepperConfig();
    void displayConfigMenu();
    void processConfigMenu();
}

// ======================== Hardware Configuration Structures ==========
struct PinConfig {
    int leftXPin, leftYPin, leftBtnPin;
    int rightXPin, rightYPin, rightBtnPin;
    int stepperPins[6][4];
    int errorLedPin;
    int oledSdaPin, oledSclPin;
};

struct JoystickConfig {
    int   deadband;
    float sensitivity;
};

struct StepperConfig {
    float stepsPerDegree;
    float maxSpeed;
    float acceleration;
    float homingSpeed;
    float minPosition;
    float maxPosition;
};

// ======================== Externe Konfigurationsvariablen ================
extern PinConfig      _pinConfig;
extern JoystickConfig _joystickConfig;
extern StepperConfig  _stepperConfig[6];

#endif // CONFIG_H
