#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H

#include <Arduino.h>
#include "RobotKinematics.h"
#include "config.h"
#include "display_system.h"
#include "joystick_system.h"
#include "stepper_system.h"
#include "menu_system.h"

// Forward declarations
class Joystick;

// Program point structure
struct ProgramPoint {
    CartesianPose position;  // Position data
    uint32_t delayMs;        // Delay after reaching this point (in milliseconds)
};

// Maximum number of program points
#define MAX_PROGRAM_POINTS 1000

// Homing menu options enum
enum HomingMenuOption {
    HOMING_MENU_START_HOMING,
    HOMING_MENU_TO_CENTER,
    HOMING_MENU_SAVE_HOME,
    HOMING_MENU_LOAD_HOME,
    HOMING_MENU_CLEAR_HOME,
    HOMING_MENU_CONFIG,
    HOMING_MENU_COUNT
};

namespace RobotSystem {
    // External variables - accessible from other modules
    extern RobotKinematics* robotKin;
    extern RobotConfig robotConfig;
    extern SystemState currentState;
    extern unsigned long stateChangeTime;
    extern int homingJointIndex;
    extern bool homingStarted;
    extern int homingMenuSelection;
    extern bool calibrationLocked;
    extern ProgramPoint programPoints[MAX_PROGRAM_POINTS];
    extern int programSize;
    
    // Initialization
    void init();
    void initRobotConfig();
    
    // Main update function
    void update();
    void processCurrentState();
    
    // State management
    SystemState getState();
    void setState(SystemState state);
    unsigned long getStateChangeTime();
    void setStateChangeTime(unsigned long time);
    void changeState(SystemState newState);
    
    // Kinematics
    RobotKinematics* getKinematics();
    bool isWithinWorkspace(const CartesianPose& pose);
    void moveToPose(const CartesianPose& pose, bool waitForCompletion = false);
    
    // Robot configuration
    RobotConfig getRobotConfig();
    void setRobotConfig(const RobotConfig& config);
    
    // Joint selection
    int getSelectedJoint();
    void setSelectedJoint(int joint);
    
    // Calibration status
    bool isCalibrationLocked();
    void setCalibrationLocked(bool locked);
    
    // Homing
    bool isHomingStarted();
    void setHomingStarted(bool started);
    int getHomingJointIndex();
    void setHomingJointIndex(int index);
    void processHomingMode();
    void processHomingMenu();
    void processHomingMenuSelection();
    bool homeJoint(int jointIndex);
    void moveToCenter();
    
    // Homing menu
    int getHomingMenuSelection();
    void setHomingMenuSelection(int selection);
    int getHomingMenuOptionCount();
    const char* getHomingMenuOptionName(int option);
    
    // Joint data
    JointAngles getCurrentJointAngles();
    
    // Home position management
    void saveRobotHome(const JointAngles& angles);
    bool loadRobotHome(JointAngles& angles);
    void clearRobotHome();
    
    // SD card operations
    bool saveRobotHomeToSD(const JointAngles& angles, const char* filename = nullptr);
    bool loadRobotHomeFromSD(JointAngles& angles, const char* filename = nullptr);
    bool clearRobotHomeFromSD(const char* filename = nullptr);
    
    // Program management
    void clearProgram();
    bool addProgramPoint(const CartesianPose& pose, uint32_t delayMs);
    int getProgramSize();
    ProgramPoint getProgramPoint(int index);
    bool updateProgramPoint(int index, const ProgramPoint& point);
    bool deleteProgramPoint(int index);
    bool executeProgram(float speedFactor = 1.0f);
}

namespace JoystickSystem {
    // Calibration data
    void getCalibrationData(float leftMin[2], float leftMax[2], 
                           float rightMin[2], float rightMax[2]);
    void setCalibrationData(const float leftMin[2], const float leftMax[2], 
                           const float rightMin[2], const float rightMax[2]);
}

#endif // ROBOT_SYSTEM_H
