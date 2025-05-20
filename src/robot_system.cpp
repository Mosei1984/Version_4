#include "robot_system.h"
#include "Debug.h"
#include "display_system.h"
#include "joystick_system.h"
#include "stepper_system.h"
#include "config.h"
#include <EEPROM.h>
#include <SD.h>

#define ROBOT_HOME_MAGIC 0x42ABCDEF
#define PROGRAM_FILE_MAGIC 0x50524F47 // "PROG" in hex

namespace RobotSystem {
    // Definition der externen Variablen
    RobotKinematics* robotKin = nullptr;
    RobotConfig robotConfig;
    SystemState currentState = STATE_STARTUP;   
    unsigned long stateChangeTime = 0;
    int homingJointIndex = 0;
    bool homingStarted = false;
    int homingMenuSelection = 0;
    bool calibrationLocked = false;
    int selectedJoint = 0; // Default to first joint
    
    // Program storage
    ProgramPoint programPoints[MAX_PROGRAM_POINTS];
    int programSize = 0;
    
    // Forward declarations for private functions
    static bool _homeJoint(int jointIndex);
    static void _moveToCenter();
    
    // Program management functions
    void clearProgram() {
        Debug::println(F("Clearing robot program"));
        programSize = 0;
    }
    
    bool addProgramPoint(const CartesianPose& pose, uint32_t delayMs) {
        if (programSize >= MAX_PROGRAM_POINTS) {
            Debug::println(F("Program memory full, cannot add more points"));
            return false;
        }
        
        programPoints[programSize].position = pose;
        programPoints[programSize].delayMs = delayMs;
        programSize++;
        
        Debug::print(F("Added program point "));
        Debug::print(programSize);
        Debug::print(F(" at X:"));
        Debug::print(pose.x);
        Debug::print(F(" Y:"));
        Debug::print(pose.y);
        Debug::print(F(" Z:"));
        Debug::println(pose.z);
        
        return true;
    }
    
    int getProgramSize() {
        return programSize;
    }
    
    ProgramPoint getProgramPoint(int index) {
        if (index < 0 || index >= programSize) {
            Debug::print(F("Invalid program point index: "));
            Debug::println(index);
            // Return empty point
            ProgramPoint emptyPoint;
            memset(&emptyPoint, 0, sizeof(ProgramPoint));
            return emptyPoint;
        }
        
        return programPoints[index];
    }
    
    bool updateProgramPoint(int index, const ProgramPoint& point) {
        if (index < 0 || index >= programSize) {
            Debug::print(F("Cannot update point, invalid index: "));
            Debug::println(index);
            return false;
        }
        
        programPoints[index] = point;
        Debug::print(F("Updated program point "));
        Debug::println(index);
        return true;
    }
    
    bool deleteProgramPoint(int index) {
        if (index < 0 || index >= programSize) {
            Debug::print(F("Cannot delete point, invalid index: "));
            Debug::println(index);
            return false;
        }
        
        // Shift remaining points
        for (int i = index; i < programSize - 1; i++) {
            programPoints[i] = programPoints[i + 1];
        }
        
        programSize--;
        Debug::print(F("Deleted program point "));
        Debug::println(index);
        return true;
    }
    
    bool executeProgram(float speedFactor) {
        if (programSize == 0) {
            Debug::println(F("No program to execute"));
            return false;
        }
        
        Debug::print(F("Executing program with "));
        Debug::print(programSize);
        Debug::print(F(" points at speed factor "));
        Debug::println(speedFactor);
        
        for (int i = 0; i < programSize; i++) {
            // Show progress
            DisplaySystem::showMessage("Executing Program", 
                                       ("Point " + String(i+1) + "/" + String(programSize)).c_str());
            
            // Move to position
            moveToPose(programPoints[i].position, true);
            
            // Apply delay (adjusted by speed factor)
            uint32_t adjustedDelay = programPoints[i].delayMs / speedFactor;
            
            // Delay with cancellation option
            unsigned long endTime = millis() + adjustedDelay;
            while (millis() < endTime) {
                // Check for cancel button press
                if (JoystickSystem::getLeftJoystick()->isPressed() && 
                    JoystickSystem::getRightJoystick()->isPressed()) {
                    DisplaySystem::showMessage("Program", "Cancelled", 1500);
                    return false;
                }
                
                // Use small delays to keep system responsive
                delay(10);
            }
        }
        
        DisplaySystem::showMessage("Program", "Completed", 1500);
        return true;
    }
    
    // Robot configuration management
    RobotConfig getRobotConfig() {
        return robotConfig;
    }
    
    void setRobotConfig(const RobotConfig& config) {
        robotConfig = config;
        
        // Update kinematic object with new configuration
        if (robotKin != nullptr) {
            delete robotKin;
        }
        
        robotKin = new RobotKinematics(robotConfig);
        
        Debug::println(F("Robot configuration updated"));
    }
    
    bool homeJoint(int jointIndex) {
        return _homeJoint(jointIndex);
    }
    
    int getSelectedJoint() {
        return selectedJoint;
    }
    
    void setSelectedJoint(int joint) {
        if (joint >= 0 && joint < 6) { // Assuming 6 joints
            selectedJoint = joint;
        }
    }

    // Initialisierung des Roboter-Systems
    void init() {
        Debug::println(F("Initializing robot system..."));
        
        // Roboter-Konfiguration initialisieren
        initRobotConfig();
        
        // Kinematik-Objekt erstellen
        robotKin = new RobotKinematics(robotConfig);
        
        // Gelenk-Winkel auf 0 initialisieren
        JointAngles init = {{0, 0, 0, 0, 0, 0}};
        robotKin->setCurrentJointAngles(init);
        
        // System-Zustand setzen
        currentState = STATE_STARTUP;
        stateChangeTime = millis();
        
        Debug::println(F("Robot system initialized"));
    }
    
    // Konfiguration des Roboters initialisieren
    void initRobotConfig() {
        Debug::println(F("Initializing robot configuration"));
        
        // Gelenk-Winkelgrenzen (in Grad)
        for (int i = 0; i < 6; i++) {
            robotConfig.jointMin[i] = -180.0 * DEG_TO_RAD;
            robotConfig.jointMax[i] = 180.0 * DEG_TO_RAD;
        }
        
        // DH-Parameter - einfacher 6DOF-Arm
        // Format: {a, alpha, d, theta}
        robotConfig.dhParams[0] = {0.0, M_PI/2, 50.0, 0.0};     // Basis
        robotConfig.dhParams[1] = {80.0, 0.0, 0.0, M_PI/2};     // Schulter
        robotConfig.dhParams[2] = {80.0, 0.0, 0.0, 0.0};        // Ellbogen
        robotConfig.dhParams[3] = {0.0, M_PI/2, 80.0, 0.0};     // Handgelenk Pitch
        robotConfig.dhParams[4] = {0.0, -M_PI/2, 0.0, 0.0};     // Handgelenk Roll
        robotConfig.dhParams[5] = {0.0, 0.0, 40.0, 0.0};        // Greifer
        
        // Werkzeug-Offset
        robotConfig.toolOffsetX = 0.0;
        robotConfig.toolOffsetY = 0.0;
        robotConfig.toolOffsetZ = 30.0;
    }
    
    // Haupt-Update-Funktion
    void update() {
        // Zustandsmachine verarbeiten
        processCurrentState();
    }
    
    // Zustandsmachine verarbeiten
    void processCurrentState() {
        switch (currentState) {
            case STATE_STARTUP:
                if (millis() - stateChangeTime > 2000) {
                    changeState(STATE_JOINT_MODE);
                }
                break;
            
            case STATE_JOINT_MODE:
                JoystickSystem::processJointModeJoysticks();
                break;
                
            case STATE_KINEMATIC_MODE:
                JoystickSystem::processKinematicModeJoysticks();
                break;
                
            case STATE_HOMING_MODE:
                processHomingMode();
                break;
                
            case STATE_CALIBRATION_MODE:
                // Anzeige aktualisieren - using the corrected function
                DisplaySystem::displayCalibrationMode();
                DisplaySystem::showMessage("Calibration Mode");
                break;
                
            case STATE_GEAR_MENU:
                DisplaySystem::processGearMenu();
                // Use alternative menu processing if available or implement a basic version
                break;
                
            case STATE_CONFIG_MODE:
                // Konfigurationsmodus-Logik hier
                break;
                
            case STATE_ERROR:
                // Fehlerbehandlung hier
                DisplaySystem::showMessage("System Error");
                break;
            
            case CALIBRATION:
                // Handle CALIBRATION state here
                // If this should be the same as STATE_CALIBRATION_MODE:
                DisplaySystem::displayCalibrationMode();
                break;
            
            case NORMAL_OPERATION:
                // Handle NORMAL_OPERATION state here
                // Choose appropriate default behavior
                JoystickSystem::processJointModeJoysticks(); // Or another default behavior
                break;
            
            default:
                // Handle any other unhandled states
                Debug::println(F("Unhandled system state in switch"));
                break;
        }
    }
    
    // Getter für den aktuellen Systemzustand
    SystemState getState() {
        return currentState;
    }
    
    // Setter für den Systemzustand
    void setState(SystemState state) {
        currentState = state;
        stateChangeTime = millis();
    }
    
    // Getter für den Zeitpunkt des letzten Zustandswechsels
    unsigned long getStateChangeTime() {
        return stateChangeTime;
    }
    
    // Setter für den Zeitpunkt des letzten Zustandswechsels
    void setStateChangeTime(unsigned long time) {
        stateChangeTime = time;
    }
    
    // Zustandswechsel mit Logging
    void changeState(SystemState newState) {
        Debug::print(F("Changing system state from "));
        Debug::print(static_cast<int>(currentState));
        Debug::print(F(" to "));
        Debug::println(static_cast<int>(newState));
        
        setState(newState);
    }
    
    // Getter für das Kinematik-Objekt
    RobotKinematics* getKinematics() {
        return robotKin;
    }
    
    // Prüfen, ob eine Pose innerhalb des Arbeitsbereichs liegt
    bool isWithinWorkspace(const CartesianPose& pose) {
        // Einfache Reichweiten-Prüfung
        float a1 = robotConfig.dhParams[1].a;
        float a2 = robotConfig.dhParams[2].a;
        float maxReach = a1 + a2;
        float minZ = robotConfig.dhParams[0].d;
        float maxZ = minZ + maxReach;
        
        // Euklidische Distanz zur Basis
        float dist = sqrt(pose.x*pose.x + pose.y*pose.y + 
                        (pose.z-minZ)*(pose.z-minZ));
        
        return (dist <= maxReach && pose.z >= minZ && pose.z <= maxZ);
    }
    
    // Bewegen zu einer bestimmten Pose
    void moveToPose(const CartesianPose& pose, bool waitForCompletion) {
        if (!isWithinWorkspace(pose)) {
            Debug::println(F("Target pose outside workspace!"));
            return;
        }
        
        // Inverse Kinematik berechnen
        JointAngles targetAngles;
        if (robotKin->inverseKinematics(pose, targetAngles)) {
            // Zielpositionen berechnen
            long targetSteps[6];
            for (int i = 0; i < 6; ++i) {
                float deg = targetAngles.angles[i] * 180.0 / M_PI;
                targetSteps[i] = deg * StepperSystem::getStepsPerDegree(i);
                StepperSystem::moveTo(i, targetSteps[i]);
            }
            
            // Auf Abschluss warten, falls gewünscht
            if (waitForCompletion) {
                bool allDone = false;
                while (!allDone) {
                    allDone = true;
                    for (int i = 0; i < 6; ++i) {
                        if (StepperSystem::getDistanceToGo(i) != 0) {
                            StepperSystem::run(i);
                            allDone = false;
                        }
                    }
                }
                
                // Kinematik mit Stepper-Positionen synchronisieren
                for (int i = 0; i < 6; ++i) {
                    float posDegrees = StepperSystem::getCurrentPosition(i) / 
                                 StepperSystem::getStepsPerDegree(i);
                    targetAngles.angles[i] = posDegrees * M_PI / 180.0;
                }
                
                robotKin->setCurrentJointAngles(targetAngles);
            }
        } else {
            Debug::println(F("Inverse kinematics failed!"));
        }
    }
    
    // Getter für den Kalibrierungs-Lock-Status
    bool isCalibrationLocked() {
        return calibrationLocked;
    }
    
    // Setter für den Kalibrierungs-Lock-Status
    void setCalibrationLocked(bool locked) {
        calibrationLocked = locked;
    }
    
    // Getter für den Homing-Status
    bool isHomingStarted() {
        return homingStarted;
    }
    
    // Setter für den Homing-Status
    void setHomingStarted(bool started) {
        homingStarted = started;
    }
    
    // Getter für den aktuellen Homing-Gelenk-Index
    int getHomingJointIndex() {
        return homingJointIndex;
    }
    
    // Setter für den Homing-Gelenk-Index
    void setHomingJointIndex(int index) {
        homingJointIndex = index;
    }
    
    // Getter für die aktuelle Homing-Menü-Auswahl
    int getHomingMenuSelection() {
        return homingMenuSelection;
    }
    
    // Setter für die Homing-Menü-Auswahl
    void setHomingMenuSelection(int selection) {
        homingMenuSelection = selection;
    }
    
    // Getter für die Anzahl der Homing-Menü-Optionen
    int getHomingMenuOptionCount() {
        return HOMING_MENU_COUNT;
    }
    
    // Getter für den Namen einer Homing-Menü-Option
    const char* getHomingMenuOptionName(int option) {
        switch (option) {
            case HOMING_MENU_START_HOMING: return "Start Homing";
            case HOMING_MENU_TO_CENTER:    return "Move to Center";
            case HOMING_MENU_SAVE_HOME:    return "Save Home";
            case HOMING_MENU_LOAD_HOME:    return "Load Home";
            case HOMING_MENU_CLEAR_HOME:   return "Clear Home";
            case HOMING_MENU_CONFIG:       return "Config";
            default:                       return "Unknown";
        }
    }
    
    // Get current joint angles
    JointAngles getCurrentJointAngles() {
        return robotKin->getCurrentJointAngles();
    }
    
    // Homing-Modus verarbeiten
    void processHomingMode() {
        // Menü immer anzeigen, wenn kein Homing läuft
        if (!homingStarted) {
            processHomingMenu();
            return;
        }
        
        // Wenn Homing läuft, Fortschritt anzeigen
        char progressMsg[32];
        sprintf(progressMsg, "Homing Joint %d", homingJointIndex + 1);
        DisplaySystem::showMessage(progressMsg);
        
        if (homingJointIndex < 6) {
            if (homeJoint(homingJointIndex)) {
                homingJointIndex++;
                Debug::print(F("Joint "));
                Debug::print(homingJointIndex);
                Debug::println(F(" referenced"));
                delay(500);
            }
        } else {
            // Nach Homing zurück ins Menü
            Debug::println(F("Homing completed"));
            homingStarted = false;
            homingJointIndex = 0;
            homingMenuSelection = 0;
            
            // Kinematik synchronisieren
            JointAngles homeAngles;
            for (int i = 0; i < 6; i++) {
                float posDegrees = StepperSystem::getCurrentPosition(i) / 
                              StepperSystem::getStepsPerDegree(i);
                homeAngles.angles[i] = posDegrees * M_PI / 180.0;
            }
            robotKin->setCurrentJointAngles(homeAngles);
        }
    }
    
    // Homing-Menü verarbeiten
    void processHomingMenu() {
        // Menü-Auswahl anzeigen
        char menuTitle[32];
        sprintf(menuTitle, "Homing Menu (%d/%d)", homingMenuSelection + 1, HOMING_MENU_COUNT);
        DisplaySystem::showMessage(menuTitle, getHomingMenuOptionName(homingMenuSelection));
        
        // Menüauswahl mit rechtem Joystick Y
        float rightY = JoystickSystem::getRightJoystick()->getNormalizedY();
        
        // Menü-Navigation (mit Entprellung)
        static unsigned long lastMenuMove = 0;
        if (millis() - lastMenuMove > 200) {
            if (rightY > 0.7f && homingMenuSelection > 0) {    
                homingMenuSelection++;
                lastMenuMove = millis();
            }
        }
        
        // Linker Button: Menüoption auswählen
        static bool lastLeftPressed = false;
                bool leftPressed = JoystickSystem::getLeftJoystick()->isPressed();
        
        if (leftPressed && !lastLeftPressed) {
            processHomingMenuSelection();
        }
        
        lastLeftPressed = leftPressed;
        
        // Rechter Button: Modus wechseln
        static bool lastRightPressed = false;
        bool rightPressed = JoystickSystem::getRightJoystick()->isPressed();
        
        if (rightPressed && !lastRightPressed) {
            if (millis() - stateChangeTime > 500) {
                // Modi durchschalten: Homing -> Calibration -> Joint -> Kinematic -> Homing
                if (currentState == STATE_HOMING_MODE) {
                    changeState(STATE_CALIBRATION_MODE);
                } else if (currentState == STATE_CALIBRATION_MODE) {
                    changeState(STATE_JOINT_MODE);
                } else if (currentState == STATE_JOINT_MODE) {
                    changeState(STATE_KINEMATIC_MODE);
                } else if (currentState == STATE_KINEMATIC_MODE) {
                    changeState(STATE_HOMING_MODE);
                    homingStarted = false;
                    homingJointIndex = 0;
                }
            }
        }
        
        lastRightPressed = rightPressed;
    }
    
    // Verarbeitet eine ausgewählte Option im Homing-Menü
    void processHomingMenuSelection() {
        switch (homingMenuSelection) {
            case HOMING_MENU_START_HOMING:
                homingStarted = true;
                homingJointIndex = 0;
                break;
                
            case HOMING_MENU_TO_CENTER:
                _moveToCenter();
                break;
                
            case HOMING_MENU_SAVE_HOME: {
                JointAngles current = robotKin->getCurrentJointAngles();
                saveRobotHome(current);
                DisplaySystem::showMessage("Home saved!", "Position stored", 1200);
                break;
            }
                
            case HOMING_MENU_LOAD_HOME: {
                JointAngles homeAngles;
                if (loadRobotHome(homeAngles)) {
                    // KEINE Bewegung! Nur Kinematik und Stepper-Positionen setzen
                    robotKin->setCurrentJointAngles(homeAngles);
                    for (int i = 0; i < 6; ++i) {
                        StepperSystem::setCurrentPosition(
                            i,
                            homeAngles.angles[i] * 180.0 / M_PI * StepperSystem::getStepsPerDegree(i)
                        );
                    }
                    Debug::println(F("Home loaded! Kinematics synchronized, no movement."));
                    DisplaySystem::showMessage("Home loaded!", "No movement", 1200);
                } else {
                    DisplaySystem::showMessage("No home", "saved!", 1200);
                }
                break;
            }
                
            case HOMING_MENU_CLEAR_HOME: {
                static unsigned long lastClearHomePress = 0;
                static int clearHomePressCount = 0;
                
                if (clearHomePressCount == 0 || millis() - lastClearHomePress > 1500) {
                    clearHomePressCount = 1;
                    lastClearHomePress = millis();
                    DisplaySystem::showMessage("Press again", "to clear home!");
                } else if (clearHomePressCount == 1) {
                    clearRobotHome();
                    DisplaySystem::showMessage("Home cleared!", "Position reset", 1200);
                    clearHomePressCount = 0;
                }
                break;
            }
                
            case HOMING_MENU_CONFIG:
                changeState(STATE_CONFIG_MODE);
                break;
        }
    }
    
    // Home-Position-Management
    
    // Home-Position im EEPROM speichern
    void saveRobotHome(const JointAngles& angles) {
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        RobotHomeData data;
        data.magic = ROBOT_HOME_MAGIC;
        
        for (int i = 0; i < 6; ++i) {
            data.jointAngles[i] = angles.angles[i];
        }
        
        EEPROM.put(EEPROM_ADDR_HOMING, data);
        Debug::println(F("Home position saved to EEPROM"));
    }
    
    // Home-Position aus EEPROM laden
    bool loadRobotHome(JointAngles& angles) {
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        RobotHomeData data;
        EEPROM.get(EEPROM_ADDR_HOMING, data);
        
        if (data.magic != ROBOT_HOME_MAGIC) {
            Debug::println(F("No valid home data found in EEPROM"));
            return false;
        }
        
        for (int i = 0; i < 6; ++i) {
            angles.angles[i] = data.jointAngles[i];
        }
        
        Debug::println(F("Home position loaded from EEPROM"));
        return true;
    }
    
    // Home-Position im EEPROM löschen
    void clearRobotHome() {
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        RobotHomeData data;
        data.magic = 0; // Clear magic number
        memset(data.jointAngles, 0, sizeof(data.jointAngles)); // Clear angles
        
        EEPROM.put(EEPROM_ADDR_HOMING, data);
        Debug::println(F("Home position cleared from EEPROM"));
    }
    
    // Home-Position auf SD-Karte speichern
    bool saveRobotHomeToSD(const JointAngles& angles, const char* filename) {
        // Standard-Dateiname, wenn keiner angegeben
        const char* fname = filename ? filename : "HOME.DAT";
        
        // SD-Karte überprüfen
        if (!SD.begin()) {
            Debug::println(F("SD card initialization failed!"));
            return false;
        }
        
        // Datei öffnen/erstellen
        File file = SD.open(fname, FILE_WRITE);
        if (!file) {
            Debug::println(F("Failed to open file for writing"));
            return false;
        }
        
        // Magic-Nummer schreiben
        uint32_t magic = ROBOT_HOME_MAGIC;
        file.write((uint8_t*)&magic, sizeof(magic));
        
        // Gelenk-Winkel schreiben
        for (int i = 0; i < 6; ++i) {
            float angle = angles.angles[i];
            file.write((uint8_t*)&angle, sizeof(float));
        }
        
        file.close();
        Debug::print(F("Home position saved to SD card as "));
        Debug::println(fname);
        return true;
    }
    
    // Home-Position von SD-Karte laden
    bool loadRobotHomeFromSD(JointAngles& angles, const char* filename) {
        // Standard-Dateiname, wenn keiner angegeben
        const char* fname = filename ? filename : "HOME.DAT";
        
        // SD-Karte überprüfen
        if (!SD.begin()) {
            Debug::println(F("SD card initialization failed!"));
            return false;
        }
        
        // Datei öffnen
        File file = SD.open(fname, FILE_READ);
        if (!file) {
            Debug::println(F("Failed to open file for reading"));
            return false;
        }
        
        // Magic-Nummer lesen und überprüfen
        uint32_t magic;
        file.read((uint8_t*)&magic, sizeof(magic));
        
        if (magic != ROBOT_HOME_MAGIC) {
            Debug::println(F("Invalid home file format"));
            file.close();
            return false;
        }
        
        // Gelenk-Winkel lesen
        for (int i = 0; i < 6; ++i) {
            float angle;
            file.read((uint8_t*)&angle, sizeof(float));
            angles.angles[i] = angle;
        }
        
        file.close();
        Debug::print(F("Home position loaded from SD card: "));
        Debug::println(fname);
        return true;
    }
    
    // Home-Position von SD-Karte löschen
    bool clearRobotHomeFromSD(const char* filename) {
        // Standard-Dateiname, wenn keiner angegeben
        const char* fname = filename ? filename : "HOME.DAT";
        
        // SD-Karte überprüfen
        if (!SD.begin()) {
            Debug::println(F("SD card initialization failed!"));
            return false;
        }
        
        // Datei löschen
        if (SD.exists(fname)) {
            if (SD.remove(fname)) {
                Debug::print(F("Home file deleted from SD card: "));
                Debug::println(fname);
                return true;
            } else {
                Debug::println(F("Failed to delete home file"));
                return false;
            }
        } else {
            Debug::println(F("Home file not found on SD card"));
            return false;
        }
    }
    
    // --- Private Hilfsfunktionen ---
    
    // Homing für ein einzelnes Gelenk durchführen
    bool _homeJoint(int jointIndex) {
        static bool homingJointStarted = false;
        static bool coarseHomingDone = false;
        static int lastSwitchState = HIGH;
        
        // Erste Initialisierung für dieses Gelenk
        if (!homingJointStarted) {
            homingJointStarted = true;
            coarseHomingDone = false;
            
            Debug::print(F("Starting homing for joint "));
            Debug::println(jointIndex + 1);
            
            // Motor aktivieren
            StepperSystem::enableMotor(jointIndex);
            
            // Schnelle Homing-Geschwindigkeit für die Grobsuche
            float homingSpeed = StepperSystem::getHomingSpeed(jointIndex);
            StepperSystem::setSpeed(jointIndex, -homingSpeed);
            
            // Aktuelle Position merken
            StepperSystem::setCurrentPosition(jointIndex, 0);
            
            // Initialen Endschalter-Zustand lesen
            int limitSwitchPin = StepperSystem::getLimitSwitchPin(jointIndex);
            lastSwitchState = digitalRead(limitSwitchPin);
            
            Debug::print(F("Limit switch pin: "));
            Debug::print(limitSwitchPin);
            Debug::print(F(", initial state: "));
            Debug::println(lastSwitchState == HIGH ? "HIGH" : "LOW");
        }
        
        // Endschalter überprüfen
        int limitSwitchPin = StepperSystem::getLimitSwitchPin(jointIndex);
        int currentSwitchState = digitalRead(limitSwitchPin);
        
        // Phase 1: Grobe Suche nach dem Endschalter
        if (!coarseHomingDone) {
            if (currentSwitchState != lastSwitchState && currentSwitchState == LOW) {
                Debug::print(F("Limit switch for joint "));
                Debug::print(jointIndex + 1);
                Debug::println(F(" reached during coarse homing"));
                
                // Motor anhalten
                StepperSystem::setSpeed(jointIndex, 0);
                StepperSystem::stop(jointIndex);
                
                delay(100);
                
                // Vom Endschalter zurückfahren (positive Richtung, weg vom Endschalter)
                StepperSystem::move(jointIndex, 100); // Etwa 5 Grad zurück bei 20 steps/degree
                float homingSpeed = StepperSystem::getHomingSpeed(jointIndex);
                StepperSystem::setSpeed(jointIndex, homingSpeed * 0.5);
                
                while (StepperSystem::getDistanceToGo(jointIndex) != 0) {
                    StepperSystem::runSpeed(jointIndex);
                }
                
                delay(100);
                
                Debug::print(F("Retreat completed, starting precise homing for joint "));
                Debug::println(jointIndex + 1);
                
                // Langsames Homing einleiten (1/4 der normalen Geschwindigkeit)
                StepperSystem::setSpeed(jointIndex, -homingSpeed * 0.25);
                
                coarseHomingDone = true;
                lastSwitchState = HIGH; // Zurücksetzen für die Feinsuche
            }
            
            lastSwitchState = currentSwitchState;
            StepperSystem::runSpeed(jointIndex);
        }
        // Phase 2: Präzise langsame Suche
        else {
            if (currentSwitchState != lastSwitchState && currentSwitchState == LOW) {
                Debug::print(F("Limit switch for joint "));
                Debug::print(jointIndex + 1);
                Debug::print(F(" reached during precise homing (Pin "));
                Debug::print(limitSwitchPin);
                Debug::println(F(")"));
                
                StepperSystem::setSpeed(jointIndex, 0);
                StepperSystem::stop(jointIndex);
                
                StepperSystem::setCurrentPosition(jointIndex, 0);
                
                StepperSystem::disableMotor(jointIndex);
                
                JointAngles currentAngles = robotKin->getCurrentJointAngles();
                currentAngles.angles[jointIndex] = 0.0f;
                robotKin->setCurrentJointAngles(currentAngles);
                
                homingJointStarted = false;
                coarseHomingDone = false;
                return true; // Homing für dieses Gelenk abgeschlossen
            }
            
            lastSwitchState = currentSwitchState;
            StepperSystem::runSpeed(jointIndex);
        }
        
        return false; // Homing für dieses Gelenk läuft noch
    }
    
    // Bewegt den Roboter zur zentralen Position
    void _moveToCenter() {
        Debug::println(F("Menu: Move to center selected"));
       
        // Zielpunkt im Arbeitsraum definieren
        float a1 = robotConfig.dhParams[1].a;
        float a2 = robotConfig.dhParams[2].a;
        float minZ = robotConfig.dhParams[0].d;
        
        CartesianPose centerPose;
        centerPose.x = 0;
                centerPose.y = 0;
        centerPose.z = minZ + (a1 + a2) / 2.0f;
        centerPose.yaw = 0;
        centerPose.pitch = 0;
        centerPose.roll = 0;
        
        Debug::print(F("Target pose: X="));
        Debug::print(centerPose.x);
        Debug::print(F(" Y="));
        Debug::print(centerPose.y);
        Debug::print(F(" Z="));
        Debug::println(centerPose.z);
        
        // IK berechnen
        JointAngles centerAngles;
        if (robotKin->inverseKinematics(centerPose, centerAngles)) {
            // Zielpositionen berechnen
            long targetSteps[6];
            long startSteps[6];
            
            for (int i = 0; i < 6; ++i) {
                float deg = centerAngles.angles[i] * 180.0 / M_PI;
                targetSteps[i] = deg * StepperSystem::getStepsPerDegree(i);
                startSteps[i] = StepperSystem::getCurrentPosition(i);
                StepperSystem::moveTo(i, targetSteps[i]);
            }
            
            // Bewegung mit Fortschrittsbalken
            bool allDone = false;
            while (!allDone) {
                allDone = true;
                long maxDist = 0, maxDistToGo = 0;
                
                for (int i = 0; i < 6; ++i) {
                    long dist = abs(targetSteps[i] - startSteps[i]);
                    long distToGo = abs(StepperSystem::getDistanceToGo(i));
                    
                    if (dist > maxDist) maxDist = dist;
                    if (distToGo > maxDistToGo) maxDistToGo = distToGo;
                    
                    if (StepperSystem::getDistanceToGo(i) != 0) {
                        StepperSystem::run(i);
                        allDone = false;
                    }
                }
                
                // Fortschritt berechnen (0.0 ... 1.0)
                float progress = 1.0f;
                if (maxDist > 0) {
                    progress = 1.0f - (float)maxDistToGo / (float)maxDist;
                    if (progress < 0) progress = 0;
                    if (progress > 1) progress = 1;
                }
                
                // Fortschrittsbalken anzeigen
                char progressText[20];
                sprintf(progressText, "Progress: %d%%", (int)(progress * 100));
                DisplaySystem::showMessage("Moving to center", progressText);
                
                delay(10);
            }
            
            // Stepper-Positionen aktualisieren
            for (int i = 0; i < 6; ++i) {
                StepperSystem::setCurrentPosition(i, StepperSystem::getTargetPosition(i));
            }
            
            // Kinematik aktualisieren
            robotKin->setCurrentJointAngles(centerAngles);
            
            // FK zur Kontrolle
            CartesianPose pose = robotKin->getCurrentPose();
            Debug::print(F("FK End pose: X="));
            Debug::print(pose.x, 2);
            Debug::print(F(" Y="));
            Debug::print(pose.y, 2);
            Debug::print(F(" Z="));
            Debug::print(pose.z, 2);
            Debug::print(F(" | Yaw="));
            Debug::print(pose.yaw * 180.0 / M_PI, 2);
            Debug::print(F(" Pitch="));
            Debug::print(pose.pitch * 180.0 / M_PI, 2);
            Debug::print(F(" Roll="));
            Debug::println(pose.roll * 180.0 / M_PI, 2);
            
            DisplaySystem::showMessage("Robot is now", "in center!", 1500);
        } else {
            Debug::println(F("IK error! Target not reachable."));
            DisplaySystem::showMessage("IK error!", "Target not reachable", 1500);
        }
    }
    
    // Define any other helper functions or public methods as needed
    
    // Function to move robot to center - public wrapper for the private function
    void moveToCenter() {
        _moveToCenter();
    }
    
    // You can define additional wrapper functions or public interfaces here
    
} // namespace RobotSystem

namespace JoystickSystem {
    // Implementation of calibration data functions
    
    // Calibration data storage
    static float _leftMinValues[2] = {0, 0};   // X, Y min values for left joystick
    static float _leftMaxValues[2] = {1023, 1023};   // X, Y max values for left joystick
    static float _rightMinValues[2] = {0, 0};  // X, Y min values for right joystick
    static float _rightMaxValues[2] = {1023, 1023};  // X, Y max values for right joystick
    
    void getCalibrationData(float leftMin[2], float leftMax[2], 
                           float rightMin[2], float rightMax[2]) {
        // Copy calibration data to provided arrays
        leftMin[0] = _leftMinValues[0];
        leftMin[1] = _leftMinValues[1];
        
        leftMax[0] = _leftMaxValues[0];
        leftMax[1] = _leftMaxValues[1];
        
        rightMin[0] = _rightMinValues[0];
        rightMin[1] = _rightMinValues[1];
        
        rightMax[0] = _rightMaxValues[0];
        rightMax[1] = _rightMaxValues[1];
    }
    
    void setCalibrationData(const float leftMin[2], const float leftMax[2], 
                           const float rightMin[2], const float rightMax[2]) {
        // Store calibration data
        _leftMinValues[0] = leftMin[0];
        _leftMinValues[1] = leftMin[1];
        
        _leftMaxValues[0] = leftMax[0];
        _leftMaxValues[1] = leftMax[1];
        
        _rightMinValues[0] = rightMin[0];
        _rightMinValues[1] = rightMin[1];
        
        _rightMaxValues[0] = rightMax[0];
        _rightMaxValues[1] = rightMax[1];
        
        // Apply calibration to joysticks
        Joystick* leftJoystick = getLeftJoystick();
        Joystick* rightJoystick = getRightJoystick();
        
        if (leftJoystick != nullptr) {
            leftJoystick->saveMinMax(_leftMinValues[0], _leftMaxValues[0], 
                                    _leftMinValues[1], _leftMaxValues[1]);
        }
        
        if (rightJoystick != nullptr) {
            rightJoystick->saveMinMax(_rightMinValues[0], _rightMaxValues[0], 
                                     _rightMinValues[1], _rightMaxValues[1]);
        }
        
        Debug::println(F("Joystick calibration data updated"));
    }

    // Other JoystickSystem functions are already defined in the existing code
}
