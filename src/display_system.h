#ifndef DISPLAY_SYSTEM_H
#define DISPLAY_SYSTEM_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RobotKinematics.h"
#include "Debug.h"

// Display-Konstanten
#define SCREEN_WIDTH 128      // OLED-Display-Breite in Pixel
#define SCREEN_HEIGHT 64      // OLED-Display-Höhe in Pixel
#define OLED_RESET    -1      // Reset-Pin (-1 = teilen mit Arduino Reset)
#define SCREEN_ADDRESS 0x3C   // I2C-Adresse des OLED-Displays (typisch: 0x3C oder 0x3D)
// UI-Konstanten
#define NUM_TABS 5            // Anzahl der Tabs in der UI
#define TAB_HEIGHT 12         // Höhe der Tab-Leiste in Pixel
#define TAB_WIDTH 25          // Breite eines einzelnen Tabs in Pixel

namespace DisplaySystem {
    // Initialisierung
    void init();
    
    // Haupt-Update-Funktion
    void update();
    
    // Zugriff auf das Display-Objekt
    Adafruit_SSD1306* getDisplay();
    // Tab-Verwaltungsfunktionen
    void selectNextTab();
    void selectPrevTab();
    void drawTabs();
    
    // Verschiedene Anzeigemodi
    void displayStartupScreen();
    void displayJointMode(RobotKinematics* kinematics, int selectedJoint);
    void displayPositionMode(RobotKinematics* kinematics);
    void displayFullPoseMode(RobotKinematics* kinematics);
    void displayTeachingMode();
    void displayCalibrationMode();  // Adding missing function
    void displayHomingMode(int jointIndex = -1);
    void displayHomingMenu(int selection, int totalOptions);
    void displayHomingProgress(int jointIndex, float progress = 0.0f);  // Adding missing function
    void displayHomingCenterProgress(float progress);  // Adding missing function
    void displaySettingsMenu();
    void displayDebugScreen();
    
    // Processing functions
    void processGearMenu();  
    
    // Spezialisierte Anzeigefunktionen
    void showMessage(const char* line1, const char* line2 = nullptr, int delayMs = 0);  
    void showModeChange(int newMode);
    void showError(const char* errorMsg);  
    void showCalibrationInProgress();
    void showCalibrationComplete();
    
    // Fortschrittsanzeige
    void drawProgressBar(int x, int y, int width, int height, float progress);
    
    // Getriebe-Menü-Funktionen
    void displayGearMenu();
    bool isGearMenuActive();
    void setGearMenuActive(bool active);
    int getSelectedGearAxis();
    void setSelectedGearAxis(int axis);
    
    // Home-Position-Verwaltung
    void displayHomeMode();
    void saveHomePositionToSD(const JointAngles& angles);
    bool loadHomePositionFromSD(JointAngles& angles);
    void listHomePositionsOnSD();
    void deleteHomePositionFromSD(const char* filename);
}

#endif // DISPLAY_SYSTEM_H
