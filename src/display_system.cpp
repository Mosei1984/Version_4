#include "display_system.h"
#include "Debug.h"
#include "joystick_system.h"
#include <Wire.h>
#include <limits.h>
#include "menu_system.h"
#include "RobotKinematics.h"
#include "robot_system.h"
#include "config.h"
#include <SD.h>

// Display instance
static Adafruit_SSD1306* display = nullptr;

// Tab icons (16×12 pixels each)
static const unsigned char PROGMEM home_icon[] = {
  0x1F, 0xF8, 0x20, 0x04, 0x40, 0x02, 0x40, 0x02, 0x80, 0x01, 0x80, 0x01,
  0xFF, 0xFF, 0x84, 0x21, 0x84, 0x21, 0x84, 0x21, 0x84, 0x21, 0x84, 0x21
};
static const unsigned char PROGMEM joint_icon[] = {
  0x00, 0x00, 0x0F, 0xF0, 0x10, 0x08, 0x20, 0x04, 0x40, 0x02, 0x40, 0x02,
  0x40, 0x02, 0x40, 0x02, 0x20, 0x04, 0x10, 0x08, 0x0F, 0xF0, 0x00, 0x00
};
static const unsigned char PROGMEM position_icon[] = {
  0x00, 0x00, 0x07, 0xE0, 0x08, 0x10, 0x13, 0xC8, 0x14, 0x28, 0x14, 0x28,
  0x14, 0x28, 0x14, 0x28, 0x14, 0x28, 0x08, 0x10, 0x07, 0xE0, 0x00, 0x00
};
static const unsigned char PROGMEM settings_icon[] = {
  0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 0x7F, 0xFE, 0x7F, 0xFE, 0x03, 0xC0,
  0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 0x7F, 0xFE, 0x7F, 0xFE, 0x03, 0xC0
};
static const unsigned char PROGMEM debug_icon[] = {
  0x00, 0x00, 0x3F, 0xFC, 0x40, 0x02, 0x80, 0x01, 0x9F, 0xF9, 0x90, 0x09,
  0x90, 0x09, 0x9F, 0xF9, 0x80, 0x01, 0x40, 0x02, 0x3F, 0xFC, 0x00, 0x00
};

static const unsigned char* const tab_icons[NUM_TABS] = {
  home_icon, joint_icon, position_icon, settings_icon, debug_icon
};

static const char* const tab_labels[NUM_TABS] = {
  "Home", "Joint", "Pos", "Set", "Debug"
};

// UI state
static int    currentTab       = 0;
static bool   gearMenuActive   = false;
static int    selectedGearAxis = 0;
static unsigned long lastJoyChk = 0;
static unsigned long msgEndTime = 0;
static char   msgText[32]      = {0};

// Helper for free RAM
extern "C" char* sbrk(int incr);
static int freeMemory() {
  char stack_dummy;
  return &stack_dummy - reinterpret_cast<char*>(sbrk(0));
}

namespace DisplaySystem {

void init() {
  DEBUG_PRINTLN("Initializing Display System");
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  if (!display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    DEBUG_PRINTLN("SSD1306 allocation failed");
    return;
  }
  displayStartupScreen();
}

void displayStartupScreen() {
  display->clearDisplay();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);
  display->setCursor(0, 0);
  display->println(F("Robot Controller"));
  display->println(F("Initializing..."));
  display->display();
  delay(1000);
  DEBUG_PRINTLN("Display initialized successfully");
}

void update() {
  // Tab navigation via joystick
  if (millis() - lastJoyChk > 200) {
    float rx = JoystickSystem::getRightX();
    if      (rx >  0.7f) selectNextTab();
    else if (rx < -0.7f) selectPrevTab();
    lastJoyChk = millis();
  }

  // Expire temporary message
  if (msgEndTime && millis() > msgEndTime) msgEndTime = 0;
  if (msgEndTime) {
    display->clearDisplay();
    display->setCursor(0, TAB_HEIGHT+3);
    display->println(msgText);
    display->display();
    return;
  }

  // Draw UI
  drawTabs();
  switch (currentTab) {
    case 0: displayHomeMode(); break;
    case 1: displayJointMode(RobotSystem::getKinematics(), RobotSystem::getSelectedJoint()); break;
    case 2: displayPositionMode(RobotSystem::getKinematics()); break;
    case 3: displaySettingsMenu(); break;
    case 4: displayDebugScreen(); break;
  }
  display->display();
}

Adafruit_SSD1306* getDisplay() {
  return display;
}

void selectNextTab() {
  currentTab = (currentTab + 1) % NUM_TABS;
  DEBUG_PRINT(F("Tab -> "));
  DEBUG_PRINTLN(tab_labels[currentTab]);
}

void selectPrevTab() {
  currentTab = (currentTab - 1 + NUM_TABS) % NUM_TABS;
  DEBUG_PRINT(F("Tab -> "));
  DEBUG_PRINTLN(tab_labels[currentTab]);
}

void drawTabs() {
  display->clearDisplay();
  // Background
  display->fillRect(0, 0, SCREEN_WIDTH, TAB_HEIGHT, SSD1306_BLACK);
  // Dividers
  for (int i = 0; i <= NUM_TABS; i++) {
    int x = i * TAB_WIDTH;
    display->drawLine(x, 0, x, TAB_HEIGHT, SSD1306_WHITE);
  }
  // Icons
  for (int i = 0; i < NUM_TABS; i++) {
    int x = i * TAB_WIDTH;
    if (i == currentTab) {
      display->fillRect(x+1, 0, TAB_WIDTH-1, TAB_HEIGHT, SSD1306_WHITE);
      display->drawBitmap(x+4, 0, tab_icons[i], 16, 12, SSD1306_BLACK);
    } else {
      display->drawBitmap(x+4, 0, tab_icons[i], 16, 12, SSD1306_WHITE);
    }
  }
  display->drawLine(0, TAB_HEIGHT, SCREEN_WIDTH, TAB_HEIGHT, SSD1306_WHITE);
}

void clearMainArea() {
  display->fillRect(0, TAB_HEIGHT+1, SCREEN_WIDTH, SCREEN_HEIGHT - TAB_HEIGHT -1, SSD1306_BLACK);
}

// Display modes

void displayHomeMode() {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("Home Screen"));
  display->println(F("Use R joystick to change tabs"));
}

void displayJointMode(RobotKinematics* kin, int selJoint) {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("Joint Mode"));
  auto angles = kin->getCurrentJointAngles().angles;
  for (int i = 0; i < NUM_TABS; i++) {
    float deg = angles[i] * 180.0f / M_PI;
    display->print(F("J")); display->print(i+1);
    display->print(F(":")); display->print(deg, 1);
    if (i == selJoint) display->print(F(" *"));
    display->println();
  }
}

void displayPositionMode(RobotKinematics* kin) {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("Position Mode"));
  auto p = kin->getCurrentPose();
  display->print(F("X: ")); display->print(p.x,2);
  display->print(F(" Y: ")); display->print(p.y,2);
  display->print(F(" Z: ")); display->println(p.z,2);
}

void displayFullPoseMode(RobotKinematics* kin) {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("Full Pose Mode"));
  auto p = kin->getCurrentPose();
  display->print(F("X:")); display->print(p.x,1);
  display->print(F(" Y:")); display->print(p.y,1);
  display->println();
  display->print(F("Z:")); display->print(p.z,1);
  display->println();
  display->print(F("R:")); display->print(p.roll,1);
  display->print(F(" P:")); display->print(p.pitch,1);
  display->print(F(" Y:")); display->println(p.yaw,1);
}

void displayTeachingMode() {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("Teaching Mode"));
}

void displayCalibrationMode() {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("Calibration Mode"));
}

void displayHomingMode(int jointIndex) {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->print(F("Homing Mode (J")); display->print(jointIndex+1);
  display->println(F(")"));
}

void displayHomingMenu(int selection, int totalOptions) {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("Homing Menu"));
  for (int i = 0; i < totalOptions; i++) {
    display->print(i==selection ? "> " : "  ");
    display->print(F("Option ")); display->println(i+1);
  }
}

void displayHomingProgress(int jointIndex, float progress) {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->print(F("Homing J")); display->print(jointIndex+1);
  display->println(F("..."));
  drawProgressBar(10,40,SCREEN_WIDTH-20,10,progress);
}

void displayHomingCenterProgress(float progress) {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("Centering..."));
  drawProgressBar(10,40,SCREEN_WIDTH-20,10,progress);
}

void displaySettingsMenu() {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("Settings Menu"));
  display->println(F("1. Calibration"));
  display->println(F("2. Homing"));
  display->println(F("3. Speed"));
}

void displayGearMenu() {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("Gear Configuration"));
}

bool isGearMenuActive() {
  return gearMenuActive;
}
void setGearMenuActive(bool active) {
  gearMenuActive = active;
}

int getSelectedGearAxis() {
  return selectedGearAxis;
}
void setSelectedGearAxis(int axis) {
  selectedGearAxis = axis;
}

void processGearMenu() {
  displayGearMenu();
}

void displayDebugScreen() {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("Debug Info"));
  display->print(F("Free RAM: ")); display->println(freeMemory());
}

void showMessage(const char* line1, const char* line2, int delayMs) {
  if (line2) {
    snprintf(msgText, sizeof(msgText), "%s\n%s", line1, line2);
  } else {
    strncpy(msgText, line1, sizeof(msgText)-1);
    msgText[sizeof(msgText)-1] = '\0';
  }
  msgEndTime = delayMs>0 ? millis()+delayMs : ULONG_MAX;
}

void showModeChange(int newMode) {
  DEBUG_PRINT(F("Mode -> ")); DEBUG_PRINTLN(newMode);
}

void showError(const char* errorMsg) {
  clearMainArea();
  display->setCursor(0, TAB_HEIGHT+3);
  display->println(F("ERROR"));
  display->println(errorMsg);
  DEBUG_PRINT(F("ERROR: ")); DEBUG_PRINTLN(errorMsg);
}

void showCalibrationInProgress() {
  showMessage("Calibrating...", nullptr, 1000);
}

void showCalibrationComplete() {
  showMessage("Calibration Done", nullptr, 1000);
}

void saveHomePositionToSD(const JointAngles& angles) {
  DEBUG_PRINTLN("Saving home pos to SD");
  // implement as needed
}

bool loadHomePositionFromSD(JointAngles& angles) {
  DEBUG_PRINTLN("Loading home pos from SD");
  return false; // implement as needed
}

void listHomePositionsOnSD() {
  DEBUG_PRINTLN("Listing home positions");
}

void deleteHomePositionFromSD(const char* filename) {
  DEBUG_PRINT(F("Deleting home pos: ")); DEBUG_PRINTLN(filename);
}

void drawProgressBar(int x, int y, int w, int h, float progress) {
  display->drawRect(x,y,w,h,SSD1306_WHITE);
  int fill = (int)((w-2)*constrain(progress,0.0f,1.0f));
  if (fill>0) display->fillRect(x+1,y+1,fill,h-2,SSD1306_WHITE);
}

} // namespace DisplaySystem
