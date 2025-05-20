#include "menu_system.h"
#include "Debug.h"
#include "display_system.h"
#include "joystick_system.h"
#include "robot_system.h"
#include "stepper_system.h"
#include <EEPROM.h>
#include <SD.h>
#include "imxrt.h" 



namespace MenuSystem {
    // Private variables in the namespace
    MenuItem menuItems[MAX_MENU_ITEMS];
    int menuItemCount = 0;
    int currentIndex = 0;
    int menuLevel = 0;
    MenuHistoryEntry menuHistory[MAX_MENU_DEPTH];
    unsigned long lastInputTime = 0;
    int selectedMotor = 0;
    const char* motorOptions[6] = {"Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"};
    float gearRatios[6] = {20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f}; // Default values
    
    // Helper variables for menu actions
    static float testSpeed = 0.5f;
    static bool invertLimitSwitches = false;
    
    // Timing for debounce
    const int inputDelayMs = 200;
    
    // Calibration state
    bool calibrationMode = false;
    int calibrationStep = 0;
    
    // Initialize the menu system
    void init() {
        Debug::println(F("Initializing menu system..."));
        
        // Start with main menu
        buildMainMenu();
        
        // Initialize timing
        lastInputTime = millis();
        
        Debug::println(F("Menu system initialized"));
    }
    
    // Main update function
    void update() {
        // Display current menu
        displayMenu();
        
        // Process user input
        processMenuInput();
        
        // Auto-exit from deep menus after inactivity
        if (menuLevel > 0 && millis() - lastInputTime > MENU_TIMEOUT_MS) {
            goBack();
        }
        
        // Process special modes
        if (RobotSystem::getState() == STATE_HOMING_MODE) {
            RobotSystem::processHomingMode();
        }
        else if (RobotSystem::getState() == STATE_GEAR_MENU) {
            DisplaySystem::processGearMenu();
        }
        else if (calibrationMode) {
            JoystickSystem::processCalibration();
        }
    }
    
    // Menu navigation
    void navigateUp() {
        if (currentIndex > 0) {
            currentIndex--;
            lastInputTime = millis();
        }
    }
    
    void navigateDown() {
        if (currentIndex < menuItemCount - 1) {
            currentIndex++;
            lastInputTime = millis();
        }
    }
    
    void selectItem() {
        if (currentIndex >= 0 && currentIndex < menuItemCount) {
            MenuItem* item = &menuItems[currentIndex];
            
            switch (item->type) {
                case MENU_ACTION:
                    executeMenuAction(item->actionFunc);
                    break;
                    
                case MENU_SUBMENU:
                    enterSubmenu(item->submenuIndex);
                    break;
                    
                case MENU_TOGGLE:
                    if (item->toggleValue) {
                        *(item->toggleValue) = !(*(item->toggleValue));
                    }
                    break;
                    
                case MENU_NUMERIC:
                    adjustNumericValue(currentIndex);
                    break;
                    
                case MENU_SELECTION:
                    adjustSelectionValue(currentIndex);
                    break;
                    
                case MENU_BACK:
                    goBack();
                    break;
            }
            
            lastInputTime = millis();
        }
    }
    
    void goBack() {
        if (menuLevel > 0) {
            menuLevel--;
            
            // If we're back to main menu
            if (menuLevel == 0) {
                buildMainMenu();
                currentIndex = menuHistory[menuLevel].menuIndex;
            } 
            else {
                // Load previous submenu
                loadSubmenu(menuHistory[menuLevel-1].submenuIndex);
                currentIndex = menuHistory[menuLevel].menuIndex;
            }
            
            lastInputTime = millis();
        }
    }
    
    // Menu definition
    void defineMenu(const MenuItem* items, int count) {
        // Ensure we don't have too many menu items
        if (count > MAX_MENU_ITEMS) {
            count = MAX_MENU_ITEMS;
        }
        
        // Copy menu items
        memcpy(menuItems, items, count * sizeof(MenuItem));
        menuItemCount = count;
        currentIndex = 0;
    }
    
    void addMenuItem(const char* text, MenuItemType type) {
        if (menuItemCount < MAX_MENU_ITEMS) {
            menuItems[menuItemCount].text = text;
            menuItems[menuItemCount].type = type;
            menuItemCount++;
        }
    }
    
    void addActionItem(const char* text, void (*actionFunc)(void)) {
        if (menuItemCount < MAX_MENU_ITEMS) {
            menuItems[menuItemCount].text = text;
            menuItems[menuItemCount].type = MENU_ACTION;
            menuItems[menuItemCount].actionFunc = actionFunc;
            menuItemCount++;
        }
    }
    
    void addSubmenuItem(const char* text, int submenuIndex) {
        if (menuItemCount < MAX_MENU_ITEMS) {
            menuItems[menuItemCount].text = text;
            menuItems[menuItemCount].type = MENU_SUBMENU;
            menuItems[menuItemCount].submenuIndex = submenuIndex;
            menuItemCount++;
        }
    }
    
    void addToggleItem(const char* text, bool* value) {
        if (menuItemCount < MAX_MENU_ITEMS) {
            menuItems[menuItemCount].text = text;
            menuItems[menuItemCount].type = MENU_TOGGLE;
            menuItems[menuItemCount].toggleValue = value;
            menuItemCount++;
        }
    }
    
    void addNumericItem(const char* text, float* value, float min, float max, float step) {
        if (menuItemCount < MAX_MENU_ITEMS) {
            menuItems[menuItemCount].text = text;
            menuItems[menuItemCount].type = MENU_NUMERIC;
            menuItems[menuItemCount].numeric.value = value;
            menuItems[menuItemCount].numeric.min = min;
            menuItems[menuItemCount].numeric.max = max;
            menuItems[menuItemCount].numeric.step = step;
            menuItemCount++;
        }
    }
    
    void addSelectionItem(const char* text, int* value, const char** options, int optionCount) {
        if (menuItemCount < MAX_MENU_ITEMS) {
            menuItems[menuItemCount].text = text;
            menuItems[menuItemCount].type = MENU_SELECTION;
            menuItems[menuItemCount].selection.value = value;
            menuItems[menuItemCount].selection.options = options;
            menuItems[menuItemCount].selection.optionCount = optionCount;
            menuItemCount++;
        }
    }
    
    void addBackItem(const char* text) {
        if (menuItemCount < MAX_MENU_ITEMS) {
            menuItems[menuItemCount].text = text;
            menuItems[menuItemCount].type = MENU_BACK;
            menuItemCount++;
        }
    }
    
    // Submenus
    void enterSubmenu(int submenuIndex) {
        if (menuLevel < MAX_MENU_DEPTH - 1) {
            // Save current menu state to history
            menuHistory[menuLevel].menuIndex = currentIndex;
            menuHistory[menuLevel].submenuIndex = submenuIndex;
            
            // Increase menu level
            menuLevel++;
            
            // Load new menu
            loadSubmenu(submenuIndex);
            currentIndex = 0;
        }
    }
    
    void exitSubmenu() {
        goBack();
    }
    
    // Menu state
    int getCurrentMenuIndex() {
        return currentIndex;
    }
    
    int getCurrentMenuLevel() {
        return menuLevel;
    }
    
    const MenuItem* getCurrentMenuItem() {
        if (currentIndex >= 0 && currentIndex < menuItemCount) {
            return &menuItems[currentIndex];
        }
        return nullptr;
    }
    
    // Generic menu display
    void displayMenu() {
        // Get display object
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(WHITE);
        
        // Show title
        display->setCursor(0, 0);
        display->println("MENU");
        display->drawLine(0, 9, display->width() - 1, 9, WHITE);
        
        // Calculate visible range with scrolling
        int visibleItems = 6; // Number of items visible at once
        int startItem = max(0, currentIndex - (visibleItems / 2));
        startItem = min(startItem, max(0, menuItemCount - visibleItems));
        
                // Display menu items
        for (int i = 0; i < visibleItems && i + startItem < menuItemCount; i++) {
            int itemIndex = i + startItem;
            int y = 11 + i * 9;
            
            // Highlight selected item
            if (itemIndex == currentIndex) {
                display->fillRect(0, y, display->width(), 9, WHITE);
                display->setTextColor(BLACK);
            } else {
                display->setTextColor(WHITE);
            }
            
            // Show item text
            display->setCursor(2, y + 1);
            display->print(menuItems[itemIndex].text);
            
            // Show type-specific markers or values
            displayItemExtras(itemIndex, display->width() - 20, y + 1);
            
            // Reset text color
            display->setTextColor(WHITE);
        }
        
        // Draw scrollbar if needed
        if (menuItemCount > visibleItems) {
            drawScrollbar(startItem, visibleItems);
        }
        
        display->display();
    }
    
    // Process joystick inputs for menu navigation
    void processMenuInput() {
        // Get joystick values
        float rightY = JoystickSystem::getRightJoystick()->getNormalizedY();
        //float rightX = JoystickSystem::getRightJoystick()->getNormalizedX();
        
        static bool lastLeftPressed = false;
        static bool lastRightPressed = false;
        static unsigned long lastMenuMove = 0;
        
        bool leftPressed = JoystickSystem::getLeftJoystick()->isPressed();
        bool rightPressed = JoystickSystem::getRightJoystick()->isPressed();
        
        // Navigate menu with debounce
        if (millis() - lastMenuMove > inputDelayMs) {
            if (rightY > 0.7f) {
                navigateUp();
                lastMenuMove = millis();
            } else if (rightY < -0.7f) {
                navigateDown();
                lastMenuMove = millis();
            }
        }
        
        // Left button: select item
        if (leftPressed && !lastLeftPressed) {
            selectItem();
        }
        
        // Right button: go back
        if (rightPressed && !lastRightPressed) {
            goBack();
        }
        
        lastLeftPressed = leftPressed;
        lastRightPressed = rightPressed;
    }
    
    // Helper function to display type-specific markers or values
    void displayItemExtras(int itemIndex, int x, int y) {
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        
        switch (menuItems[itemIndex].type) {
            case MENU_SUBMENU:
                display->print(" >");
                break;
                
            case MENU_TOGGLE:
                if (menuItems[itemIndex].toggleValue) {
                    display->print(*(menuItems[itemIndex].toggleValue) ? " [x]" : " [ ]");
                }
                break;
                
            case MENU_NUMERIC:
                if (menuItems[itemIndex].numeric.value) {
                    display->setCursor(x, y);
                    display->print(*(menuItems[itemIndex].numeric.value), 1);
                }
                break;
                
            case MENU_SELECTION:
                if (menuItems[itemIndex].selection.value && 
                    menuItems[itemIndex].selection.options && 
                    *(menuItems[itemIndex].selection.value) < menuItems[itemIndex].selection.optionCount) {
                    
                    int value = *(menuItems[itemIndex].selection.value);
                    display->setCursor(x, y);
                    display->print(menuItems[itemIndex].selection.options[value]);
                }
                break;
                
            default:
                break;
        }
    }
    
    // Draw a scrollbar
    void drawScrollbar(int startItem, int visibleItems) {
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        
        int barHeight = 54; // Height of scrollbar area
        int handleHeight = max(5, barHeight * visibleItems / menuItemCount);
        int handleY = 10 + (barHeight - handleHeight) * startItem / max(1, menuItemCount - visibleItems);
        
        // Draw scrollbar track
        display->drawRect(display->width() - 3, 10, 3, barHeight, WHITE);
        
        // Draw scrollbar handle
        display->fillRect(display->width() - 3, handleY, 3, handleHeight, WHITE);
    }
    
    // Adjust a numeric value
    void adjustNumericValue(int index) {
        float* value = menuItems[index].numeric.value;
        float min = menuItems[index].numeric.min;
        float max = menuItems[index].numeric.max;
        float step = menuItems[index].numeric.step;
        
        // Show adjustment screen
        bool adjusting = true;
        float initialValue = *value;
        
        while (adjusting) {
            Adafruit_SSD1306* display = DisplaySystem::getDisplay();
            display->clearDisplay();
            display->setTextSize(1);
            display->setTextColor(WHITE);
            
            // Title
            display->setCursor(0, 0);
            display->println(menuItems[index].text);
            display->drawLine(0, 9, display->width() - 1, 9, WHITE);
            
            // Value display
            display->setTextSize(2);
            display->setCursor(20, 20);
            display->print(*value, 1);
            
            // Arrows and range
            display->setTextSize(1);
            display->setCursor(0, 40);
            display->print("Min: ");
            display->print(min, 1);
            display->setCursor(70, 40);
            display->print("Max: ");
            display->print(max, 1);
            
            // Controls hint
            display->setCursor(0, 55);
            display->print("Y: Adj | L: OK | R: Cancel");
            
            display->display();
            
            // Input handling
            float rightY = JoystickSystem::getRightJoystick()->getNormalizedY();
            bool leftPressed = JoystickSystem::getLeftJoystick()->isPressed();
            bool rightPressed = JoystickSystem::getRightJoystick()->isPressed();
            
            static bool lastLeftPressed = false;
            static bool lastRightPressed = false;
            static unsigned long lastValueChange = 0;

            
            // Change value with joystick
            if (millis() - lastValueChange > 150) {
                if (rightY > 0.5f) {
                    *value = std::min(*value + step, max);  // Remove the equals sign after min
                    lastValueChange = millis();
                } else if (rightY < -0.5f) {
                    *value = std::max(*value - step, min);  // Remove the equals sign after max
                    lastValueChange = millis();
                }
            }

            
            // Confirm or cancel
            if (leftPressed && !lastLeftPressed) {
                adjusting = false;  // Accept changes
            }
            
            if (rightPressed && !lastRightPressed) {
                *value = initialValue;  // Revert changes
                adjusting = false;
            }
            
            lastLeftPressed = leftPressed;
            lastRightPressed = rightPressed;
            
            delay(10);
        }
    }
    
    // Adjust a selection value
    void adjustSelectionValue(int index) {
        int* value = menuItems[index].selection.value;
        const char** options = menuItems[index].selection.options;
        int optionCount = menuItems[index].selection.optionCount;
        
        if (!value || !options || optionCount <= 0) {
            return;
        }
        
        // Ensure valid value
        *value = constrain(*value, 0, optionCount - 1);
        
        // Show adjustment screen
        bool adjusting = true;
        int initialValue = *value;
        
        while (adjusting) {
            Adafruit_SSD1306* display = DisplaySystem::getDisplay();
            display->clearDisplay();
            display->setTextSize(1);
            display->setTextColor(WHITE);
            
            // Title
            display->setCursor(0, 0);
            display->println(menuItems[index].text);
            display->drawLine(0, 9, display->width() - 1, 9, WHITE);
            
            // Current selection
            display->setTextSize(1);
            display->setCursor(0, 15);
            display->print("Option: ");
            
            display->setTextSize(1);
            display->setCursor(10, 25);
            display->print(options[*value]);
            
            // Controls hint
            display->setCursor(0, 55);
            display->print("Y: Change | L: OK | R: Cancel");
            
            display->display();
            
            // Input handling
            float rightY = JoystickSystem::getRightJoystick()->getNormalizedY();
            bool leftPressed = JoystickSystem::getLeftJoystick()->isPressed();
            bool rightPressed = JoystickSystem::getRightJoystick()->isPressed();
            
            static bool lastLeftPressed = false;
            static bool lastRightPressed = false;
            static unsigned long lastValueChange = 0;
            
            // Change value with joystick
            if (millis() - lastValueChange > 200) {
                if (rightY > 0.5f) {
                    *value = (*value + 1) % optionCount;
                    lastValueChange = millis();
                } else if (rightY < -0.5f) {
                    *value = (*value - 1 + optionCount) % optionCount;
                    lastValueChange = millis();
                }
            }
            
            // Confirm or cancel
            if (leftPressed && !lastLeftPressed) {
                adjusting = false;  // Accept changes
            }
            
            if (rightPressed && !lastRightPressed) {
                *value = initialValue;  // Revert changes
                adjusting = false;
            }
            
            lastLeftPressed = leftPressed;
            lastRightPressed = rightPressed;
            
            delay(10);
        }
    }
    
    // Execute menu action and handle errors
    bool executeMenuAction(void (*actionFunc)(void)) {
        if (!actionFunc) {
            return false;
        }
        
        // Execute action within try-catch-like error checking
        bool success = true;
        
        // Indicate that action is being executed
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(WHITE);
        display->setCursor(20, 20);
        display->print("Executing...");
        display->display();
        
        // Execute action
        noInterrupts(); // Critical section
        actionFunc();
        interrupts();
        
        return success;
    }
    
    // Load a specific submenu
    void loadSubmenu(int submenuIndex) {
        menuItemCount = 0;
        
        switch (submenuIndex) {
            case 1: // System-Menü
                buildSystemMenu();
                break;
                
            case 2: // Homing-Menü
                buildHomePositionMenu();
                break;
                
            case 3: // Kalibrierungsmenü
                buildMotorTestMenu();
                break;
                
            case 4: // Gear-Menü
                buildGearRatioMenu();
                break;
                
            case 5: // SD-Karten-Menü
                buildSDCardMenu();
                break;
                
            case 6: // Limit-Switch-Menü
                buildLimitSwitchMenu();
                break;
                
            default:
                // Standardmenüpunkte
                addBackItem("Zurück zum Hauptmenü");
                break;
        }
    }
    
    // Menu builders
    void buildMainMenu() {
        menuItemCount = 0;
        
        addSubmenuItem("System", 1);
        addSubmenuItem("Home Position", 2);
        addSubmenuItem("Motor Test", 3);
        addSubmenuItem("Gear Ratios", 4);
        addSubmenuItem("SD Card", 5);
        addSubmenuItem("Limit Switches", 6);
        addActionItem("Start Homing", startHoming);
        addActionItem("Move to Center", moveToCenter);
    }
    
    void buildSystemMenu() {
        menuItemCount = 0;
        
        addActionItem("System Info", showSystemInfo);
        addActionItem("Joystick Calib.", startJoystickCalibration);
        addActionItem("Run Diagnostics", runDiagnostics);
        addActionItem("Restart System", restartSystem);
        addActionItem("Debug Info", showDebugInfo);
        addActionItem("Factory Reset", factoryReset);
        addBackItem();
    }
    
    void buildHomePositionMenu() {
        menuItemCount = 0;
        
        addActionItem("Current Home", showCurrentHomePosition);
        addActionItem("Save Home", saveHome);
        addActionItem("Load Home", loadHome);
        addActionItem("Clear Home", clearHome);
        addActionItem("Move To Home", moveToHome);
        addBackItem();
    }
    
    void buildMotorTestMenu() {
        menuItemCount = 0;
        
        addSelectionItem("Select Motor", &selectedMotor, motorOptions, 6);
        addNumericItem("Test Speed", &testSpeed, 0.1f, 1.0f, 0.1f);
        addActionItem("Move Forward", moveMotorForward);
        addActionItem("Move Backward", moveMotorBackward);
        addActionItem("Stop", stopMotor);
        addActionItem("Test All Motors", testAllMotors);
        addBackItem();
    }
    
    void buildLimitSwitchMenu() {
        menuItemCount = 0;
        
        addToggleItem("Invert Switches", &invertLimitSwitches);
        addActionItem("Check All Switches", checkAllLimitSwitches);
        addBackItem();
    }
    
    void buildGearRatioMenu() {
        menuItemCount = 0;
        
        addActionItem("Motor 1 (Base)", configureGearRatio1);
        addActionItem("Motor 2 (Shoulder)", configureGearRatio2);
        addActionItem("Motor 3 (Elbow)", configureGearRatio3);
        addActionItem("Motor 4 (Wrist P)", configureGearRatio4);
        addActionItem("Motor 5 (Wrist R)", configureGearRatio5);
        addActionItem("Motor 6 (Gripper)", configureGearRatio6);
        addActionItem("Save All", saveAllGearRatios);
        addActionItem("Load All", loadAllGearRatios);
        addBackItem();
    }
    
    void buildSDCardMenu() {
        menuItemCount = 0;
        
        addActionItem("List Files", listSDFiles);
        addActionItem("Load Program", loadProgramFromSD);
        addActionItem("Save Program", saveProgramToSD);
        addActionItem("Load Settings", loadSettingsFromSD);
                addActionItem("Save Settings", saveSettingsToSD);
        addBackItem();
    }
    
    // Action Functions
    void showSystemInfo() {
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(WHITE);
        
        display->setCursor(0, 0);
        display->println("System Information");
        display->drawLine(0, 9, display->width() - 1, 9, WHITE);
        
        display->setCursor(0, 12);
        display->print("Robot Control v1.0");
        
        display->setCursor(0, 22);
        display->print("Free Memory: ");
        display->print(getFreeMemory());
        display->print(" bytes");
        
        display->setCursor(0, 32);
        display->print("Uptime: ");
        unsigned long uptime = millis() / 1000;
        display->print(uptime / 3600);
        display->print("h ");
        display->print((uptime % 3600) / 60);
        display->print("m ");
        display->print(uptime % 60);
        display->print("s");
        
        display->setCursor(0, 55);
        display->print("Press any button to exit");
        display->display();
        
        // Wait for any button press
        bool exitScreen = false;
        while (!exitScreen) {
            if (JoystickSystem::getLeftJoystick()->isPressed() || 
                JoystickSystem::getRightJoystick()->isPressed()) {
                exitScreen = true;
            }
            delay(50);
        }
        
        // Debounce
        delay(200);
    }
    
    void factoryReset() {
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(WHITE);
        
        display->setCursor(15, 10);
        display->println("!! WARNING !!");
        display->setCursor(0, 25);
        display->println("Factory Reset will erase");
        display->setCursor(0, 35);
        display->println("all settings. Continue?");
        
        display->setCursor(0, 55);
        display->print("Left: YES | Right: NO");
        display->display();
        
        // Wait for button press
        bool confirmed = false;
        bool cancelled = false;
        while (!confirmed && !cancelled) {
            if (JoystickSystem::getLeftJoystick()->isPressed()) {
                confirmed = true;
            }
            if (JoystickSystem::getRightJoystick()->isPressed()) {
                cancelled = true;
            }
            delay(50);
        }
        
        if (confirmed) {
            display->clearDisplay();
            display->setCursor(0, 20);
            display->println("Resetting to defaults...");
            display->display();
            
            // Reset EEPROM data
            for (int i = 0; i < 1024; i++) {
                EEPROM.write(i, 0xFF);
            }
            
            // Reset gear ratios
            for (int i = 0; i < 6; i++) {
                gearRatios[i] = 20.0f;
            }
            
            // Reset all other settings to defaults
            // ...
            
            delay(1000);
            display->clearDisplay();
            display->setCursor(0, 20);
            display->println("Factory reset complete.");
            display->setCursor(0, 35);
            display->println("System will restart.");
            display->display();
            delay(2000);
            
            // Restart system
            
        }
        
        // Debounce
        delay(200);
    }
    
    void startCalibration() {
        calibrationMode = true;
        calibrationStep = 0;
        
        DisplaySystem::showMessage("Calibration", "Starting...", 1000);
    }
    
    void processCalibrationStep() {
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        
        switch (calibrationStep) {
            case 0: // Introduction
                display->clearDisplay();
                display->setTextSize(1);
                display->setTextColor(WHITE);
                display->setCursor(0, 0);
                display->println("Joystick Calibration");
                display->drawLine(0, 9, display->width() - 1, 9, WHITE);
                
                display->setCursor(0, 15);
                display->println("Move joysticks through");
                display->setCursor(0, 25);
                display->println("full range of motion.");
                
                display->setCursor(0, 40);
                display->println("Press LEFT button to start");
                display->setCursor(0, 50);
                display->println("Press RIGHT to cancel");
                display->display();
                
                if (JoystickSystem::getLeftJoystick()->isPressed()) {
                    calibrationStep = 1;
                    delay(300); // Debounce
                }
                
                if (JoystickSystem::getRightJoystick()->isPressed()) {
                    calibrationMode = false;
                    DisplaySystem::showMessage("Calibration", "Cancelled", 1000);
                    delay(300); // Debounce
                }
                break;
                
            case 1: // Calibrating Left Joystick
                JoystickSystem::getLeftJoystick()->calibrate();
                calibrationStep = 2;
                break;
                
            case 2: // Calibrating Right Joystick
                JoystickSystem::getRightJoystick()->calibrate();
                calibrationStep = 3;
                break;
                
            case 3: // Completed
                DisplaySystem::showMessage("Calibration", "Complete!", 1000);
                calibrationMode = false;
                break;
        }
    }
    
    void startHoming() {
        RobotSystem::setState(STATE_HOMING_MODE);
        RobotSystem::setHomingStarted(true);
        RobotSystem::setHomingJointIndex(0);
    }
    
    void moveToCenter() {
        RobotSystem::setHomingStarted(false);
        RobotSystem::moveToCenter();
    }
    
    void saveHome() {
        JointAngles current = RobotSystem::getKinematics()->getCurrentJointAngles();
        RobotSystem::saveRobotHome(current);
        DisplaySystem::showMessage("Home Saved!", nullptr, 1200);
    }
    
    void loadHome() {
        JointAngles homeAngles;
        if (RobotSystem::loadRobotHome(homeAngles)) {
            // No movement, just synchronize kinematics and stepper positions
            RobotSystem::getKinematics()->setCurrentJointAngles(homeAngles);
            for (int i = 0; i < 6; ++i) {
                StepperSystem::steppers[i]->setCurrentPosition(
                    homeAngles.angles[i] * 180.0 / M_PI * StepperSystem::enhancedSteppers[i].stepsPerDegree
                );
            }
            Debug::println(F("Home loaded! Kinematics synchronized, no movement."));
            DisplaySystem::showMessage("Home Loaded!", "No movement", 1200);
        } else {
            DisplaySystem::showMessage("No home", "saved!", 1200);
        }
    }
    
    void clearHome() {
        // Confirmation dialog
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(WHITE);
        
        display->setCursor(0, 10);
        display->println("Are you sure you want to");
        display->setCursor(0, 20);
        display->println("clear home position?");
        
        display->setCursor(0, 55);
        display->print("Left: YES | Right: NO");
        display->display();
        
        // Wait for button press
        bool confirmed = false;
        bool cancelled = false;
        while (!confirmed && !cancelled) {
            if (JoystickSystem::getLeftJoystick()->isPressed()) {
                confirmed = true;
            }
            if (JoystickSystem::getRightJoystick()->isPressed()) {
                cancelled = true;
            }
            delay(50);
        }
        
        if (confirmed) {
            RobotSystem::clearRobotHome();
            DisplaySystem::showMessage("Home cleared!", nullptr, 1200);
        }
        
        // Debounce
        delay(200);
    }
    
    void startJoystickCalibration() {
        startCalibration();
    }
    
    void testMotors() {
        buildMotorTestMenu();
    }
    
    void testLimitSwitches() {
        buildLimitSwitchMenu();
    }
    
    void saveGearRatios() {
        // Save gear ratios to EEPROM
        for (int i = 0; i < 6; i++) {
            EEPROM.put(200 + (i * sizeof(float)), gearRatios[i]);
        }
        
        // Save gear ratios to stepper config
        for (int i = 0; i < 6; i++) {
            extern StepperConfig _stepperConfig[6];
            _stepperConfig[i].stepsPerDegree = gearRatios[i];
        }
        
        DisplaySystem::showMessage("Gear Ratios", "Saved", 1000);
    }
    
    void loadGearRatios() {
        // Check if values in EEPROM are valid
        float testValue;
        EEPROM.get(200, testValue);
        
        // Check if testValue is a valid gear ratio (should be between 1 and 100)
        if (testValue >= 1.0f && testValue <= 100.0f) {
            // Load gear ratios from EEPROM
            for (int i = 0; i < 6; i++) {
                EEPROM.get(200 + (i * sizeof(float)), gearRatios[i]);
            }
            
            // Load gear ratios to stepper config
            for (int i = 0; i < 6; i++) {
                extern StepperConfig _stepperConfig[6];
                _stepperConfig[i].stepsPerDegree = gearRatios[i];
            }
            
            DisplaySystem::showMessage("Gear Ratios", "Loaded", 1000);
        } else {
            DisplaySystem::showMessage("No Saved", "Gear Ratios", 1000);
        }
    }
    
    void showCurrentHomePosition() {
        JointAngles current = RobotSystem::getKinematics()->getCurrentJointAngles();
        
        // Display current position on the screen
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(WHITE);
        
        display->setCursor(0, 0);
        display->println("Current Home Position");
        display->drawLine(0, 9, display->width() - 1, 9, WHITE);
        
        for (int i = 0; i < 6; i++) {
            display->setCursor(0, 12 + i * 8);
            display->print("J");
            display->print(i+1);
            display->print(": ");
            display->print(current.angles[i] * 180.0 / M_PI, 1);
            display->print(" deg");
        }
        
        display->setCursor(0, 55);
        display->print("Press any button to exit");
        display->display();
        
        // Wait for any button press
        bool exitScreen = false;
        while (!exitScreen) {
            if (JoystickSystem::getLeftJoystick()->isPressed() || 
                JoystickSystem::getRightJoystick()->isPressed()) {
                exitScreen = true;
            }
            delay(50);
        }
        
        // Debounce
        delay(200);
    }
    
    void moveToHome() {
        JointAngles homeAngles;
        if (RobotSystem::loadRobotHome(homeAngles)) {
            // Confirmation dialog
            Adafruit_SSD1306* display = DisplaySystem::getDisplay();
            display->clearDisplay();
            display->setTextSize(1);
            display->setTextColor(WHITE);
            
            display->setCursor(0, 10);
            display->println("Move robot to home");
            display->setCursor(0, 20);
            display->println("position? Robot will");
            display->setCursor(0, 30);
            display->println("move to saved position!");
            
            display->setCursor(0, 55);
            display->print("Left: YES | Right: NO");
            display->display();
            
            // Wait for button press
            bool confirmed = false;
            bool cancelled = false;
            while (!confirmed && !cancelled) {
                if (JoystickSystem::getLeftJoystick()->isPressed()) {
                    confirmed = true;
                }
                if (JoystickSystem::getRightJoystick()->isPressed()) {
                    cancelled = true;
                }
                delay(50);
            }
            
            if (confirmed) {
                // Convert joint angles to Cartesian pose
                CartesianPose pose = RobotSystem::getKinematics()->forwardKinematics(homeAngles);
                
                // Move to pose with progress indication
                RobotSystem::moveToPose(pose, true);
                
                DisplaySystem::showMessage("Moved to Home", "Position", 1200);
            }
        } else {
            DisplaySystem::showMessage("No home", "saved!", 1200);
        }
        
        // Debounce
        delay(200);
    }
    
    void moveMotorForward() {
        if (selectedMotor >= 0 && selectedMotor < 6) {
            // Enable motor
            extern PinConfig _pinConfig;
            digitalWrite(_pinConfig.stepperPins[selectedMotor][2], LOW);
            
            // Set speed
            StepperSystem::moveJoint(selectedMotor, testSpeed);
            
            DisplaySystem::showMessage("Moving motor", "forward...");
            delay(1000);
            
            // Stop motor
            StepperSystem::moveJoint(selectedMotor, 0);
            
            // Disable motor
            digitalWrite(_pinConfig.stepperPins[selectedMotor][2], HIGH);
            
            DisplaySystem::showMessage("Motor stopped", nullptr, 500);
        }
    }
    
    void moveMotorBackward() {
        if (selectedMotor >= 0 && selectedMotor < 6) {
            // Enable motor
            extern PinConfig _pinConfig;
            digitalWrite(_pinConfig.stepperPins[selectedMotor][2], LOW);
            
            // Set speed
            StepperSystem::moveJoint(selectedMotor, -testSpeed);
            
            DisplaySystem::showMessage("Moving motor", "backward...");
                        delay(1000);
            
            // Stop motor
            StepperSystem::moveJoint(selectedMotor, 0);
            
            // Disable motor
            digitalWrite(_pinConfig.stepperPins[selectedMotor][2], HIGH);
            
            DisplaySystem::showMessage("Motor stopped", nullptr, 500);
        }
    }
    
    void stopMotor() {
        if (selectedMotor >= 0 && selectedMotor < 6) {
            // Stop motor
            StepperSystem::moveJoint(selectedMotor, 0);
            
            // Disable motor
            extern PinConfig _pinConfig;
            digitalWrite(_pinConfig.stepperPins[selectedMotor][2], HIGH);
            
            DisplaySystem::showMessage("Motor stopped", nullptr, 500);
        }
    }
    
    void testAllMotors() {
        // Confirmation dialog
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(WHITE);
        
        display->setCursor(0, 10);
        display->println("Test all motors?");
        display->setCursor(0, 20);
        display->println("Robot will move each");
        display->setCursor(0, 30);
        display->println("joint slowly.");
        
        display->setCursor(0, 55);
        display->print("Left: YES | Right: NO");
        display->display();
        
        // Wait for button press
        bool confirmed = false;
        bool cancelled = false;
        while (!confirmed && !cancelled) {
            if (JoystickSystem::getLeftJoystick()->isPressed()) {
                confirmed = true;
            }
            if (JoystickSystem::getRightJoystick()->isPressed()) {
                cancelled = true;
            }
            delay(50);
        }
        
        if (confirmed) {
            // Test each motor
            for (int i = 0; i < 6; i++) {
                // Enable motor
                extern PinConfig _pinConfig;
                digitalWrite(_pinConfig.stepperPins[i][2], LOW);
                
                display->clearDisplay();
                display->setTextSize(1);
                display->setTextColor(WHITE);
                display->setCursor(0, 10);
                display->print("Testing Joint ");
                display->print(i+1);
                display->setCursor(0, 25);
                display->print("Moving forward...");
                display->display();
                
                // Move forward
                StepperSystem::moveJoint(i, testSpeed * 0.5);
                delay(500);
                StepperSystem::moveJoint(i, 0);
                delay(200);
                
                display->setCursor(0, 35);
                display->print("Moving backward...");
                display->display();
                
                // Move backward
                StepperSystem::moveJoint(i, -testSpeed * 0.5);
                delay(500);
                StepperSystem::moveJoint(i, 0);
                
                // Disable motor
                digitalWrite(_pinConfig.stepperPins[i][2], HIGH);
                
                display->setCursor(0, 45);
                display->print("Joint ");
                display->print(i+1);
                display->print(" test complete.");
                display->display();
                delay(500);
            }
            
            DisplaySystem::showMessage("All motors", "tested", 1000);
        }
        
        // Debounce
        delay(200);
    }
    
    void checkAllLimitSwitches() {
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        bool exitTest = false;
        
        extern PinConfig _pinConfig;
        
        while (!exitTest) {
            display->clearDisplay();
            display->setTextSize(1);
            display->setTextColor(WHITE);
            
            display->setCursor(0, 0);
            display->println("Limit Switch Test");
            display->drawLine(0, 9, display->width() - 1, 9, WHITE);
            
            // Read and display all limit switch states
            for (int i = 0; i < 6; i++) {
                int limitSwitchPin = _pinConfig.stepperPins[i][3];
                int switchState = digitalRead(limitSwitchPin);
                
                // If switches are inverted, flip the state
                if (invertLimitSwitches) {
                    switchState = !switchState;
                }
                
                display->setCursor(0, 12 + i*8);
                display->print("J");
                display->print(i+1);
                display->print(": ");
                
                if (switchState == LOW) {
                    display->print("TRIGGERED");
                } else {
                    display->print("released");
                }
            }
            
            display->setCursor(0, 55);
            display->print("Press any button to exit");
            display->display();
            
            // Check for exit
            if (JoystickSystem::getLeftJoystick()->isPressed() || 
                JoystickSystem::getRightJoystick()->isPressed()) {
                exitTest = true;
                delay(200); // Debounce
            }
            
            delay(100);
        }
    }
    
    void runDiagnostics() {
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(WHITE);
        
        display->setCursor(0, 0);
        display->println("System Diagnostics");
        display->drawLine(0, 9, display->width() - 1, 9, WHITE);
        
        display->setCursor(0, 12);
        display->print("Testing Motors... ");
        display->display();
        
        bool motorsOk = testMotorsQuick();
        display->print(motorsOk ? "OK" : "FAIL");
        display->display();
        
        display->setCursor(0, 22);
        display->print("Testing Switches... ");
        display->display();
        
        bool switchesOk = testSwitchesQuick();
        display->print(switchesOk ? "OK" : "FAIL");
        display->display();
        
        display->setCursor(0, 32);
        display->print("Testing Joysticks... ");
        display->display();
        
        bool joysticksOk = testJoysticksQuick();
        display->print(joysticksOk ? "OK" : "FAIL");
        display->display();
        
        display->setCursor(0, 42);
        display->print("Memory: ");
        display->print(getFreeMemory());
        display->print(" bytes free");
        display->display();
        
        display->setCursor(0, 55);
        display->print("Press any button to exit");
        display->display();
        
        // Wait for any button press
        bool exitScreen = false;
        while (!exitScreen) {
            if (JoystickSystem::getLeftJoystick()->isPressed() || 
                JoystickSystem::getRightJoystick()->isPressed()) {
                exitScreen = true;
            }
            delay(50);
        }
        
        // Debounce
        delay(200);
    }
    
    void restartSystem() {
        // Confirmation dialog
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(WHITE);
        
        display->setCursor(0, 10);
        display->println("Restart System?");
        display->setCursor(0, 20);
        display->println("All unsaved settings");
        display->setCursor(0, 30);
        display->println("will be lost.");
        
        display->setCursor(0, 55);
        display->print("Left: YES | Right: NO");
        display->display();
        
        // Wait for button press
        bool confirmed = false;
        bool cancelled = false;
        while (!confirmed && !cancelled) {
            if (JoystickSystem::getLeftJoystick()->isPressed()) {
                confirmed = true;
            }
            if (JoystickSystem::getRightJoystick()->isPressed()) {
                cancelled = true;
            }
            delay(50);
        }
        
        if (confirmed) {
            display->clearDisplay();
            display->setCursor(0, 20);
            display->println("Restarting system...");
            display->display();
            delay(1000);
            
            // Reset the Arduino
            
        }
        
        // Debounce
        delay(200);
    }
    
    void showDebugInfo() {
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        bool exitDebug = false;
        
        while (!exitDebug) {
            // Get the current joint angles and positions
            JointAngles angles = RobotSystem::getKinematics()->getCurrentJointAngles();
            CartesianPose pose = RobotSystem::getKinematics()->getCurrentPose();
            
            display->clearDisplay();
            display->setTextSize(1);
            display->setTextColor(WHITE);
            
            display->setCursor(0, 0);
            display->println("Debug Information");
            display->drawLine(0, 9, display->width() - 1, 9, WHITE);
            
            // Show position data
            display->setCursor(0, 12);
            display->print("X:");
            display->print(pose.x, 1);
            display->print(" Y:");
            display->print(pose.y, 1);
            display->print(" Z:");
            display->print(pose.z, 1);
            
            // Show joint data (first 3 joints)
            display->setCursor(0, 22);
            for (int i = 0; i < 3; i++) {
                display->print("J");
                display->print(i+1);
                display->print(":");
                display->print(angles.angles[i] * 180.0 / M_PI, 0);
                display->print(" ");
            }
            
            // Show joint data (last 3 joints)
            display->setCursor(0, 32);
            for (int i = 3; i < 6; i++) {
                display->print("J");
                display->print(i+1);
                display->print(":");
                display->print(angles.angles[i] * 180.0 / M_PI, 0);
                display->print(" ");
            }
            
            // System info
            display->setCursor(0, 42);
            display->print("Mem:");
            display->print(getFreeMemory());
            display->print(" State:");
            display->print((int)RobotSystem::getState());
            
            display->setCursor(0, 55);
            display->print("Press any button to exit");
            display->display();
            
            // Check for exit
            if (JoystickSystem::getLeftJoystick()->isPressed() || 
                JoystickSystem::getRightJoystick()->isPressed()) {
                exitDebug = true;
                delay(200); // Debounce
            }
            
            delay(100);
        }
    }
    
    void listSDFiles() {
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(WHITE);
        
        display->setCursor(0, 0);
        display->println("SD Card Files");
        display->drawLine(0, 9, display->width() - 1, 9, WHITE);
        display->display();
        
        if (!SD.begin()) {
            display->setCursor(0, 20);
            display->println("SD card init failed!");
            display->setCursor(0, 55);
            display->print("Press any button to exit");
            display->display();
            
            // Wait for any button press
            bool exitScreen = false;
            while (!exitScreen) {
                if (JoystickSystem::getLeftJoystick()->isPressed() || 
                    JoystickSystem::getRightJoystick()->isPressed()) {
                    exitScreen = true;
                }
                delay(50);
            }
            return;
        }
        
        // Open root directory
        File root = SD.open("/");
        if (!root || !root.isDirectory()) {
            display->setCursor(0, 20);
            display->println("Failed to open root!");
            display->display();
            delay(2000);
            return;
        }
        
        // List files
        int fileCount = 0;
        int page = 0;
        int filesPerPage = 5;
        File entry;
        
        // Count files
        while (entry = root.openNextFile()) {
            fileCount++;
            entry.close();
        }
        
        // Reset directory
        root.close();
        root = SD.open("/");
        
        bool exitListing = false;
        
        while (!exitListing) {
            display->clearDisplay();
            display->setTextSize(1);
            display->setTextColor(WHITE);
            
            display->setCursor(0, 0);
            display->print("SD Files (");
            display->print(fileCount);
            display->print(") - Page ");
            display->print(page + 1);
            display->drawLine(0, 9, display->width() - 1, 9, WHITE);
            
            // Skip files for previous pages
            root.close();
            root = SD.open("/");
            
            for (int i = 0; i < page * filesPerPage; i++) {
                entry = root.openNextFile();
                if (!entry) break;
                entry.close();
            }
            
            // Show files for current page
            int filesOnPage = 0;
            for (int i = 0; i < filesPerPage; i++) {
                entry = root.openNextFile();
                if (!entry) break;
                
                // Display file name
                display->setCursor(0, 12 + i * 8);
                String fileName = entry.name();
                if (fileName.length() > 20) {
                    fileName = fileName.substring(0, 17) + "...";
                }
                display->print(fileName);
                
                entry.close();
                filesOnPage++;
            }
            
            // Navigation help
            display->setCursor(0, 55);
            display->print("L:Exit | R:");
            if (fileCount > filesPerPage && page < (fileCount / filesPerPage)) {
                display->print("Next Page");
            } else {
                display->print("First Page");
            }
            
            display->display();
            
            // Wait for button press
            bool buttonPressed = false;
            while (!buttonPressed) {
                if (JoystickSystem::getLeftJoystick()->isPressed()) {
                    exitListing = true;
                    buttonPressed = true;
                    delay(200); // Debounce
                }
                
                if (JoystickSystem::getRightJoystick()->isPressed()) {
                    if (fileCount > filesPerPage && page < (fileCount / filesPerPage)) {
                        page++;
                    } else {
                        page = 0;
                    }
                    buttonPressed = true;
                    delay(200); // Debounce
                }
                                delay(50);
            }
        }
        
        root.close();
    }
    
void loadProgramFromSD() {
    String fileName = getFileName("Load Program");
    
    if (fileName.length() == 0) {
        return; // User cancelled
    }
    
    Adafruit_SSD1306* display = DisplaySystem::getDisplay();
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(WHITE);
    
    display->setCursor(0, 0);
    display->println("Loading Program");
    display->drawLine(0, 9, display->width() - 1, 9, WHITE);
    
    display->setCursor(0, 15);
    display->print("File: ");
    display->println(fileName);
    display->display();
    
    // Initialize SD card
    if (!SD.begin()) {
        DisplaySystem::showMessage("SD Error", "Card init failed!", 1500);
        return;
    }
    
    // Check if file exists
    if (!SD.exists(fileName.c_str())) {
        DisplaySystem::showMessage("File Error", "File not found!", 1500);
        return;
    }
    
    // Open file for reading
    File programFile = SD.open(fileName.c_str(), FILE_READ);
    if (!programFile) {
        DisplaySystem::showMessage("File Error", "Cannot open file!", 1500);
        return;
    }
    
    // Read magic number to verify file type
    uint32_t magicNumber;
    programFile.read((uint8_t*)&magicNumber, sizeof(magicNumber));
    
    if (magicNumber != 0x50524F47) { // "PROG" in hex
        programFile.close();
        DisplaySystem::showMessage("File Error", "Invalid format!", 1500);
        return;
    }
    
    // Read program point count
    uint16_t pointCount;
    programFile.read((uint8_t*)&pointCount, sizeof(pointCount));
    
    // Read all program points
    bool success = true;
    
    // First clear any existing program
    RobotSystem::clearProgram();
    
    // Progress display
    display->clearDisplay();
    display->setCursor(0, 0);
    display->println("Loading Program");
    display->drawLine(0, 9, display->width() - 1, 9, WHITE);
    
    display->setCursor(0, 20);
    display->print("Reading ");
    display->print(pointCount);
    display->println(" points...");
    display->display();
    
    // Read each program point
    for (int i = 0; i < pointCount; i++) {
        CartesianPose pose;
        programFile.read((uint8_t*)&pose, sizeof(CartesianPose));
        
        uint32_t delayMs;
        programFile.read((uint8_t*)&delayMs, sizeof(delayMs));
        
        // Add to program
        RobotSystem::addProgramPoint(pose, delayMs);
        
        // Show progress periodically
        if (i % 5 == 0 || i == pointCount - 1) {
            display->clearDisplay();
            display->setCursor(0, 0);
            display->println("Loading Program");
            display->drawLine(0, 9, display->width() - 1, 9, WHITE);
            
            display->setCursor(0, 20);
            display->print("Progress: ");
            display->print(i + 1);
            display->print("/");
            display->println(pointCount);
            
            // Progress bar
            int barWidth = display->width() - 20;
            int progress = map(i + 1, 0, pointCount, 0, barWidth);
            display->drawRect(10, 35, barWidth, 10, WHITE);
            display->fillRect(10, 35, progress, 10, WHITE);
            
            display->display();
        }
    }
    
    programFile.close();
    
    if (success) {
        DisplaySystem::showMessage("Program Loaded", "Successfully!", 1500);
    } else {
        DisplaySystem::showMessage("Load Error", "Some points failed", 1500);
    }
}

void saveProgramToSD() {
    // Check if there's a program to save
    int programSize = RobotSystem::getProgramSize();
    if (programSize == 0) {
        DisplaySystem::showMessage("Error", "No program to save", 1500);
        return;
    }
    
    String fileName = getFileName("Save Program");
    
    if (fileName.length() == 0) {
        return; // User cancelled
    }
    
    Adafruit_SSD1306* display = DisplaySystem::getDisplay();
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(WHITE);
    
    display->setCursor(0, 0);
    display->println("Saving Program");
    display->drawLine(0, 9, display->width() - 1, 9, WHITE);
    
    display->setCursor(0, 15);
    display->print("File: ");
    display->println(fileName);
    display->display();
    
    // Initialize SD card
    if (!SD.begin()) {
        DisplaySystem::showMessage("SD Error", "Card init failed!", 1500);
        return;
    }
    
    // Check if file exists and confirm overwrite if it does
    if (SD.exists(fileName.c_str())) {
        display->clearDisplay();
        display->setCursor(0, 10);
        display->println("File exists!");
        display->setCursor(0, 25);
        display->println("Overwrite existing file?");
        display->setCursor(0, 55);
        display->print("L: Yes | R: No");
        display->display();
        
        // Wait for user confirmation
        bool confirmed = false;
        bool cancelled = false;
        
        while (!confirmed && !cancelled) {
            if (JoystickSystem::getLeftJoystick()->isPressed()) {
                confirmed = true;
                delay(200); // Debounce
            }
            if (JoystickSystem::getRightJoystick()->isPressed()) {
                cancelled = true;
                delay(200); // Debounce
            }
            delay(50);
        }
        
        if (cancelled) {
            DisplaySystem::showMessage("Save Cancelled", "", 1000);
            return;
        }
        
        // Remove existing file
        SD.remove(fileName.c_str());
    }
    
    // Open file for writing
    File programFile = SD.open(fileName.c_str(), FILE_WRITE);
    if (!programFile) {
        DisplaySystem::showMessage("File Error", "Cannot create file!", 1500);
        return;
    }
    
    // Write magic number to identify file type
    uint32_t magicNumber = 0x50524F47; // "PROG" in hex
    programFile.write((uint8_t*)&magicNumber, sizeof(magicNumber));
    
    // Write program point count
    uint16_t pointCount = programSize;
    programFile.write((uint8_t*)&pointCount, sizeof(pointCount));
    
    // Progress display
    display->clearDisplay();
    display->setCursor(0, 0);
    display->println("Saving Program");
    display->drawLine(0, 9, display->width() - 1, 9, WHITE);
    
    display->setCursor(0, 20);
    display->print("Saving ");
    display->print(pointCount);
    display->println(" points...");
    display->display();
    
    // Write each program point
    bool success = true;
    for (int i = 0; i < pointCount; i++) {
        // Get program point
        ProgramPoint point = RobotSystem::getProgramPoint(i);
        
        // Write position data
        programFile.write((uint8_t*)&point.position, sizeof(CartesianPose));
        
        // Write delay
        programFile.write((uint8_t*)&point.delayMs, sizeof(uint32_t));
        
        // Show progress periodically
        if (i % 5 == 0 || i == pointCount - 1) {
            display->clearDisplay();
            display->setCursor(0, 0);
            display->println("Saving Program");
            display->drawLine(0, 9, display->width() - 1, 9, WHITE);
            
            display->setCursor(0, 20);
            display->print("Progress: ");
            display->print(i + 1);
            display->print("/");
            display->println(pointCount);
            
            // Progress bar
            int barWidth = display->width() - 20;
            int progress = map(i + 1, 0, pointCount, 0, barWidth);
            display->drawRect(10, 35, barWidth, 10, WHITE);
            display->fillRect(10, 35, progress, 10, WHITE);
            
            display->display();
        }
    }
    
    programFile.close();
    
    if (success) {
        DisplaySystem::showMessage("Program Saved", "Successfully!", 1500);
    } else {
        DisplaySystem::showMessage("Save Error", "Some points failed", 1500);
    }
}

void loadSettingsFromSD() {
    String fileName = getFileName("Load Settings");
    
    if (fileName.length() == 0) {
        return; // User cancelled
    }
    
    Adafruit_SSD1306* display = DisplaySystem::getDisplay();
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(WHITE);
    
    display->setCursor(0, 0);
    display->println("Loading Settings");
    display->drawLine(0, 9, display->width() - 1, 9, WHITE);
    
    display->setCursor(0, 15);
    display->print("File: ");
    display->println(fileName);
    display->display();
    
    // Initialize SD card
    if (!SD.begin()) {
        DisplaySystem::showMessage("SD Error", "Card init failed!", 1500);
        return;
    }
    
    // Check if file exists
    if (!SD.exists(fileName.c_str())) {
        DisplaySystem::showMessage("File Error", "File not found!", 1500);
        return;
    }
    
    // Open file for reading
    File settingsFile = SD.open(fileName.c_str(), FILE_READ);
    if (!settingsFile) {
        DisplaySystem::showMessage("File Error", "Cannot open file!", 1500);
        return;
    }
    
    // Read magic number to verify file type
    uint32_t magicNumber;
    settingsFile.read((uint8_t*)&magicNumber, sizeof(magicNumber));
    
    if (magicNumber != 0x53455454) { // "SETT" in hex
        settingsFile.close();
        DisplaySystem::showMessage("File Error", "Invalid format!", 1500);
        return;
    }
    
    // Read version
    uint16_t version;
    settingsFile.read((uint8_t*)&version, sizeof(version));
    
    // Different handling based on version
    if (version > 1) {
        DisplaySystem::showMessage("File Error", "Unsupported version", 1500);
        settingsFile.close();
        return;
    }
    
    display->clearDisplay();
    display->setCursor(0, 0);
    display->println("Loading Settings");
    display->drawLine(0, 9, display->width() - 1, 9, WHITE);
    display->setCursor(0, 20);
    display->println("Reading settings...");
    display->display();
    
    // Read gear ratios
    settingsFile.read((uint8_t*)gearRatios, static_cast<int>(sizeof(float) * 6));
    
    // Update stepper config with loaded gear ratios
    extern StepperConfig _stepperConfig[6];
    for (int i = 0; i < 6; i++) {
        _stepperConfig[i].stepsPerDegree = gearRatios[i];
    }
    
    // Read joystick calibration if present
if (settingsFile.available() >= static_cast<int>(sizeof(float) * 8)) {
    float leftMin[2], leftMax[2], rightMin[2], rightMax[2];
    
    settingsFile.read((uint8_t*)leftMin, static_cast<int>(sizeof(float) * 2));
    settingsFile.read((uint8_t*)leftMax, static_cast<int>(sizeof(float) * 2));
    settingsFile.read((uint8_t*)rightMin, static_cast<int>(sizeof(float) * 2));
    settingsFile.read((uint8_t*)rightMax, static_cast<int>(sizeof(float) * 2));
    
    // Apply joystick calibration
    JoystickSystem::setCalibrationData(leftMin, leftMax, rightMin, rightMax);
}

// Read robot configuration if present
if (settingsFile.available() >= static_cast<int>(sizeof(RobotConfig))) {
    RobotConfig config;
    settingsFile.read((uint8_t*)&config, static_cast<int>(sizeof(RobotConfig)));
    
    // Apply robot configuration
    RobotSystem::setRobotConfig(config);


    }
    
    settingsFile.close();
    DisplaySystem::showMessage("Settings Loaded", "Successfully!", 1500);
}

void saveSettingsToSD() {
    String fileName = getFileName("Save Settings");
    
    if (fileName.length() == 0) {
        return; // User cancelled
    }
    
    Adafruit_SSD1306* display = DisplaySystem::getDisplay();
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(WHITE);
    
    display->setCursor(0, 0);
    display->println("Saving Settings");
    display->drawLine(0, 9, display->width() - 1, 9, WHITE);
    
    display->setCursor(0, 15);
    display->print("File: ");
    display->println(fileName);
    display->display();
    
    // Initialize SD card
    if (!SD.begin()) {
        DisplaySystem::showMessage("SD Error", "Card init failed!", 1500);
        return;
    }
    
    // Check if file exists and confirm overwrite if it does
    if (SD.exists(fileName.c_str())) {
        display->clearDisplay();
        display->setCursor(0, 10);
        display->println("File exists!");
        display->setCursor(0, 25);
        display->println("Overwrite existing file?");
        display->setCursor(0, 55);
        display->print("L: Yes | R: No");
        display->display();
        
        // Wait for user confirmation
        bool confirmed = false;
        bool cancelled = false;
        
        while (!confirmed && !cancelled) {
            if (JoystickSystem::getLeftJoystick()->isPressed()) {
                confirmed = true;
                delay(200); // Debounce
            }
            if (JoystickSystem::getRightJoystick()->isPressed()) {
                cancelled = true;
                delay(200); // Debounce
            }
            delay(50);
        }
        
        if (cancelled) {
            DisplaySystem::showMessage("Save Cancelled", "", 1000);
            return;
        }
        
        // Remove existing file
        SD.remove(fileName.c_str());
    }
    
    // Open file for writing
    File settingsFile = SD.open(fileName.c_str(), FILE_WRITE);
    if (!settingsFile) {
        DisplaySystem::showMessage("File Error", "Cannot create file!", 1500);
        return;
    }
    
    // Write magic number to identify file type
    uint32_t magicNumber = 0x53455454; // "SETT" in hex
    settingsFile.write((uint8_t*)&magicNumber, sizeof(magicNumber));
    
    // Write version
    uint16_t version = 1;
    settingsFile.write((uint8_t*)&version, sizeof(version));
    
    display->clearDisplay();
    display->setCursor(0, 0);
    display->println("Saving Settings");
    display->drawLine(0, 9, display->width() - 1, 9, WHITE);
    display->setCursor(0, 20);
    display->println("Writing settings...");
    display->display();
    
    // Write gear ratios
    settingsFile.write((uint8_t*)gearRatios, sizeof(float) * 6);
    
    // Get joystick calibration data
    float leftMin[2], leftMax[2], rightMin[2], rightMax[2];
    JoystickSystem::getCalibrationData(leftMin, leftMax, rightMin, rightMax);
    
    // Write joystick calibration
    settingsFile.write((uint8_t*)leftMin, sizeof(float) * 2);
    settingsFile.write((uint8_t*)leftMax, sizeof(float) * 2);
    settingsFile.write((uint8_t*)rightMin, sizeof(float) * 2);
    settingsFile.write((uint8_t*)rightMax, sizeof(float) * 2);
    
    // Get robot configuration
    RobotConfig config = RobotSystem::getRobotConfig();
    
    // Write robot configuration
    settingsFile.write((uint8_t*)&config, sizeof(RobotConfig));
    
    settingsFile.close();
    DisplaySystem::showMessage("Settings Saved", "Successfully!", 1500);
}

    
    // Diagnostic Helper Functions
    bool testMotorsQuick() {
        // Just check if all stepper motors are responding
        bool allOk = true;
        
        for (int i = 0; i < 6; i++) {
            allOk = allOk && (StepperSystem::steppers[i] != nullptr);
        }
        
        return allOk;
    }
    
    bool testSwitchesQuick() {
        // Read all limit switches to make sure they're connected
        bool allOk = true;
        extern PinConfig _pinConfig;
        
        for (int i = 0; i < 6; i++) {
            int limitSwitchPin = _pinConfig.stepperPins[i][3];

            if (limitSwitchPin <= 0) {
            allOk = false;
            continue;
        }
            pinMode(limitSwitchPin, INPUT_PULLUP);
            digitalRead(limitSwitchPin);
        }
        
        return allOk;
    }
    
    bool testJoysticksQuick() {
        // Check if joysticks are connected and reading values
        return (JoystickSystem::getLeftJoystick() != nullptr && 
                JoystickSystem::getRightJoystick() != nullptr);
    }
    
    bool testMemoryQuick() {
        // Just check if we have a reasonable amount of free memory
        return (getFreeMemory() > 512); // At least 512 bytes free
    }
    
    int getFreeMemory() {
        // This is a simple way to estimate free memory on AVR
        extern int __heap_start, *__brkval;
        int v;
        return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
    }
    
    // Gear ratio configuration helpers
    String getFileName(const char* prompt) {
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        String fileName = "DEFAULT.TXT";
        bool exitEdit = false;
        int cursorPos = 0;
        
        while (!exitEdit) {
            display->clearDisplay();
            display->setTextSize(1);
            display->setTextColor(WHITE);
            
            display->setCursor(0, 0);
            display->println(prompt);
            display->drawLine(0, 9, display->width() - 1, 9, WHITE);
            
            display->setCursor(0, 15);
            display->print("Filename: ");
            
            display->setCursor(0, 30);
            display->print("> ");
            display->print(fileName);
            
            // Show cursor
            if ((millis() / 500) % 2 == 0) {
                display->drawLine(10 + cursorPos * 6, 39, 16 + cursorPos * 6, 39, WHITE);
            }
            
            display->setCursor(0, 55);
            display->print("L:Save | R:Cancel | Y:Edit");
            display->display();
            
            // Check joystick input
            if (JoystickSystem::getLeftJoystick()->isPressed()) {
                exitEdit = true; // Confirm and exit
                delay(200); // Debounce
            }
            
            if (JoystickSystem::getRightJoystick()->isPressed()) {
                exitEdit = true;
                fileName = ""; // Cancel and return empty filename
                delay(200); // Debounce
            }
            
            // Implement cursor movement and editing here
            // This is a simplified version just returning a default filename
            
            delay(50);
        }
        
        return fileName;
    }
    
    void configureGearRatio(int motorIndex) {
        if (motorIndex < 0 || motorIndex >= 6) {
            return;
        }
        
        float ratio = gearRatios[motorIndex];
        float initialRatio = ratio;
        bool exitEdit = false;
        
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        
        while (!exitEdit) {
            display->clearDisplay();
            display->setTextSize(1);
            display->setTextColor(WHITE);
            
            display->setCursor(0, 0);
            display->print("Gear Ratio - Joint ");
            display->println(motorIndex + 1);
            display->drawLine(0, 9, display->width() - 1, 9, WHITE);
            
            display->setCursor(0, 15);
            display->print("Current: ");
            display->print(ratio, 2);
            display->print(" steps/deg");
            
            display->setCursor(0, 30);
            display->print("Use Y-axis to adjust");
            
            display->setCursor(0, 55);
            display->print("L:Save | R:Cancel");
            display->display();
            
            // Check input
            float rightY = JoystickSystem::getRightJoystick()->getNormalizedY();
            bool leftPressed = JoystickSystem::getLeftJoystick()->isPressed();
            bool rightPressed = JoystickSystem::getRightJoystick()->isPressed();
            
            static unsigned long lastAdjust = 0;
            if (millis() - lastAdjust > 150) {
                if (rightY > 0.5f) {
                    ratio += 0.1f;
                    lastAdjust = millis();
                } else if (rightY < -0.5f) {
                    ratio -= 0.1f;
                    if (ratio < 0.1f) ratio = 0.1f;
                    lastAdjust = millis();
                }
            }
            
            if (leftPressed) {
                gearRatios[motorIndex] = ratio;
                
                // Update stepper config
                extern StepperConfig _stepperConfig[6];
                _stepperConfig[motorIndex].stepsPerDegree = ratio;
                
                exitEdit = true;
                delay(200); // Debounce
            }
            
            if (rightPressed) {
                ratio = initialRatio;
                exitEdit = true;
                delay(200); // Debounce
            }
            
            delay(10);
        }
    }
    
    // Gear configuration actions
    void configureGearRatio1() { configureGearRatio(0); }
    void configureGearRatio2() { configureGearRatio(1); }
    void configureGearRatio3() { configureGearRatio(2); }
    void configureGearRatio4() { configureGearRatio(3); }
    void configureGearRatio5() { configureGearRatio(4); }
    void configureGearRatio6() { configureGearRatio(5); }
    
    void saveAllGearRatios() {
        // Save gear ratios to EEPROM
        for (int i = 0; i < 6; i++) {
            EEPROM.put(200 + (i * sizeof(float)), gearRatios[i]);
        }
        
        // Save gear ratios to stepper config
        for (int i = 0; i < 6; i++) {
            extern StepperConfig _stepperConfig[6];
            _stepperConfig[i].stepsPerDegree = gearRatios[i];
        }
        
        DisplaySystem::showMessage("All Gear Ratios", "Saved", 1000);
    }
    
    void loadAllGearRatios() {
        // Check if values in EEPROM are valid
        float testValue;
        EEPROM.get(200, testValue);
        
        // Check if testValue is a valid gear ratio (should be between 1 and 100)
        if (testValue >= 1.0f && testValue <= 100.0f) {
            // Load gear ratios from EEPROM
            for (int i = 0; i < 6; i++) {
                EEPROM.get(200 + (i * sizeof(float)), gearRatios[i]);
            }
            
            // Load gear ratios to stepper config
            for (int i = 0; i < 6; i++) {
                extern StepperConfig _stepperConfig[6];
                _stepperConfig[i].stepsPerDegree = gearRatios[i];
            }
            
            DisplaySystem::showMessage("All Gear Ratios", "Loaded", 1000);
        } else {
            DisplaySystem::showMessage("No Saved", "Gear Ratios", 1000);
        }
    }
    
} // namespace MenuSystem
