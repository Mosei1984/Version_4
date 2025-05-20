#ifndef MENU_SYSTEM_H
#define MENU_SYSTEM_H

#include <Arduino.h>

// Maximale Anzahl von Menüpunkten und Verschachtelungstiefe
#define MAX_MENU_ITEMS 20
#define MAX_MENU_DEPTH 5
#define MENU_TIMEOUT_MS 60000 // 60 Sekunden Timeout für Menüs

// Menüpunkt-Typen
enum MenuItemType {
    MENU_ACTION,     // Führt eine Aktion aus
    MENU_SUBMENU,    // Öffnet ein Untermenü
    MENU_TOGGLE,     // Umschalter (Ein/Aus)
    MENU_NUMERIC,    // Numerischer Wert
    MENU_SELECTION,  // Auswahl aus einer Liste
    MENU_BACK        // Zurück zum übergeordneten Menü
};

// Struktur für einen Menüpunkt
struct MenuItem {
    const char* text;             // Anzeigetext
    MenuItemType type;            // Typ des Menüpunkts
    union {
        void (*actionFunc)(void); // Funktionszeiger für MENU_ACTION
        int submenuIndex;         // Index des Untermenüs für MENU_SUBMENU
        bool* toggleValue;        // Zeiger auf bool für MENU_TOGGLE
        struct {                  // Für MENU_NUMERIC
            float* value;
            float min;
            float max;
            float step;
        } numeric;
        struct {                  // Für MENU_SELECTION
            int* value;
            const char** options;
            int optionCount;
        } selection;
    };
};

// Struktur für Menü-History (für Rücknavigation)
struct MenuHistoryEntry {
    int menuIndex;
    int submenuIndex;
};

namespace MenuSystem {
    // Initialisierung
    void init();
    
    // Haupt-Update-Funktion
    void update();
    
    // Menü-Navigation
    void navigateUp();
    void navigateDown();
    void selectItem();
    void goBack();
    
    // Menü-Definition
    void defineMenu(const MenuItem* items, int count);
    void addMenuItem(const char* text, MenuItemType type);
    void addActionItem(const char* text, void (*actionFunc)(void));
    void addSubmenuItem(const char* text, int submenuIndex);
    void addToggleItem(const char* text, bool* value);
    void addNumericItem(const char* text, float* value, float min, float max, float step);
    void addSelectionItem(const char* text, int* value, const char** options, int optionCount);
    void addBackItem(const char* text = "Zurück");
    
    // Menü-Anzeige
    void displayMenu();
    void processMenuInput();
    
    // Untermenüs
    void enterSubmenu(int submenuIndex);
    void exitSubmenu();
    void loadSubmenu(int submenuIndex);
    
    // Menüpunkt-Bearbeitung
    void adjustNumericValue(int index);
    void adjustSelectionValue(int index);
    bool executeMenuAction(void (*actionFunc)(void));
    
    // Hilfsfunktionen zur Anzeige
    void displayItemExtras(int itemIndex, int x, int y);
    void drawScrollbar(int startItem, int visibleItems);
    
    // Menü-State-Management
    int getCurrentMenuIndex();
    int getCurrentMenuLevel();
    const MenuItem* getCurrentMenuItem();
    
    // Menü-Builder
    void buildMainMenu();
    void buildHomePositionMenu();
    void buildMotorTestMenu();
    void buildLimitSwitchMenu();
    void buildSystemMenu();
    void buildGearRatioMenu();
    void buildSDCardMenu();
    
    // Aktionsfunktionen
    void showSystemInfo();
    void factoryReset();
    void startCalibration();
    void startHoming();
    void moveToCenter();
    void saveHome();
    void loadHome();
    void clearHome();
    void startJoystickCalibration();
    void testMotors();
    void testLimitSwitches();
    void saveGearRatios();
    void loadGearRatios();
    void showCurrentHomePosition();
    void moveToHome();
    void moveMotorForward();
    void moveMotorBackward();
    void stopMotor();
    void testAllMotors();
    void checkAllLimitSwitches();
    void runDiagnostics();
    void restartSystem();
    void showDebugInfo();
    void listSDFiles();
    void loadProgramFromSD();
    void saveProgramToSD();
    void loadSettingsFromSD();
    void saveSettingsToSD();
    
    // Diagnose-Hilfsfunktionen
    bool testMotorsQuick();
    bool testSwitchesQuick();
    bool testJoysticksQuick();
    bool testMemoryQuick();
    int getFreeMemory();
    
    // Spezialfunktionen
    String getFileName(const char* prompt);
    void configureGearRatio(int motorIndex);
    void configureGearRatio1();
    void configureGearRatio2();
    void configureGearRatio3();
    void configureGearRatio4();
    void configureGearRatio5();
    void configureGearRatio6();
    void saveAllGearRatios();
    void loadAllGearRatios();
}

#endif // MENU_SYSTEM_H
