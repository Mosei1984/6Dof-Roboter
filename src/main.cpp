#include <Arduino.h>
#include "Debug.h"
#include "TimerLoop.h"
#include "config.h"
#include "StepperSystem.h"
#include "JoystickSystem.h"
#include "DisplaySystem.h"
#include "RobotSystem.h"

// Function declarations for the TimerLoop callbacks
void controlCallback();
void joystickCallback();
void displayCallback();

// Robot arm bitmap for startup screen
extern const unsigned char robotArmBitmap[];

void setup() {
    Serial.begin(115200);
    Debug::enabled = true;
    Debug::println(F("6DOF Robot starting..."));
    
    // Load configurations
    loadDefaultPinConfig();
    loadDefaultJoystickConfig();
    loadDefaultStepperConfig();
    
    // Initialize subsystems
    StepperSystem::init();
    JoystickSystem::init();
    DisplaySystem::init();
    RobotSystem::init();
    
    // Display startup message
    DisplaySystem::displayStartupScreen();
    delay(1500);
    
    // Show robot bitmap at startup
    Adafruit_SSD1306* display = DisplaySystem::getDisplay();
    display->clearDisplay();
    display->drawBitmap(0, 0, robotArmBitmap, 128, 64, SSD1306_WHITE);
    display->display();
    delay(2000);
    
    // Start timer-based control system
    TimerLoop::begin(controlCallback);  // Takes exactly one function pointer
    
    Debug::println(F("System initialized, starting in Homing mode"));
    RobotSystem::setState(RobotSystem::STATE_HOMING_MODE);
    RobotSystem::setStateChangeTime(millis());
}

void loop() {
    // Joysticks zuerst auslesen
    JoystickSystem::update();  // Dies ruft auch processButtonInput auf
    
    // Dann TimerLoop und andere Verarbeitung
    TimerLoop::loop(joystickCallback, displayCallback);  // Takes exactly two function pointers
}

// Control callback - runs at 1kHz
void controlCallback() {
    // Update stepper motors at high frequency
    StepperSystem::update();
}

// Joystick callback - runs at 100Hz
void joystickCallback() {
    // NICHT nochmal JoystickSystem::update oder processButtonInput hier aufrufen!
    // Die Button-Verarbeitung findet bereits in JoystickSystem::update statt
    
    // Process current state logic - ohne Button handling
    RobotSystem::processCurrentState();
}

// Display callback - runs at 60Hz
void displayCallback() {
    // Update the display based on current state
    DisplaySystem::update();
}