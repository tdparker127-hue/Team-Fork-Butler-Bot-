#include <Arduino.h>
#include "gamepad_controller.h"
#include "wireless.h"
#include "robot_drive.h"

static ControllerPtr myGamepad = nullptr;
static unsigned long lastGamepadUpdate = 0;
static const unsigned long GAMEPAD_TIMEOUT_MS = 500;

static void onConnectedController(ControllerPtr ctl) {
    if (myGamepad == nullptr) {
        Serial.printf("Gamepad connected: model=%s\n", ctl->getModelName().c_str());
        myGamepad = ctl;
    } else {
        Serial.println("Gamepad already connected, ignoring new connection");
    }
}

static void onDisconnectedController(ControllerPtr ctl) {
    if (ctl == myGamepad) {
        Serial.println("Gamepad disconnected — stopping motors");
        myGamepad = nullptr;
        updateSetpoints(0, 0);
    }
}

void setupGamepad() {
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Only allow connections from the specific PS5 controller MAC address.
    // Set PS5_CONTROLLER_ADDR in gamepad_controller.h to your controller's MAC.
    const bd_addr_t addr = {
        PS5_CONTROLLER_ADDR[0], PS5_CONTROLLER_ADDR[1],
        PS5_CONTROLLER_ADDR[2], PS5_CONTROLLER_ADDR[3],
        PS5_CONTROLLER_ADDR[4], PS5_CONTROLLER_ADDR[5]
    };
    

    Serial.println("Bluepad32 ready — waiting for PS5 controller...");
}

bool isGamepadConnected() {
    return myGamepad != nullptr && myGamepad->isConnected();
}

void updateGamepad() {
    bool dataUpdated = BP32.update();
    if (!dataUpdated) return;

    if (!isGamepadConnected()) {
        // Safety timeout: if no gamepad for too long, ensure motors are stopped
        if (millis() - lastGamepadUpdate > GAMEPAD_TIMEOUT_MS) {
            updateSetpoints(0, 0);
        }
        return;
    }

    lastGamepadUpdate = millis();

    if (!myGamepad->hasData()) return;

    // Map Bluepad32 axes (-511..512) to normalized range (-1.0..1.0)
    controllerMessage.joystick1.x = myGamepad->axisX() / 512.0f;
    controllerMessage.joystick1.y = -(myGamepad->axisY() / 512.0f); // invert Y so up = positive
    controllerMessage.joystick2.x = myGamepad->axisRX() / 512.0f;
    controllerMessage.joystick2.y = -(myGamepad->axisRY() / 512.0f);

    // Map buttons
    controllerMessage.buttonL = myGamepad->l1();
    controllerMessage.buttonR = myGamepad->r1();

    // Map D-pad
    uint8_t dpad = myGamepad->dpad();
    controllerMessage.dPad.up = (dpad & DPAD_UP) != 0;
    controllerMessage.dPad.down = (dpad & DPAD_DOWN) != 0;
    controllerMessage.dPad.left = (dpad & DPAD_LEFT) != 0;
    controllerMessage.dPad.right = (dpad & DPAD_RIGHT) != 0;

    controllerMessage.millis = millis();
    freshWirelessData = true;

    // Debug print
    Serial.printf("GP L:(%.2f,%.2f) R:(%.2f,%.2f) L1:%d R1:%d\n",
                  controllerMessage.joystick1.x, controllerMessage.joystick1.y,
                  controllerMessage.joystick2.x, controllerMessage.joystick2.y,
                  controllerMessage.buttonL, controllerMessage.buttonR);
}
