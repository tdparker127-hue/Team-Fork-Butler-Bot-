#ifndef JETSERIAL_LINK_H
#define JETSERIAL_LINK_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

struct JetMessage {
    float joystickLx;
    float joystickLy;
    float joystickRx;
    float joystickRy;
};

