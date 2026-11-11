
#pragma once
#include <Arduino.h>

enum class ButtonEvent {
    NONE,
    SHORT_PRESS,
    DOUBLE_PRESS,
    LONG_PRESS
};

class ButtonHandler {
public:
    void begin();

    ButtonEvent update(
        uint32_t now,
        bool immediateShort = false
    );

private:
    bool rawPrevious_ = false;
    bool stable_ = false;
    bool pendingShort_ = false;

    uint32_t rawChangedMs_ = 0;
    uint32_t pressBeginMs_ = 0;
    uint32_t firstReleaseMs_ = 0;
};
