
#include "ButtonHandler.h"
#include "config.h"

void ButtonHandler::begin() {
    // External 10k pulldown, active HIGH
    pinMode(BUTTON_PIN, INPUT);

    rawPrevious_ = digitalRead(BUTTON_PIN);
    stable_ = rawPrevious_;
    pendingShort_ = false;
}

ButtonEvent ButtonHandler::update(
    uint32_t now,
    bool immediateShort
) {
    bool raw = digitalRead(BUTTON_PIN) == HIGH;

    if (raw != rawPrevious_) {
        rawPrevious_ = raw;
        rawChangedMs_ = now;
    }

    if (raw != stable_ &&
        (uint32_t)(now - rawChangedMs_) >= DEBOUNCE_MS) {

        stable_ = raw;

        if (stable_) {
            pressBeginMs_ = now;
        } else {
            uint32_t duration = now - pressBeginMs_;

            if (duration >= LONG_PRESS_MS) {
                pendingShort_ = false;
                return ButtonEvent::LONG_PRESS;
            }

            if (duration >= MIN_PRESS_MS) {
                if (immediateShort) {
                    pendingShort_ = false;
                    return ButtonEvent::SHORT_PRESS;
                }

                if (pendingShort_ &&
                    (uint32_t)(now - firstReleaseMs_)
                        <= DOUBLE_PRESS_MS) {

                    pendingShort_ = false;
                    return ButtonEvent::DOUBLE_PRESS;
                }

                pendingShort_ = true;
                firstReleaseMs_ = now;
            }
        }
    }

    if (pendingShort_ &&
        !stable_ &&
        (uint32_t)(now - firstReleaseMs_)
            > DOUBLE_PRESS_MS) {

        pendingShort_ = false;
        return ButtonEvent::SHORT_PRESS;
    }

    return ButtonEvent::NONE;
}
