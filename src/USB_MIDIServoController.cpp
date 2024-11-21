#include "USB_MIDIServoController.h"

// Static member initialization
MIDIServoController* MIDIServoController::instance = nullptr;

MIDIServoController::MIDIServoController() {
    instance = this;
}

void MIDIServoController::begin(MIDI_NAMESPACE::MidiInterface<MIDI_NAMESPACE::SerialMIDI<Adafruit_USBD_MIDI>>& midiInterface) {
    // Set up MIDI
    midiInterface.setHandleControlChange(staticControlChangeHandler);
}

void MIDIServoController::setServoPin(uint8_t servoIndex, uint8_t pin, int minUs, int maxUs, int centerUs) {
    if (servoIndex >= MIDI_MAX_SERVOS) return;

    ServoConfig& config = servos[servoIndex];

    if (config.active) {
        config.servo.detach();
    }

    config.minUs = minUs;
    config.maxUs = maxUs;
    config.centerUs = centerUs;
    config.currentPos = centerUs;
    config.targetPos = centerUs;

    config.servo.attach(pin, minUs, maxUs);
    config.servo.writeMicroseconds(centerUs);
    config.active = true;
}

void MIDIServoController::setServoCCs(uint8_t servoIndex, uint8_t coarseCCPosition, uint8_t fineCCPosition, uint8_t coarseCCSpeed, uint8_t fineCCSpeed) {
    if (servoIndex >= MIDI_MAX_SERVOS) return;

    ServoConfig& config = servos[servoIndex];
    config.coarseCCPosition = coarseCCPosition;
    config.fineCCPosition = fineCCPosition;
    config.coarseCCSpeed = coarseCCSpeed;
    config.fineCCSpeed = fineCCSpeed;
}

void MIDIServoController::setServoPosition(uint8_t servoIndex, int microseconds) {
    if (servoIndex >= MIDI_MAX_SERVOS || !servos[servoIndex].active) return;

    ServoConfig& config = servos[servoIndex];
    config.targetPos = constrain(microseconds, config.minUs, config.maxUs);
}

void MIDIServoController::setServoSpeed(uint8_t servoIndex, float speedUsPerMs) {
    if (servoIndex >= MIDI_MAX_SERVOS || !servos[servoIndex].active) return;

    servos[servoIndex].speed = speedUsPerMs;
}

void MIDIServoController::update() {
    unsigned long currentMillis = millis();
    unsigned long deltaTime = currentMillis - previousMillis;
    previousMillis = currentMillis;

    for (uint8_t i = 0; i < MIDI_MAX_SERVOS; i++) {
        ServoConfig& config = servos[i];

        if (!config.active || config.currentPos == config.targetPos) continue;

        float maxMove = config.speed * deltaTime;
        float diff = config.targetPos - config.currentPos;
        float move = constrain(diff, -maxMove, maxMove);

        config.currentPos += move;
        config.currentPos = constrain(config.currentPos, config.minUs, config.maxUs);
        config.servo.writeMicroseconds(round(config.currentPos));
    }
}

void MIDIServoController::handleControlChange(byte channel, byte number, byte value) {
    for (uint8_t i = 0; i < MIDI_MAX_SERVOS; i++) {
        ServoConfig& config = servos[i];

        if (number == config.coarseCCPosition) {
            config.lastCoarsePosition = value;
            uint16_t fullValue = (config.lastCoarsePosition << 7) | (config.fineCCPosition < 0xFF ? 0 : value);
            config.targetPos = mapCCToMicroseconds(fullValue, config);
        } else if (number == config.fineCCPosition) {
            uint16_t fullValue = (config.lastCoarsePosition << 7) | value;
            config.targetPos = mapCCToMicroseconds(fullValue, config);
        }

        if (number == config.coarseCCSpeed) {
            config.lastCoarseSpeed = value;
            uint16_t fullValue = (config.lastCoarseSpeed << 7) | (config.fineCCSpeed < 0xFF ? 0 : value);
            config.speed = mapCCToSpeed(fullValue);
        } else if (number == config.fineCCSpeed) {
            uint16_t fullValue = (config.lastCoarseSpeed << 7) | value;
            config.speed = mapCCToSpeed(fullValue);
        }
    }
}

void MIDIServoController::staticControlChangeHandler(byte channel, byte number, byte value) {
    if (instance) {
        instance->handleControlChange(channel, number, value);
    }
}

int MIDIServoController::mapCCToMicroseconds(uint16_t value, const ServoConfig& config) {
    return map(value, 0, 16383, config.minUs, config.maxUs);
}

float MIDIServoController::mapCCToSpeed(uint16_t value) {
    return map(value, 0, 16383, 100, 2000) / 1000.0;
}
