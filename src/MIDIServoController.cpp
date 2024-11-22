#include "MIDIServoController.h"

// Static member initialization
MIDIServoController* MIDIServoController::instance = nullptr;

MIDIServoController::MIDIServoController() {
    instance = this;
}

void MIDIServoController::begin(MIDI_NAMESPACE::MidiInterface<MIDI_NAMESPACE::SerialMIDI<Adafruit_USBD_MIDI>>& midiInterface) {
    // Set up MIDI
    midiInterface.setHandleControlChange(staticControlChangeHandler);
    Debug::info("MIDIServoController initialized");

}

void MIDIServoController::setServoPin(uint8_t servoIndex, uint8_t pin, int minUs, int maxUs, int centerUs) {
    if (servoIndex >= MIDI_MAX_SERVOS) {
        Debug::error("Invalid servo index: " + String(servoIndex));
        return;
    }

    ServoConfig& config = servos[servoIndex];

    if (config.active) {
        Debug::info("Detaching servo " + String(servoIndex) + " from pin " + String(pin));
        config.servo.detach();
    }

    config.minUs = minUs;
    config.maxUs = maxUs;
    config.centerUs = centerUs;
    config.currentPos = centerUs;
    config.targetPos = centerUs;

    Debug::info("Servo " + String(servoIndex) + 
                " on pin " + String(pin) + 
                " (min: " + String(minUs) + 
                "us, max: " + String(maxUs) + 
                "us, center: " + String(centerUs) + "us)");
                
    config.servo.setPeriodHertz(200);
    config.servo.attach(pin, minUs, maxUs);
    config.servo.writeMicroseconds(centerUs);
    config.active = true;
}

void MIDIServoController::setServoCCs(uint8_t servoIndex, uint8_t coarseCCPosition, uint8_t fineCCPosition, uint8_t CCSpeed) {
    if (servoIndex >= MIDI_MAX_SERVOS) {
        Debug::error("Invalid servo index for CC assignment: " + String(servoIndex));
        return;
    }

    ServoConfig& config = servos[servoIndex];
    config.coarseCCPosition = coarseCCPosition;
    config.fineCCPosition = fineCCPosition;
    config.CCSpeed = CCSpeed;
}

void MIDIServoController::setServoPosition(uint8_t servoIndex, int microseconds) {
    if (servoIndex >= MIDI_MAX_SERVOS) {
        Debug::error("Invalid servo index for position: " + String(servoIndex));
        return;
    }
    
    if (!servos[servoIndex].active) {
        Debug::warn("Attempt to move inactive servo: " + String(servoIndex));
        return;
    }
    ServoConfig& config = servos[servoIndex];
    int constrainedPos = constrain(microseconds, config.minUs, config.maxUs);
    
    if (constrainedPos != microseconds) {
        Debug::warn("Servo " + String(servoIndex) + 
                   " position constrained from " + String(microseconds) + 
                   " to " + String(constrainedPos));
    }
    
    Debug::verbose("Servo " + String(servoIndex) + 
                  " target position: " + String(constrainedPos) + "us");
    
    config.targetPos = constrainedPos;
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
        int newPos = round(config.currentPos);
        
        Debug::verbose("S" + String(i) + 
                      " to " + String(newPos) + 
                      " (spd: " + String(config.speed) + 
                      " dT: " + String(deltaTime) + "ms)");
        
        config.servo.writeMicroseconds(newPos);
    }
}

void MIDIServoController::handleControlChange(byte channel, byte number, byte value) {
    Debug::debug("CC received - Channel: " + String(channel) + 
                 ", CC: " + String(number) + 
                 ", Value: " + String(value));

    for (uint8_t i = 0; i < MIDI_MAX_SERVOS; i++) {
        ServoConfig& config = servos[i];

        if (number == config.coarseCCPosition) {
            config.lastCoarsePosition = value;
            uint16_t fullValue = (config.lastCoarsePosition << 7) | (config.fineCCPosition < 0xFF ? 0 : value);
            config.targetPos = mapCCToMicroseconds(fullValue, config);
            Debug::debug("Servo " + String(i) + 
                        " coarse position update: " + String(config.targetPos) + "us");
        } 
        else if (number == config.fineCCPosition) {
            uint16_t fullValue = (config.lastCoarsePosition << 7) | value;
            config.targetPos = mapCCToMicroseconds(fullValue, config);
            Debug::debug("Servo " + String(i) + 
                        " fine position update: " + String(config.targetPos) + "us");
        }

        if (number == config.CCSpeed) {
            config.speed = mapCCToSpeed(value);
            Debug::debug("Servo " + String(i) + 
                        " speed update: " + String(config.speed) + " us/ms");
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
    return map(value, 0, 127, 500, 22000) / 1000.0;
}
