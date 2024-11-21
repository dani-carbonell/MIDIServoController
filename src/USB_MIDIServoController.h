#ifndef MIDI_SERVO_CONTROLLER_H
#define MIDI_SERVO_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <MIDI.h>
#include <Adafruit_TinyUSB.h> // USB MIDI support

class MIDIServoController {
public:
    static const uint8_t MIDI_MAX_SERVOS = 16; // Support up to 16 servos

    MIDIServoController();

    // Initialize the controller
    void begin(MIDI_NAMESPACE::MidiInterface<MIDI_NAMESPACE::SerialMIDI<Adafruit_USBD_MIDI>>& midiInterface);

    // Configure a servo
    void setServoPin(uint8_t servoIndex, uint8_t pin, 
                     int minUs = 500, int maxUs = 2500, 
                     int centerUs = 1500);

    // Set CC numbers for a servo (supports 14-bit CCs)
    void setServoCCs(uint8_t servoIndex, 
                     uint8_t coarseCCPosition, 
                     uint8_t fineCCPosition,
                     uint8_t coarseCCSpeed = 0xFF, 
                     uint8_t fineCCSpeed = 0xFF);

    // Manual servo control (alternative to MIDI)
    void setServoPosition(uint8_t servoIndex, int microseconds);
    void setServoSpeed(uint8_t servoIndex, float speedUsPerMs);

    // Update servo positions (call in loop())
    void update();

    // MIDI Callback (attach this to MIDI library)
    void handleControlChange(byte channel, byte number, byte value);

    // Static instance for callback
    static MIDIServoController* instance;

private:
    struct ServoConfig {
        Servo servo;
        bool active = false;
        int minUs = 500;
        int maxUs = 2500;
        int centerUs = 1500;

        float currentPos = 1500;
        float targetPos = 1500;
        float speed = 1.0; // microseconds per millisecond

        // 14-bit CC support
        uint8_t coarseCCPosition = 0xFF; // Disabled
        uint8_t fineCCPosition = 0xFF;   // Disabled
        uint8_t coarseCCSpeed = 0xFF;   // Disabled
        uint8_t fineCCSpeed = 0xFF;     // Disabled

        // Temporary storage for 14-bit CC values
        uint8_t lastCoarsePosition = 0;
        uint8_t lastCoarseSpeed = 0;
    };

    ServoConfig servos[MIDI_MAX_SERVOS];
    unsigned long previousMillis = 0;

    // Helper methods
    int mapCCToMicroseconds(uint16_t value, const ServoConfig& config);
    float mapCCToSpeed(uint16_t value);
    static void staticControlChangeHandler(byte channel, byte number, byte value);
};

#endif // MIDI_SERVO_CONTROLLER_H
