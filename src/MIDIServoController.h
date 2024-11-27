#ifndef MIDI_SERVO_CONTROLLER_H
#define MIDI_SERVO_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <MIDI.h>
#include <Adafruit_TinyUSB.h> // USB MIDI support
#include <Debug.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define MIDI_MAX_SERVOS 16
#elif defined(CONFIG_IDF_TARGET_ESP32S2) 
#define MIDI_MAX_SERVOS 8
#else
#define MIDI_MAX_SERVOS 16 // Default to 16 servos
#endif

class MIDIServoController {
public:
    // Remove conflicting constant and use the macro instead
    MIDIServoController();

    // Initialize the controller
    void begin(MIDI_NAMESPACE::MidiInterface<MIDI_NAMESPACE::SerialMIDI<Adafruit_USBD_MIDI>>& midiInterface);

    // Configure a servo
    void setServoPin(uint8_t servoIndex, uint8_t pin, 
                     int minUs = 500, int maxUs = 2500, 
                     int centerUs = 1500);

    // Set CC numbers for a servo
    void setServoCCs(uint8_t servoIndex, 
                     uint8_t coarseCCPosition, 
                     uint8_t fineCCPosition,
                     uint8_t CCSpeed = 0xFF);

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
        float speed = 22.0; // microseconds per millisecond

        // 14-bit CC support
        uint8_t coarseCCPosition = 0xFF; // Disabled
        uint8_t fineCCPosition = 0xFF;   // Disabled

        // 7-bit CC support
        uint8_t CCSpeed = 0xFF;   // Disabled

        // Temporary storage for 14-bit CC values
        uint8_t lastCoarsePosition = 0;
    };

    ServoConfig servos[MIDI_MAX_SERVOS];
    unsigned long previousMillis = 0;

    // Helper methods
    int mapCCToMicroseconds(uint16_t value, const ServoConfig& config);
    float mapCCToSpeed(uint16_t value);
    static void staticControlChangeHandler(byte channel, byte number, byte value);
};

#endif // MIDI_SERVO_CONTROLLER_H
