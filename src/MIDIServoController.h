#ifndef MIDI_SERVO_CONTROLLER_H
#define MIDI_SERVO_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <MIDI.h>
#include <Adafruit_TinyUSB.h> // USB MIDI support
#include <Debug.h>


// Configuration constants
#define MAX_SHIFT_REGISTERS 8  // Maximum 8 shift registers (64 bits)
#ifndef NUM_SHIFT_REGISTERS
#define NUM_SHIFT_REGISTERS 1  // Default to 1 shift register if not defined
#endif

#if NUM_SHIFT_REGISTERS > MAX_SHIFT_REGISTERS
#error "NUM_SHIFT_REGISTERS cannot exceed MAX_SHIFT_REGISTERS (8)"
#endif

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
    // Add new methods
    void setShiftRegisterPins(uint8_t dataPin, uint8_t clockPin, uint8_t latchPin);
    void mapNoteToShiftRegister(uint8_t note, uint8_t bitPosition, uint8_t velocityThreshold = 0);
    
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

    struct NoteMapping {
        uint8_t bitPosition = 0xFF; // Disabled by default
        uint8_t velocityThreshold = 0;
        bool active = false;
    };

    ServoConfig servos[MIDI_MAX_SERVOS];
    unsigned long previousMillis = 0;
    NoteMapping noteMappings[128]; // One for each MIDI note
    uint8_t shiftRegData[MAX_SHIFT_REGISTERS] = {0};
    uint8_t dataPin = 0xFF;
    uint8_t clockPin = 0xFF;
    uint8_t latchPin = 0xFF;
    bool shiftRegEnabled = false;
    
    // Helper methods
    int mapCCToMicroseconds(uint16_t value, const ServoConfig& config);
    float mapCCToSpeed(uint16_t value);
    static void staticControlChangeHandler(byte channel, byte number, byte value);
    static void staticNoteOnHandler(byte channel, byte note, byte velocity);
    static void staticNoteOffHandler(byte channel, byte note, byte velocity);
    void updateShiftRegister();
    void handleNoteOn(byte channel, byte note, byte velocity);
    void handleNoteOff(byte channel, byte note, byte velocity);

};

#endif // MIDI_SERVO_CONTROLLER_H
