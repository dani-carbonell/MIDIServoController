#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <ESP32Servo.h>
#include <MIDIServoController.h>

// Create a USB MIDI instance
Adafruit_USBD_MIDI usb_midi;

// Create the MIDI instance for USB MIDI
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

// Create the MIDI Servo Controller
MIDIServoController servoController;

void setup() {
    // Start USB MIDI
    usb_midi.begin();
    
    // Allocate PWM timers (for ESP32)
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Configure servos
    servoController.begin(MIDI);
    
    // Set pins and limits for each servo
    servoController.setServoPin(0, 3, 500, 2500, 1500);  // Servo 1
    servoController.setServoPin(1, 5, 500, 2500, 1500);  // Servo 2
    servoController.setServoPin(2, 7, 500, 2500, 1500);  // Servo 3
    servoController.setServoPin(3, 9, 500, 2500, 1500);  // Servo 4
    servoController.setServoPin(4, 11, 500, 2500, 1500); // Servo 5
    servoController.setServoPin(5, 13, 500, 2500, 1500); // Servo 6
    servoController.setServoPin(6, 15, 500, 2500, 1500); // Servo 7
    servoController.setServoPin(7, 17, 500, 2500, 1500); // Servo 8

    // Set 14-bit CC numbers for servos
    // Position (Coarse, Fine), Speed
    servoController.setServoCCs(0, 20, 52, 30);  // Servo 1
    servoController.setServoCCs(1, 21, 53, 31);  // Servo 2
    servoController.setServoCCs(2, 22, 54, 32);  // Servo 3
    servoController.setServoCCs(3, 23, 55, 33);  // Servo 4
    servoController.setServoCCs(4, 24, 56, 34);  // Servo 5
    servoController.setServoCCs(5, 25, 57, 35);  // Servo 6
    servoController.setServoCCs(6, 26, 58, 36);  // Servo 7
    servoController.setServoCCs(7, 27, 59, 37);  // Servo 8
}

void loop() {
    // Read USB MIDI messages
    MIDI.read();
    
    // Update servo positions
    servoController.update();
}
