/*
  TinyUSB MIDI Multi Controller Demo
  
  This example demonstrates how to use the MIDIServoController library with an ESP32 LOLIN S2 mini.
  It creates a USB MIDI device that can control:
  
  -Up to 8 servos using MIDI CC messages.
        Each servo can be controlled with 14-bit position resolution and speed control.

  -Up to 64 shift register outputs using MIDI NoteOnOff messages. (With the TPIC6C596 modules you can dasy chain them and drive lights, motors and valves)
        Each output can be mapped to any note and can have a treshold to activate it.

  Author: Dani Carbonell (danicarbonellrubio@gmail.com)
  
  This example code is in the public domain.
  
  https://github.com/danicarbonell/MIDIServoController
*/
#include <Arduino.h>
#include <MIDI.h>
#include <Adafruit_TinyUSB.h>
#include <MIDIServoController.h>

#define NUM_SHIFT_REGISTERS 1  // For 8 outputs

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

MIDIServoController controller;

// Pin Definitions
const uint8_t SHIFT_DATA_PIN = 16;
const uint8_t SHIFT_CLOCK_PIN = 18;
const uint8_t SHIFT_LATCH_PIN = 33;

void setup() {

    Serial.begin(115200);
    //Debug::begin(Serial, Debug::DEBUG_DEBUG);

    // Initialize USB MIDI
    usb_midi.setStringDescriptor("Servo-SR Controller");
    usb_midi.begin();

    // Allocate PWM timers (for ESP32)
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    //delay(1000);

    // Initialize controller
    controller.begin(MIDI);
    
    // Configure shift register
    controller.setShiftRegisterPins(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, SHIFT_LATCH_PIN);
    
    // Valve Mappings
    controller.mapNoteToShiftRegister(36, 0, 1);
    controller.mapNoteToShiftRegister(37, 1, 1);
    controller.mapNoteToShiftRegister(38, 2, 1);
    controller.mapNoteToShiftRegister(39, 3, 1);
    controller.mapNoteToShiftRegister(40, 4, 1);
    controller.mapNoteToShiftRegister(41, 5, 1);
    controller.mapNoteToShiftRegister(42, 6, 1);
    controller.mapNoteToShiftRegister(43, 7, 1);
    
    
    // Set pins and limits for each servo
    controller.setServoPin(0, 3, 500, 2500, 1500);  // Servo 1
    controller.setServoPin(1, 5, 500, 2500, 1500);  // Servo 2
    controller.setServoPin(2, 7, 500, 2500, 1500);  // Servo 3
    controller.setServoPin(3, 9, 500, 2500, 1500);  // Servo 4
    controller.setServoPin(4, 11, 500, 2500, 1500); // Servo 5
    controller.setServoPin(5, 2, 500, 2500, 1500); // Servo 6
    controller.setServoPin(6, 4, 500, 2500, 1500); // Servo 7
    controller.setServoPin(7, 6, 500, 2500, 1500); // Servo 8

    // Set 14-bit CC numbers for servos
    // Position (Coarse, Fine), Speed
    controller.setServoCCs(0, 20, 52, 30);  // Servo 1
    controller.setServoCCs(1, 21, 53, 31);  // Servo 2
    controller.setServoCCs(2, 22, 54, 32);  // Servo 3
    controller.setServoCCs(3, 23, 55, 33);  // Servo 4
    controller.setServoCCs(4, 24, 56, 34);  // Servo 5
    controller.setServoCCs(5, 25, 57, 35);  // Servo 6
    controller.setServoCCs(6, 26, 58, 36);  // Servo 7
    controller.setServoCCs(7, 27, 59, 37);  // Servo 8

}

void loop() {
    MIDI.read();
    controller.update();  // Update servo positions
}