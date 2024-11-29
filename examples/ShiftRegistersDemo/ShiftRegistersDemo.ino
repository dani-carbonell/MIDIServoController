/*
  TinyUSB MIDI Shift Registers Controller Demo
  
  This example demonstrates how to use the MIDIServoController library with an ESP32 LOLIN S2 mini.
  It creates a USB MIDI device that can control up to 64 shiftregister outputs using MIDI NoteOnOff messages.
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
    usb_midi.setStringDescriptor("Shift Register Controller");
    usb_midi.begin();

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

}

void loop() {
    MIDI.read();
}