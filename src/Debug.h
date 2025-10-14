#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

class Debug {
private:
    static uint8_t _debugLevel;
    static Stream* _debugStream;
    static bool _timestampEnabled;
    static unsigned long _startTime;

    // Private constructor to prevent instantiation
    Debug() {}

    static void printTimestamp() {
        if (_timestampEnabled) {
            unsigned long currentTime = millis() - _startTime;
            _debugStream->print("[");
            _debugStream->print(currentTime);
            _debugStream->print("ms] ");
        }
    }

public:
    static const uint8_t DEBUG_NONE = 0;
    static const uint8_t DEBUG_ERROR = 1;
    static const uint8_t DEBUG_WARN = 2;
    static const uint8_t DEBUG_INFO = 3;
    static const uint8_t DEBUG_DEBUG = 4;
    static const uint8_t DEBUG_VERBOSE = 5;

    static void begin(Stream& stream, uint8_t level = DEBUG_INFO) {
        _debugStream = &stream;
        _debugLevel = level;
        _startTime = millis();
        _timestampEnabled = true;
    }

    static void enableTimestamp(bool enable) {
        _timestampEnabled = enable;
    }

    static void setLevel(uint8_t level) {
        _debugLevel = level;
    }

    static bool isEnabled() {
        return _debugLevel > DEBUG_NONE;
    }

    template<typename T>
    static void error(const T& message) {
        if (_debugLevel >= DEBUG_ERROR) {
            printTimestamp();
            _debugStream->print("ERROR: ");
            _debugStream->println(message);
        }
    }

    template<typename T>
    static void warn(const T& message) {
        if (_debugLevel >= DEBUG_WARN) {
            printTimestamp();
            _debugStream->print("WARN: ");
            _debugStream->println(message);
        }
    }

    template<typename T>
    static void info(const T& message) {
        if (_debugLevel >= DEBUG_INFO) {
            printTimestamp();
            _debugStream->print("INFO: ");
            _debugStream->println(message);
        }
    }

    template<typename T>
    static void debug(const T& message) {
        if (_debugLevel >= DEBUG_DEBUG) {
            printTimestamp();
            _debugStream->print("DEBUG: ");
            _debugStream->println(message);
        }
    }

    template<typename T>
    static void verbose(const T& message) {
        if (_debugLevel >= DEBUG_VERBOSE) {
            printTimestamp();
            _debugStream->print("VERBOSE: ");
            _debugStream->println(message);
        }
    }

    static void hexDump(const uint8_t* data, size_t length) {
        if (_debugLevel >= DEBUG_DEBUG) {
            for (size_t i = 0; i < length; i++) {
                if (data[i] < 0x10) _debugStream->print("0");
                _debugStream->print(data[i], HEX);
                _debugStream->print(" ");
                if ((i + 1) % 16 == 0) _debugStream->println();
            }
            _debugStream->println();
        }
    }
};

#endif // DEBUG_H