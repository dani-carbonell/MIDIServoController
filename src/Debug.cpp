#include "Debug.h"

// Static member definitions
uint8_t Debug::_debugLevel = Debug::DEBUG_NONE;
Stream* Debug::_debugStream = nullptr;
bool Debug::_timestampEnabled = false;
unsigned long Debug::_startTime = 0;