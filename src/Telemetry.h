#ifndef MIDISC_TELEMETRY_H
#define MIDISC_TELEMETRY_H

#include <Arduino.h>

// Telemetry is compiled in by default. The runtime `enabled` flag stays false
// until enableTelemetry(Stream&) is called, so the dead code path costs nothing
// in production. Override to 0 with a build flag if you need to strip it out.
#ifndef MIDISC_TELEMETRY
#define MIDISC_TELEMETRY 1
#endif

#if MIDISC_TELEMETRY

namespace MIDISCTelemetry {

enum EventKind : uint8_t {
    EV_NOTE_ON  = 1,
    EV_NOTE_OFF = 2,
    EV_CC       = 3,
    EV_SYNC     = 4,
};

struct Event {
    uint32_t t_rx;
    uint32_t t_done;
    uint16_t seq;
    uint8_t  kind;
    uint8_t  num;
    uint8_t  val;
    uint8_t  _pad[3];
};

class Telemetry {
public:
    void begin(Stream& outStream, uint16_t bufSize = 256);
    bool isEnabled() const { return enabled; }

    void recordEvent(uint8_t kind, uint8_t num, uint8_t val,
                     uint32_t t_rx, uint32_t t_done);
    void recordHandlerCost(uint32_t handler_us);
    void recordLoopTick(uint32_t loop_dt_us, uint32_t update_us);

    void flush(uint32_t now_us);

    void setSeqCCs(uint8_t hi, uint8_t lo);
    bool tryConsumeSeqCC(uint8_t cc, uint8_t value);
    uint16_t currentSeq() const { return seq; }

    void setFlushPeriodUs(uint32_t us) { flush_period_us = us; }

private:
    bool enabled = false;
    Stream* out = nullptr;
    Event* buf = nullptr;
    uint16_t cap = 0;
    volatile uint16_t head = 0;
    volatile uint16_t tail = 0;
    uint32_t dropped = 0;

    uint32_t loop_min = UINT32_MAX, loop_max = 0;
    uint64_t loop_sum = 0;
    uint32_t loop_n = 0;

    uint32_t handler_min = UINT32_MAX, handler_max = 0;
    uint64_t handler_sum = 0;
    uint32_t handler_n = 0;

    uint32_t upd_min = UINT32_MAX, upd_max = 0;
    uint64_t upd_sum = 0;
    uint32_t upd_n = 0;

    uint8_t seq_hi_cc = 0xFF;
    uint8_t seq_lo_cc = 0xFF;
    uint8_t seq_hi = 0;
    uint8_t seq_lo = 0;
    uint16_t seq = 0;

    uint32_t last_flush_us = 0;
    uint32_t flush_period_us = 5000;

    void resetStats();
};

} // namespace MIDISCTelemetry

#endif // MIDISC_TELEMETRY

#endif // MIDISC_TELEMETRY_H
