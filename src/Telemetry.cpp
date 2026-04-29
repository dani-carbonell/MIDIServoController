#include "Telemetry.h"

#if MIDISC_TELEMETRY

namespace MIDISCTelemetry {

void Telemetry::begin(Stream& outStream, uint16_t bufSize) {
    out = &outStream;
    cap = bufSize;
    if (buf) delete[] buf;
    buf = new Event[cap];
    head = tail = 0;
    dropped = 0;
    last_flush_us = micros();
    resetStats();
    enabled = true;
}

void Telemetry::recordEvent(uint8_t kind, uint8_t num, uint8_t val,
                            uint32_t t_rx, uint32_t t_done) {
    if (!enabled) return;
    uint16_t next = (uint16_t)((head + 1) % cap);
    if (next == tail) {
        dropped++;
        return;
    }
    Event& e = buf[head];
    e.t_rx = t_rx;
    e.t_done = t_done;
    e.seq = seq;
    e.kind = kind;
    e.num = num;
    e.val = val;
    head = next;
}

void Telemetry::recordHandlerCost(uint32_t handler_us) {
    if (!enabled) return;
    if (handler_us < handler_min) handler_min = handler_us;
    if (handler_us > handler_max) handler_max = handler_us;
    handler_sum += handler_us;
    handler_n++;
}

void Telemetry::recordLoopTick(uint32_t loop_dt, uint32_t upd_dt) {
    if (!enabled) return;
    if (loop_dt > 0) {
        if (loop_dt < loop_min) loop_min = loop_dt;
        if (loop_dt > loop_max) loop_max = loop_dt;
        loop_sum += loop_dt;
        loop_n++;
    }
    if (upd_dt < upd_min) upd_min = upd_dt;
    if (upd_dt > upd_max) upd_max = upd_dt;
    upd_sum += upd_dt;
    upd_n++;
}

void Telemetry::flush(uint32_t now_us) {
    if (!enabled || !out) return;
    if ((uint32_t)(now_us - last_flush_us) < flush_period_us) return;
    last_flush_us = now_us;

    while (tail != head) {
        const Event& e = buf[tail];
        out->print('M'); out->print(',');
        out->print(e.seq); out->print(',');
        out->print(e.t_rx); out->print(',');
        out->print(e.t_done); out->print(',');
        out->print(e.kind); out->print(',');
        out->print(e.num); out->print(',');
        out->println(e.val);
        tail = (uint16_t)((tail + 1) % cap);
    }

    out->print('S'); out->print(',');
    out->print(now_us); out->print(',');
    out->print(loop_n); out->print(',');
    out->print(loop_n ? loop_min : 0); out->print(',');
    out->print(loop_max); out->print(',');
    out->print(loop_n ? (uint32_t)(loop_sum / loop_n) : 0); out->print(',');
    out->print(handler_n); out->print(',');
    out->print(handler_n ? handler_min : 0); out->print(',');
    out->print(handler_max); out->print(',');
    out->print(handler_n ? (uint32_t)(handler_sum / handler_n) : 0); out->print(',');
    out->print(upd_n); out->print(',');
    out->print(upd_n ? upd_min : 0); out->print(',');
    out->print(upd_max); out->print(',');
    out->print(upd_n ? (uint32_t)(upd_sum / upd_n) : 0); out->print(',');
    out->println(dropped);

    resetStats();
}

void Telemetry::resetStats() {
    loop_min = handler_min = upd_min = UINT32_MAX;
    loop_max = handler_max = upd_max = 0;
    loop_sum = handler_sum = upd_sum = 0;
    loop_n = handler_n = upd_n = 0;
}

void Telemetry::setSeqCCs(uint8_t hi, uint8_t lo) {
    seq_hi_cc = hi;
    seq_lo_cc = lo;
}

bool Telemetry::tryConsumeSeqCC(uint8_t cc, uint8_t value) {
    if (!enabled) return false;
    if (seq_hi_cc != 0xFF && cc == seq_hi_cc) {
        seq_hi = value;
        return true;
    }
    if (seq_lo_cc != 0xFF && cc == seq_lo_cc) {
        seq_lo = value;
        seq = (uint16_t)(((uint16_t)seq_hi << 7) | (uint16_t)seq_lo);
        return true;
    }
    return false;
}

} // namespace MIDISCTelemetry

#endif
