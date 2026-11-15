// Laser rangefinder driver for Meridian.
//
// The rest of the firmware only sees the RangeFinder interface, so a different module (or a corrected
// protocol) only means changing this file.
//
// ASSUMPTION, NOT YET CONFIRMED: the ERDI LRF0306C uses the same serial protocol as ERDI's LRF0310C
// (1535 nm sister model). That protocol comes from the LRF0310C manual (third-party copy at
// manuals.plus); no ERDI document for the LRF0306C was found. Before trusting it, get the LRF0306C
// manual from ERDI and check:
//   - baud 115200 8N1 (factory default in the LRF0310C manual; 57600 and 9600 are options)
//   - frame: EE 16 | len | device 03 | command | params | checksum (low 8 bits of device+command+params)
//   - single range 0x02 reply: EE 16 06 03 02 status int_hi int_lo tenths checksum, metres
//   - status 0x00 single, 0x01 front, 0x02 rear target, 0x04 out of range
//   - a POWER_ON pin (pulled high to run, about 0.5 s start-up)
// The LRF0310C manual's own tables disagree in places (e.g. "7" parameter bytes for a 4-byte reply),
// so the parser checks the length byte and the checksum rather than assuming sizes.
#pragma once

#include <cstdint>
#include <cstddef>

namespace lrf {

constexpr uint8_t kHdr0 = 0xEE, kHdr1 = 0x16, kDevice = 0x03;
enum : uint8_t {
    kCmdSelfCheck = 0x01, kCmdSingle = 0x02, kCmdTargetMode = 0x03, kCmdContinuous = 0x04,
    kCmdStop = 0x05, kCmdAnomaly = 0x06, kCmdFrequency = 0xA1,
};
// Target mode parameter for kCmdTargetMode.
enum : uint8_t { kTargetFirst = 0x01, kTargetLast = 0x02, kTargetMulti = 0x03 };

// Build a command frame. Returns its length (at most 4 + 1 + 1 + 4 + 1 = 11 bytes).
inline size_t build(uint8_t cmd, const uint8_t* params, size_t np, uint8_t* out) {
    out[0] = kHdr0; out[1] = kHdr1;
    out[2] = static_cast<uint8_t>(2 + np);
    out[3] = kDevice; out[4] = cmd;
    uint8_t sum = static_cast<uint8_t>(kDevice + cmd);
    for (size_t i = 0; i < np; ++i) { out[5 + i] = params[i]; sum = static_cast<uint8_t>(sum + params[i]); }
    out[5 + np] = sum;
    return 6 + np;
}

// One decoded reply frame.
struct Frame {
    uint8_t cmd = 0;
    uint8_t params[16] = {};
    uint8_t np = 0;
};

// Byte-at-a-time frame parser. feed() returns true when a complete frame with a good checksum is ready.
class Parser {
  public:
    bool feed(uint8_t b, Frame& out) {
        switch (state_) {
            case 0: if (b == kHdr0) state_ = 1; return false;
            case 1: state_ = (b == kHdr1) ? 2 : (b == kHdr0 ? 1 : 0); return false;
            case 2:
                if (b < 2 || b > 2 + sizeof(Frame::params)) { state_ = (b == kHdr0) ? 1 : 0; ++bad_; return false; }
                len_ = b; idx_ = 0; sum_ = 0; state_ = 3; return false;
            case 3:
                buf_[idx_++] = b; sum_ = static_cast<uint8_t>(sum_ + b);
                if (idx_ == len_) state_ = 4;
                return false;
            case 4: {
                state_ = 0;
                if (b != sum_ || buf_[0] != kDevice) { ++bad_; return false; }
                out.cmd = buf_[1];
                out.np = static_cast<uint8_t>(len_ - 2);
                for (uint8_t i = 0; i < out.np; ++i) out.params[i] = buf_[2 + i];
                return true;
            }
        }
        state_ = 0;
        return false;
    }
    uint32_t badFrames() const { return bad_; }
    void reset() { state_ = 0; }

  private:
    uint8_t state_ = 0, len_ = 0, idx_ = 0, sum_ = 0;
    uint8_t buf_[2 + sizeof(Frame::params)] = {};
    uint32_t bad_ = 0;
};

// Result of one measurement.
struct Result {
    enum Kind : uint8_t { kNone = 0, kRange, kNoTarget, kTimeout, kFault } kind = kNone;
    float range_m = 0;       // valid for kRange
    uint8_t echo = 0;        // status byte low nibble: 0 single, 1 front, 2 rear target
};

// Decode a ranging reply (single 0x02 or continuous 0x04). Returns false if the frame is not one.
inline bool decode_range(const Frame& f, Result& r) {
    if ((f.cmd != kCmdSingle && f.cmd != kCmdContinuous) || f.np < 4) return false;
    const uint8_t status = f.params[0];
    const uint8_t echo = status & 0x0F;
    r.echo = echo;
    const float m = (f.params[1] << 8 | f.params[2]) + f.params[3] * 0.1f;
    if (echo == 0x04 || m <= 0.0f) { r.kind = Result::kNoTarget; r.range_m = 0; return true; }
    r.kind = Result::kRange; r.range_m = m;
    return true;
}

// The interface the firmware uses. A Port needs: int available(); int read(); size_t write(const uint8_t*, size_t).
class RangeFinder {
  public:
    virtual ~RangeFinder() {}
    virtual void begin(uint32_t nowMs) = 0;
    virtual void single(uint32_t nowMs) = 0;
    virtual void continuous(bool on, uint32_t nowMs) = 0;
    virtual void targetMode(uint8_t mode) = 0;     // kTargetFirst or kTargetLast
    // Call every loop. Returns true when a new Result is ready.
    virtual bool poll(uint32_t nowMs, Result& out) = 0;
    virtual bool busy() const = 0;
    virtual bool isContinuous() const = 0;
    virtual bool present() const = 0;              // answered at least once since begin()
};

template <class Port>
class ErdiRangeFinder : public RangeFinder {
  public:
    explicit ErdiRangeFinder(Port& p) : port_(p) {}
    static constexpr uint32_t kReplyTimeoutMs = 1500;   // single shot reply wait (guess; manual gives none)
    static constexpr uint32_t kStartupMs = 600;         // POWER_ON to first command (manual: about 0.5 s)

    void begin(uint32_t nowMs) override { readyAt_ = nowMs + kStartupMs; selfCheckPending_ = true; }

    void single(uint32_t nowMs) override {
        if (cont_) return;                       // continuous mode already streams ranges
        send(kCmdSingle, nullptr, 0);
        waiting_ = true; sentAt_ = nowMs;
    }
    void continuous(bool on, uint32_t nowMs) override {
        if (on) {
            const uint8_t hz[2] = {0x01, 0x00};  // 1 Hz (frequency param per the manual's example frame)
            send(kCmdFrequency, hz, 2);
            send(kCmdContinuous, nullptr, 0);
        } else {
            send(kCmdStop, nullptr, 0);
        }
        cont_ = on; waiting_ = false; lastContMs_ = nowMs;
    }
    void targetMode(uint8_t mode) override { send(kCmdTargetMode, &mode, 1); }

    bool poll(uint32_t nowMs, Result& out) override {
        if (selfCheckPending_ && (int32_t)(nowMs - readyAt_) >= 0) {
            selfCheckPending_ = false;
            send(kCmdSelfCheck, nullptr, 0);
        }
        Frame f;
        while (port_.available() > 0) {
            const int c = port_.read();
            if (c < 0) break;
            if (!parser_.feed(static_cast<uint8_t>(c), f)) continue;
            present_ = true;
            Result r;
            if (decode_range(f, r)) {
                waiting_ = false;
                if (f.cmd == kCmdContinuous) lastContMs_ = nowMs;
                out = r;
                return true;
            }
            if (f.cmd == kCmdSelfCheck && f.np >= 4) {
                // Status1 bit0 = FPGA system normal (LRF0310C manual). Anything else counts as a fault.
                selfCheckOk_ = (f.params[2] & 0x01) != 0;
                if (!selfCheckOk_) { out = Result{}; out.kind = Result::kFault; return true; }
            }
            if (f.cmd == kCmdAnomaly) { waiting_ = false; out = Result{}; out.kind = Result::kFault; return true; }
        }
        if (waiting_ && nowMs - sentAt_ > kReplyTimeoutMs) {
            waiting_ = false;
            out = Result{}; out.kind = Result::kTimeout;
            return true;
        }
        return false;
    }
    bool busy() const override { return waiting_ || cont_; }
    bool isContinuous() const override { return cont_; }
    bool present() const override { return present_; }
    bool selfCheckOk() const { return selfCheckOk_; }
    uint32_t badFrames() const { return parser_.badFrames(); }

  private:
    void send(uint8_t cmd, const uint8_t* p, size_t n) {
        uint8_t buf[12] = {};
        const size_t len = build(cmd, p, n, buf);
        port_.write(buf, len);
    }
    Port& port_;
    Parser parser_;
    bool waiting_ = false, cont_ = false, present_ = false, selfCheckPending_ = false, selfCheckOk_ = true;
    uint32_t sentAt_ = 0, readyAt_ = 0, lastContMs_ = 0;
};

}  // namespace lrf
