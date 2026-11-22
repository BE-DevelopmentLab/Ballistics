// Laser rangefinder driver for Meridian.
//
// The rest of the firmware only sees the RangeFinder interface, so a different module only changes this file.
//
// Protocol from ERDI's LRF0306C manual (ballistic-calc/LRF0306C.pdf, checked 2026-11-15):
//   - UART 3.3 V, 115200 8N1 factory default (57600 / 38400 / 9600 selectable)
//   - frame: EE 16 | len (2..7) | device 03 | command | 0..5 params | checksum = low 8 bits of device+command+params
//   - single range 0x02 / continuous 0x04 reply: EE 16 06 03 cmd status int_hi int_lo tenths checksum, metres
//   - status 0x00 single, 0x01 front, 0x02 back target, 0x04 over range (multi-target mode: low nibble, high = index)
//   - the module sends 0x06 by itself on a ranging error
//   - POWER_ON is low by default. Pull it high and wait about 0.3 s before any command (start-up <= 350 ms).
//     Standby draws <= 0.8 W with POWER_ON high but <= 10 mW with it low, so this driver keeps the laser off
//     between shots and switches it on for each ranging (about 0.35 s extra on the first shot after idling).
// Not yet tried on a real module.
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
    // power(true/false) drives the module's POWER_ON pin; pass nullptr if POWER_ON is tied high.
    explicit ErdiRangeFinder(Port& p, void (*power)(bool) = nullptr) : port_(p), power_(power) {}
    static constexpr uint32_t kReplyTimeoutMs = 1500;   // single shot reply wait (manual: response <= 20 ms after start-up)
    static constexpr uint32_t kStartupMs = 400;         // POWER_ON high to first command (manual: about 0.3 s, <= 350 ms)
    static constexpr uint32_t kIdleOffMs = 60000;       // switch the laser off after this long without ranging

    void begin(uint32_t nowMs) override {
        powerOn(nowMs);                                  // first power-up: run the self-check
        queue(kCmdSelfCheck);
        queue(kCmdTargetMode, target_);
    }
    void single(uint32_t nowMs) override {
        if (cont_) return;                               // continuous mode already streams ranges
        powerOn(nowMs);
        queue(kCmdSingle);
        waiting_ = true; sentAt_ = nowMs;
    }
    void continuous(bool on, uint32_t nowMs) override {
        if (on) {
            powerOn(nowMs);
            queue(kCmdFrequency, 0x01, 0x00);            // 1 Hz (the LRF0306C does 1 to 3 Hz)
            queue(kCmdContinuous);
        } else if (powered_) {
            queue(kCmdStop);
        }
        cont_ = on; waiting_ = false; lastUseMs_ = nowMs;
    }
    void targetMode(uint8_t mode) override {
        target_ = mode;
        if (powered_) queue(kCmdTargetMode, mode);       // otherwise sent at the next power-up
    }

    bool poll(uint32_t nowMs, Result& out) override {
        // Send queued commands once the module has had time to start.
        if (powered_ && nq_ && (int32_t)(nowMs - readyAt_) >= 0) {
            for (uint8_t i = 0; i < nq_; ++i) send(q_[i].cmd, q_[i].p, q_[i].n);
            if (waiting_) sentAt_ = nowMs;               // the reply clock starts when the command really goes out
            nq_ = 0;
        }
        Frame f;
        while (port_.available() > 0) {
            const int c = port_.read();
            if (c < 0) break;
            if (!parser_.feed(static_cast<uint8_t>(c), f)) continue;
            present_ = true;
            Result r;
            if (decode_range(f, r)) {
                waiting_ = false; lastUseMs_ = nowMs;
                out = r;
                return true;
            }
            if (f.cmd == kCmdSelfCheck && f.np >= 4) {
                // params: Status3 reserved, Status2 echo intensity, Status1 (bit0 FPGA normal), Status0 (bit0 5V6 power normal)
                selfCheckOk_ = (f.params[2] & 0x01) && (f.params[3] & 0x01);
                if (!selfCheckOk_) { out = Result{}; out.kind = Result::kFault; return true; }
            }
            if (f.cmd == kCmdAnomaly) { waiting_ = false; out = Result{}; out.kind = Result::kFault; return true; }
        }
        if (waiting_ && nq_ == 0 && nowMs - sentAt_ > kReplyTimeoutMs) {
            waiting_ = false; lastUseMs_ = nowMs;
            out = Result{}; out.kind = Result::kTimeout;
            return true;
        }
        if (powered_ && !waiting_ && !cont_ && nq_ == 0 && nowMs - lastUseMs_ > kIdleOffMs && power_) {
            power_(false); powered_ = false;
        }
        return false;
    }
    bool busy() const override { return waiting_ || cont_; }
    bool isContinuous() const override { return cont_; }
    bool present() const override { return present_; }
    bool powered() const { return powered_; }
    bool selfCheckOk() const { return selfCheckOk_; }
    uint32_t badFrames() const { return parser_.badFrames(); }
    // Switch the laser off now (sleep).
    void off() { if (power_) power_(false); powered_ = false; cont_ = false; waiting_ = false; nq_ = 0; }

  private:
    struct Cmd { uint8_t cmd, p[2], n; };
    void powerOn(uint32_t nowMs) {
        lastUseMs_ = nowMs;
        if (powered_) return;
        if (power_) power_(true);
        powered_ = true;
        readyAt_ = nowMs + (power_ ? kStartupMs : 0);
        if (power_ && booted_) queue(kCmdTargetMode, target_);   // the module forgets settings when powered off (assumed)
        booted_ = true;
    }
    void queue(uint8_t cmd) { if (nq_ < 6) q_[nq_++] = Cmd{cmd, {0, 0}, 0}; }
    void queue(uint8_t cmd, uint8_t a) { if (nq_ < 6) q_[nq_++] = Cmd{cmd, {a, 0}, 1}; }
    void queue(uint8_t cmd, uint8_t a, uint8_t b) { if (nq_ < 6) q_[nq_++] = Cmd{cmd, {a, b}, 2}; }
    void send(uint8_t cmd, const uint8_t* p, size_t n) {
        uint8_t buf[12] = {};
        const size_t len = build(cmd, p, n, buf);
        port_.write(buf, len);
    }
    Port& port_;
    void (*power_)(bool);
    Parser parser_;
    Cmd q_[6] = {};
    uint8_t nq_ = 0, target_ = kTargetFirst;
    bool waiting_ = false, cont_ = false, present_ = false, selfCheckOk_ = true, powered_ = false, booted_ = false;
    uint32_t sentAt_ = 0, readyAt_ = 0, lastUseMs_ = 0;
};

}  // namespace lrf
