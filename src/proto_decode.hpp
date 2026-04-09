#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <memory>

// ── Decoder output ──────────────────────────────────────────────────────
struct DecoderOutput {
    std::string summary;
    std::string detail;
    bool   valid       = false;
    int    error_count = 0;
};

// ── Base decoder interface ──────────────────────────────────────────────
class ProtoDecoder {
public:
    virtual ~ProtoDecoder() = default;
    virtual bool feed_bit(uint8_t bit) = 0;
    virtual DecoderOutput get_output() = 0;
    virtual void reset() = 0;
    virtual uint64_t sync_word() const = 0;
    virtual int sync_bits() const = 0;
};

// ── POCSAG decoder ──────────────────────────────────────────────────────
class PocsagDecoder : public ProtoDecoder {
    enum State { SYNC_SEARCH, IN_BATCH };
    State state_ = SYNC_SEARCH;
    uint32_t shift_reg_ = 0;
    int batch_bit_count_ = 0;
    uint32_t codewords_[16] = {};
    DecoderOutput last_out_;
    int preamble_cnt_ = 0;

    int bch_correct(uint32_t& cw);
    DecoderOutput decode_batch();
public:
    bool feed_bit(uint8_t bit) override;
    DecoderOutput get_output() override { return last_out_; }
    void reset() override;
    uint64_t sync_word() const override { return 0x7CD215D8ULL; }
    int sync_bits() const override { return 32; }
};

// ── ACARS decoder ───────────────────────────────────────────────────────
class AcarsDecoder : public ProtoDecoder {
    enum State { SYNC_SEARCH, IN_FRAME };
    State state_ = SYNC_SEARCH;
    uint16_t shift16_ = 0;
    std::vector<uint8_t> frame_bits_;
    DecoderOutput last_out_;

    DecoderOutput decode_frame();
public:
    bool feed_bit(uint8_t bit) override;
    DecoderOutput get_output() override { return last_out_; }
    void reset() override;
    uint64_t sync_word() const override { return 0x2B2AULL; }
    int sync_bits() const override { return 16; }
};

// ── Generic hex fallback ────────────────────────────────────────────────
class GenericHexDecoder : public ProtoDecoder {
    uint8_t byte_val_ = 0;
    int bit_count_ = 0;
    int total_bytes_ = 0;
    DecoderOutput last_out_;
    char hex_buf_[512] = {};
    int hex_pos_ = 0;
public:
    bool feed_bit(uint8_t bit) override;
    DecoderOutput get_output() override { return last_out_; }
    void reset() override;
    uint64_t sync_word() const override { return 0; }
    int sync_bits() const override { return 0; }
};

// ── Factory ─────────────────────────────────────────────────────────────
std::unique_ptr<ProtoDecoder> create_decoder(int decoder_id);
