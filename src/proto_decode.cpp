#include "proto_decode.hpp"
#include <cstdio>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────
// POCSAG Decoder
// ─────────────────────────────────────────────────────────────────────────

static constexpr uint32_t POCSAG_SYNC   = 0x7CD215D8;
static constexpr uint32_t POCSAG_IDLE   = 0x7A89C197;

void PocsagDecoder::reset()
{
    state_ = SYNC_SEARCH;
    shift_reg_ = 0;
    batch_bit_count_ = 0;
    preamble_cnt_ = 0;
    memset(codewords_, 0, sizeof(codewords_));
}

int PocsagDecoder::bch_correct(uint32_t& cw)
{
    // BCH(31,21,5) syndrome check on bits 31..1 (bit 0 = even parity)
    uint32_t data = cw >> 1;
    static constexpr uint32_t GEN = 0x769; // x^10+x^8+x^6+x^5+x^3+x^0

    uint32_t temp = data;
    for (int i = 30; i >= 10; i--) {
        if (temp & (1u << i))
            temp ^= (GEN << (i - 10));
    }
    uint32_t syndrome = temp & 0x3FF;
    if (syndrome == 0) return 0;

    // Try single-bit error correction
    for (int i = 0; i < 31; i++) {
        uint32_t test = cw ^ (1u << (i + 1));
        uint32_t td = test >> 1;
        uint32_t t = td;
        for (int j = 30; j >= 10; j--) {
            if (t & (1u << j)) t ^= (GEN << (j - 10));
        }
        if ((t & 0x3FF) == 0) {
            cw = test;
            return 1;
        }
    }
    return -1; // uncorrectable
}

DecoderOutput PocsagDecoder::decode_batch()
{
    DecoderOutput out;
    int errors_total = 0;
    uint32_t address = 0;
    int func = 0;
    bool in_message = false;
    std::string alpha_msg, num_msg;

    for (int i = 0; i < 16; i++) {
        uint32_t cw = codewords_[i];
        int ec = bch_correct(cw);
        if (ec < 0) { errors_total++; continue; }
        errors_total += ec;

        // Even parity check
        uint32_t p = cw;
        p ^= p >> 16; p ^= p >> 8; p ^= p >> 4;
        p ^= p >> 2; p ^= p >> 1;
        if (p & 1) errors_total++;

        if (cw == POCSAG_IDLE) { in_message = false; continue; }

        bool is_addr = !(cw & 0x80000000u);

        if (is_addr) {
            // Emit previous
            if (in_message && (!alpha_msg.empty() || !num_msg.empty())) {
                char line[256];
                const char* msg = alpha_msg.empty() ? num_msg.c_str() : alpha_msg.c_str();
                snprintf(line, sizeof(line), "ADDR=%u F=%d: %s", address, func, msg);
                if (!out.detail.empty()) out.detail += "\n";
                out.detail += line;
                if (out.summary.empty()) out.summary = line;
            }
            address = ((cw >> 13) & 0x3FFFF) * 8 + (i / 2);
            func = (cw >> 11) & 0x3;
            in_message = true;
            alpha_msg.clear(); num_msg.clear();
        } else if (in_message) {
            uint32_t data = (cw >> 11) & 0xFFFFF;
            if (func == 0) {
                // Numeric: BCD
                for (int d = 0; d < 5; d++) {
                    int digit = (data >> (16 - d*4)) & 0xF;
                    static const char bcd[] = "0123456789*U -)(";
                    num_msg += bcd[digit & 0xF];
                }
            } else {
                // Alphanumeric: 7-bit packed
                for (int b = 19; b >= 6; b -= 7) {
                    int ch = 0;
                    for (int k = 0; k < 7 && (b-k) >= 0; k++)
                        ch |= ((data >> (b-k)) & 1) << k;
                    if (ch >= 32 && ch < 127)
                        alpha_msg += (char)ch;
                    else if (ch == '\n' || ch == '\r')
                        alpha_msg += ' ';
                }
            }
        }
    }

    // Emit final
    if (in_message && (!alpha_msg.empty() || !num_msg.empty())) {
        char line[256];
        const char* msg = alpha_msg.empty() ? num_msg.c_str() : alpha_msg.c_str();
        snprintf(line, sizeof(line), "ADDR=%u F=%d: %s", address, func, msg);
        if (!out.detail.empty()) out.detail += "\n";
        out.detail += line;
        if (out.summary.empty()) out.summary = line;
    }

    out.valid = (errors_total < 8 && !out.summary.empty());
    out.error_count = errors_total;
    return out;
}

bool PocsagDecoder::feed_bit(uint8_t bit)
{
    shift_reg_ = (shift_reg_ << 1) | (bit & 1);

    if (state_ == SYNC_SEARCH) {
        // Detect preamble (10101010... pattern)
        if ((shift_reg_ & 0x3) == 0x2 || (shift_reg_ & 0x3) == 0x1)
            preamble_cnt_++;
        else
            preamble_cnt_ = 0;

        if (shift_reg_ == POCSAG_SYNC) {
            state_ = IN_BATCH;
            batch_bit_count_ = 0;
            memset(codewords_, 0, sizeof(codewords_));
            return false;
        }
    } else { // IN_BATCH
        int cw_bit = batch_bit_count_ % 32;
        int cw_num = batch_bit_count_ / 32;

        if (cw_num < 16) {
            codewords_[cw_num] |= ((uint32_t)(bit & 1)) << (31 - cw_bit);
        }
        batch_bit_count_++;

        if (batch_bit_count_ >= 512) {
            last_out_ = decode_batch();
            state_ = SYNC_SEARCH;
            return last_out_.valid;
        }
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────
// ACARS Decoder
// ─────────────────────────────────────────────────────────────────────────

void AcarsDecoder::reset()
{
    state_ = SYNC_SEARCH;
    shift16_ = 0;
    frame_bits_.clear();
}

DecoderOutput AcarsDecoder::decode_frame()
{
    DecoderOutput out;
    std::string text;
    for (size_t i = 0; i + 7 < frame_bits_.size(); i += 8) {
        uint8_t ch = 0;
        for (int b = 0; b < 7; b++)
            ch |= (frame_bits_[i+b] & 1) << b;
        if (ch >= 32 && ch < 127) text += (char)ch;
        else if (ch == '\r' || ch == '\n') text += ' ';
    }
    out.detail = text;
    out.summary = "ACARS: " + text.substr(0, std::min(text.size(), (size_t)80));
    out.valid = text.size() > 5;
    return out;
}

bool AcarsDecoder::feed_bit(uint8_t bit)
{
    shift16_ = (shift16_ << 1) | (bit & 1);

    if (state_ == SYNC_SEARCH) {
        if ((shift16_ & 0xFFFF) == 0x2B2A) {
            state_ = IN_FRAME;
            frame_bits_.clear();
            return false;
        }
    } else {
        frame_bits_.push_back(bit);
        if (frame_bits_.size() >= 8) {
            uint8_t last_byte = 0;
            for (int i = 0; i < 8; i++)
                last_byte |= (frame_bits_[frame_bits_.size()-8+i] & 1) << i;
            if (last_byte == 0x7F && frame_bits_.size() > 40) {
                last_out_ = decode_frame();
                state_ = SYNC_SEARCH;
                return last_out_.valid;
            }
        }
        if (frame_bits_.size() > 2000) state_ = SYNC_SEARCH;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────
// Generic Hex Decoder
// ─────────────────────────────────────────────────────────────────────────

void GenericHexDecoder::reset()
{
    byte_val_ = 0; bit_count_ = 0; total_bytes_ = 0; hex_pos_ = 0;
}

bool GenericHexDecoder::feed_bit(uint8_t bit)
{
    byte_val_ = (byte_val_ << 1) | (bit & 1);
    bit_count_++;
    if (bit_count_ >= 8) {
        hex_pos_ += snprintf(hex_buf_ + hex_pos_,
                            sizeof(hex_buf_) - hex_pos_,
                            "%02X ", byte_val_);
        byte_val_ = 0; bit_count_ = 0;
        total_bytes_++;
        if (total_bytes_ >= 16) {
            hex_buf_[hex_pos_] = '\0';
            last_out_.summary = hex_buf_;
            last_out_.detail = hex_buf_;
            last_out_.valid = true;
            hex_pos_ = 0; total_bytes_ = 0;
            return true;
        }
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────
// Factory
// ─────────────────────────────────────────────────────────────────────────

std::unique_ptr<ProtoDecoder> create_decoder(int decoder_id)
{
    switch (decoder_id) {
    case 1:  return std::make_unique<PocsagDecoder>();
    case 2:  return std::make_unique<AcarsDecoder>();
    default: return std::make_unique<GenericHexDecoder>();
    }
}
