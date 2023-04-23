#pragma once

#include "rtp_packet.hpp"

namespace espp {
  class RtpJpegPacket : public RtpPacket {
  public:
    explicit RtpJpegPacket(std::string_view data) : RtpPacket(data) {
      parse_mjpeg_header();
    }

    ~RtpJpegPacket() {}

    int get_type_specific() const { return type_specific_; }
    int get_offset() const { return offset_; }
    int get_q() const { return q_; }
    int get_width() const { return width_; }
    int get_height() const { return height_; }

    int get_mjpeg_header_size() const { return MJPEG_HEADER_SIZE; }
    std::string_view get_mjpeg_header() {
      return std::string_view(get_payload().data(), MJPEG_HEADER_SIZE);
    }

    int get_q_table_size() const { return Q_TABLE_SIZE; }
    int get_num_q_tables() const { return q_tables_.size(); }
    std::string_view get_q_table(int index) const {
      if (index < get_num_q_tables()) {
        return q_tables_[index];
      }
      return {};
    }

    std::string_view get_jpeg_data() const {
      auto payload = get_payload();
      return std::string_view(payload.data() + jpeg_data_start_, jpeg_data_size_);
    }


  protected:
    static constexpr int MJPEG_HEADER_SIZE = 8;
    static constexpr int QUANT_HEADER_SIZE = 4;
    static constexpr int NUM_Q_TABLES = 2;
    static constexpr int Q_TABLE_SIZE = 64;

    void parse_mjpeg_header() {
      auto payload = get_payload();
      type_specific_ = payload[0];
      offset_ = (payload[1] << 16) | (payload[2] << 8) | payload[3];
      frag_type_ = payload[4];
      q_ = payload[5];
      width_ = payload[6] * 8;
      height_ = payload[7] * 8;

      size_t offset = MJPEG_HEADER_SIZE;

      // If the Q value is between 128 and 256, then the packet contains
      // quantization tables.
      if (128 <= q_ && q_ <= 256) {
        int num_quant_bytes = payload[11];
        int expected_num_quant_bytes = NUM_Q_TABLES * Q_TABLE_SIZE;
        if (num_quant_bytes == expected_num_quant_bytes) {
          q_tables_.resize(NUM_Q_TABLES);
          offset += QUANT_HEADER_SIZE;
          for (int i = 0; i < NUM_Q_TABLES; i++) {
            q_tables_[i] = std::string_view(payload.data() + offset, Q_TABLE_SIZE);
            offset += Q_TABLE_SIZE;
          }
        }
      }

      jpeg_data_start_ = offset;
      jpeg_data_size_ = payload.size() - jpeg_data_start_;
    }

    int type_specific_;
    int offset_;
    int frag_type_;
    int q_{0};
    int width_{0};
    int height_{0};
    int jpeg_data_start_{0};
    int jpeg_data_size_{0};
    std::vector<std::string_view> q_tables_;
  };
} // namespace espp
