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
      return std::string_view(packet_.data() + RTP_HEADER_SIZE, MJPEG_HEADER_SIZE);
    }

    int get_q_table_size() const { return Q_TABLE_SIZE; }
    int get_num_q_tables() const { return q_table_indices_.size(); }
    std::string_view get_q_table(int index) const {
      if (index < get_num_q_tables()) {
        return std::string_view(packet_.data() + q_table_indices_[index], Q_TABLE_SIZE);
      }
      return {};
    }

    std::string_view get_jpeg_data() const {
      int quant_size = get_num_q_tables() * Q_TABLE_SIZE;
      return std::string_view(packet_.data() + RTP_HEADER_SIZE + MJPEG_HEADER_SIZE + quant_size,
                              packet_.size() - RTP_HEADER_SIZE - MJPEG_HEADER_SIZE - quant_size);
    }


  protected:
    static constexpr int MJPEG_HEADER_SIZE = 8;
    static constexpr int NUM_Q_TABLES = 2;
    static constexpr int Q_TABLE_SIZE = 64;

    void parse_mjpeg_header() {
      int offset = RTP_HEADER_SIZE;
      type_specific_ = packet_[offset];
      offset_ = (packet_[offset + 1] << 16) | (packet_[offset + 2] << 8) | packet_[offset + 3];
      frag_type_ = packet_[offset + 4];
      q_ = packet_[offset + 5];
      width_ = packet_[offset + 6] * 8;
      height_ = packet_[offset + 7] * 8;

      // If the Q value is between 128 and 256, then the packet contains
      // quantization tables.
      if (128 <= q_ && q_ <= 256) {
        int num_quant_bytes = packet_[offset + 11];
        int expected_num_quant_bytes = NUM_Q_TABLES * Q_TABLE_SIZE;
        if (num_quant_bytes == expected_num_quant_bytes) {
          q_table_indices_.resize(NUM_Q_TABLES);
          for (int i = 0; i < NUM_Q_TABLES; i++) {
            q_table_indices_[i] = offset + 12 + (i * Q_TABLE_SIZE);
          }
        }
      }
    }

    int type_specific_;
    int offset_;
    int frag_type_;
    int q_;
    int width_;
    int height_;
    std::vector<int> q_table_indices_;
  };
} // namespace espp
