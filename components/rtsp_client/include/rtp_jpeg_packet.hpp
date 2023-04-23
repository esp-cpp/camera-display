#pragma once

#include "rtp_packet.hpp"

namespace espp {
  /// RTP packet for JPEG video.
  /// The RTP payload for JPEG is defined in RFC 2435.
  class RtpJpegPacket : public RtpPacket {
  public:
    /// Construct an RTP packet from a buffer.
    /// @param data The buffer containing the RTP packet.
    explicit RtpJpegPacket(std::string_view data) : RtpPacket(data) {
      parse_mjpeg_header();
    }

    ~RtpJpegPacket() {}

    /// Get the type-specific field.
    /// @return The type-specific field.
    int get_type_specific() const { return type_specific_; }

    /// Get the offset field.
    /// @return The offset field.
    int get_offset() const { return offset_; }

    /// Get the fragment type field.
    /// @return The fragment type field.
    int get_q() const { return q_; }

    /// Get the fragment type field.
    /// @return The fragment type field.
    int get_width() const { return width_; }

    /// Get the fragment type field.
    /// @return The fragment type field.
    int get_height() const { return height_; }

    /// Get the mjepg header.
    /// @return The mjepg header.
    std::string_view get_mjpeg_header() {
      return std::string_view(get_payload().data(), MJPEG_HEADER_SIZE);
    }

    /// Get whether the packet contains quantization tables.
    /// @note The quantization tables are optional. If they are present, the
    /// number of quantization tables is always 2.
    /// @note This check is based on the value of the q field. If the q field
    ///       is 128-256, the packet contains quantization tables.
    /// @return Whether the packet contains quantization tables.
    bool has_q_tables() const { return q_ >= 128 && q_ <= 256; }

    /// Get the number of quantization tables.
    /// @note The quantization tables are optional. If they are present, the
    /// number of quantization tables is always 2.
    /// @note Only the first packet in a frame contains quantization tables.
    /// @return The number of quantization tables.
    int get_num_q_tables() const { return q_tables_.size(); }

    /// Get the quantization table at the specified index.
    /// @param index The index of the quantization table.
    /// @return The quantization table at the specified index.
    std::string_view get_q_table(int index) const {
      if (index < get_num_q_tables()) {
        return q_tables_[index];
      }
      return {};
    }

    /// Get the JPEG data.
    /// The jpeg data is the payload minus the mjpeg header and quantization
    /// tables.
    /// @return The JPEG data.
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

      if (has_q_tables()) {
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
