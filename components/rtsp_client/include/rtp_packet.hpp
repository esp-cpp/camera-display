#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace espp {
  /// RtpPacket is a class to parse RTP packet.
  class RtpPacket {
  public:
    /// Construct an RtpPacket from a string_view.
    /// Store the string_view in the packet_ vector and parses the header.
    /// @param data The string_view to parse.
    explicit RtpPacket(std::string_view data) {
      packet_.assign(data.begin(), data.end());
      parse_rtp_header();
    }

    ~RtpPacket() {}

    /// Getters for the RTP header fields.
    int get_version() const { return version_; }
    bool get_padding() const { return padding_; }
    bool get_extension() const { return extension_; }
    int get_csrc_count() const { return csrc_count_; }
    bool get_marker() const { return marker_; }
    int get_payload_type() const { return payload_type_; }
    int get_sequence_number() const { return sequence_number_; }
    int get_timestamp() const { return timestamp_; }
    int get_ssrc() const { return ssrc_; }

    /// Get a string_view of the whole packet.
    /// @return A string_view of the whole packet.
    std::string_view get_data() const {
      return std::string_view(packet_.data(), packet_.size());
    }

    /// Get a string_view of the RTP header.
    /// @return A string_view of the RTP header.
    std::string_view get_packet_header() const {
      return std::string_view(packet_.data(), RTP_HEADER_SIZE);
    }

    /// Get a string_view of the payload.
    /// @return A string_view of the payload.
    std::string_view get_payload() const {
      return std::string_view(packet_.data() + RTP_HEADER_SIZE, payload_size_);
    }

  protected:
    static constexpr int RTP_HEADER_SIZE = 12;

    void parse_rtp_header() {
      version_ = (packet_[0] & 0xC0) >> 6;
      padding_ = (packet_[0] & 0x20) >> 5;
      extension_ = (packet_[0] & 0x10) >> 4;
      csrc_count_ = packet_[0] & 0x0F;
      marker_ = (packet_[1] & 0x80) >> 7;
      payload_type_ = packet_[1] & 0x7F;
      sequence_number_ = (packet_[2] << 8) | packet_[3];
      timestamp_ = (packet_[4] << 24) | (packet_[5] << 16) | (packet_[6] << 8) | packet_[7];
      ssrc_ = (packet_[8] << 24) | (packet_[9] << 16) | (packet_[10] << 8) | packet_[11];
      payload_size_ = packet_.size() - RTP_HEADER_SIZE;
    }

    std::vector<char> packet_;
    int version_;
    bool padding_;
    bool extension_;
    int csrc_count_;
    bool marker_;
    int payload_type_;
    int sequence_number_;
    int timestamp_;
    int ssrc_;
    int payload_size_;
  };
}  // namespace espp
