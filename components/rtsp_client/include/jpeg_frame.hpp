#pragma once

#include "rtp_jpeg_packet.hpp"
#include "jpeg_header.hpp"

namespace espp {
  /// A class that represents a complete JPEG frame.
  ///
  /// This class is used to collect the JPEG scans that are received in RTP
  /// packets and to serialize them into a complete JPEG frame.
  class JpegFrame {
  public:

    /// Construct a JpegFrame from a RtpJpegPacket.
    ///
    /// This constructor will parse the header of the packet and add the JPEG
    /// data to the frame.
    ///
    /// @param packet The packet to parse.
    explicit JpegFrame(const RtpJpegPacket& packet)
      : header_(packet.get_width(), packet.get_height(), packet.get_q_table(0), packet.get_q_table(1)) {
      // add the jpeg data
      scans_.push_back(packet.get_jpeg_data());
    }

    /// Get the width of the frame.
    /// @return The width of the frame.
    int get_width() const {
      return header_.get_width();
    }

    /// Get the height of the frame.
    /// @return The height of the frame.
    int get_height() const {
      return header_.get_height();
    }

    /// Append a RtpJpegPacket to the frame.
    /// This will add the JPEG data to the frame.
    /// @param packet The packet containing the scan to append.
    void append(const RtpJpegPacket& packet) {
      add_scan(packet);
    }

    /// Append a JPEG scan to the frame.
    /// This will add the JPEG data to the frame.
    /// @param packet The packet containing the scan to append.
    void add_scan(const RtpJpegPacket& packet) {
      add_scan(packet.get_jpeg_data());
    }

    /// Append a JPEG scan to the frame.
    /// This will add the JPEG data to the frame.
    /// @param scan The jpeg scan to append.
    void add_scan(std::string_view scan) {
      scans_.push_back(scan);
    }

    /// Serialize the frame.
    ///
    /// This will serialize the header and all scans into a single buffer which
    /// can be sent over the network. You can get the serialized data using
    /// get_data().
    ///
    /// @note This method must be called before get_data() can be used.
    /// @note This method should only be called once.
    void serialize() {
      auto header_data = header_.get_data();
      auto scan_bytes = 0;
      for (auto& scan : scans_) {
        scan_bytes += scan.size();
      }
      data_.resize(header_data.size() + scan_bytes + 2);;

      int offset = 0;
      // serialize the header
      memcpy(data_.data(), header_data.data(), header_data.size());
      offset += header_data.size();
      // serialize the scans
      for (auto& scan : scans_) {
        memcpy(data_.data() + offset, scan.data(), scan.size());
        offset += scan.size();
      }
      // add the EOI marker
      data_[offset++] = 0xFF;
      data_[offset++] = 0xD9;
    }

    /// Get the serialized data.
    ///
    /// This will return the serialized data. You must call serialize() before
    /// calling this method.
    ///
    /// @return The serialized data.
    std::string_view get_data() const {
      return std::string_view(data_.data(), data_.size());
    }

  protected:
    JpegHeader header_;
    std::vector<std::string_view> scans_;
    std::vector<char> data_;
  };
} // namespace espp
