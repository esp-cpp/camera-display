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
      // add the jpeg header
      serialize_header();
      // add the jpeg data
      add_scan(packet);
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

    /// Check if the frame is complete.
    /// @return True if the frame is complete, false otherwise.
    bool is_complete() const {
      return finalized_;
    }

    /// Append a RtpJpegPacket to the frame.
    /// This will add the JPEG data to the frame.
    /// @param packet The packet containing the scan to append.
    void append(const RtpJpegPacket& packet) {
      add_scan(packet);
    }

    /// Append a JPEG scan to the frame.
    /// This will add the JPEG data to the frame.
    /// @note If the packet contains the EOI marker, the frame will be
    ///       finalized, and no further scans can be added.
    /// @param packet The packet containing the scan to append.
    void add_scan(const RtpJpegPacket& packet) {
      add_scan(packet.get_jpeg_data());
      if (packet.get_marker()) {
        finalize();
      }
    }

    /// Get the serialized data.
    /// This will return the serialized data.
    /// @return The serialized data.
    std::string_view get_data() const {
      return std::string_view(data_.data(), data_.size());
    }

  protected:
    /// Serialize the header.
    void serialize_header() {
      auto header_data = header_.get_data();
      data_.resize(header_data.size());
      memcpy(data_.data(), header_data.data(), header_data.size());
    }

    /// Append a JPEG scan to the frame.
    /// This will add the JPEG data to the frame.
    /// @param scan The jpeg scan to append.
    void add_scan(std::string_view scan) {
      if (finalized_) {
        // TODO: handle this error
        return;
      }
      data_.insert(std::end(data_), std::begin(scan), std::end(scan));
    }

    /// Add the EOI marker to the frame.
    /// This will add the EOI marker to the frame. This must be called before
    /// calling get_data().
    /// @note This will prevent any further scans from being added to the frame.
    void finalize() {
      if (!finalized_) {
        finalized_ = true;
        // add_eoi();
      } else {
        // TODO: handle this error
        // already finalized
      }
    }

    /// Add the EOI marker to the frame.
    void add_eoi() {
      data_.push_back(0xFF);
      data_.push_back(0xD9);
    }

    JpegHeader header_;
    bool finalized_ = false;
    std::vector<char> data_;
  };
} // namespace espp
