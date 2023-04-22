# pragma once

#include <string>
#include <string_view>
#include <vector>

namespace espp {
  /// A class to generate a JPEG header for a given image size and quantization tables.
  /// The header is generated once and then cached for future use.
  /// The header is generated according to the JPEG standard and is compatible with
  /// the ESP32 camera driver.
  class JpegHeader {
  public:
    /// Create a JPEG header for a given image size and quantization tables.
    /// @param width The image width in pixels.
    /// @param height The image height in pixels.
    /// @param q0_table The quantization table for the Y channel.
    /// @param q1_table The quantization table for the Cb and Cr channels.
    explicit JpegHeader(int width, int height, std::string_view q0_table, std::string_view q1_table)
      : width_(width), height_(height), q0_table_(q0_table), q1_table_(q1_table) {
      serialize();
    }

    ~JpegHeader() {}

    /// Get the image width.
    /// @return The image width in pixels.
    int get_width() const {
      return width_;
    }

    /// Get the image height.
    /// @return The image height in pixels.
    int get_height() const {
      return height_;
    }

    /// Get the JPEG header data.
    /// @return The JPEG header data.
    std::string_view get_data() {
      return std::string_view(data_.data(), data_.size());
    }

  protected:

    static constexpr int SOF0_SIZE = 19;
    static constexpr int DQT_HEADER_SIZE = 4;

    // JFIF APP0 Marker for version 1.2 with 72 DPI and no thumbnail
    static constexpr char JFIF_APP0_DATA[] = {
      0xFF, 0xE0, // APP0 marker
      0x00, 0x10, // Length of APP0 data (16 bytes)
      0x4A, 0x46, 0x49, 0x46, 0x00, // Identifier: ASCII "JFIF\0"
      0x01, 0x02, // Version number (1.2)
      0x01, // Units: 1 = dots per inch
      0x00, 0x48, // X density (72 DPI)
      0x00, 0x48, // Y density (72 DPI)
      0x00, 0x00 // No thumbnail
    };

    static constexpr char HUFFMAN_TABLES[] = {
      // 1st table
      // Default luminance DC Huffman table
      0xff, 0xc4, 0x00, 0x1f, 0x00, // header
      0x00, 0x01, 0x05, 0x01, 0x01,
      0x01, 0x01, 0x01, 0x01, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x01, 0x02, 0x03,
      0x04, 0x05, 0x06, 0x07, 0x08,
      0x09, 0x0a, 0x0b,
      // 2nd table
      // Default luminance AC Huffman table
      0xff, 0xc4, 0x00, 0xb5, 0x10, // header
      0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d, 0x01, 0x02,
      0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71,
      0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33,
      0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a,
      0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53,
      0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73,
      0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92,
      0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
      0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
      0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4,
      0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
    };

    // Scan header (SOS)
    static constexpr char SOS[] = {
      0xFF, 0xDA, // SOS marker
      0x00, 0x0C, // length
      0x03, // number of components
      0x01, 0x00, 0x02, 0x11, 0x03, 0x11, 0x00, 0x3F, 0x00 // components
    };


    int add_sof0(int offset) {
      // add the SOF0 marker
      data_[offset++] = 0xFF;
      data_[offset++] = 0xC0;
      // add the length of the marker
      data_[offset++] = 0x00;
      data_[offset++] = 0x11;
      // add the precision
      data_[offset++] = 0x08;
      // add the height
      data_[offset++] = (height_ >> 8) & 0xFF;
      data_[offset++] = height_ & 0xFF;
      // add the width
      data_[offset++] = (width_ >> 8) & 0xFF;
      data_[offset++] = width_ & 0xFF;
      // add the number of components
      data_[offset++] = 0x03;
      // add the Y component
      data_[offset++] = 0x01;
      data_[offset++] = 0x21;
      data_[offset++] = 0x00;
      // add the Cb component
      data_[offset++] = 0x02;
      data_[offset++] = 0x11;
      data_[offset++] = 0x01;
      // add the Cr component
      data_[offset++] = 0x03;
      data_[offset++] = 0x11;
      data_[offset++] = 0x01;
      return offset;
    }

    void serialize() {
      int header_size =
        2 +
        sizeof(JFIF_APP0_DATA) +
        DQT_HEADER_SIZE +
        q0_table_.size() +
        DQT_HEADER_SIZE +
        q1_table_.size() +
        SOF0_SIZE +
        sizeof(HUFFMAN_TABLES) +
        sizeof(SOS);
      // serialize the jpeg header to the data_ vector
      data_.resize(header_size);
      int offset = 0;

      // add the SOI marker
      data_[offset++] = 0xFF;
      data_[offset++] = 0xD8;

      // add the JFIF APP0 marker
      memcpy(data_.data() + offset, JFIF_APP0_DATA, sizeof(JFIF_APP0_DATA));
      offset += sizeof(JFIF_APP0_DATA);

      // add the DQT marker for luminance
      data_[offset++] = 0xFF;
      data_[offset++] = 0xDB;
      data_[offset++] = 0x00;
      data_[offset++] = 0x43;
      data_[offset++] = 0x00;
      memcpy(data_.data() + offset, q0_table_.data(), q0_table_.size());
      offset += q0_table_.size();

      // add the DQT marker for chrominance
      data_[offset++] = 0xFF;
      data_[offset++] = 0xDB;
      data_[offset++] = 0x00;
      data_[offset++] = 0x43;
      data_[offset++] = 0x01;
      memcpy(data_.data() + offset, q1_table_.data(), q1_table_.size());
      offset += q1_table_.size();

      // add the SOF0
      offset = add_sof0(offset);

      // add huffman tables
      memcpy(data_.data() + offset, HUFFMAN_TABLES, sizeof(HUFFMAN_TABLES));
      offset += sizeof(HUFFMAN_TABLES);

      // add the SOS marker
      memcpy(data_.data() + offset, SOS, sizeof(SOS));
      offset += sizeof(SOS);
    }

    int width_;
    int height_;
    std::string_view q0_table_;
    std::string_view q1_table_;

    std::vector<char> data_;
  };
} // namespace espp
