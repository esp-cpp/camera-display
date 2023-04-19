import socket
import sys
import threading

import cv2
import io
import struct
import numpy as np

import io

'''

NOTE: This code is designed to handle MJPEG video streams over RTP/RTCP.

Some useful references:
* https://www.rfc-editor.org/rfc/rfc2435
  RTP Payload Format for JPEG-compressed Video
* https://en.wikipedia.org/wiki/JPEG_File_Interchange_Format
  JFIF - the JPEG File Interchange Format
* https://en.wikipedia.org/wiki/JPEG
  Wikipedia page for JPEG

The huffman tables are not transferred with the images, but the quantization
tables are. The rest of the JPEG header/data is stripped from the stream, such
that to properly display it, you need to rebuild the jpeg header data based on
the simplified RTP & JFIF header data. Once you've done that, the jpeg frames
can be decoded properly.

Somewhat unrelated, you can convert mp4 to mjpeg and mp3:

```bash
ffmpeg -i input.mp4 -vf "fps=30,scale=-1:176:flags=lanczos,crop=220:in_h:(in_w-220)/2:0" -q:v 9 220_30fps.mjpeg
# for MP3
ffmpeg -i input.mp4 -ar 44100 -ac 1 -q:a 9 44100.mp3
# for PCM
ffmpeg -i input.mp4 -f u16be -acodec pcm_u16le -ar 44100 -ac 1 44100_u16le.pcm
```

'''

huffman_table = [
    # 1st table
    # Default luminance DC Huffman table
    0xff, 0xc4, 0x00, 0x1f, 0x00, # header
    0x00, 0x01, 0x05, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0a, 0x0b,
    # 2nd table
    # Default luminance AC Huffman table
    0xff, 0xc4, 0x00, 0xb5, 0x10, # header
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

]

class RtspClient:
    def __init__(self, server, port):
        self.server = server
        self.port = port
        self.cseq = 0
        self.session_id = ""

    def connect(self):
        self.sock = socket.create_connection((self.server, self.port))
        self.send_request("OPTIONS", "*")

    def send_request(self, method, uri, headers=None):
        if headers is None:
            headers = {}

        request = f"{method} {uri} RTSP/1.0\r\n"
        request += f"CSeq: {self.cseq}\r\n"
        if self.session_id:
            request += f"Session: {self.session_id}\r\n"

        for key, value in headers.items():
            request += f"{key}: {value}\r\n"

        request += "User-Agent: RtspClient\r\n"
        request += "\r\n"

        self.sock.sendall(request.encode())
        response = self.sock.recv(4096)
        print("Response:", response.decode())

        self.cseq += 1

    def describe(self, uri):
        self.send_request("DESCRIBE", uri, {"Accept": "application/sdp"})

    def setup(self, uri, transport):
        self.send_request("SETUP", uri, {"Transport": transport})

    def play(self, uri):
        self.send_request("PLAY", uri)

    def pause(self, uri):
        self.send_request("PAUSE", uri)

    def teardown(self, uri):
        self.send_request("TEARDOWN", uri)

    def start_receiving_video_stream(self, rtp_port, rtcp_port):
        self.rtp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rtp_socket.bind(("0.0.0.0", rtp_port))

        self.rtcp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rtcp_socket.bind(("0.0.0.0", rtcp_port))

        print(f"Listening for RTP packets on port {rtp_port}")
        print(f"Listening for RTCP packets on port {rtcp_port}")

        # NOTE: right now the rtp_thread must be run in the main thread context
        #       (mac os cannot run cv2.imshow in another thread), and we are not
        #       receiving any RTCP packets, so we've simply stopped spawning
        #       those threads and are instead direclty running the rtp_thread's
        #       function

        # TODO: get threaded cv2.imshow working (or have it simply update the
        #       frame and have opencv show happen in main thread context)

        # rtp_thread = threading.Thread(target=self.handle_rtp_packet)
        # rtcp_thread = threading.Thread(target=self.handle_rtcp_packet)

        # rtp_thread.start()
        # rtcp_thread.start()

        # rtp_thread.join()
        # rtcp_thread.join()

        self.handle_rtp_packet()

    def handle_rtp_packet(self):
        # for this example we'll show the received video stream in an opencv
        # window
        buf = bytearray()
        cv2.namedWindow('MJPEG Stream', cv2.WINDOW_NORMAL)

        while True:
            # Process RTP packet in rtp_data
            rtp_data, addr = self.rtp_socket.recvfrom(8192)
            print(f"received rtp packet, len={len(rtp_data)}")
            rtp_header = rtp_data[:12]
            rtp_payload = rtp_data[12:] # NAL unit


            version, payload_and_marker, seq, timestamp, ssrc = struct.unpack('>BBHII', rtp_header)
            payload_type = (payload_and_marker & 0x7F)
            marker_bit = (payload_and_marker & 0b10000000) >> 7

            # print("\tRTP Header:", [hex(x) for x in list(rtp_header)])
            print(f"\tMarker: {marker_bit}, Payload Type: {payload_type}")
            # print(f"Sequence number: {seq}, Timestamp: {timestamp}, SSRC: {ssrc}")

            jpeg_header = rtp_payload[:8]
            type_specific, frag_offset, frag_type, q, width, height = struct.unpack('>B3s BBBB', rtp_payload[:8])
            frag_offset = int.from_bytes(frag_offset, byteorder='big')
            q = int(q)
            width = int(width) * 8
            height = int(height) * 8

            # print("\tJPEG header:", [hex(x) for x in list(jpeg_header)])
            print(f"\tJpeg image: width={width}, height={height}, type_spec={type_specific}, q={q}, frag_type={frag_type}, frag_offset={frag_offset}")

            jpeg_data = rtp_payload[8:]

            if 64 <= frag_type <= 127:
                # there must be a restart marker header
                print("\tHave restart header")
                restart_header = rtp_payload[8:12]
                restart_interval = int((restart_header[0] << 8) | restart_header[1])
                f_bit = True if restart_header[2] & 0x80 else False
                l_bit = True if restart_header[2] & 0x80 else False
                restart_count = int(((restart_header[2] & 0x3F) << 8) | restart_header[3])
                print(f"\tRestart interval={restart_interval}, f={f_bit}, l={l_bit}, restart_count={restart_count}")
                jpeg_data = rtp_payload[12:]

            if 128 <= q <= 255:
                print("\tGetting quantization table header")
                # bytes 8,9,10 are all 0 based on CStreamer.cpp lines 109-111
                num_quant_bytes = rtp_payload[11]
                quant_size = 64
                expected_quant_bytes = 2 * quant_size
                if num_quant_bytes != expected_quant_bytes:
                    print(f"Unexpected quant bytes: {num_quant_bytes}, expected {expected_quant_bytes}")
                else:
                    q0_offset = 12
                    q1_offset = q0_offset + quant_size
                    q1_end = q1_offset + quant_size
                    q0 = rtp_payload[q0_offset:q1_offset]
                    q1 = rtp_payload[q1_offset:q1_end]
                    jpeg_data = rtp_payload[q1_end:]

            if frag_offset == 0:
                # Create a binary stream to construct the JPEG header
                jpeg_header = io.BytesIO()

                # Start Of Image (SOI) marker
                jpeg_header.write(b'\xFF\xD8')

                jfif_app0_marker = bytearray([
                    0xFF, 0xE0,  # APP0 marker
                    0x00, 0x10,  # Length (16 bytes)
                    0x4A, 0x46, 0x49, 0x46, 0x00,  # JFIF identifier
                    0x01, 0x02,  # JFIF version 1.2
                    0x01,        # Units: DPI
                    0x00, 0x48,  # Xdensity: 72 DPI
                    0x00, 0x48,  # Ydensity: 72 DPI
                    0x00, 0x00   # No thumbnail (width 0, height 0)
                ])
                jpeg_header.write(jfif_app0_marker)

                # Quantization table (DQT) marker for luminance
                # marker(0xFFDB), size (0x0043 = 67), index (0x00)
                jpeg_header.write(b'\xFF\xDB\x00\x43\x00')
                jpeg_header.write(bytearray(q0))

                # Quantization table (DQT) marker for chrominance
                # marker(0xFFDB), size (0x0043 = 67), index (0x01)
                jpeg_header.write(b'\xFF\xDB\x00\x43\x01')
                jpeg_header.write(bytearray(q1))

                # Frame header (SOF0) marker
                sof0_marker = bytearray([
                    0xFF, 0xC0,  # SOF0 marker
                    0x00, 0x11,  # Length (17 bytes)
                    0x08,        # Data precision: 8 bits
                    *height.to_bytes(2, 'big'), # 0x01, 0xE0,  # Image height: 240
                    *width.to_bytes(2, 'big'), # 0x01, 0xE0,  # Image width: 240
                    0x03,        # Number of components: 3 (YCbCr)
                    0x01, 0x21, 0x00,  # Component 1 (Y):  horizontal sampling factor = 2, vertical sampling factor = 1, quantization table ID = 0
                    0x02, 0x11, 0x01,  # Component 2 (Cb): horizontal sampling factor = 1, vertical sampling factor = 1, quantization table ID = 1
                    0x03, 0x11, 0x01   # Component 3 (Cr): horizontal sampling factor = 1, vertical sampling factor = 1, quantization table ID = 1
                ])
                jpeg_header.write(sof0_marker)

                jpeg_header.write(bytes(huffman_table))

                # Scan header (SOS) marker
                # marker(0xFFDA), size of SOS (0x000C), num components(0x03),
                # component specification parameters,
                # spectral selection (0x003F),
                # successive appromiation parameters (0x00)
                jpeg_header.write(b'\xFF\xDA\x00\x0C\x03\x01\x00\x02\x11\x03\x11\x00\x3F\x00')

                jpeg_header_bytes = bytearray(jpeg_header.getvalue())

                print(f"\tAdded header of length {len(jpeg_header_bytes)}")

                buf = jpeg_header_bytes

            # make sure we add the actual jpeg segment data
            buf.extend(jpeg_data)

            if marker_bit:
                # Add the JPEG end marker (EOI)
                buf.extend(b'\xFF\xD9')
                print(f"Decoding image size={len(buf)}")
                frame = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    print(f"Decoded frame: {frame.shape}\n\n")
                    # our images are flipped vertically, fix it :)
                    # 0 = vertical, 1 = horizontal, -1 = both vertical and horiztonal
                    frame = cv2.flip(frame, 0)
                    cv2.imshow('MJPEG Stream', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

    def handle_rtcp_packet(self):
        while True:
            rtcp_data, addr = self.rtcp_socket.recvfrom(8192)
            print("Received rtcp packet:", rtcp_data)
            # Process RTCP packet in rtcp_data
            # ...
            # The handle_rtcp_packet function currently does nothing, but you
            # can implement it to process RTCP packets, such as sender reports
            # or receiver reports, depending on your application requirements.

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python rtsp_client.py <server> <port>")
        sys.exit(1)

    server, port = sys.argv[1], int(sys.argv[2])
    client = RtspClient(server, port)
    client.connect()

    # Replace the following line with the actual RTSP URI you want to stream
    rtsp_uri = f"rtsp://{server}:{port}/mjpeg/1"

    # Call DESCRIBE method to get SDP information
    client.describe(rtsp_uri)

    # The following lines are placeholders for RTP and RTCP ports
    # You should parse the RTSP SETUP response and set the RTP and RTCP ports accordingly
    rtp_port = 5000
    rtcp_port = 5001

    # Set up the transport header with the RTP and RTCP ports
    transport_header = f"RTP/AVP;unicast;client_port={rtp_port}-{rtcp_port}"
    client.setup(rtsp_uri, transport_header)

    # Start streaming
    print("Streaming:", rtsp_uri)
    client.play(rtsp_uri)

    # Start receiving video stream
    client.start_receiving_video_stream(rtp_port, rtcp_port)
