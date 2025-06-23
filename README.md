# camera-display

Example for [ESP32-S3-BOX](https://www.adafruit.com/product/5290)
([docs](https://github.com/espressif/esp-box)),
[ESP32-S3-BOX-3](https://www.espressif.com/en/news/ESP32-S3-BOX-3)([mouser](https://www.mouser.com/ProductDetail/Espressif-Systems/ESP32-S3-BOX-3?qs=HoCaDK9Nz5chOY9AUo%2F%2FvA%3D%3D)),
and [LilyGo T-Deck](https://www.lilygo.cc/products/t-deck) which receives an
MJPEG camera stream from the [camera-streamer
app](https://github.com/esp-cpp/camera-streamer) over WiFi and displays them on
the screen.

To facilitate discovery, this sample uses mDNS to find the camera-streamer app
(or any RTSP server that advertises itself as `_rtsp._tcp.local`).

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [camera-display](#camera-display)
  - [Use](#use)
    - [Program](#program)
    - [Configure](#configure)
  - [Development](#development)
    - [Environment](#environment)
    - [Build and Flash](#build-and-flash)
  - [Hardware](#hardware)
  - [Software](#software)

<!-- markdown-toc end -->

https://user-images.githubusercontent.com/213467/236601479-fcd982f5-80b1-4ce5-aef6-ab2b90f3d0b8.mp4

![image](https://github.com/esp-cpp/camera-display/assets/213467/3b08febf-433b-42e1-b139-132d64e35a07)
![image](https://github.com/esp-cpp/camera-display/assets/213467/cc179d89-6083-4324-af5e-05003beeda72)

## Use

You must first program your hardware. Afterwards, you can configure it via a USB
connection using its built-in CLI.

### Program

The ESP32-TimerCam will require one-time programming to function.

Download the release `programmer` executable from the latest [releases
page](https://github.com/esp-cpp/camera-display/releases) for `windows`,
`macos`, or `linux` - depending on which computer you want to use to perform the
one-time programming. There are a few programmers pre-built for either the
`ESP-BOX` or the `LilyGo T-Deck`.

1. Download the programmer
2. Unzip it
3. Double click the `exe` (if windows), or open a terminal and execute it from
   the command line `./camera-display-<hardware>_programmer_v2.0.0_macos.bin`,
   where hardware is one of `esp-box` or `t-deck`.

### Configure

To configure it, simply connect it to your computer via USB and open the serial
port in a terminal (e.g. `screen`, `PuTTY`, etc.) at 115200 baud. Once there,
you can use it as you would any other CLI - and the `help` command will provide
info about the commands available.

Any SSID/Password you set will be securely saved in the board's NVS, which is
managed by the ESP-IDF WiFi subsystem.

```console
sta> help
Commands available:
 - help
	This help message
 - exit
	Quit the session
 - log <verbosity>
	Set the log verbosity for the wifi sta.
 - connect
	Connect to a WiFi network with the given SSID and password.
 - connect <ssid> <password>
	Connect to a WiFi network with the given SSID and password.
 - disconnect
	Disconnect from the current WiFi network.
 - ssid
	Get the current SSID (Service Set Identifier) of the WiFi connection.
 - rssi
	Get the current RSSI (Received Signal Strength Indicator) of the WiFi connection.
 - ip
	Get the current IP address of the WiFi connection.
 - connected
	Check if the WiFi is connected.
 - mac
	Get the current MAC address of the WiFi connection.
 - bssid
	Get the current BSSID (MAC addressof the access point) of the WiFi connection.
 - channel
	Get the current WiFi channel of the connection.
 - config
	Get the current WiFi configuration.
 - scan <int>
	Scan for available WiFi networks.
 - memory
	Display minimum free memory.
```

## Development

If you wish to modify / recompile the code, you will need to set up your
development environment to be able to build and flash your target hardware.

### Environment

This project is an ESP-IDF project, currently [ESP-IDF
v.5.4](https://github.com/espressif/esp-idf).

For information about setting up `ESP-IDF v5.4`, please see [the official
ESP-IDF getting started
documentation](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/get-started/index.html).

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Hardware

This sample is designed to run on the ESP32-S3-BOX, ESP32-S3-BOX-3, and LilyGo
T-Deck all of which have a 320x240 LCD (over SPI) running on a ESP32-S3.

## Software

This sample has two main tasks: 

1. RTSP client that receives mjpeg frames split into RTP packets, turns them 
   back into JPEG images, and pushes them into a queue.
2. Display task, which pulls image data from the queue, decodes the jpeg, and
   displays it on the screen.
   
It is built with these libraries:

* [ESPP](https://github.com/esp-cpp/espp)
* [JPEGDEC](https://github.com/bitbank2/JPEGDEC)
* [ESP-IDF](https://github.com/espressif/esp-idf)
* [mDNS](https://docs.espressif.com/projects/esp-protocols/mdns/docs/latest/en/index.html)
