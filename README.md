# camera-display

Example for [ESP32-S3-BOX](https://www.adafruit.com/product/5290)
([docs](https://github.com/espressif/esp-box)) which receives an MJPEG camera stream
from the [camera-streamer app](https://github.com/esp-cpp/camera-streamer) over
WiFi and displays them on the screen.

To facilitate discovery, this sample uses mDNS to find the camera-streamer app
(or any RTSP server that advertises itself as `_rtsp._tcp.local`).

https://user-images.githubusercontent.com/213467/236601479-fcd982f5-80b1-4ce5-aef6-ab2b90f3d0b8.mp4

## Hardware

This sample is designed to run on the ESP32-S3-BOX which has a 320x240 LCD (over
SPI) running on a ESP32-S3.

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
