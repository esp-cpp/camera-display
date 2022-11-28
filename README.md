# camera-display

Example for [ESP32-S3-BOX](https://www.adafruit.com/product/5290)
([docs](https://github.com/espressif/esp-box)) which receives JPEG camera images
from the [camera-streamer app](https://github.com/esp-cpp/camera-streamer) over
WiFi and displays them on the screen.

## Hardware

This sample is designed to run on the ESP32-S3-BOX which has a 320x240 LCD (over
SPI) running on a ESP32-S3.

## Software

This sample has two main tasks: 

1. Receiver task, which runs a TCP server that receives jpeg image packets
   (whicha have an 8-byte header which includes image length) and puts them into
   a queue.
2. Display task, which pulls image data from the queue, decodes the jpeg, and
   displays it on the screen.
   
It is built with these libraries:

* [ESPP](https://github.com/esp-cpp/espp)
* [JPEGDEC](https://github.com/bitbank2/JPEGDEC)
* [ESP-IDF](https://github.com/espressif/esp-idf)
