# AirClock - Wifi Clock and Air Quality Monitor

Feng Zhou, July-August, 2020

## Overview

This is a Wifi clock and air quality monitor based on [Kevin Norman's Air Quaility Meter project](https://kn100.me/where-embedded-meets-the-internet-building-your-own-air-quality-meter/)(See [PDF](hardware.pdf) for a local copy).  Hardware is similar. Additions are a button between pin 
12 and GND, and a LiPo battery. Most of the changes are on the software side. I added easy 
Wifi configuration (the original project uses hard-coded Wifi SSID/password), an always-accurate clock 
(network-synced), and a button to switch between functionalities.

Enjoy!

## Final product

See [inside image](doc/AirClock_1.jpg), [front panel](doc/AirClock_2.jpg), and [breadboard version](doc/AirClock_breadboard.jpg).

## Instructions

1. Get the necessary components as mentioned in the original [article](hardware.pdf), in addition to the
   following:

   a. A micro switch. See image above for what I use.
   b. (Optional) A LiPo battery. The battery connector on the Wemos Lolin32 is JST XH2-2.54mm. The one I use has 2000mAh capacity.
   c. (Optional) I used a 5x7cm prototype PCB for the circuits and a 125x80x32mm ABS case for holding everything.

2. Assembly everything according to Kevin's article.

3. Install [Arduino IDE](https://www.arduino.cc/en/main/software) if you have not yet done so.

4. Within Arduino IDE, use `Sketch->Library->Manage Libraries` to install the following libraries:
  * BSEC Library by Bosch Sensortec (v1.5.1474) (needs extra changes to install, see 'Talking to the BME680' in [Kevin's article](https://kn100.me/where-embedded-meets-the-internet-building-your-own-air-quality-meter/))
  * [LiquidCrystal I2C](https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library) 1.1.2
  * NTPClient 3.2.0
  * [BigNumbers_I2C](https://github.com/Anush-DP/BigNumbers_I2C)
  * PubSubClient 2.8.0
  * EasyButton 2.0
  * PageBuilder 1.4.2
  * AutoConnect 1.1.7

5. Open `airclock/airclock.ino` in Arduino IDE.

6. Connect the dev board through USB, and install the firmware with `Sketch->Upload`.

## Usage

Once installed, the firmware works as follows,

1. The first time when the wifi clock is powered up, it needs some setup for it to connect
   to the Wifi. Just follow the instructions on the screen. Use your phone to connect to the
   clock's Wifi network. Then in the pop-up screen on your phone, choose "add new AP", put in
   Wifi information and you will be done. Remember the board only support 2.4Ghz networks, 
   not 5Ghz ones.

2. After the setup is done, the clock should show accurate time as displayed in the picture
   above. 

3. Press the button once to turn on backlight. Press it again to switch to air quality 
monitoring, where you can see air quality index, measure temperature, humidity, pressure, 
VOC and CO2. Press it again to show Wifi configuration page. The last part is not finished 
yet.

## Notes
 
 * Time is synced from 'ntp.aliyun.com' every 10 minutes. Change it to other servers in airclock.ino if
   if you need to.

 * For initial use, the air quality sensor (BME680) needs quite a long time to 'calibrate' itself.
   Before that is done, the readings will be off and it is indicated with a '?' at bottom right 
   of screen. It may take 2-3 days for this process to complete.
   
 * MQTT reporting to server is currently disabled in the firmware as I do not need it. Refer to 
   Kevin's original code if you want to restore it.

## Changelog

 * 0.6
  * In wifi screen, long press to re-setup Wifi.
  * Reduce clock frequency to 80Mhz to save power.
  * Only turn on Wifi at NTP time to save power. Power is now at around 20-30mA @ 5V.
 * 0.5
  * Initial version.