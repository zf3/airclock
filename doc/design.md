# ESP32-based Wifi Clock and Air Quality Monitor

Feng Zhou, July-August, 2020

# Changes

2020-8-2 Optimizaing power

Original power @ 5V: 0.07A. 

 * Change clock to 80Mhz: `setCpuFrequencyMhz(80);`:
   0.05A 

# Future work

 * Lower power consumption by adding
   power management to the firmware (currently unoptimized version is at around 80mA @ 5V). 

 * Press buttom for 10 seconds in Wifi screen to clear Wifi AutoConnect data.

 * Show current SSID in Wifi screen.

 * Finish configuration-through-wifi. It would be nice to be able to do things like setting
   alarms, changing time zones and etc.

 * MQTT for reporting air quality to server.

# Design notes

## Buttons

 * [Push button with ESP32 – GPIO pins as digital input](https://microcontrollerslab.com/push-button-esp32-gpio-digital-input/)
  * GPIO 12 should be OK. There's internal pull-up resistors, needs to be enabled.

## Soft AP for setting up Wifi

 * [ESP8266/ESP32 Connect WiFi Made Easy](https://www.hackster.io/hieromon-ikasamo/esp8266-esp32-connect-wifi-made-easy-d75f45)

## WebServer for configuration

 * For setting time zone, alarm time, NTP server address and etc.

## EEPROM

 * Save wifi ssid/password, wall clock time, air quality sensor data

## Time keeping and NTP

 * NTP to get time after each power-up.
 * cn.ntp.org.cn
 * [Getting Date and Time with ESP32 on Arduino IDE (NTP Client)](https://randomnerdtutorials.com/esp32-ntp-client-date-time-arduino-ide/)
 * [Accuracy is about 1%, so need to sync once per hour to get to 1 min accuracy](https://github.com/espressif/arduino-esp32/issues/3641)

## Display BigNumber

 * `#include <BigNumbers_I2C.h>`

## Fancy 2004 LCD demo

## Power Saving

 * [ESP32 Power Modes](https://lastminuteengineers.com/esp32-sleep-modes-power-consumption/)
   * With radio: 160-260mA
   * Without radio: 3-20mA
 * [ESP32: Tips to increase battery life](https://www.savjee.be/2019/12/esp32-tips-to-increase-battery-life/)


