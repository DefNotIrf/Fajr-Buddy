# Fajr Buddy – IoT-Based Fajr Prayer Assistant

**Fajr Buddy** is an assistive technology system designed to help users consistently wake up for **Fajr prayer**, especially those with irregular sleep patterns or difficulty waking before sunrise. The system combines time-based and environmental triggers with IoT features to deliver reliable, user-friendly alerts.

---

## Project Motivation

Fajr prayer occurs before sunrise, making it challenging for some to wake up on time, especially for individuals with sleep disorders or inconsistent routines. Fajr Buddy solves this by:

- Automatically detecting **Fajr prayer time** via Real-Time Clock (RTC)
- Monitoring **sunlight conditions** via an LDR sensor
- Triggering buzzer and light alerts, with **manual snooze override**
- Providing full **IoT control** via the Blynk app for remote access

---

## Features

- ⏰ **Fajr Prayer Time Alert**  
  When RTC matches preset prayer time, system activates buzzer and LED to wake the user.

- 🌄 **Sunrise Detection**  
  Uses an LDR sensor to detect ambient sunlight; triggers a secondary reminder if Fajr was missed.

- 💤 **Snooze & Manual Override**  
  Snooze button temporarily disables automatic mode. A second button allows manual control of LED.

- 📱 **IoT Control via Blynk (ESP32)**  
  Remotely control the buzzer, LED, or snooze using a smartphone app.

---

## Hardware Components

- ESP32 (WiFi-enabled microcontroller)  
- Real-Time Clock Module (e.g. DS3231)  
- LDR (light-dependent resistor)  
- Buzzer  
- LED  
- Two push buttons  
- Blynk app (on Android/iOS)

---

## System Workflow

1. **Time Trigger**: RTC matches Fajr time → Buzzer + LED ON  
2. **User Response**:  
   - Press Snooze → System disables auto mode, enables manual control  
   - Ignore → System checks for sunrise  
3. **Sunrise Detection**:  
   - LDR detects sunlight → Triggers alert again  
4. **Remote Control**:  
   - All features accessible via Blynk app

---

## Blynk IoT Integration

- Connects system to the smartphone via WiFi  
- Remotely control:
  - Snooze
  - LED ON/OFF
  - Reset triggers  
- Allows real-time monitoring and updates

---

## 📝 License

This project is licensed under the MIT License. See [LICENSE](./LICENSE) for details.

---

## 👤 Author

**Muhammad Irfan Rosdin**  
Mechatronics Engineering, IIUM  
---


