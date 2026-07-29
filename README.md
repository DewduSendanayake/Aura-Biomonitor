# Chest Strap V3

## Overview

Chest Strap V3 is a wearable physiological monitoring platform built
around the **ESP32-C3 Super Mini**. The system acquires physiological
and motion data, processes key health metrics, and transmits results to
a mobile application using **Bluetooth Low Energy (BLE)**.

------------------------------------------------------------------------

## Hardware

-   ESP32-C3 Super Mini
-   ADS1292R ECG Front-End
-   MPU6500 6-Axis IMU
-   DS18B20 Body Temperature Sensor
-   Bluetooth Low Energy (BLE)

------------------------------------------------------------------------

# Current Features

## ADS1292R Driver

-   SPI communication
-   Device initialization
-   Continuous ECG acquisition
-   DRDY interrupt handling
-   24-bit ECG sample conversion
-   Self-test support

## MPU6500 Driver

-   I²C communication
-   Accelerometer acquisition
-   Gyroscope acquisition
-   Internal temperature acquisition
-   WHO_AM_I verification

## DS18B20 Driver

-   OneWire communication
-   Non-blocking temperature conversion
-   Body temperature acquisition

## BLE Communication

-   Nordic UART Service (NUS)
-   BLE advertising
-   Mobile connectivity
-   Data transmission

------------------------------------------------------------------------

# Firmware Architecture

    ADS1292R
          │
    MPU6500
          │
    DS18B20
          │
          ▼
    Health Engine
          │
          ▼
    Health Packet
          │
          ▼
    BLE UART
          │
          ▼
    Mobile Application

------------------------------------------------------------------------

# Sampling Rates

  Module            Rate
  ------------- --------
  ECG             500 Hz
  IMU             100 Hz
  Temperature       1 Hz
  BLE               1 Hz

------------------------------------------------------------------------

# Health Engine (Version 1)

Current responsibilities:

-   Collect ECG data
-   Collect IMU data
-   Collect body temperature
-   Build BLE packets
-   Transmit one packet every second

Current BLE packet includes:

-   Timestamp
-   Heart Rate *(placeholder)*
-   Respiration Rate *(placeholder)*
-   Body Temperature
-   Step Count *(basic implementation)*
-   Motion State *(basic implementation)*

------------------------------------------------------------------------

# Project Status

  Feature                 Status
  ---------------------- --------
  ECG Acquisition           ✅
  IMU Acquisition           ✅
  Body Temperature          ✅
  BLE Communication         ✅
  Scheduler                 ✅
  Health Engine V1          ✅
  Packet Generation         ✅
  Mobile Communication      ✅

------------------------------------------------------------------------

# Planned Enhancements

-   ECG filtering
-   R-peak detection
-   Heart rate calculation
-   Respiration monitoring
-   Accurate step counting
-   Activity recognition
-   Heart Rate Variability (HRV)
-   Stress estimation
-   Binary BLE protocol with CRC
-   Ring-buffer based data management
-   Live ECG visualization
-   Session recording
-   Battery monitoring

------------------------------------------------------------------------

# Long-Term Vision

The objective is to develop a complete wearable physiological monitoring
platform capable of continuously acquiring physiological signals,
extracting clinically relevant health metrics, and delivering processed
information to a mobile application over Bluetooth Low Energy.

------------------------------------------------------------------------

## Author

**Thamara Banneheka**

Electrical & Electronic Engineer\
University of Kelaniya -- M.Sc. Data Analytics\
SLIIT -- B.Sc. (Hons.) Electrical & Electronic Engineering
