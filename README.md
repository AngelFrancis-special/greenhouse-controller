# Greenhouse Controller

Embedded greenhouse monitoring and control project developed by Francis Nzekwe.

## Overview

The system monitors **temperature, humidity, and CO2**, averages sensor samples, compares the measurements against configurable high/low limits, and controls greenhouse actuators including a **fan, heater, cooler, sprinkler, and motorized vent**.

The project uses two controllers communicating over serial: a **PIC18F45K22** for the greenhouse sensing/control side and an **mbed LPC1768** for receiving, validating, parsing, and executing controller messages.

## System Architecture

```text
Sensors
  |
  v
PIC18F45K22
  |  ADC / control logic
  |
  +----> Fan
  +----> Heater
  +----> Cooler
  +----> Sprinkler
  +----> Stepper-driven vent
  |
  | UART / serial message
  v
mbed LPC1768
  |
  +----> Receive buffer
  +----> Checksum validation
  +----> Message parsing
  +----> Address/command validation
  +----> Greenhouse monitor display
```

## PIC18F45K22

The PIC firmware includes:

- ADC configuration and sensor acquisition
- Temperature, humidity, and CO2 processing
- Ten-sample sensor averaging
- High/low configurable limits
- Push-button limit adjustment
- Fan/heater/cooler/sprinkler control
- Stepper-motor vent positioning
- Timer-based timing
- UART/serial transmission
- XOR checksum generation
- Display/status reporting

## LPC1768

The LPC1768 firmware receives controller messages at **19200 baud**, stores incoming bytes in a receive buffer, detects complete messages, validates the checksum, parses comma-delimited fields, verifies addressing/command conditions, and updates the corresponding sensor limits.

The message format implemented in the source includes fields for controller/address, sensor channel, limit type, and limit value.

## Source Files

- `GREENHOUSE_CONTROLLER_PIC.c` — PIC18F45K22 greenhouse sensing and actuator-control firmware.
- `GREENHOUSE_CONTROLLER_MBED.cpp` — mbed LPC1768 serial receiver, checksum validation, parsing, and command execution.

## Original Project Information

Author: Francis Nzekwe

## Note

This repository contains the original project source supplied from the project files. The source comments and implementation are retained rather than rewritten as a new implementation.
