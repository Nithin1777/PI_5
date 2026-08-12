# Operational & Serial Control Guide: SmartElex 15D Motor Driver

---

## 1. Executive Summary & Board Overview

This document outlines the wiring, hardware configuration, serial command protocol, and software setup for operating the **SmartElex 15D Dual Channel DC Motor Driver** in **Serial Mode** under Ubuntu Linux.

### Key Specifications

* **Board Model:** SmartElex 15D Dual Channel DC Motor Driver
* **Operating Input Voltage ($\text{V}_{\text{IN}}$):** $7\text{V}$ to $22\text{V DC}$
* **Continuous Current:** $15\text{A}$ per channel
* **Peak Current:** $30\text{A}$ per channel ($10\text{s}$)
* **Control Modes:** Radio Control (RC), PWM, Analog, and **TTL Serial / USB**

---

## 2. Hardware & Switch Setup

### DIP Switch Configuration

To enable **Serial Mode at 115200 Baud**, set the 4-position DIP switch on the board to position **8 (`1000`)**:

| Switch Pin | State | Description |
| --- | --- | --- |
| **SW1** | **ON** (`1`) | Serial Mode Selector |
| **SW2** | **OFF** (`0`) | Baud Rate Selector ($115200\text{ Baud}$) |
| **SW3** | **OFF** (`0`) | Unused in Serial Mode |
| **SW4** | **OFF** (`0`) | Unused in Serial Mode |

---

## 3. Terminal & Power Wiring

> **Critical Note:** The USB connection powers only the logic layer. The motor terminals will **not** output voltage unless an external power supply ($7\text{V} - 22\text{V}$) is attached to the **`VIN` / `GND**` screw terminals.

1. **Power Supply:** Connect main power ($7\text{V} - 22\text{V}$) to the **`VIN` (+)** and **`GND` (-)** screw terminals.
2. **Motor Terminals:**
* **Channel 1:** Connect Motor 1 across `M1+` and `M1-`.
* **Channel 2:** Connect Motor 2 across `M2+` and `M2-`.


3. **Data Connection:** Connect the driver’s USB port (or USB-to-Serial bridge) to your host system (e.g., Ubuntu PC).

---

## 4. Serial Command Protocol

When configured in Serial Mode, the board Listens for a **6-character packet string** sent over the serial port.

### Packet Structure

$$\mathbf{* \; [DIR_1] \; [SPD_1] \; [DIR_2] \; [SPD_2] \; \#}$$

| Field | Character | Description / Valid Values |
| --- | --- | --- |
| **Header** | `*` | Delimiter indicating the start of a configuration packet |
| **Motor 1 Direction** | `1` or `0` | `1` = Forward, `0` = Reverse |
| **Motor 1 Speed** | `0` to `9` | Speed mapping ($0 = 0\%$ duty cycle, $9 = 100\%$ duty cycle) |
| **Motor 2 Direction** | `1` or `0` | `1` = Forward, `0` = Reverse |
| **Motor 2 Speed** | `0` to `9` | Speed mapping ($0 = 0\%$ duty cycle, $9 = 100\%$ duty cycle) |
| **Footer** | `#` | Delimiter indicating end of packet (applies settings) |

---

## 5. Verified Command Quick Reference

| Command | Motor 1 State | Motor 2 State | Usage Scenario |
| --- | --- | --- | --- |
| **`*1919#`** | $100\%$ Speed (Forward) | $100\%$ Speed (Forward) | Full speed ahead (Both channels active) |
| **`*1910#`** | $100\%$ Speed (Forward) | $0\%$ Speed (Stopped) | Differential control / Turning right |
| **`*1019#`** | $0\%$ Speed (Stopped) | $100\%$ Speed (Forward) | Differential control / Turning left |
| **`*0909#`** | $100\%$ Speed (Reverse) | $100\%$ Speed (Reverse) | Full speed reverse |
| **`*1515#`** | $\approx 55\%$ Speed (Forward) | $\approx 55\%$ Speed (Forward) | Medium speed forward |
| **`*1010#`** | $0\%$ Speed (Stopped) | $0\%$ Speed (Stopped) | **Emergency Stop / Standby** |

---

## 6. Ubuntu Linux Integration & Testing

### 1. Identify System Device Port

Plug in the USB cable and locate the assigned device path:

```bash
dmesg | grep -i "tty"

```

*(Typically mapped to `/dev/ttyACM0` or `/dev/ttyUSB0`).*

### 2. User Permissions Fix

Grant your user permission to access hardware serial interfaces:

```bash
sudo usermod -aG dialout $USER

```

*(Log out and back in for permissions to apply).*

### 3. Interactive Control via Terminal (`tio`)

Connect directly using `tio` at 115200 baud:

```bash
tio /dev/ttyACM0 -b 115200

```

Once connected, type commands like `*1919#` directly into the terminal window to actuate the outputs. *(To quit `tio`, press `Ctrl+T` then `Q`).*

---

## 7. Python Implementation Script

Below is a complete test script using `pyserial` to programmatically control the motor driver:

```python
import serial
import time

# Serial Port Configuration
PORT = '/dev/ttyACM0'  # Adjust to match your system (/dev/ttyUSB0 or /dev/ttyACM0)
BAUD = 115200

def send_motor_command(ser, command_str):
    print(f"Sending Command: {command_str}")
    ser.write(command_str.encode('utf-8'))
    time.sleep(0.1)

try:
    # Initialize serial interface
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # Wait for port initialization

    # 1. Drive both channels 100% Forward
    send_motor_command(ser, "*1919#")
    time.sleep(3)

    # 2. Drive Channel 1 Forward, Channel 2 Off
    send_motor_command(ser, "*1910#")
    time.sleep(3)

    # 3. Stop all outputs
    send_motor_command(ser, "*1010#")
    print("Test Complete. Outputs disabled.")

except Exception as e:
    print(f"Serial Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()

```




