# STM32- AES Encryption 

## STM32 Peripheral Driver Development Project

This project focuses on **from-scratch implementation of STM32 peripherals** to gain a deep, practical understanding of microcontroller internals and data movement.

While a cryptographic library is used for data processing, the primary contribution of this project lies in **manual GPIO, I2C, and LCD driver development**, written at the **register level**, without relying on high-level HAL abstractions - in a way writing my own compact HAL.

---

## Project Focus

This project is intentionally designed to emphasize:
- Peripheral initialization via registers
- Bit-level control of GPIO and I2C
- LCD interfacing over I2C
- Understanding how application data flows through firmware into hardware


---

## Features

- Fully hand-written GPIO driver
- I2C master driver implemented from scratch
- LCD driver over GPIO (16x02)
- CMSIS-based, bare-metal style code
- Clear separation between application logic and drivers
- Emphasis on readability and debuggability

---

## System Overview
[ Application Logic ]
        │
        │ 
(plaintext / processed data)
        ↓
  [ AES Library ]
        │
        │ 
  (byte stream)
        ↓
   [ I2C Driver ]
        ↓
      START
        ↓
    Address
        ↓
       Data
        ↓
       STOP
        │
        ↓
[ STM32 I2C Registers ]
        │
        ↓
[ I2C Backpack ]
        │
        ↓
[ LCD Controller ]
        │
        ↓
[ Display Output ]


This flow was implemented deliberately to understand how **high-level data ultimately reaches physical hardware pins**.

---

## Hardware Setup

| Component | Description | Notes |
|---------|-------------|-------|
| STM32 MCU | Main microcontroller | CMSIS / Register-level |
| LCD (16x2) | Display module | Controlled via custom LCD driver |
| GPIO Pins | Control and signaling | Manually configured |

---

## Software Modules

| Module | Description |
|------|-------------|
| GPIO_Init() | Configures GPIO modes, speed, pull-ups |
| I2C_Init() | Sets up I2C timing, addressing, enable |
| I2C_Write() | Implements START, data transfer, STOP |
| lcd_Init() | LCD initialization sequence |
| LCD_print_string() | Prints any string on the LCD |

All drivers are implemented without HAL function calls from ground up

---

##  Project Structure

.
├── Drivers/
│   ├── Inc/        # Driver headers
│   └── Src/        # Driver implementations
├── Inc/            # Application headers
├── Src/            # Application source files
├── Startup/        # Startup code and vector table
├── README.md


---

##  Development Setup

### Requirements
- STM32 development board
- STM32CubeIDE or equivalent GCC-based toolchain
- CMSIS headers (if needed)
- LCD (16X02) module
- Basic debugging tools (through Stm's debugging interface)

---

## Learning Outcomes

- Strong intuition for STM32 peripheral registers
- Confidence in writing reusable drivers
- Clear understanding of I2C transaction flow
- Practical experience interfacing external hardware
- Better foundation for firmware and SoC-level work

---

## Future Improvements

- Add DMA-based I2C transactions
- Improve error handling and timeouts
- Extend drivers to support multiple LCDs
- Add documentation for register-level timing analysis
- Port drivers to another STM32 family

---

## Notes

- AES is library-based and used only for data generation
- Core contribution is peripheral driver development
- Project is intended for learning and experimentation

