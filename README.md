# KX032 Accelerometer Driver for STM32 (Low-Layer - LL)

An efficient, lightweight, and professional C library for the **Kionix KX032-1021** accelerometer, specifically designed for **STM32** microcontrollers using the **Low-Layer (LL) APIs**.



## 🚀 Key Features
* **Performance:** Built with STM32 LL drivers for minimal memory footprint and high execution speed.
* **Platform:** Optimized for STM32 (tested on Series like F1, F4, and L4).
* **Functionality:** Full support for G-range selection, Output Data Rate (ODR) configuration, and I2C communication.
* **Portability:** Easy to integrate into STM32CubeIDE, Keil uVision, or IAR.

## 🛠 Prerequisites
* STM32CubeIDE or any toolchain supporting STM32 LL.
* An STM32 microcontroller with an available I2C interface.
* **KX032** sensor module.

## 📂 Installation
1.  Clone this repository into your project's `Drivers/Hardware` folder:
    ```bash
    git clone [https://github.com/JavidTahery/KX032-STM32.git](https://github.com/JavidTahery/KX032-STM32.git)
    ```
2.  Include `kx032_ll.h` in your `main.c`.
3.  Add the source files to your IDE project path.

## 💻 Quick Start Example
```c
#include "kx032_ll.h"

...
