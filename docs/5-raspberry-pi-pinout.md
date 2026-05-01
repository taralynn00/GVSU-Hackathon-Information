5. [Raspberry Pi Pinout](#raspberry-pi-pinout)  



# Raspberry Pi Pinout:
**GPIO Pins:** The majority of the pins are labeled GPIO followed by a number (e.g., GPIO 2, GPIO 17). These are the main programmable pins.

They can be configured as inputs (to read signals from sensors, like detecting if a button is pressed) or outputs (to send signals to components, like turning on an LED).

**Power Pins (VCC):** These pins supply electrical power to external components.
- Pins 1 and 17 provide **3.3V** (volts) power.
- Pins 2 and 4 provide **5V** power.

![alt text](img/rasp-pi-pinout.png)
Many of these GPIO pins also have specialized functions (like I2C, SPI, UART, or PWM, noted in parentheses) which are used for more complex communication with specific types of devices.
