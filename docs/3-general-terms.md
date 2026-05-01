
3. [General Terms](#general-terms)  


# General Terms:
- **GPIO**: General-purpose input output, a pin that can either be read from or written to.
- **GND**: Ground, which is just 0 volts in this context. You almost always want all GNDs connected between different components.
- **Vcc**: Always positive and often referred to as the logic voltage or control voltage, the minimum voltage level that a pin must detect to register as high or "on" (1). On a Raspberry Pi, this is 3.3V, on some devices, this is 5V. Raspberry Pis regulate the 5V input down to 3.3V, this is why they have a 5V supply pin and a 3.3V supply pin.
- **PWM**: Pulse-width modulation, this is how a simple dimming LED or a servo would be controlled.
- **MOSI**: Master-Out Slave-In, used in I2C communication between devices.
- **MISO**: Master-In Slave-Out, used in I2C communication between devices.
- **Cathode**: The “negative” side of a component, like an LED.
- **Anode**: The “positive” side of a component, like an LED.
