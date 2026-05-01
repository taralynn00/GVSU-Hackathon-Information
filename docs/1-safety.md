1. [Safety](#️-safety)
   1. [Power Supply](#power-supply)


# ⚠️ Safety:
**Safe Shutdown Procedures**
Always follow proper shutdown procedures to protect the operating system and hardware.

*Software Shutdown is Key:* Use the command `sudo shutdown -h now` (or `sudo reboot`) before removing power from the Pi. Simply pulling the plug or battery while the Pi is running can corrupt the SD card and make the operating system unbootable.

*Wait for the Lights:* After a shutdown command, wait for the Pi's green activity light to stop blinking (it may turn off completely) before disconnecting the power supply.

**Component and Handling Safety**
- *Mind the Voltage and Polarity:* The provided documentation states the power supply is to be set to 4.75-5.25V. We found best voltage setting to be 5.35-37V. Turning the knob too fast and above ~5.4V can instantly destroy the Raspberry Pi and/or the Hat. 
  
- *Keep Wires Tidy:* Ensure no loose wire ends, metal tools, or components can accidentally touch different pins or traces on the Pi or Hat. A short circuit can cause excessive current draw, generate heat, and permanently damage the board or cause the battery to become a fire hazard.
  
- Ensure the Pi has proper airflow, especially when running processor-intensive tasks.
  
- Handle the boards by their edges and avoid touching the metal components, chips, or connectors directly.
  
- *Use Insulating Material:* Place the Raspberry Pi on a non-conductive surface (like wood, plastic, or an anti-static mat) while working.
  

### Power Supply
Power Supply Setting: 5.35-37V
*Note: Turn the knob slowly until it clicks. Once it clicks, the voltage should start around 3V. Slowly continue turning the knob until it hits ~5.35 volts.*
