7. [Hats in IoT Systems](#hats-in-iot-systems)
   1. [Touch-Sense (MPR121) HAT](#touch-sense-mpr121-hat)
   2. [Sense Hat](#sensehat)
   3. [Servo Diagram](#servo-diagram)
   4. [GPIO - Buttons, LEDs, Servo, Buzzers](#gpio---buttons-leds-servo-buzzers)


# HATs in IoT Systems
- A HAT (Hardware Attached on Top) is an expansion board that connects directly to the Raspberry Pi’s 40-pin GPIO header, adding new capabilities such as sensing, display, or control functions.
- HATs simplify IoT prototyping by combining sensors, inputs, and communication interfaces (I²C, SPI, UART) into a single, compact module.
- They enable students to focus on data collection and analysis instead of circuit wiring, making them ideal for hackathon use.
- Always power off the Pi before attaching a HAT, and ensure I²C and SPI are enabled via `sudo raspi-config` before running code.

**Usage Notes:**
- Press the HAT down firmly for a proper GPIO connection.
- Attach the HAT only when the Pi is powered off.
- Requires I²C and SPI to be enabled.

## Touch-Sense (MPR121) HAT
- Purpose: Adds capacitive touch sensing—detecting proximity or touch through conductive surfaces—for creative or user-input IoT designs.

- Product & Tutorial:
  - [Adafruit MPR121 Capacitive Touch Sensor](https://www.adafruit.com/product/2340)
  - [Python/CircuitPython Setup Guide](https://learn.adafruit.com/adafruit-mpr121-12-key-capacitive-touch-sensor-breakout-tutorial/python-circuitpython)

Note: `Adafruit-Blinka` depends on `lgpio`, which is a system package (not installed in your venv). So, your venv needs to be created with system packages visible.
1. Ensure lgpio is installed by running `sudo apt install -y python3-lgpio liblgpio1`
2. Create venv with system packages visible: `python -m venv --system-site-packages myVenv`
3. Activate Venv: `source /home/techshow1/myVenv/bin/activate`
4. Install https://pypi.org/project/Adafruit-Blinka/ by running `pip install Adafruit-Blinka` in venv
5. Run `pip3 install adafruit-circuitpython-mpr121` in Venv **DO NOT USE SUDO**

Example Code:
```
import time
import board
import busio
import adafruit_mpr121

i2c = busio.I2C(board.SCL, board.SDA)
mpr121 = adafruit_mpr121.MPR121(i2c)

while True:
  if mpr121[0].value:
    print("Pin 0 touched!")
  else:
    print("Pin 0 not touched!")
  time.sleep(0.5)
```

*Note: Whatever is attached to the Sense Hat Holes, it assumes that is the default capacitance, any added component will make the pin go high.*

## SenseHat:
![alt text](img/sense-hat.png)
- `sudo apt install sense-hat`
- Enable I2C using `sudo raspi-config`

[Sense HAT - Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/accessories/sense-hat.html)
Examples can be found in the folder: “/usr/src/sense-hat/examples/python-sense-hat/”

## Servo Diagram:
![alt text](img/servo-diagram.png)
Since servos operate with PWM, the servo control wire (yellow) must be connected to a GPIO pin using PWM on your Raspberry Pi.

```
from gpiozero import Servo
from time import sleep

servo = Servo(12) 

while True:
    # Move the servo to its minimum position
    servo.min()
    sleep(1)

    # Move the servo to its middle position
    servo.mid()
    sleep(1)

    # Move the servo to its maximum position
    servo.max()
    sleep(1)

    # Move the servo to 45 degrees
    servo.angle = 45
    sleep(1)
```

## GPIO - Buttons, LEDs, Servo, Buzzers:
[Installing GPIO Zero — gpiozero 2.0.1 Documentation](https://gpiozero.readthedocs.io/en/stable/installing.html#raspberry-pi)

*Notes:*
- Should already be installed
- Need to give permissions. Run this in the Pi’s terminal: `sudo usermod -a -G gpio <your_username>` 
- Servo: Since the Raspberry Pi is a fully fledged computer and multiple pieces of code run at the same time, timing is not as precise. GPIO Zero library supports servos, but they are jittery because the PWM signal is not consistent enough. This is why a separate library is needed to have non-jittery servos, it does some complex code stuff involving interrupts. You can use GPIO Zero if you just move a servo to a position and then turn off the PWM signal.
- Continuous Servo: Uses the same servo libraries as above. I think you can control speed but not position, this needs to be tested. “Position "90" (1.5ms pulse) is stop, "180" (2ms pulse) is full speed forward, "0" (1ms pulse) is full speed backward. They may require some simple calibration, simply tell the servo to 'stop' and then gently adjust the potentiometer in the recessed hole with a small screwdriver until the servo stops moving.” https://www.adafruit.com/product/154?srsltid=AfmBOoptop99bL5_lxYjCYqGNZttixjbZLOcAF2bz4I5EZUxTgE32DbG

### GPIO Switch Example Code:
GPIO Zero Button Test (Switches use the same code):
```
from gpiozero import Button

button = Button(2)
button.wait_for_press()
print("The button was pressed!")
```

*Wiring note: you can either connect to the middle and left pin or the middle and right pin. The switch toggles between connecting either side.*
![alt text](img/wiring-note.png)
