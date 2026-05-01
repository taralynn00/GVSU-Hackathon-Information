6. [Misc. Diagrams](#misc-diagrams)  
   1. [Breadboard Diagram](#breadboard-diagram)
   2. [Button Diagram](#button-diagram)
   3. [LEDs](#leds)
   4. [PWM Diagram](#pwm-diagram)
   5. [Motion Sensors](#motion-sensors)


# Misc. Diagrams
These could be helpful when implementing different tools in your project!

## Breadboard Diagram:
All points with lines between them are connected, and will be of the same voltage. Typically the side “rails” are used to supply power and ground to components on the breadboard, where the red line is power and the blue line is ground. If using both sets of rails for the same voltages, they should be connected to the same line on the other side of the board.

Here is a video to learn about how to use a breadboard!
[💡 How to Use a Breadboard: A Beginner's Guide to Prototyping 🔧](https://youtu.be/oPMgerbKBwM?si=SqExYYlA8SQw6aky)

![alt text](img/breadboard-diagram.png)

## Button Diagram:
**Reading the Input in Code**
Since the switch only provides an electrical signal, you need a program (usually written in Python) to read and interpret that signal.

![alt text](img/button-diagram.png)

**Basic Setup**:
One side of the button is connected to GND, and one side is connected to the pin you want to use, which has a pull-up resistor.

```
from gpiozero import Button

button = Button(4)
button.wait_for_press()
print("The button was pressed!")
```

### Pull-up/Pull-down resistor:
Inside the Raspberry Pi, there is a resistor that connects a GPIO pin to either 3.3V (pull-up) or GND (pull-down), preventing a short from happening when the button is pressed. If a short happens (GND is connected directly to power), a bunch of current flows, just like if you connect the two sides of a battery. Typically, internal pull-up resistors are used as described in the basic setup above, but note that if you use a pull-down resistor, which would connect the pin to GND when the button isn’t pressed, the button would have to have one side connected to power and the other connected to the pin you want to use, instead.

Code example:
```
import RPi.GPIO as GPIO
import time

BUTTON_GPIO = 17 # Choose any available GPIO pin

GPIO.setmode(GPIO.BCM) # Use the Broadcom pin numbering scheme
  - Set the pin as an input and enable the internal PULL-UP resistor
GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
    # Read the state of the button
    # Because we used a pull-up, the value is LOW (0) when pressed
    if GPIO.input(BUTTON_GPIO) == GPIO.LOW:
        print("Button was pressed!")
    
    time.sleep(0.1) # Wait a bit to prevent reading the button too many times
```

Using the internal pull-up resistor is usually the simplest way to wire this switch to a Raspberry Pi, as it only requires connecting the switch between a GPIO input pin and a Ground (GND) pin.
![alt text](img/resistor.png)

## LEDs:
One side is connected to GND, the other side is connected to a pin. The resistor can be on either side of the LED, as long as the anode of the LED is connected to the pin you’re powering the LED from, whether that be directly or through the resistor. The larger the resistor, the dimmer the LED will be, but it is important to make sure that you have a resistor large enough to safely power the LED. For example, using the calculator below, which lists the typical voltage drop across each LED color, we can calculate the minimum resistance needed to connect a red LED, as shown below. To be safe, we’ll assume that the red LED uses 2.1V across itself (forward voltage). Since we know the GPIO pin supplies 3.3V (supply voltage) when on, assuming a standard forward current of 20 mA, at least a 60-ohm resistor would be needed to safely operate the LED. It is always safe to increase this value.
[LED Series Resistor Calculator | DigiKey Electronics](https://www.digikey.com/en/resources/conversion-calculators/conversion-calculator-led-series-resistor)
[4 Band Resistor Color Code Calculator](https://www.digikey.com/en/resources/conversion-calculators/conversion-calculator-resistor-color-code)

![alt text](img/led-diagram.png)

## PWM Diagram:
![alt text](img/pwm-diagram.png)
T is the time it takes for one cycle to complete, or rather, the period. The brightness of an LED can be controlled with PWM: with a 25% duty cycle, the LED would appear dimmer than with a 75% duty cycle, since the percentages correspond to the fraction of the period the LED is actually on. It should be noted that the type of PWM available differs by pin on the Raspberry Pi. All GPIOs can perform software PWM, but only specific pins support hardware PWM, which has better performance. For the Pi 5, the hardware PWM pins are GPIO 12, 13, 18, and 19. The code below brings the LED from “off” to full brightness, and then full brightness to “off”, and repeats.

```
from gpiozero import PWMLED
from time import sleep

led = PWMLED(17)

# Fade the LED from off to full brightness and back

while True:
    for brightness in range(101):
        led.value = brightness / 100.0
        sleep(0.02)
    for brightness in range(100, -1, -1):
        led.value = brightness / 100.0
        sleep(0.02)
```

## Motion Sensors:
The component shown is a PIR (Passive Infrared) Motion Sensor. It detects movement by sensing changes in heat (infrared radiation) in its surrounding area. When it senses motion, its OUTPUT signal changes, telling the Raspberry Pi that something has moved.
![alt text](img/motion-sensor.png)
