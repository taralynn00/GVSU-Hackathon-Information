# 💻 GVSU x GRPS Hackathon Technical Information

### Documentation
Reading documentation can be very helpful! Consider it a manual of how to use whatever object or software you are using.
[Click here to see some documentation about Raspberry Pi!](https://www.raspberrypi.com/documentation/)

**Some Useful Calculators:**
[Online Conversion Calculators | DigiKey Electronics](https://www.digikey.com/en/resources/online-conversion-calculators)

# ⚠️ Safety:
**Safe Shutdown Procedures**
Always follow proper shutdown procedures to protect the operating system and hardware.

*Software Shutdown is Key:* Use the command `sudo shutdown -h now` (or `sudo reboot`) before removing power from the Pi. Simply pulling the plug or battery while the Pi is running can corrupt the SD card and make the operating system unbootable.

*Wait for the Lights:* After a shutdown command, wait for the Pi's green activity light to stop blinking (it may turn off completely) before disconnecting the power supply.

**Component and Handling Safety**
- *Mind the Voltage and Polarity:* The provided documentation states the power supply is set to 5.37V. While this is close to the recommended 5V for the Pi, always double-check the polarity (positive and negative connections) when hooking up the battery or power supply. Turning the knob too fast and above 5.37V can instantly destroy the Raspberry Pi and/or the Hat.
  
- *Keep Wires Tidy:* Ensure no loose wire ends, metal tools, or components can accidentally touch different pins or traces on the Pi or Hat. A short circuit can cause excessive current draw, generate heat, and permanently damage the board or cause the battery to become a fire hazard.
  
- Ensure the Pi has proper airflow, especially when running processor-intensive tasks.
  
- Handle the boards by their edges and avoid touching the metal components, chips, or connectors directly.
  
- *Use Insulating Material:* Place the Raspberry Pi on a non-conductive surface (like wood, plastic, or an anti-static mat) while working.
  

### Power Supply
Power Supply Setting: 5.37v
*Note: Turn the knob slowly until it clicks. Once it clicks, the voltage should start around 3V. Slowly continue turning the knob until it hits 5.37 volts.*

## Commands to Know:
### Setup Commands:
`sudo apt update`
`sudo apt upgrade`
`sudo apt upgrade pip` (or pip3)
`sudo raspi-config`

Using the arrow keys:
Navigate to Interface Options > I2C and enable it.
Navigate to Interface Options > SPI and enable it.
*end of setup commands*

### Reboot: 
`sudo reboot`

### Shut Down:
`sudo shutdown -h now`

### Get into folder:
`cd /home/techshow1/Documents/TestCode/`
or if you were already in "/home/techshow1” (This is where the terminal would typically start you if you ssh in.) you could just do `cd Documents` for example

### Move up one folder:
`cd ~/[folder name]`

### Move Back to root (In this case "/home/techshow1”):
`cd ..`

### List all files in Folder:
`ls`

### Show Python Library Version:
`pip show pigpio`

## General Terms:
- **GPIO**: General-purpose input output, a pin that can either be read from or written to.
- **GND**: Ground, which is just 0 volts in this context. You almost always want all GNDs connected between different components.
- **Vcc**: Always positive and often referred to as the logic voltage or control voltage, the minimum voltage level that a pin must detect to register as high or "on" (1). On a Raspberry Pi, this is 3.3V, on some devices, this is 5V. Raspberry Pis regulate the 5V input down to 3.3V, this is why they have a 5V supply pin and a 3.3V supply pin.
- **PWM**: Pulse-width modulation, this is how a simple dimming LED or a servo would be controlled.
- **MOSI**: Master-Out Slave-In, used in I2C communication between devices.
- **MISO**: Master-In Slave-Out, used in I2C communication between devices.
- **Cathode**: The “negative” side of a component, like an LED.
- **Anode**: The “positive” side of a component, like an LED.

# Create Virtual Environment:
A virtual environment (venv) is essentially a clean, isolated workspace for your Python project.

Imagine you have two different projects:
- Project A needs an older version of a specific package (a tool).
- Project B needs the newest version of that same package.

If you installed all packages in the same place, installing the new version for Project B would break Project A.

The venv solves this by creating a separate, private box in your computer for each project. Consider your computer as a big box with mutiple smaller boxes inside of it. When you activate a venv, you ensure that any packages you install only exist inside that box, preventing any errors or side effects with your other projects (boxes) or your main computer setup.

`python -m venv testVenv`
*It will create this inside of whatever folder you’re in.*
*"testVenv" can be replaced with whatever you want to name your virtual environment.*

## Start Virtual Environment:
This is where you want to create that box so you don't mess with other projects. Using 'venv' in the commmand is calling the specific Python module that handles creating virtual environments.
`source /home/techshow1/Documents/TestCode/testVenv/bin/activate`

## Deactivate Virtual Environment:
To deactivate and leave the venv, you must already be inside of the venv. Navigate back to the venv folder and run this command:
`deactivate`

## Where to create python files in a venv:
You typically put them inside of the venv folder itself. In this example: "/home/techshow1/Documents/TestCode/testVenv”


# Raspberry Pi Pinout:
**GPIO Pins:** The majority of the pins are labeled GPIO followed by a number (e.g., GPIO 2, GPIO 17). These are the main programmable pins.

They can be configured as inputs (to read signals from sensors, like detecting if a button is pressed) or outputs (to send signals to components, like turning on an LED).

**Power Pins (VCC):** These pins supply electrical power to external components.
- Pins 1 and 17 provide **3.3V** (volts) power.
- Pins 2 and 4 provide **5V** power.

![alt text](rasp-pi-pinout.png)
Many of these GPIO pins also have specialized functions (like I2C, SPI, UART, or PWM, noted in parentheses) which are used for more complex communication with specific types of devices.

# Breadboard Diagram:
All points with lines between them are connected, and will be of the same voltage. Typically the side “rails” are used to supply power and ground to components on the breadboard, where the red line is power and the blue line is ground. If using both sets of rails for the same voltages, they should be connected to the same line on the other side of the board.

![alt text](breadboard-diagram.png)

## Button Diagram:
**Reading the Input in Code**
Since the switch only provides an electrical signal, you need a program (usually written in Python) to read and interpret that signal.

![alt text](button-diagram.png)

**Basic Setup**:
One side of the button is connected to GND, and one side is connected to the pin you want to use, which has a pull-up resistor (in the code below, the pin is 4).

```
from gpiozero import Button

button = Button(4)
button.wait_for_press()
print("The button was pressed!")```
```

# Pull-up/Pull-down resistor:
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
![alt text](resistor.png)

# LEDs:
One side is connected to GND, the other side is connected to a pin. The resistor can be on either side of the LED, as long as the anode of the LED is connected to the pin you’re powering the LED from, whether that be directly or through the resistor. The larger the resistor, the dimmer the LED will be, but it is important to make sure that you have a resistor large enough to safely power the LED. For example, using the calculator below, which lists the typical voltage drop across each LED color, we can calculate the minimum resistance needed to connect a red LED, as shown below. To be safe, we’ll assume that the red LED uses 2.1V across itself (forward voltage). Since we know the GPIO pin supplies 3.3V (supply voltage) when on, assuming a standard forward current of 20 mA, at least a 60-ohm resistor would be needed to safely operate the LED. It is always safe to increase this value.
[LED Series Resistor Calculator | DigiKey Electronics](https://www.digikey.com/en/resources/conversion-calculators/conversion-calculator-led-series-resistor)
[4 Band Resistor Color Code Calculator](https://www.digikey.com/en/resources/conversion-calculators/conversion-calculator-resistor-color-code)

![alt text](led-diagram.png)

# PWM Diagram:
![alt text](led-diagram.png)
T is the time it takes for one cycle to complete, or rather, the period. The brightness of an LED can be controlled with PWM: with a 25% duty cycle, the LED would appear dimmer than with a 75% duty cycle, since the percentages correspond to the fraction of the period the LED is actually on. It should be noted that the type of PWM available differs by pin on the Raspberry Pi. All GPIOs can perform software PWM, but only specific pins support hardware PWM, which has better performance. For the Pi 5, the hardware PWM pins are GPIO 12, 13, 18, and 19. The code below brings the LED from “off” to full brightness, and then full brightness to “off”, and repeats.

`from gpiozero import PWMLED`
`from time import sleep`

`led = PWMLED(17)`

`# Fade the LED from off to full brightness and back`

`while True:`
    `for brightness in range(101):`
        `led.value = brightness / 100.0`
        `sleep(0.02)`
    `for brightness in range(100, -1, -1):`
        `led.value = brightness / 100.0`
        `sleep(0.02)`

# Servo Diagram:
![alt text](led-diagram.png)
Since servos operate with PWM, the servo control wire (yellow) must be connected to a GPIO pin using PWM on your Raspberry Pi.

`from gpiozero import Servo`
`from time import sleep`

`servo = Servo(12)`

`while True:`
    `# Move the servo to its minimum position`
    `servo.min()`
    `sleep(1)`
    ``
    `# Move the servo to its middle position`
    `servo.mid()`
    `sleep(1)`
    ``
    `# Move the servo to its maximum position`
    `servo.max()`
    `sleep(1)`
    ``
    `# Move the servo to 45 degrees`
    `servo.angle = 45`
    `sleep(1)`

# Touch-Sense Hat:
[Adafruit Capacitive Touch HAT for Raspberry Pi](https://www.adafruit.com/product/2340)
[Python & CircuitPython | Capacitive Touch Sensor Breakout Tutorial](https://learn.adafruit.com/adafruit-mpr121-12-key-capacitive-touch-sensor-breakout-tutorial/python-circuitpython)

Example Code:
`import time`
`import board`
`import busio`
`import adafruit_mpr121`
``
`i2c = busio.I2C(board.SCL, board.SDA)`
`mpr121 = adafruit_mpr121.MPR121(i2c)`

`while True:`
  `if mpr121[0].value:`
    `print("Pin 0 touched!")`
  `else:`
    `print("Pin 0 not touched!")`
  `time.sleep(0.5)`

*Note: Whatever is attached to the Sense Hat Holes, it assumes that is the default capacitance, any added component will make the pin go high.*

# SenseHat:
![alt text](led-diagram.png)
`sudo apt install sense-hat`
Enable I2C using `sudo raspi-config`
[Sense HAT - Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/accessories/sense-hat.html)
Examples can be found in the folder: “/usr/src/sense-hat/examples/python-sense-hat/”

## GPIO - Buttons, LEDs, Servo, Buzzers:
[Installing GPIO Zero — gpiozero 2.0.1 Documentation](https://gpiozero.readthedocs.io/en/stable/installing.html#raspberry-pi)

*Notes:*
- Should already be installed
- Need to give permissions. Run this in the Pi’s terminal: `sudo usermod -a -G gpio <your_username>` 
- Servo: Since the Raspberry Pi is a fully fledged computer and multiple pieces of code run at the same time, timing is not as precise. GPIO Zero library supports servos, but they are jittery because the PWM signal is not consistent enough. This is why a separate library is needed to have non-jittery servos, it does some complex code stuff involving interrupts. You can use GPIO Zero if you just move a servo to a position and then turn off the PWM signal.
- Continuous Servo: Uses the same servo libraries as above. I think you can control speed but not position, this needs to be tested. “Position "90" (1.5ms pulse) is stop, "180" (2ms pulse) is full speed forward, "0" (1ms pulse) is full speed backward. They may require some simple calibration, simply tell the servo to 'stop' and then gently adjust the potentiometer in the recessed hole with a small screwdriver until the servo stops moving.” https://www.adafruit.com/product/154?srsltid=AfmBOoptop99bL5_lxYjCYqGNZttixjbZLOcAF2bz4I5EZUxTgE32DbG

# Code we Ran and Tested:
GPIO Zero Button Test (Switches use the same code):
`from gpiozero import Button`
``
`button = Button(2)`
`button.wait_for_press()`
`print("The button was pressed!")`

*Wiring note: you can either connect to the middle and left pin or the middle and right pin. The switch toggles between connecting either side.*
![alt text](wiring-note.png)

# Digital LED Test:
`from gpiozero import LED`
`from time import sleep`

`red = LED(2)`

`while True:`
    `red.on()`
    `sleep(1)`
    `red.off()`
    `sleep(1)`

# Dimmable LED:
`from gpiozero import PWMLED`
`from time import sleep`

`led = PWMLED(2)`

`while True:`
    `led.value = 0  # off`
    `sleep(1)`
    `led.value = 0.5  # half brightness`
    `sleep(1)`
    `led.value = 1  # full brightness`
    `sleep(1)`

# Jittery Servo:
`from gpiozero import AngularServo`
`from time import sleep`
`pin = 17`
`servo = AngularServo(pin, min_angle=-90, max_angle=90)`

`while True:`
    `servo.angle = -90`
    `sleep(2)`
    `servo.angle = -45`
    `sleep(2)`
    `servo.angle = 0`
    `sleep(2)`
    `servo.angle = 45`
    `sleep(2)`
    `servo.angle = 90`
    `sleep(2)`


# Motion Sensor:
![alt text](motion-sensor.png)
`from gpiozero import Button`
`import time`

`if __name__ == '__main__':`
  `MotionSensor = Button(26)`
  
  
  `while True:`
    `time.sleep(1)`
    ``
    `if MotionSensor.is_pressed:`
        `print("Motion is detected")`
    `else:`
        `print("Motion is not detected")`

## Neopixels:
[Pi5Neo Documentation](https://github.com/vanshksingh/Pi5Neo?tab=readme-ov-file)

**Setup:**
`sudo raspi-config`
Navigate to Interface Options > SPI and enable it. *(the SPI is used with the NeoPixels)*

## Using Pi5Neo in a Virtual Environment
This is a very important step when making projects and using libraries.

`python3 -m venv myenv`  Creates a virtual environment named 'myenv'
`source myenv/bin/activate`  Activates the virtual environment
`pip install pi5neo`  Installs pi5neo within the virtual environment

`from pi5neo import Pi5Neo`
`import time`

`def rainbow_cycle(neo, delay=0.1):`
    `colors = [`
        `(255, 0, 0),`  # Red
        `(255, 127, 0),`  # Orange
        `(255, 255, 0),`  # Yellow
        `(0, 255, 0),`  # Green
        `(0, 0, 255),`  # Blue
        `(75, 0, 130),`  # Indigo
        `(148, 0, 211)`  # Violet
    `]`
    `for color in colors:`
        `neo.fill_strip(*color)`
        `neo.update_strip()`
        `time.sleep(delay)`

`neo = Pi5Neo('/dev/spidev0.0', 10, 800)`
`rainbow_cycle(neo)`

## I2C OLED:
1. Enable I2C Interface:
The luma.oled library primarily uses the I2C interface to communicate with OLED displays.
Enable I2C: Open a terminal on your Raspberry Pi and run: `sudo raspi-config`

Navigate to Interface Options > I2C and enable it.
Reboot your Raspberry Pi if prompted.

2. Install luma.oled and Dependencies:
Update package list: `sudo apt update`
Install build dependencies:
`sudo apt install build-essential python3-dev python3-pip python3-pil libjpeg-dev zlib1g-dev` `libfreetype6-dev liblcms2-dev libopenjp2-7`
-> These dependencies could cause problems in other areas, especially because they’re system wide.

`Install luma.oled.`
`pip3 install luma.oled -> doesn’t work.`
`pip3 install luma.oled -i https://pypi.tuna.tsinghua.edu.cn/simple/ -> doesn’t work`
`sudo apt install python3-luma.oled`


1. Verify Installation and Test:
Add your user to necessary groups if needed.
`sudo usermod -a -G spi,gpio techshow1`

1. Wiring:
Gnd->Gnd
Vcc->5v
SDA->GPIO 2 or Pin 3 (labeled SDA in Pi 5 pin Diagram)
SCL->GPIO 3 or Pin 5

1. Code that worked:
**Code Summary:**
The luma.oled library only has a few functions. It basically just sends a 1 bit pil image to the OLED screen.
[See “luma.oled.device.ssd1306“ at this link](https://luma-oled.readthedocs.io/en/latest/api-documentation.html)
The pil library is used to make the images, and the device.display function of the luma.oled library expects a pil image object.

`from luma.core.interface.serial import i2c`
`from luma.oled.device import ssd1306`
`from PIL import ImageDraw, ImageFont, Image`
`import time`

`serial = i2c(port=1, address=0x3C)`  # Adjust address if needed
`device = ssd1306(serial)`

`with Image.new("1", device.size) as img:`
  `draw = ImageDraw.Draw(img)`
  `font_size = 20`
  `font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font_size)` 
  `# font = ImageFont.load_default() `
  `x_pos = 10`
  `y_pos = 10`
  `draw.text((x_pos, y_pos), "Hello, Pi 5!", font=font, fill=255)`
  `device.display(img)`
  `while(True):`
    `time.sleep(1)`

*Note: The screen seems to clear when the program completes. If the I2C can’t connect an error will occur.*

*Note: These are the available fonts as defaults.*
![alt text](fonts.png)
Here’s a different font type and size (20 is the size of the font in pixels):
`ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)`

# Scrolling Test Example:
`from luma.core.interface.serial import i2c`
`from luma.oled.device import ssd1306`

`from PIL import ImageDraw, ImageFont, Image`
`import time`

`serial = i2c(port=1, address=0x3C)`  # Adjust address if needed
`device = ssd1306(serial)`

`def clearImage(img):`
  `for x in range(device.width):`
    `for y in range(device.height):`
        `img.putpixel((x, y), 0)`

#img is a 1 bit pil image
`with Image.new("1", device.size) as img:`
  `draw = ImageDraw.Draw(img)`
  `font_size = 20`
  `font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font_size)` 
  `# font = ImageFont.load_default() `
  `x_pos = 0`
  `y_pos = 10`
  `text_to_display = "Hello, Pi 5!"`
  `draw.text((x_pos, y_pos), text_to_display, font=font, fill=255)`
  `device.display(img)`
  `img.save("/home/techshow1/Documents/font_example.png")`
  
  `while(True):`
    `clearImage(img)`
    `x_pos +=1`
    `if(x_pos > 128):`
      `x_pos = -100`
    `draw.text((x_pos, y_pos), text_to_display, font=font, fill=255)`
    `device.display(img)`

# Putting Images on Device:
This takes an image, resizes it to fit the device, and uses dithering to convert a colored image to black and white. Note that the display has a resolution of 128x64 pixels, so if you don’t want the image to get distorted, you’ll have to make sure the image has a similar ratio.

`from luma.core.interface.serial import i2c`
`from luma.oled.device import ssd1306`
`from PIL import ImageDraw, ImageFont, Image`
`import time`

`serial = i2c(port=1, address=0x3C)`  # Adjust address if needed
`device = ssd1306(serial)`

`source_image = Image.open("/home/techshow1/Documents/Giraffe.jpg").resize(device.size)`

`convertedImage = source_image.convert("1")`

`convertedImage.save("/home/techshow1/Documents/image_example.png")`

`device.display(convertedImage)`

`while(True):`
  `time.sleep(1)`

## Audio and Microphone Setup

The Raspberry Pi 5 has built-in support for USB audio devices through PulseAudio. We did not need to install any additional libraries — only to import subprocess and time.
Instead of using the device’s interface name, we used “pulse” as the audio interface.

There is also a script named test.py that can be used to verify both the audio output and the microphone input.

# Testing the Microphone
To confirm that the microphone is working, use the following command in Terminal:
`arecord -D pulse -f cd -d 3 test.wav`
This records a **3-second audio clip** using the PulseAudio interface and saves it as test.wav in the current directory.
You can then play it back with:
`aplay test.wav`

# Enable SPI on Raspberry P
1. Open Terminal
2. Run the configuration tool:
   `sudo raspi-config`
3. Navigate to Interface Options
    ![alt text](interface-options.png)
4. Select SPI
    ![alt text](select-spi.png)
    When asked “Would you like the SPI interface to be enabled?”, select Yes
    ![alt text](enable-spi.png)
5. Finish and reboot
    Select Finish
    When prompted, choose Yes to reboot the Pi.
*(If not prompted, `run sudo reboot`.)*


# Time Blurb about Time library
In programming, the time library is used to manage and measure time-related tasks. It provides access to the computer’s system clock, allowing you to track the current time, measure how long code takes to run, pause execution, and format or convert timestamps.

Common options and functions include:
- **time.time()** - returns the current time in seconds since January 1, 1970 (the Unix epoch).
- **time.sleep(seconds)** – pauses program execution for a set number of seconds.
- **time.ctime()** – converts a timestamp into a readable date and time.
- **time.localtime()** and **time.gmtime()** – return structured time data in local or UTC format.
- **time.strftime()** – formats time into a custom string *(e.g., "2025-11-07 10:30 AM")*.

Overall, the time library is essential for any program that needs to handle timing, scheduling, or performance tracking.

## Current-Limiting Protection
In sensor and indicator circuits, resistors limit current flow through components such as **LEDs, photodiodes, or buzzers**, preventing component burnout.

![alt text](current-limiting.png)

# Resistors
[Resistors explained simply!](https://www.youtube.com/watch?v=x6jajfprWZo)
In Internet of Things (IoT) hardware design, resistors are among the most fundamental components ensuring that electronic subsystems operate within safe and predictable limits.

They regulate the flow of current in mixed-signal circuits that interconnect sensors, actuators, and microcontrollers such as the Raspberry Pi or ESP32.

## Key Takeaways
- Resistors serve as the control backbone of IoT electronics, ensuring reliable data acquisition, safe operation, and efficient power management.
- Their applications span from simple GPIO protection to advanced analog signal conditioning and power metering.
- A deep understanding of resistor behavior under real-world electrical and environmental conditions empowers IoT developers to design resilient, scalable, and energy-aware systems.

## Transistors
[How To Use Transistors In YOUR Projects! || Transistors Explained || Transistors As A Switch](https://www.youtube.com/watch?si=DzobngMC4t_jnj9s&v=-qRNJhU1OLM&feature=youtu.be)

- In Internet of Things (IoT) hardware, transistors function as the critical interface between computation and physical action.

- They act as solid-state switches or amplifiers, allowing low-power logic signals from microcontrollers (such as the Raspberry Pi, Arduino, or ESP32) to control higher-power devices such as motors, solenoids, relays, and heating elements.

- The transistor is a three-terminal semiconductor device, with terminals that control and conduct electrical current through an internal junction.

- By regulating the flow of electricity between two terminals (output path) based on a small signal applied to a third terminal (control path), a transistor enables digital systems to interact seamlessly with analog or electromechanical components.

- Within IoT systems, transistors are essential for power control, signal conditioning, and digital actuation, forming the foundation for embedded automation and sensor networks.

# Structure and Basic Functionality
- A transistor has three active terminals — the control terminal (Base in BJTs or Gate in MOSFETs), the input terminal (Collector or Drain), and the output terminal (Emitter or Source).

- It uses a small input signal at the control terminal to modulate a larger output current through the other two terminals.

- This property allows transistors to perform two critical roles:

  - Switching: Turning electronic components or circuits on and off under microcontroller command.

  - Amplification: Strengthening weak analog signals for processing or measurement.

- In IoT systems, this makes transistors indispensable in modules for motor drivers, sensor amplifiers, and wireless communication circuits.

- Their switching speed, current gain, and power-handling capabilities vary by type — the correct selection depends on the IoT application’s voltage, current, and control requirements.

# Key Takeaways
- Transistors form the switching and amplification core of IoT device control systems.

- They enable digital microcontrollers to operate safely across mixed-voltage environments while driving diverse electromechanical components.

- Selection between BJT, Darlington, and MOSFET devices depends on trade-offs among speed, current capacity, and control voltage.

- In IoT prototyping and hackathons, mastery of transistor-based circuit design enables students to convert conceptual “smart ideas” into functional, interactive, and autonomous prototypes.

# How to Automatically Run Scripts When Your Computer Starts
This guide explains in simple terms how to make your computer automatically run a script or program every time it starts or when you log in. You don’t need to be a programmer, just follow the steps below.

Imagine you built a small weather station or camera project on your Raspberry Pi. Every time you restart it, you have to open the terminal and run a command to start your program. That gets tiring quickly. By using the methods below, you can make your Raspberry Pi automatically start your script every time it powers on, no typing needed. 

The @reboot method tells your Pi, “Run this every time I start up,” while the systemd method keeps your program running in the background even if something crashes.
Once you set this up, you can just plug in your Pi and your project will launch on its own. This makes your setup work like a real automated system, ideal for sensors, data loggers, dashboards, or smart displays that need to run all the time.


**For Linux Users (including Raspberry Pi)**
Linux systems can run scripts automatically at startup or login. You can choose between a quick cron method or a more reliable systemd service.
**Option 1:** Using `@reboot` in crontab 
1.	Open the Terminal.
2.	Type:
3.	crontab -e
This opens your personal startup list.

4.	Scroll to the bottom and add this line:
5.	`@reboot /home/pi/myscript.sh`
  - Replace "/home/pi/myscript.sh" with the full path to your script.
6.	Save and exit (Ctrl + X, then Y, then Enter).
7.	Make the script executable:
8.	chmod +x /home/pi/myscript.sh
9.	Reboot your Pi. The script will start automatically after every boot.

**Option 2:** Using systemd (More Reliable and Professional Way)
1.	Open Terminal.
2.	Create a new service file:
- `sudo nano /etc/systemd/system/myscript.service`
3.	Paste this inside: [Unit]
4. Description=Run my script at boot
5. After=network-online.target
6. Wants=network-online.target
7. [Service]
8. ExecStart=/home/pi/myscript.sh
9. WorkingDirectory=/home/pi
10. StandardOutput=inherit
11. StandardError=inherit
12. Restart=always
13. User=pi
14. [Install]
15. WantedBy=multi-user.target
Change paths or username if needed.
16. Save and exit (Ctrl + X, then Y, then Enter).
17. Enable and start the service:
18. sudo systemctl daemon-reload
19. sudo systemctl enable myscript.service
20. sudo systemctl start myscript.service
21. Check its status:
22. systemctl status myscript.service
23. Reboot to confirm it launches automatically.

Extra Tips
- Always use the full path to your script and test it manually first.
- Add a line at the top of your script to define the shell, for example:
  - `#!/bin/bash`
  - If your script depends on Wi-Fi or other services, start it with a delay:
      - `sleep 10`
    - To view logs, use:
      - `journalctl -u myscript.service -e`


# 📡 RFID – Using the RC522 Reader on Raspberry Pi 5

This section provides a clear, polished, and beginner‑friendly guide for using the **RC522 RFID module** with the **Raspberry Pi 5** during the hackathon. It includes wiring instructions, installation steps, usage details, and troubleshooting to help students of all skill levels work confidently with RFID technology.

RFID (Radio‑Frequency Identification) enables the Raspberry Pi to detect and interact with cards, tags, or key fobs **wirelessly**. This makes it ideal for interactive hackathon projects involving user check‑ins, game mechanics, access control, or sensor‑based installations.

Common use cases include:

- Player check‑in systems for games or challenges
- Unlocking features or awarding points based on scanned tags
- Triggering LEDs, sound effects, or animations
- Creating scavenger hunts or puzzle systems
- Logging progress or activity during an event
- Interactive displays that respond to different tags

Each RFID card or fob contains a **unique UID**, which your program can use to identify participants or actions.

---

# 🛠️ Step 1 — Hardware Setup (Do This Before Installing Software)

Before running any code, you must correctly connect the RC522 module to the Raspberry Pi 5. All wiring **must be completed while the Raspberry Pi is powered off**, as connecting wires to a powered device can damage the board or the RFID module.

Make sure:

- The Raspberry Pi is fully **powered off**.
- All wires go to the correct pins.
- The RC522 is powered using **3.3V only** (never 5V).

---

# 🔌 RC522 → Raspberry Pi 5 Wiring

Below is a clear mapping that shows the **RC522 pin number**, the **signal name**, and the matching **Raspberry Pi 5 pin**. This format helps beginners identify exactly where each wire goes.

![alt text](rfid-rc522-raspberry.pi-wiring-diagram.png)

```
RC522 Pin   | RC522 Signal Name | Raspberry Pi Pin | Raspberry Pi Signal Name
------------|-------------------|------------------|-----------------------------
Pin 1       | SDA / SS          | Pin 24           | GPIO 8 (CE0)
Pin 2       | SCK               | Pin 23           | GPIO 11 (SCLK)
Pin 3       | MOSI              | Pin 19           | GPIO 10 (MOSI)
Pin 4       | MISO              | Pin 21           | GPIO 9 (MISO)
Pin 6       | GND               | Pin 6            | Ground
Pin 7       | RST               | Pin 16           | GPIO 23 (Reset)
Pin 8       | 3.3V              | Pin 1            | 3V3 Power (NOT 5V)
```

Explanation of signals:

- **SDA/SS** – Selects the RC522 for communication.
- **SCK** – Clock line used by the SPI interface.
- **MOSI** – Data line from Pi → RC522.
- **MISO** – Data line from RC522 → Pi.
- **RST** – Resets the module when needed.
- **3.3V/GND** – Power connections; must be correct for the module to function.

Once wiring is complete, power the Raspberry Pi back on.

---
## Enable SPI on Raspberry Pi 5

Before the RFID reader can communicate with the Pi, you must enable SPI in the system settings:

Open Terminal

Run:
```sudo raspi-config```
Go to Interface Options -> Select SPI -> Choose Yes to enable it (to navigate to yes use right arrow key)
Exit the menu and reboot the Pi
```sudo reboot```

***SPI must be enabled for the RC522 to work properly.***
---

# 📥 Step 2 — Installing the RFID Library (Required)

### 2. Install Pi 5–compatible library

Because the hackathon uses **Raspberry Pi 5**, many older online tutorials will not work. They rely on outdated GPIO libraries that are incompatible with newer Pi hardware.

The correct, Pi‑5‑compatible MFRC522 library is already included with the hackathon materials, but you can reinstall it using the commands below.

### 🔧 Required installation commands:

```
sudo apt update
sudo apt install -y python3-pip python3-spidev git
```

Download the compatible RFID library:

```
git clone https://github.com/danjperron/MFRC522-python.git
cd MFRC522-python
```

---

# 🧪 Step 3 — Testing the Reader

After installing the library and entering the folder, run the provided reader script:

```
python3 Read.py
```

Then place your RFID card or key fob directly on top of the module.

You should see output similar to:

```
Card Detected
Card read UID: Example: 33CE8153
```

Each card has a different UID. You will use these UIDs inside your project to identify users, unlock features, or trigger events.

This MFRC522 library is the **only stable and supported option** for Raspberry Pi 5, which is the platform used in this hackathon.

---

# 🚀 Project Ideas

Here are several project ideas suitable for teams with varying experience levels:

- **RFID Login Station** — Register each participant using their card.
- **Color Badge System** — Assign colors or LED patterns to each UID.
- **Puzzle Lock / Treasure Hunt** — Scanning specific tags unlocks clues.
- **Task Progress Tracker** — Track which stations participants have visited.
- **Bonus Unlock System** — Special cards grant advantages in a game.
- **Interactive Display** — Show unique messages, animations, or icons on an OLED screen.

RFID makes it simple to build interactive systems that respond to physical objects.

---

# 🐞 Troubleshooting Guide

If the RC522 does not work as expected, try the following:

- **No output at all:** Ensure SPI is enabled using `sudo raspi-config`.
- **Reader lights up but shows no UID:** Move the card closer or recheck the 3.3V connection.
- **Same UID repeats constantly:** Normal behavior if the card stays on the reader.
- **Permission errors:** Verify your user is in the required SPI/GPIO groups.
- **Unstable or inconsistent reads:** Shorten jumper wires and check all connections.

If problems continue, ensure the correct MFRC522 library (Pi‑5 compatible) is installed and that all wiring is secure.

---

# 🎉 You're Ready to Build!

Your RFID module is now fully set up and ready to integrate into your hackathon project. With wiring complete, the correct software installed, and the ability to read card UIDs, you can begin building interactive systems that respond to RFID tags in creative and exciting ways.

Let your imagination guide your project - RFID opens the door to fun, engaging, and dynamic interactions!


### I²C Communication in IoT Systems
# Overview
 **I²C (Inter-Integrated Circuit)** is a serial communication protocol that enables multiple digital devices — such as microcontrollers, sensors, and displays — to communicate using only **two shared signal lines**.


It is one of the most widely used communication standards in Internet of Things (IoT) systems because it allows efficient, synchronized data transfer between a central controller (the master) and numerous peripheral devices (the slaves) with minimal wiring.


Typical IoT use cases include connecting **temperature sensors, OLED displays, accelerometers, real-time clocks, and air-quality monitors** to a single Raspberry Pi or microcontroller.


I²C combines the simplicity of **UART communication** (few wires) with the flexibility of **SPI** (multi-device support), making it ideal for sensor networks and embedded device clusters.
![alt text](ic-communication.png)

### HATs in IoT Systems
# Overview
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
    [Adafruit MPR121 Capacitive Touch Sensor](https://www.adafruit.com/product/2340)
    [Python/CircuitPython Setup Guide](https://learn.adafruit.com/adafruit-mpr121-12-key-capacitive-touch-sensor-breakout-tutorial/python-circuitpython)
