8. [Neopixels](#neopixels)
   1. [Using Pi5Neo in a Virtual Environment](#using-pi5neo-in-a-virtual-environment)

# Neopixels:
[Pi5Neo Documentation](https://github.com/vanshksingh/Pi5Neo?tab=readme-ov-file)
When using the neopixel technology and libraries, you will need to include this in a venv so the packages within the library do not affect any other project. 
[Click here to go to the Venv documentation.](#create-virtual-environment)

**Setup:**
`sudo raspi-config`
Navigate to Interface Options > SPI and enable it. *(the SPI is used with the NeoPixels)*

## Using Pi5Neo in a Virtual Environment
Using the Pi5Neo library for NeoPixels within a Python virtual environment (venv) on your Raspberry Pi 5 is the recommended and best practice approach, especially with the recent changes in Raspberry Pi OS (Bookworm and later) which discourages system-wide package installation.

`python3 -m venv myenv`  Creates a virtual environment named 'myenv'
`source myenv/bin/activate`  Activates the virtual environment
`pip install pi5neo`  (or pip3) Installs pi5neo within the virtual environment

Click here to read the documentation about creating virtual environments if you need more info!
[Create Virtual Environment](#create-virtual-environment)

**REMEMBER**
You must enable the SPI interface on your Raspberry Pi 5 before running any code.
[Click here to view more info about enabling the SPI interface.](#enable-spi-on-raspberry-pi)
