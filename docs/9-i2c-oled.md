9. [I2C OLED](#i2c-oled)
   1. [Scrolling Test Example for I2C OLED](#scrolling-test-example-for-i2c-oled)
   2. [Putting Images on Device for I2C OLED](#putting-images-on-device-for-i2c-oled)

# I2C OLED:
1. Enable I2C Interface:
The luma.oled library primarily uses the I2C interface to communicate with OLED displays.
Enable I2C: Open a terminal on your Raspberry Pi and run: `sudo raspi-config`

Navigate to Interface Options > I2C and enable it.
Reboot your Raspberry Pi if prompted.

2. Install luma.oled and Dependencies:
Update package list: `sudo apt update`
Install build dependencies:
- `sudo apt install build-essential python3-dev python3-pip python3-pil libjpeg-dev zlib1g-dev libfreetype6-dev liblcms2-dev libopenjp2-7` (use pip3 if pip is not compatible/not working)
-> These dependencies could cause problems in other areas, especially because they’re system wide so make sure to use a venv.

- `Install luma.oled.`
- `sudo apt install python3-luma.oled`


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

```
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import ImageDraw, ImageFont, Image
import time

serial = i2c(port=1, address=0x3C)  # Adjust address if needed
device = ssd1306(serial)

with Image.new("1", device.size) as img:
  draw = ImageDraw.Draw(img)
  font_size = 20
  font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font_size) 
  # font = ImageFont.load_default() 
  x_pos = 10
  y_pos = 10
  draw.text((x_pos, y_pos), "Hello, Pi 5!", font=font, fill=255)
  device.display(img)
  while(True):
    time.sleep(1)
```

*Note: The screen seems to clear when the program completes. If the I2C can’t connect an error will occur.*

*Note: These are the available fonts as defaults.*
![alt text](img/fonts.png)

Here’s a different font type and size (20 is the size of the font in pixels):
`ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)`

## Scrolling Test Example for I2C OLED:
```
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306

from PIL import ImageDraw, ImageFont, Image
import time

serial = i2c(port=1, address=0x3C)  # Adjust address if needed
device = ssd1306(serial)

def clearImage(img):
  for x in range(device.width):
    for y in range(device.height):
        img.putpixel((x, y), 0)

#img is a 1 bit pil image
with Image.new("1", device.size) as img:
  draw = ImageDraw.Draw(img)
  font_size = 20
  font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font_size) 
  # font = ImageFont.load_default() 
  x_pos = 0
  y_pos = 10
  text_to_display = "Hello, Pi 5!"
  draw.text((x_pos, y_pos), text_to_display, font=font, fill=255)
  device.display(img)
  img.save("/home/techshow1/Documents/font_example.png")
  
  while(True):
    clearImage(img)
    x_pos +=1
    if(x_pos > 128):
      x_pos = -100
    draw.text((x_pos, y_pos), text_to_display, font=font, fill=255)
    device.display(img)
```


## Putting Images on Device for I2C OLED:
This takes an image, resizes it to fit the device, and uses dithering to convert a colored image to black and white. Note that the display has a resolution of 128x64 pixels, so if you don’t want the image to get distorted, you’ll have to make sure the image has a similar ratio.

```
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306

from PIL import ImageDraw, ImageFont, Image
import time

serial = i2c(port=1, address=0x3C)  # Adjust address if needed
device = ssd1306(serial)

source_image = Image.open("/home/techshow1/Documents/Giraffe.jpg").resize(device.size)
convertedImage = source_image.convert("1")

convertedImage.save("/home/techshow1/Documents/image_example.png")
device.display(convertedImage)

while(True):
  time.sleep(1)
```
