
2. [Commands To Know](#commands-to-know)  



# Commands to Know:
## Setup Commands:
- `sudo apt update`
- `sudo apt upgrade`
- `sudo apt upgrade pip` (or pip3)
- `sudo raspi-config`

- Using the arrow keys:
  - Navigate to Interface Options > I2C and enable it.
  - Navigate to Interface Options > SPI and enable it.
*End of setup commands. Note: `sudo update` and `sudo upgrade` are used daily **IF** installing any new utilities, packages, libraries, etc.*

## Reboot: 
`sudo reboot`

## Shut Down:
Running this command ultimately is clicking the power off button within the OS.
`sudo shutdown -h now`
From here, turn the kob on the power supply towards 0. Once the knob is completely turned off, unplug the power supply.

## Get into folder:
`cd /home/techshow1/Documents/TestCode/`
or if you were already in "/home/techshow1” (This is where the terminal would typically start you if you ssh in.) you could just do `cd Documents` for example

## Move up one folder:
`cd ~/[folder name]`

## Move Back to root (In this case "/home/techshow1”):
`cd ..`

## List all files in Folder:
`ls`

## Show Python Library Version:
`pip show pigpio` (or pip3)
