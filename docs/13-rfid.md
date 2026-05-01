13. [RFID – Using the RC522 Reader on Raspberry Pi 5](#-rfid--using-the-rc522-reader-on-raspberry-pi-5)
    1.  [Hardware Setup](#hardware-setup-do-this-before-installing-software)
    2.  [RC522 → Raspberry Pi 5 Wiring](#-rc522--raspberry-pi-5-wiring)
    3.  [Installing the RFID Library (Required)](#-installing-the-rfid-library-required)
    4.  [Testing the Reader](#testing-the-reader)


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

## Hardware Setup (Do This Before Installing Software)

Before running any code, you must correctly connect the RC522 module to the Raspberry Pi 5. All wiring **must be completed while the Raspberry Pi is powered off**, as connecting wires to a powered device can damage the board or the RFID module.

Make sure:

- The Raspberry Pi is fully **powered off**.
- All wires go to the correct pins.
- The RC522 is powered using **3.3V only** (never 5V).

---

## 🔌 RC522 → Raspberry Pi 5 Wiring

Below is a clear mapping that shows the **RC522 pin number**, the **signal name**, and the matching **Raspberry Pi 5 pin**. This format helps beginners identify exactly where each wire goes.

![alt text](img/rfid-rc522-raspberry-pi-wiring-diagram.jpg)

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

Before moving onto the next step, make sure you have enabled your SPI!
[Enable SPI Documentation](#enable-spi-on-raspberry-pi)

## Installing the RFID Library (Required)

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

## Testing the Reader

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

## Troubleshooting Guide

If the RC522 does not work as expected, try the following:

- **No output at all:** Ensure SPI is enabled using `sudo raspi-config`.
- **Reader lights up but shows no UID:** Move the card closer or recheck the 3.3V connection.
- **Same UID repeats constantly:** Normal behavior if the card stays on the reader.
- **Permission errors:** Verify your user is in the required SPI/GPIO groups.
- **Unstable or inconsistent reads:** Shorten jumper wires and check all connections.

If problems continue, ensure the correct MFRC522 library (Pi‑5 compatible) is installed and that all wiring is secure.

Your RFID module is now fully set up and ready to integrate into your hackathon project. With wiring complete, the correct software installed, and the ability to read card UIDs, you can begin building interactive systems that respond to RFID tags in creative and exciting ways.
