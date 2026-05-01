10. [Audio and Microphone Setup](#audio-and-microphone-setup)
    1.  [Enable SPI on Raspberry Pi](#enable-spi-on-raspberry-pi)  

# Audio and Microphone Setup

The Raspberry Pi 5 has built-in support for USB audio devices through PulseAudio. We did not need to install any additional libraries — only to import subprocess and time.
Instead of using the device’s interface name, we used “pulse” as the audio interface.

There is also a script named test.py that can be used to verify both the audio output and the microphone input.

## Enable SPI on Raspberry Pi
1. Open Terminal
2. Run the configuration tool:
   `sudo raspi-config`
3. Navigate to Interface Options  
    - ![alt text](img/interface-options.png) 
4. Select SPI  
    - ![alt text](img/select-spi.png)  
    When asked “Would you like the SPI interface to be enabled?”, select Yes   
    - ![alt text](img/enable-spi.png)  
5. Finish and reboot
    Select Finish
    When prompted, choose Yes to reboot the Pi.
*(If not prompted, `run sudo reboot`.)*

## Testing the Microphone
To confirm that the microphone is working, use the following command in Terminal:
`arecord -D pulse -f cd -d 3 test.wav`
This records a **3-second audio clip** using the PulseAudio interface and saves it as test.wav in the current directory.
You can then play it back with:
`aplay test.wav`

```
import subprocess
import time

def test_speakers():
    print("🔊 Testing speakers...")

    # Play a built-in system sound (440 Hz sine wave for 1 sec)
    try:
        subprocess.run([
            "speaker-test",
            "--test", "sine",
            "--frequency", "440",
            "--nloops", "1"
        ], check=True)
        print("✅ Speaker test finished.")
    except Exception as e:
        print("❌ Speaker test failed:", e)


def test_microphone():
    print("🎤 Testing microphone (3-second recording)...")

    try:
        # Record 3 seconds from default mic
        subprocess.run([
            "arecord",
            "-D", "pulse",  # change if your mic uses a different device
            "-f", "cd",
            "-d", "3",
            "mic_test.wav"
        ], check=True)

        print("✅ Recording saved as mic_test.wav")
        print("▶ Playing back the recording...")

        # Play recording
        subprocess.run(["aplay", "mic_test.wav"], check=True)

    except Exception as e:
        print("❌ Microphone test failed:", e)


if __name__ == "__main__":
    print("=== Raspberry Pi 5 Speaker & Mic Test ===")
    time.sleep(1)
    test_speakers()
    time.sleep(1)
    test_microphone()
    print("✅ Test complete.")
```
