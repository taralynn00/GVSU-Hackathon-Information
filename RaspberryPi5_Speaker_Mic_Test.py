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
