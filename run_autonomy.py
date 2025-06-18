import subprocess
import time
import os
import sys

# Automatically get current folder
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LINE_CONTROLLER_PATH = os.path.join(BASE_DIR, "linecontroller.py")

def start_pigpiod():
    try:
        subprocess.run(["pgrep", "pigpiod"], check=True)
        print("pigpiod is already running.")
    except subprocess.CalledProcessError:
        print("🔧 Starting pigpiod...")
        subprocess.run(["sudo", "pigpiod"])
        time.sleep(1)

def start_linefollower():
    print("Starting line follower...")
    subprocess.run(["python3", LINE_CONTROLLER_PATH])

if __name__ == "__main__":
    try:
        start_pigpiod()
        start_linefollower()
    except KeyboardInterrupt:
        print("\nStopping deployment...")
        sys.exit(0)
