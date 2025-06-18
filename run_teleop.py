import subprocess
import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLER_PATH = os.path.join(BASE_DIR, "controller.py")

def start_teleop():
    print("Starting teleoperation...")
    subprocess.run(["python3", CONTROLLER_PATH])

if __name__ == "__main__":
    try:
        start_teleop()
    except KeyboardInterrupt:
        print("\nStopping teleop...")
        sys.exit(0)
