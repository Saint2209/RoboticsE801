import os
import subprocess
import sys

def prepare_environment():
    """Prepares the environment by installing pip and dependencies."""
    print("Preparing environment...")

    # Step 1: Install pip if it's not installed
    if subprocess.run(["python3", "Movement/get-pip.py"]).returncode == 0:
        print("pip installation successful.")
    else:
        print("Error: Failed to install pip. Please check your environment.")
        return

    # Step 2: Install dependencies from requirements.txt
    requirements_path = "Movement/requirements.txt"
    if os.path.exists(requirements_path):
        print(f"Installing dependencies from {requirements_path}...")
        subprocess.run(["python3", "-m", "pip", "install", "-r", requirements_path])
        print("Dependencies installed.")
    else:
        print(f"Error: {requirements_path} not found.")
        return

    print("Environment prepared successfully.")

if __name__ == "__main__":
    prepare_environment()
