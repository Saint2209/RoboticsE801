import os
import subprocess
import time

# Global variable to track if ROS 2 is already running
ros2_process = None

def list_options():
    """Displays the available task options to the user."""
    print("Please choose a task to run:")
    print("1. PID")
    print("2. Right Edge Detection")
    print("3. Obstacle Avoidance")
    print("4. Combination")
    print("0. Exit")


def run_ros2_launch():
    """Launch ROS 2 bringup once and listen continuously."""
    global ros2_process
    if ros2_process is None:
        print("Starting ROS 2 bringup...")
        # Run the ROS 2 launch command to start the robot (launches in the background)
        ros2_process = subprocess.Popen(
            "ros2 launch turtlebot3_bringup robot.launch.py",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        # Allow some time for ROS 2 to initialize
        time.sleep(2)
        print("ROS 2 bringup started.")
    else:
        print("ROS 2 is already running.")

def stop_ros2_launch():
    """Stops the ROS 2 bringup process."""
    global ros2_process
    if ros2_process:
        print("Stopping ROS 2 bringup...")
        ros2_process.terminate()  # Terminate the ROS 2 bringup process
        ros2_process = None
        print("ROS 2 bringup stopped.")
    else:
        print("ROS 2 is not running.")


def run_task(choice):
    """Runs the selected task based on user input."""
    if choice == 1:
        # Running Tasks - PID
        script = "Tasks/1. PID/main.py"
    elif choice == 2:
        # Running Tasks - Right Edge Detection
        script = "Tasks/2. Right Edge Detection/main.py"
    elif choice == 3:
        # Running Tasks - Obstacle Avoidance
        script = "Tasks/3. Obstacle Avoidance/main.py"
    elif choice == 4:
        # Running Tasks - Combination
        script = "Tasks/4. Combination/main.py"
    else:
        print("Invalid choice. Exiting.")
        return

    # Check if the script exists before running it
    if os.path.exists(script):
        print(f"Running {script}...")

        # Step 1: Export TURTLEBOT3_MODEL (one-time setup)
        subprocess.run("export TURTLEBOT3_MODEL=burger", shell=True, check=True)

        # Step 2: Launch ROS 2 (if not already running)
        run_ros2_launch()

        # Step 3: Run the Python script
        subprocess.run(["python3", script])  # Ensure python3 is used
    else:
        print(f"Error: {script} not found.")


def main():
    while True:
        list_options()
        try:
            choice = int(input("Enter your choice (0 to exit): "))
            if choice == 0:
                print("Exiting...")
                stop_ros2_launch()  # Stop ROS 2 bringup when exiting
                break
            run_task(choice)
        except ValueError:
            print("Invalid input. Please enter a number.")


if __name__ == "__main__":
    main()
