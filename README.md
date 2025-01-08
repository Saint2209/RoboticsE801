# README for CM24681: Fuzzy Logic and Movement Systems

## Project Overview

This project, titled **Fuzzy Logic and Movement Systems**, demonstrates the implementation of various systems and algorithms for movement control and obstacle handling using Python. It includes tasks such as PID control, right-edge detection, and obstacle avoidance, alongside a combined approach integrating these functionalities.

## Important Note

**These scripts are intended to be executed only after SSHing into the robot's system.** Ensure that you are connected to the robot via SSH and have the necessary permissions to run these scripts.

## Directory Structure

```
Dir/
│
├── prep.py              # Prepares the environment for execution (configures dependencies and environment settings).
├── run.py               # Entry point script to execute the entire system.
│
├── Movement/
│   ├── FuzzyLogicSystem.py   # Implements the fuzzy logic system.
│   ├── get-pip.py            # Script to install pip for dependency management.
│   ├── PIDCalculator.py      # PID calculator module.
│   ├── Readme                # Existing documentation for the 'Movement' module.
│   ├── requirements.txt      # Dependencies required for the project.
│
├── Tasks/
│   ├── 1. PID/
│   │   └── main.py           # Main script for PID control.
│   │
│   ├── 2. Right Edge Detection/
│   │   ├── main.py           # Script for  Fuzzy logic: right edge detection.
│   │   ├── right_edge.csv    # Sample CSV data for testing.
│   │   └── right_edge.json   # JSON configuration or output for edge detection.
│   │
│   ├── 3. Obstacle Avoidance/
│   │   ├── main.py           # Script for Fuzzy logic: obstacle avoidance logic.
│   │   ├── object_avoidance.csv
│   │   └── object_avoidance.json
│   │
│   ├── 4. Combination/
│       ├── main.py           # Combines the Fuzzy behaviors tasks.
│       ├── object_avoidance.csv
│       ├── object_avoidance.json
│       ├── right_edge.csv
│       └── right_edge.json
```

## Key Features

1. **PID Control**:
   - Implements Proportional, Integral, and Derivative control logic.
   - Script: `Tasks/1. PID/main.py`

2. **Right Edge Detection**:
   - Detects the right edge of a path using sensor data.
   - Script: `Tasks/2. Right Edge Detection/main.py`
   - Input/Output: `right_edge.csv`, `right_edge.json`

3. **Obstacle Avoidance**:
   - Identifies and avoids obstacles in the path.
   - Script: `Tasks/3. Obstacle Avoidance/main.py`
   - Input/Output: `object_avoidance.csv`, `object_avoidance.json`

4. **Combination Task**:
   - Integrates PID control, edge detection, and obstacle avoidance into a cohesive system.
   - Script: `Tasks/4. Combination/main.py`

5. **Fuzzy Logic System**:
   - Utilizes fuzzy logic to make decisions based on input conditions.
   - Script: `Movement/FuzzyLogicSystem.py`

## Installation and Setup

1. **SSH into the Robot**:
   - Use SSH to connect to the robot's system:
     ```bash
     ssh user@robot-ip-address
     ```

2. **Clone the Repository**:
   - Clone the repository onto the robot:
     ```bash
     git clone <repository-url>
     cd Dir
     ```

3. **Prepare the Environment**:
   - Run the `prep.py` script to install dependencies and configure the environment:
     ```bash
     python prep.py
     ```

4. **Run the System**:
   - Use the `run.py` script as the main entry point:
     ```bash
     python run.py
     ```

## Usage

- Modify the configuration files (`right_edge.json`, `object_avoidance.json`) to tailor the system to your requirements.
- Scripts in the `Tasks` folder can be executed individually for specific tasks or combined using the `run.py` script.

## Dependencies

The necessary dependencies will be automatically installed when running `prep.py`.

## Notes

- For detailed documentation, refer to the `Readme` file under the `Movement` directory.
- Sample data files (`.csv` and `.json`) are provided for testing and demonstration purposes.

## Author

**Thabang Maribana[https://github.com/Saint2209]**  
MSc Artificial Intelligence Candidate
University of Essex, 2024/2025

## Credits

This README was prepared with the assistance of ChatGPT, an AI model developed by OpenAI.
