# Active Control: MIT Rocket Team
*Active Control Algorithms and Simulation Testing for MIT Rocket Team.*

*For a more comprehensive explanation, see [README.pdf](https://github.com/MDNich/ActiveControl_MIT_RktTeam/blob/main/README.pdf).*

This repository contains tools and code for advanced simulation and control of rocket flight using both Java (OpenRocket) and Python. The workflow integrates custom Java simulation logic with Python scripting and visualization.

## Overview
A Python-based tool for rocket flight simulation that integrates with OpenRocket to analyze and visualize flight characteristics. This project focuses on processing and visualizing rocket flight data, particularly useful for analyzing pitch rates and velocities during flight.

## Prerequisites
- Python 3.12+
- Java Development Kit (JDK) 17
- `matplotlib`
- `numpy`
- `jpype`

## Environment Setup

Python Dependencies:
   ```bash
   pip install numpy matplotlib jpype1
   ```
## Usage Instructions

### 1. Compiling and Running

- **Use the `OR_interfaceTest + JAR` run configuration** in your IDE to:
  - Compile the Java code (including your modifications).
  - Run the Python script that interfaces with the compiled Java classes.

### 2. Editing Simulation and Control Logic

- **Simulation Flow:**  
  Edit only `ModifiedEventSimulationEngine.java` to manage the overall simulation flow, such as event handling, simulation stepping, and integration with listeners.

- **Controller Behavior:**  
  Edit `NewControlStepListener.java` to modify the controller's behavior and properties. This file is responsible for how the control system interacts with the simulation at each step.

- **Python Scripting:**  
  Edit `openRocketInterface.py` (located in `sim/src/py/` or similar) to change how the Python script interacts with the Java code. This script is used to run specific tests, automate simulation runs, and collect results.

## Python Script: `openRocketInterface.py`

This script uses `jpype` to launch the JVM and interface directly with the compiled Java classes. It allows you to:

- **Control Java Variables:**  
  The script can set and get various simulation parameters in Java, such as:
  - Time step size (variable `prefDt` in `openRocketInterface.py`)
  - Controller gains and setpoints 
  - Initial conditions (e.g., launch angle, velocity)
  - Simulation duration and event triggers

- **Run and Automate Simulations:**  
  You can script multiple runs, parameter sweeps, or custom test scenarios by calling Java methods from Python.

- **Graphing and Visualization:**  
  The script collects simulation output (e.g., altitude, velocity, control surface deflections, error signals) and generates plots for analysis. The default graph plots two panels, showing
  - Altitude, Velocity vs. time
  - Control output (fin cant) and rotational velocity vs. time

## Repository File Structure

This repository is organized as follows:

- `README.md` 
  Project documentation and usage instructions.

- `bib/`  
  Reference materials and research papers.

- `canard/`, `ctrl/`, `wing/`  
  Subdirectories for specific rocket components, control models, and aerodynamic studies.

- `clone/`  
  Contains OpenRocket source code and related files:
  - `openrocket/` and `openrocket-release-24.12.RC.01/`  
    OpenRocket Java source, build scripts, and documentation.
    `openrocket/` is a symlink to the openrocket version used, currently `openrocket-release-24.12.RC.01/`.

- `land.tbd/`  
  Landing simulation (tbd - i.e. in progress) documentation and related files.

- `sim/`  
  Main simulation code and data:
  - `src/py/`  
    Python scripts for simulation and Java-Python integration (e.g., `openRocketInterface.py`).
 - `src/java/`  
    A symlink to the Java source of the used openrocket distribution - for ease of navigation.
  - `dat/`  
    Simulation results and data output.

- `stabil/`  
  Additional files pertaining to the roll controller, mostly ideation.

- `test/`  
  Test scripts and utilities.

Each directory contains files relevant to its purpose, such as source code, data, documentation, or research materials. The main simulation workflow is in the `sim/` directory, with integration to Java code in `clone/openrocket-release-24.12.RC.01/`.

---
