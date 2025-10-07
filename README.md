# RF Coverage Mapping Simulator

An interactive Python simulator for RF (Wi-Fi) coverage mapping with autonomous and manual robot navigation using gradient ascent optimization.

---

## Overview

This simulator models and visualizes RSSI-based RF coverage mapping using both autonomous and manual control modes.  
It supports gradient-based navigation, trajectory export, and real-time visualization for research, education, and poster demonstrations.

### Key Components
- Analytical RSSI field generation using NumPy and Gaussian smoothing  
- Real-time PyGame visualization  
- Autonomous gradient ascent navigation (SPSA-like)  
- Manual control via keyboard input  
- Trajectory export to CSV for data analysis  

---

## Features
- Realistic RSSI field generation with Gaussian smoothing  
- Real-time gradient ascent and manual navigation modes  
- Live RSSI display and iteration counter  
- Visual robot tracking and path history  
- Map regeneration and CSV export  
- Configurable parameters via JSON  

---

## Installation

### Requirements
- Python 3.10 or higher  
- pip package manager  

### Setup
Clone this repository and install dependencies:
```bash
pip install -r requirements.txt
```
---

## Usage
Run the simulator from the `wifi_simulator` directory:
```bash
cd wifi_simulator
python main.py
```
## Project Structure
wifi_simulator/
│
├── main.py              # Entry point and visualization loop
├── map_generator.py     # RSSI field generation
├── robot.py             # Robot state and movement logic
├── simulation.py        # Auto/manual navigation control
├── utils.py             # Helper functions and exports
├── config.json          # Simulation parameters
├── requirements.txt     # Dependencies
└── README.md


## Configuration
Simulation parameters can be modified in config.json:

{
  "map_size": 100,
  "window_size": 600,
  "robot_radius": 8,
  "learning_rate": 0.5,
  "gradient_delta": 2.0,
  "noise_std": 0.3,
  "manual_speed": 2.0,
  "manual_speed_boost": 3.0,
  "gaussian_sigma": 8.0,
  "rssi_min": -90,
  "rssi_max": -30,
  "fps": 60,
  "colormap": "plasma"
}
