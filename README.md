# Industrial Robot Simulation

<img width="803" height="747" alt="image" src="https://github.com/user-attachments/assets/aef543d3-34f8-401c-a070-7080b3098f20" />


A Python-based simulation of an industrial robot arm performing pick-and-place tasks on a conveyor belt. This project demonstrates basic robotics concepts, state machine logic, and real-time visualization using `matplotlib`.

## Features

*   **Robot Simulation**:
    *   Simulates a robot arm with movement and grasping capabilities.
    *   Implements a state machine (IDLE, MOVING, PICKING, PLACING, ERROR).
    *   Handles edge cases like mechanical failures and grasp misses.
*   **Dynamic Environment**:
    *   Conveyor belt system with moving objects.
    *   Randomized object generation with positional noise (misplacement).
*   **Real-Time Visualization**:
    *   Live animation of the robot and environment.
    *   Visual feedback for robot state (Blue = Normal, Red = Error).
    *   On-screen dashboard showing success rates and failure counts.

## Requirements

*   Python 3.x
*   `numpy`
*   `matplotlib`

## Installation

1.  Clone the repository:
    ```bash
    git clone https://github.com/yourusername/industrial-robot-sim.git
    cd industrial-robot-sim
    ```

2.  Install dependencies:
    ```bash
    pip install numpy matplotlib
    ```

## Usage

Run the simulation script:

```bash
python robot_simulation.py
```

A window will open displaying the simulation. The robot will attempt to pick up objects from the conveyor belt and place them in the drop zone.

*   **Close the window** to stop the simulation.

## Simulation Details

*   **Success Rate**: Tracks the percentage of successful pick-and-place operations.
*   **Failures**:
    *   **Grasp Failure**: 10% chance to miss an object during pickup.
    *   **Mechanical Failure**: 0.5% chance per frame for the arm to malfunction (requires auto-reset).
