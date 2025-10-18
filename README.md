# DroneHordes - A Swarm Control Simulation Sandbox

**Project Status: Experimental / Under Development**

This project serves as a sandbox environment for exploring emergent behaviors and control strategies for a drone swarm.

The primary goal is not to create a physically accurate collision simulation, but rather to investigate which parameters and behaviors are most effective for a human operator to manage a "horde" of semi-autonomous agents within a defined space.

---

## Concept

The simulation explores different behavioral modes that can be switched in real-time. The core idea is to lay the groundwork for understanding how a set of simple rules (e.g., Boids, mouse following) can be fine-tuned to perform complex collective tasks.

---

## Getting Started

### Clone

Just clone this git repository main branch.

```bash
git clone https://github.com/cesardelarosa/DroneHordes.git && cd DroneHordes
```

### Prerequisites

You will need Python 3 and the libraries listed in `requirements.txt`.

```bash
# Install dependencies
pip install -r requirements.txt
```
*(Currently: `pygame` and `numpy`)*

### Running the Simulation

The simulation can be run from the command line. Use the `-n` or `--count` argument to specify the number of drones.

```bash
# Run with the default number of drones (50)
python main.py

# Run with 200 drones
python main.py -n 200
```

---

## Controls

While the simulation is running, you can switch the swarm's behavior using the following keys:

* **1**: **Ideal Gas Mode**
    * Drones move with constant velocity and reflect off walls.
* **2**: **Follow Mouse Mode**
    * Drones are attracted to the cursor's position, with applied friction.
* **3**: **Flocking (Boids) Mode**
    * Drones apply the three classic Boids rules (Separation, Alignment, Cohesion) to move as a cohesive flock.
* **Q** or **ESC**: Exit the simulation.

---

## Code Structure

* **`main.py`**: Handles the main application loop, event processing, and rendering (Pygame).
* **`config.py`**: Contains all constants and tunable parameters (e.g., speeds, radii, behavior weights).
* **`swarm.py`**: Defines the `Drone` and `Swarm` classes. Manages the state, physics (collision resolution), and statistics of the swarm.
* **`behaviors.py`**: Implements the different behavioral modes using the Strategy design pattern, allowing for easy extension with new behaviors.

---

## Future Roadmap

This project is in its early stages. The plan is to continue exploring the following areas:

* **Hybrid Behaviors:** Combine modes, such as "Follow Mouse" with "Boids," to create a flock that follows a leader (the cursor).
* **Objective-Based Control:** Implement target zones on the screen (e.g., "go here" waypoints or "avoid this area" zones).
* **Horde Ergonomics:** Investigate how the parameters in `config.py` (e.g., aggression, cohesion, separation) affect the swarm's "manageability" from an operator's perspective.
* **Performance Optimization:** Implement spatial partitioning techniques (such as a Quadtree) to efficiently simulate thousands of drones.
