# Product Requirements Document: Orbit RL

## 1. Overview

This project aims to create a 2D N-body gravitational simulation in Rust, featuring a user/AI-controllable spacecraft. The simulation will serve as an environment for Reinforcement Learning (RL) agents, implemented in Python, to learn complex orbital mechanics tasks, such as achieving stable orbits, transferring between celestial bodies, and rendezvous maneuvers.

## 2. Goals

* Develop a performant and reasonably accurate 2D N-body physics simulation in Rust.
* Implement a controllable spacecraft within the simulation with its own propulsion physics.
* Create a clear interface (API) between the Rust simulation and external controllers (initially Python RL agents).
* Provide visualization of the simulation state.
* Enable Python RL agents to observe the simulation state and control the spacecraft's thrusters.
* Successfully train RL agents to perform basic orbital tasks.

## 3. Functional Requirements

### 3.1. Rust Simulation Core (`orbit_rl` crate)

* **Physics Engine:**
  * Calculate gravitational forces between N celestial bodies (planets, moons, stars).
  * Configurable gravitational constant (`G`).
  * Implement a stable numerical integration method (e.g., Velocity Verlet, Runge-Kutta 4) for updating positions and velocities of celestial bodies and the spacecraft.
  * Handle collisions (initially, simple detection and maybe stopping the simulation or marking the spacecraft as crashed). Define collision behavior (e.g., elastic, inelastic, spacecraft destruction).
  * Allow configuration of initial simulation state (body masses, positions, velocities) potentially via a configuration file (e.g., TOML, JSON).
* **Celestial Bodies:**
  * Represent bodies with properties: mass, radius, position, velocity.
  * Bodies should interact gravitationally with all other bodies (including the spacecraft).
  * Bodies should be visually distinct (e.g., size related to mass/radius, different colors).
* **Spacecraft:**
  * Represent the spacecraft with properties: mass (potentially variable if fuel is consumed), position, velocity.
  * Implement controllable thrusters:
    * Allow applying force in a specific direction relative to the spacecraft's orientation.
    * Specify maximum thrust force.
    * (Optional) Model fuel consumption and limited fuel supply.
    * (Optional) Allow spacecraft rotation/orientation control.
  * Spacecraft is affected by gravity from all celestial bodies.
  * Spacecraft has a visual representation.
* **Simulation Control:**
  * Ability to step the simulation forward by a discrete time step (`dt`).
  * Ability to reset the simulation to its initial state.
  * Ability to get the current state of all bodies and the spacecraft (positions, velocities, etc.).
* **Visualization (Optional but Recommended for Debugging):**
  * Use a Rust library (e.g., `ggez`, `bevy`, `macroquad`) to render the simulation state in real-time.
  * Display celestial bodies and the spacecraft.
  * (Optional) Display velocity vectors or predicted trajectories.
  * (Optional) Allow basic camera controls (pan, zoom).

### 3.2. Python Interface / RL Environment Wrapper

* **Communication Bridge:**
  * Establish a mechanism for Python to call Rust functions and exchange data (e.g., PyO3, CFFI, ZeroMQ, gRPC, simple text protocol over stdin/stdout). PyO3 is likely the most idiomatic approach for tight integration.
* **Environment API (Gymnasium/PettingZoo standard):**
  * Wrap the Rust simulation in a Python class that adheres to a standard RL environment interface like Gymnasium (`gymnasium.Env`).
  * `reset()`: Resets the Rust simulation and returns the initial observation.
  * `step(action)`: Takes an action (e.g., thrust vector/magnitude, duration), steps the Rust simulation, calculates the reward, determines if the episode is done, and returns the next observation, reward, terminated, truncated, info.
  * `observation_space`: Define the structure and bounds of the state information provided to the agent (e.g., positions/velocities of bodies and spacecraft, relative positions/velocities, spacecraft fuel).
  * `action_space`: Define the structure and bounds of the actions the agent can take (e.g., continuous values for thrust vector components, discrete actions for fixed thrusters).
  * `reward_function`: Define how rewards are calculated based on the simulation state and the agent's goals (e.g., negative reward for fuel use, positive reward for approaching target orbit, large penalty for crashing).
  * `termination/truncation conditions`: Define when an episode ends (e.g., crash, out of fuel, time limit reached, goal achieved).

### 3.3. RL Agent (Python)

* Use standard RL libraries (e.g., Stable Baselines3, Ray RLlib, Tianshou) to implement/train agents.
* Agents will interact with the Python environment wrapper.
* Training scripts to run the simulation and learning process.
* Evaluation scripts to test trained agent performance.

## 4. Non-Functional Requirements

* **Performance:** The Rust simulation should be fast enough to allow for reasonably quick RL training iterations.
* **Accuracy:** The physics simulation should be accurate enough for meaningful orbital mechanics learning (choice of integrator is key).
* **Modularity:** Separate concerns between the core physics simulation, the spacecraft logic, the visualization, and the Python interface.
* **Configuration:** Allow easy configuration of simulation parameters and RL environment settings.
* **Cross-Platform:** Should ideally run on common operating systems (Linux, macOS, Windows).

## 5. Milestones (Potential Breakdown)

1. **M1: Core Rust Simulation:** Implement basic N-body physics (gravity, integration) for celestial bodies only. Add basic visualization.
2. **M2: Spacecraft Implementation:** Add the spacecraft object, basic thruster physics (apply force), and include it in the simulation loop and visualization.
3. **M3: Python Interface (PyO3):** Set up PyO3 bindings. Expose basic simulation control (`reset`, `step`, `get_state`) to Python. Test calling Rust from Python.
4. **M4: Gymnasium Environment:** Create the Python `gymnasium.Env` wrapper. Define initial observation/action spaces and a simple reward function/termination condition.
5. **M5: Basic RL Training:** Implement a simple RL agent (e.g., PPO, SAC) using a library and train it on a very basic task (e.g., reach a specific point, increase orbital energy).
6. **M6: Refinement & Complex Tasks:** Improve physics accuracy (integrator, collisions), refine reward functions, define more complex tasks (stable orbit, orbital transfer), enhance visualization.

## 6. Open Questions / Future Considerations

* Specific choice of numerical integrator? (Start simple like Verlet, potentially move to RK4 if needed).
* Detailed collision physics model? (Simple detection first).
* Spacecraft rotation model? (Start without rotation).
* Variable spacecraft mass (fuel consumption)? (Start with fixed mass).
* Specific IPC method if not PyO3?
* Configuration file format and structure?
* Advanced visualization features (trails, telemetry)?
* Parallelization for multiple environments during training?
