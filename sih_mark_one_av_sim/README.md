# MARK-One: Adaptive Path Planning & Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads
### Smart India Hackathon 2026 | Problem Statement ID: SIH26037 | Theme: Smart Vehicles

---

## 📌 Project Overview

Autonomous vehicle (AV) navigation systems designed for Western highways rely heavily on pristine lane markings, predictable driving discipline, and structured road geometry. In India, however, autonomous vehicles face **unstructured road conditions**:
- **Missing or broken lane markings**: Vehicles must navigate unmarked roads without lane-following assumptions.
- **Organic, variable road geometry**: Uneven road widths, unpaved gravel/dirt shoulders, eroded asphalt edges, and chokepoints.
- **High-density mixed traffic**: Simultaneous interaction with two-wheelers, auto-rickshaws, pedestrians, heavy commercial vehicles, and street vendors.
- **Biological & surface hazards**: Sacred cattle wandering or resting in the roadway, abrupt pedestrian crossings, potholes, construction barricades, and debris.

**MARK-One** is an interactive, real-time engineering simulation and prototype developed for **Smart India Hackathon 2026**. It demonstrates a robust, **closed-loop autonomous driving pipeline** that perceives unstructured environments, forecasts irregular actor movements, dynamically assesses risk, and continuously replans collision-free trajectories using an adaptive trajectory lattice planner.

---

## 🏗️ System Architecture & Closed-Loop Pipeline

The simulation implements the closed-loop architecture proposed in Team MARK-One's technical presentation:

```
┌────────────────────────────────────────────────────────────────────────┐
│                        CLOSED-LOOP AV PIPELINE                         │
└────────────────────────────────────────────────────────────────────────┘

    [ Multi-Sensor Perception ]
       ├── 64-Beam Forward LiDAR (Geometric Raycasting & Point Clouds)
       ├── 77 GHz FMCW Radar (Doppler Velocity & Closing Speed)
       └── 4K Vision Camera (Semantic Segmentation & Pothole Detection)
                   │
                   ▼
    [ Multi-Sensor Fusion Engine ]
       └── State Estimation & High-Confidence Track Association
                   │
                   ▼
    [ Trajectory Prediction & Risk Engine ]
       ├── Spatiotemporal Motion Forecasting (3.0s Horizon)
       └── Risk Index (TTC, Clearance, Biological Vulnerability Multipliers)
                   │
                   ▼
    [ Adaptive Trajectory Lattice Planner ]
       ├── Quintic Polynomial Candidate Generation (C2 Continuous)
       └── Multi-Objective Cost Function Optimization J(τ)
                   │
                   ▼
    [ Pure Pursuit Lateral & Longitudinal Control ]
       ├── Kinematic Bicycle Model Tracking (Steering Angle δ)
       └── Curvature-Aware Speed Governor & Emergency Braking
                   │
                   ▼
    [ Real-Time World & Actuator Update ]
       └── Updates Ego Position ──► Feeds back into Perception
```

---

## 🧮 Mathematical Formulations & Algorithms

### 1. Vehicle Dynamics: Kinematic Bicycle Model
The autonomous ego vehicle is modeled using the planar kinematic bicycle model:
$$\dot{x} = v \sin(\theta)$$
$$\dot{y} = v \cos(\theta)$$
$$\dot{\theta} = \frac{v}{L} \tan(\delta)$$
$$\dot{v} = a$$

Where:
- $(x, y)$: Vehicle lateral and longitudinal positions (m)
- $\theta$: Heading angle relative to road forward axis (rad)
- $v$: Vehicle longitudinal speed (m/s)
- $\delta$: Front wheel steering angle (clamped to $\pm 34.4^\circ$, rate-limited to $1.2$ rad/s)
- $L$: Wheelbase ($2.7$ m)
- $a$: Acceleration / Deceleration command ($-6.0 \le a \le +2.5 \text{ m/s}^2$)

### 2. Multi-Objective Trajectory Cost Function
The lattice planner samples a family of quintic polynomial candidate trajectories $x(s)$ across the drivable asphalt width over a dynamic lookahead distance $S \in [14, 32]\text{m}$. Each candidate trajectory $\tau$ is evaluated by:

$$J(\tau) = w_{col} J_{collision} + w_{bound} J_{boundary} + w_{smooth} J_{smoothness} + w_{center} J_{centerline} + w_{pothole} J_{pothole}$$

- **Collision Repulsion Cost ($J_{collision}$)**: Integrates an exponential repulsive potential field around all tracked obstacles:
  $$J_{collision} = \sum_{t} \sum_{obs} \exp\left( - \frac{(x(t) - x_{obs}(t))^2 + (y(t) - y_{obs}(t))^2}{2 \sigma^2} \right) \cdot W_{vuln}$$
  *(If clearance violates the hard safety margin, $J_{collision} = \infty$, rendering the trajectory infeasible).*
- **Road Boundary Cost ($J_{boundary}$)**: Penalizes trajectories approaching or crossing unpaved gravel shoulders and ditches.
- **Biological Vulnerability Weight ($W_{vuln}$)**: Elevated safety margins applied to biological actors:
  - Cattle: $W_{vuln} = 2.0$
  - Pedestrians: $W_{vuln} = 2.2$
- **Surface Damage Penalty ($J_{pothole}$)**: Penalizes tire path intersections with detected road potholes to prevent rim/suspension damage.
- **Smoothness Cost ($J_{smoothness}$)**: Minimizes trajectory curvature $\kappa$ and lateral acceleration to prevent passenger discomfort and wheel slip.

### 3. Trajectory Tracking: Pure Pursuit Controller
The lateral steering angle is computed using lookahead distance $L_d(v) = k_v \cdot v + L_0$:
$$\alpha = \text{atan2}(x_{target} - x_{ego}, y_{target} - y_{ego}) - \theta_{ego}$$
$$\delta = \text{atan2}\left( \frac{2 L \sin(\alpha)}{L_d} \right)$$

---

## 🚦 Validation Scenarios

The prototype includes **10 interactive scenarios** covering challenging Indian road situations:

1. **Scenario 1: Normal Indian Mixed Traffic** — Autos, bikes, and cars weaving irregularly with fragmented lane markings.
2. **Scenario 2: Slow Vehicle Corridor Overtake** — Slow auto-rickshaw blocking current lane; AV evaluates oncoming gaps and safely overtakes.
3. **Scenario 3: Informal Pedestrian Crossing** — Pedestrian abruptly walks across the road; AV detects high risk, slows down, and swerves around them.
4. **Scenario 4: Severe Pothole Avoidance** — Deep asphalt craters directly on nominal track; AV detects surface defect and executes smooth evasive maneuver.
5. **Scenario 5: Parked Vehicle & Roadside Encroachment** — Parked delivery tempo and roadside vendor pinch the road; AV threads the remaining gap.
6. **Scenario 6: Mixed Multi-Obstacle Chaos** — Car on right, pothole in center, bike weaving on left; multi-criteria optimization.
7. **Scenario 7: Sudden Cut-In / Emergency Deceleration** — Fast motorcycle cuts directly into ego vehicle buffer; AV executes rapid lateral replan and braking.
8. **Scenario 8: Narrow Chokepoint & Barricades** — Unstructured rural road narrows with construction barriers on both flanks.
9. **Scenario 9: Cattle-Crossing Event (SIH PPT Scenario #05)** — Sacred cow halts in middle of drivable lane; AV applies biological buffer & gentle detour.
10. **Scenario 10: Dense Market Area (SIH PPT Scenario #04)** — High-density bazaar zone with hand-carts, parked bikes, pedestrians, and tight gaps.

### ✨ Automated 75-Second SIH Presentation Demo Mode
Click the **"✨ SIH DEMO MODE"** button to trigger a choreographed 75-second multi-phase demonstration with an on-screen narrative ticker guiding judges through perception initialization, boundary reasoning, overtaking, cattle avoidance, pothole replanning, and chokepoint convergence.

---

## 🎮 Interactive Judge Features

- **Live Obstacle Injector**: Select an obstacle (**Cow, Pothole, Pedestrian, Auto, Barricade, Bike**) and **click anywhere on the road canvas** to drop it directly in front of the vehicle. Watch the lattice planner recalculate candidate paths in real-time!
- **Sensor Overlay Toggles**: Inspect individual perception layers (64-beam LiDAR rays, Radar Doppler cones, Camera vision frustum, Lattice candidates, Safe corridor).
- **Simulation Speed Controls**: Slow-motion ($0.5\times$) for detailed algorithmic analysis, normal ($1.0\times$), or fast ($2.0\times$).
- **Single-Step Physics**: Pause and step frame-by-frame ($16.6\text{ms}$) to inspect the decision matrix and cost breakdown.

---

## 🚀 How to Run the Simulation

### Option 1: One-Click Python Launcher (Recommended)
Open a terminal in this directory and execute:
```bash
python run_sim.py
```
This starts the local zero-dependency HTTP server and automatically opens `http://localhost:8000/index.html` in your default browser.

### Option 2: Standard Python HTTP Server
```bash
python -m http.server 8000
```
Then navigate to: [http://localhost:8000](http://localhost:8000)

---

## 📂 Project Structure

```
sih_mark_one_av_sim/
├── index.html                   # Master Dashboard UI and Simulation Viewport
├── css/
│   └── style.css                # Dark Automotive HUD Theme & Telemetry Styles
├── js/
│   ├── config.js                # Physics parameters, sensor specs & cost weights
│   ├── main.js                  # Closed-loop orchestrator & animation loop
│   ├── models/
│   │   ├── Road.js              # Variable-width unstructured Indian road model
│   │   ├── Vehicle.js           # Kinematic bicycle model with steer & brake dynamics
│   │   └── Obstacle.js          # Multi-class Indian obstacles (Cattle, Auto, Pothole, etc.)
│   ├── perception/
│   │   ├── Sensors.js           # Simulated 64-beam LiDAR, FMCW Radar, and Camera
│   │   └── SensorFusion.js      # Multi-modal track association & state fusion
│   ├── risk/
│   │   └── RiskAssessment.js    # TTC calculation, clearance metrics & threat classification
│   ├── planning/
│   │   ├── TrajectoryPredictor.js # Spatiotemporal short-term motion forecasting
│   │   └── LatticePlanner.js    # Multi-objective candidate lattice planner
│   ├── control/
│   │   └── VehicleController.js # Pure Pursuit lateral tracker & longitudinal speed governor
│   ├── scenarios/
│   │   └── ScenarioManager.js   # 10 Indian road scenarios & 75s Demo Mode director
│   └── ui/
│       ├── Renderer.js          # High-DPI Canvas rendering engine
│       └── Dashboard.js         # Telemetry HUD update bindings & controls
├── run_sim.py                   # One-click Python server launcher
├── verify_sim.py                # Automated test script verifying files & endpoints
└── README.md                    # Comprehensive documentation
```

---

## 🔍 Limitations & Future Enhancements

### Limitations of Current Prototype
1. **Planar 2.5D Assumption**: Pitch and roll road gradient variations (banking angles) are simplified to a 2D planar ground representation.
2. **Sensor Noise Modeling**: Current sensor outputs assume Gaussian measurement noise; extreme monsoon precipitation or dense dust storms are not yet physically simulated.
3. **Rule-Assisted Lattice**: Candidate offsets are sampled at uniform discrete lateral intervals rather than continuous optimization via nonlinear Model Predictive Control (NMPC).

### Future Roadmap for Smart India Hackathon 2026 Deployment
- **Integration with Real LiDAR SDK**: Interface with physical Livox / Velodyne LiDAR ROS 2 topics.
- **Deep Reinforcement Learning (DRL) Hybridization**: Pre-filter candidate trajectories using a policy network trained on real Indian driving datasets (e.g. IDD - Indian Driving Dataset).
- **V2X Communication**: Integrate Vehicle-to-Everything beaconing for blind corner warning at unsignalized rural intersections.

---

### Team MARK-One — Smart India Hackathon 2026
*Advancing Autonomous Mobility for the Realities of Indian Roads.*
