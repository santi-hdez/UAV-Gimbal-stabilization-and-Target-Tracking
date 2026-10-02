# UAV Gimbal Stabilization and Moving-Target Tracking

MATLAB/Simulink project for modeling and controlling a two-axis gimbal mounted on a moving UAV.

The model computes the line of sight (LOS) from a moving UAV to a moving ground target, transforms the target direction into the UAV body frame, and controls the gimbal azimuth and elevation to maintain target tracking despite UAV attitude motion, sensor noise, actuator dynamics, and attitude disturbances.

---

## Overview

The simulation includes:

- Moving UAV and ground-target trajectories
- Time-varying UAV roll, pitch, and yaw
- World-to-body coordinate transformation
- LOS azimuth and elevation calculation
- Angular measurement noise
- Independent azimuth and elevation PID controllers
- Two-axis rotational gimbal dynamics
- Torque, angular-rate, and angular-position constraints
- Tracking-error monitoring
- 3D visualization of the UAV, target, LOS, camera boresight, and field of view

---

## Simulink Model

The simulation can be divided conceptually into four stages:

1. **UAV and target kinematics**
2. **LOS geometry and sensor model**
3. **Gimbal control**
4. **Gimbal dynamics and monitoring**

![Simulink model](media/model_overview.png)

### UAV and target kinematics

The UAV and target positions are generated as functions of time.

The relative target position in world coordinates is

$$
\mathbf{r}_W = \mathbf{p}_T - \mathbf{p}_U
$$

where $\mathbf{p}_U$ and $\mathbf{p}_T$ are the UAV and target positions, respectively.

The UAV attitude is represented by roll $\phi$, pitch $\theta$, and yaw $\psi$.

### World-to-body transformation

The relative target vector is transformed from world coordinates into the UAV body frame using

$$
\mathbf{r}_B = R^T \mathbf{r}_W
$$

with the body-to-world rotation matrix

$$
R = R_z(\psi)R_y(\theta)R_x(\phi)
$$

The body coordinate convention used in the model is:

- +x: forward
- +y: lateral
- +z: upward

Zero gimbal azimuth and elevation therefore correspond to a camera boresight aligned with the UAV +x body axis.

---

### Line-of-sight angles

The desired gimbal angles are calculated from the target vector expressed in body coordinates:

$$
\mathrm{az}_{\mathrm{des}} =
\mathrm{atan2}(y_B,x_B)
$$

and

$$
\mathrm{el}_{\mathrm{des}} =
\mathrm{atan2}\left(z_B,\sqrt{x_B^2+y_B^2}\right)
$$

Angular measurement noise is added to the desired LOS angles before they are supplied to the controller.

---

### Gimbal controller

Independent PID controllers are used for the azimuth and elevation axes.

The controller uses the angular tracking error between the measured desired LOS direction and the actual gimbal orientation to generate the commanded torque for each axis.

Current controller parameters are:

| Axis | Kp | Ki | Kd | N |
|---|---:|---:|---:|---:|
| Azimuth | 2.0 | 0.5 | 0.20 | 100 |
| Elevation | 1.8 | 0.4 | 0.18 | 100 |

Back-calculation anti-windup is enabled with `Kb = 1`.

The azimuth error is wrapped to the interval [-pi, pi] to avoid discontinuities when crossing the angular boundary.

---

### Gimbal dynamics

Each gimbal axis is modeled using the rotational equation

$$
J\ddot{\theta} + b\dot{\theta} = \tau
$$

where:

- $J$ is the rotational inertia
- $b$ is the damping coefficient
- $\theta$ is the gimbal angle
- $\tau$ is the applied torque

The current model parameters are:

| Parameter | Azimuth | Elevation |
|---|---:|---:|
| Inertia J | 0.035 | 0.025 |
| Damping b | 0.06 | 0.05 |
| Maximum torque | ±1.5 N m | ±1.5 N m |
| Maximum angular rate | ±120 deg/s | ±120 deg/s |
| Angular range | Continuous | -90 to +30 deg |

---

## Disturbances and sensor noise

The UAV trajectory contains time-varying position and attitude components, requiring the gimbal to continuously adapt its orientation to maintain the target within the camera line of sight.

Additional attitude disturbances are introduced during the simulation to examine the transient response of the tracking controller.

Angular measurement noise is also included in the LOS measurements.

Together, these effects provide a simple test environment for evaluating the tracking response under changing operating conditions.

---

## 3D Visualization

A MATLAB script reconstructs the simulation using data exported from Simulink and generates a 3D visualization of the tracking problem.

The visualization displays:

- UAV position and attitude
- Moving ground target
- Recent UAV and target trajectories
- UAV body coordinate axes
- True target LOS
- Actual camera boresight
- Camera field-of-view cone
- Gimbal azimuth and elevation
- Control torques
- True 3D pointing error

The camera direction is first constructed in UAV body coordinates from the simulated gimbal angles and then transformed into world coordinates using the UAV attitude.

The true 3D pointing error is calculated from the angle between the LOS unit vector and camera boresight:

$$
\epsilon =
\cos^{-1}\left(
\mathbf{u}_{\mathrm{LOS}} \cdot
\mathbf{u}_{\mathrm{cam}}
\right)
$$

The UAV geometry and gimbal size in the animation are deliberately enlarged for visibility and are not drawn to physical scale.

The field-of-view cone is also intended primarily as a visualization aid rather than as a optical camera model.
---

## Repository structure

```text
uav-gimbal-tracking-simulink/
│
├── README.md
├── LICENSE
│
├── model/
│   └── UAV_Gimbal_Stabilization_and_Moving_Target_Tracking.slx
│
├── animation_script/
│   ├── animate_gimbal_tracking.m
│
├── media/
│   ├── uav_gimbal_tracking.gif
│   ├── model_overview.png

```
---

## Running the project

1. Clone or download the repository.

2. Open MATLAB and Simulink.

3. Open the Simulink model:

   `model/UAV_Gimbal_Stabilization_and_Moving_Target_Tracking.slx`

4. Run the Simulink simulation.

5. Run:

   `scripts/plot_monitoring_results.m`

   to generate the tracking-performance plots.

6. Run:

   `scripts/animate_gimbal_tracking.m`

   to generate the 3D visualization and animated GIF.

---

## Result

![tracking](media/movie_rho.gif)

---

## Author

**Santiago Hernández Díaz**

PhD candidate in Physics
University of Tübingen


