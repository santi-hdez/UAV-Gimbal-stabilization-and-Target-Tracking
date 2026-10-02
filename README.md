# UAV Gimbal Stabilization and Moving-Target Tracking

MATLAB/Simulink project for modeling and controlling a two-axis gimbal mounted on a moving UAV.

The model computes the line of sight (LOS) from a moving UAV to a moving ground target, transforms the target direction into the UAV body frame, and controls the gimbal azimuth and elevation to maintain target tracking despite UAV attitude motion, sensor noise, actuator dynamics, and attitude disturbances.

![UAV gimbal tracking animation](media/uav_gimbal_tracking.gif)

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

## Simulink Model

![Simulink model](docs/model_overview.png)

The simulation can be divided conceptually into four stages:

1. **UAV and target kinematics**
2. **LOS geometry and sensor model**
3. **Gimbal control**
4. **Gimbal dynamics and monitoring**

### UAV and target kinematics

The UAV and target positions are generated as functions of time.

The relative target position in world coordinates is

$$
\mathbf{r}_W = \mathbf{p}_T - \mathbf{p}_U
$$

The UAV attitude is represented by roll, pitch, and yaw.

### World-to-body transformation

The relative target vector is transformed from world coordinates into the UAV body frame using

$$
\mathbf{r}_B = R^T \mathbf{r}_W
$$

with

$$
R = R_z(\psi) R_y(\theta) R_x(\phi)
$$

The body coordinate convention used in the model is:

- +x: forward
- +y: lateral
- +z: upward

Zero gimbal azimuth and elevation therefore correspond to a camera boresight aligned with the UAV +x body axis.

### Line-of-sight angles

The desired gimbal angles are calculated from the target vector expressed in body coordinates:

$$
\mathrm{az}_{\text{des}} =
\operatorname{atan2}(y_B,x_B)
$$

and

$$
\mathrm{el}_{\text{des}} =
\operatorname{atan2}
\left(
z_B,\sqrt{x_B^2+y_B^2}
\right)
$$

Band-limited angular noise is added to the desired LOS measurements before they are supplied to the controller.

### Gimbal controller

Independent PID controllers are used for the azimuth and elevation axes.

Current controller parameters are:

| Axis | Kp | Ki | Kd | N |
|---|---:|---:|---:|---:|
| Azimuth | 2.0 | 0.5 | 0.20 | 100 |
| Elevation | 1.8 | 0.4 | 0.18 | 100 |

Back-calculation anti-windup is enabled with `Kb = 1`.

### Gimbal dynamics

Each gimbal axis is represented using the rotational equation

$$
J\ddot{\theta} + b\dot{\theta} = \tau
$$

The current model parameters are:

| Parameter | Azimuth | Elevation |
|---|---:|---:|
| Inertia J | 0.035 | 0.025 |
| Damping b | 0.06 | 0.05 |
| Maximum angular rate | ±120 deg/s | ±120 deg/s |
| Angular range | ±160 deg | -90 to +30 deg |

These parameters are illustrative values used for the simulation and are not intended to represent a specific commercial gimbal.

## Disturbance and sensor noise

The UAV attitude contains time-varying roll, pitch, and yaw components.

Additional attitude disturbances are introduced during the simulation to evaluate the response of the tracking controller.

The angular measurement noise blocks use a sample time of `0.01 s` and are parameterized to represent approximately `0.05 deg` angular noise.

The disturbances and noise allow the closed-loop response and recovery of the gimbal controller to be examined.

## 3D Visualization

The MATLAB visualization reconstructs the simulation from the exported Simulink signals.

It displays:

- UAV position and attitude
- Moving target
- Recent UAV and target trajectories
- UAV body coordinate axes
- True target LOS
- Actual camera boresight
- Camera field-of-view cone
- Gimbal azimuth and elevation
- Control torques
- True 3D pointing error

The true 3D pointing error is calculated from the angle between the LOS unit vector and camera boresight:

$$
\epsilon =
\cos^{-1}
\left(
\mathbf{u}_{\mathrm{LOS}} \cdot
\mathbf{u}_{\mathrm{cam}}
\right)
$$

The UAV geometry and gimbal size in the animation are deliberately enlarged for visibility and are not drawn to physical scale.

## Results

### Azimuth tracking

![Azimuth tracking](media/azimuth_tracking.png)

### Elevation tracking

![Elevation tracking](media/elevation_tracking.png)

### Tracking error

![Tracking error](media/tracking_errors.png)

The plots illustrate the response of the two-axis controller to the time-varying LOS command, UAV attitude motion, measurement noise, and imposed attitude disturbances.

## Running the project

1. Open MATLAB and Simulink.
2. Open:

   `model/UAV_Gimbal_Stabilization_and_Moving_Target_Tracking.slx`

3. Run the Simulink model.
4. Run:

   `scripts/plot_monitoring_results.m`

   to generate the monitoring plots.

5. Run:

   `scripts/animate_gimbal_tracking.m`

   to generate the 3D visualization and GIF.

## Files

- `UAV_Gimbal_Stabilization_and_Moving_Target_Tracking.slx`  
  Main Simulink model.

- `animate_gimbal_tracking.m`  
  Creates the 3D tracking visualization and animated GIF.

- `plot_monitoring_results.m`  
  Generates tracking-performance plots from the simulation results.

## Notes

This is a simplified engineering model developed as a personal simulation and controls project.

The UAV geometry, gimbal dimensions, actuator parameters, and sensor characteristics are illustrative and are not intended to reproduce a specific real-world platform.
