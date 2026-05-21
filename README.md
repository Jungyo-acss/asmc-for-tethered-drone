# ASMC-Controller

Implementation-oriented **Adaptive Sliding Mode Control (ASMC)** framework for a cooperative **Winch-tethered UAV** system, integrated with **PX4 SITL + Gazebo + ROS**.

---

## 1. Control

### 1.1 Hierarchical Outer–Inner Loop

This project uses a cascaded control architecture for a winch-tethered quadrotor:

- **Outer translational loop**: computes the commanded acceleration from position and velocity tracking errors using an adaptive sliding mode control structure.
- **Thrust–attitude mapping**: converts the commanded acceleration into the desired collective thrust and desired attitude.
- **Inner attitude loop**: performs robust geometric attitude tracking on **SO(3)** using the attitude error, angular-velocity error, and attitude error function.
- **Winch regulation**: regulates the tether length based on the relative distance to reduce slack–taut transients.

### 1.2 Control Error Variables

- **Outer-loop sliding surface**

  $$
  s_p = e_v + \Lambda_p e_p
  $$

- **Inner-loop attitude errors**
```math
e_R = \frac{1}{2}\left(R_d^\top R - R^\top R_d\right)^\vee,
\qquad
e_\Omega = \Omega - R^\top R_d \Omega_d
```

where $e_p$ and $e_v$ are the position and velocity errors, and $e_R$ and $e_\Omega$ are the geometric attitude and angular-velocity errors on $SO(3)$.

### 1.3 Practical Add-ons

- Adaptive switching gain for bounded tether-induced disturbances and model mismatch.
- Boundary-layer saturation for chattering reduction.
- Adaptive throttle bias compensation for slow-varying loading changes during tether deployment/retrieval.
- Distance-based winch regulation to mitigate abrupt slack–taut transients.
---

## 2. Model

### 2.1 Drone Model
- UAV: **`if750a`** model

### 2.2 Winch Model
- Custom winch model created using:
  - **cylinder** (drum)
  - **box** (structure / tether-related parts)

### 2.3 Tether Model
- Multi-link tether model with **100 cylinder links**
- Links are connected via **ball joints** to emulate flexible cable dynamics

### 2.4 UGV Model
- UGV: **`r1_rover`** model


## 3. How to Build

### 3.1 Install / Setup PX4

1) Download PX4-Autopilot
```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
```

2) Build PX4 SITL(Gazebo)
```bash
make px4_sitl gazebo
```

### 3.2 Build ROS workspace

Download **offb_example** under `catkin_ws/src`
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3.3 Build/Register Winch Plugin
1) Build the plugin
- Put the plugin package under `catkin_ws/src`, then build the workspace:
```bash
cd ~/catkin_ws/src
# (place winch_control_plugin here)
cd ..
catkin_make
```
- Verify that the shared library is generated:
```bash
ls -lh ~/catkin_ws/devel/lib/libwinch_control_plugin.so
```
2) Export plugin path(example)
```bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:<your_plugin_path>
```

### 3.4 Apply Plugin to **if750a.sdf**
1) Edit **if750a.sdf**
2) Add the winch plugin block(**libwinch_control_plugin.so**)
3) Confirm the winch joint/link attachment works in Gazebo

## How to Run

### 4.1 Launch QGroundControl(QGC)
Open QGC first

### 4.2 Run PX4 + Gazebo
```bash
cd PX4-Autopilot
roslaunch px4 add.launch
```
### 4.3 Run Offboard Controller
```bash
roslaunch offb_example offb_example.launch
```

### 4.4 Apply Force
```bash
python3 apply_force_2.py -**magnitude**-**duration**
```

### 4.5 Tune Parameters(GUI)
Parameters can be tuned via GUI based on the cfg file(dynamic reconfigure)
