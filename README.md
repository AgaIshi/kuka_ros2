# kuka_lbr_impedance_ros2

A **self-contained ROS 2 workspace** for **Cartesian impedance control** of the
KUKA LBR (iiwa / med) arms — with runtime stiffness tuning, a GUI teleop, and
live stiffness-ellipsoid visualization. The same torque-based controllers run on
**any LBR variant** (iiwa7/14, med7/14) in **Gazebo simulation** by changing a
single launch argument.

Everything needed is vendored in `src/` — just clone, build, and run.

---

## Repository layout

This repo **is** the colcon workspace (build from the repo root):

```
kuka_lbr_impedance_ros2/
└── src/
    ├── kuka_control/            # launch + config (bring up arm + controller)
    ├── controllers/             # torque controllers (ros2_effort_controller)
    │   ├── cartesian_impedance_controller/   # + our stiffness/GUI/ellipsoid additions
    │   ├── joint_impedance_controller/
    │   ├── gravity_compensation/
    │   ├── effort_controller_base/
    │   └── debug_msg/
    ├── lbr-stack/               # KUKA FRI hardware interface, descriptions, bringup
    ├── controller_evaluation/   # benchmark trajectories + tracking plots
    └── assets/
```

### What this project adds on top of the upstream stacks
- **Runtime stiffness tuning** — single-key `std_msgs/String` commands on
  `…/stiffness_command`; live values on `…/current_stiffness`.
- **GUI teleop** (`stiffness_teleop_gui.py`) — window with per-axis bars, +/-
  buttons, keyboard shortcuts, and a Close button.
- **Stiffness ellipsoids** in RViz — green = translational, blue = rotational,
  at the end-effector, scaled by the stiffness matrix.
- **Rx/Ry/Rz** stiffness commands and a Gazebo **contact world** (box).

---

## Prerequisites

- **Ubuntu 22.04 + ROS 2 Humble**
- **Gazebo (Ignition / `ros_gz`)**
- `sudo apt install ros-humble-rviz2 python3-colcon-common-extensions python3-rosdep`
- Python (evaluation): `pip install numpy matplotlib` (optional scripts also use
  `scipy roboticstoolbox-python pynput h5py`)

---

## Build

```bash
git clone https://github.com/AgaIshi/kuka_lbr_impedance_ros2.git
cd kuka_lbr_impedance_ros2

# system dependencies
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# build  (single-threaded avoids out-of-memory on low-RAM machines)
colcon build --symlink-install --parallel-workers 1

source install/setup.bash
```

> Every new terminal must run:
> `source /opt/ros/humble/setup.bash && source ~/kuka_lbr_impedance_ros2/install/setup.bash`

---

## Run in simulation

**1. Gazebo + RViz (arm, box, ellipsoids):**
```bash
ros2 launch kuka_control gazebo_rviz.launch.py
# choose arm:     model:=med7     (iiwa7 | iiwa14 | med7 | med14, default iiwa7)
# headless:       rviz:=false
# other control:  ctrl:=joint_impedance_controller
```
Wait for `Finished Impedance on_activate`.

**2. Stiffness GUI:**
```bash
ros2 run cartesian_impedance_controller stiffness_teleop_gui.py
```

**3. Move the end-effector (Cartesian target):**
```bash
ros2 topic pub -r 10 /lbr/cartesian_impedance_controller/target_frame geometry_msgs/msg/PoseStamped \
"{header: {frame_id: lbr_link_0}, pose: {position: {x: 0.4, y: 0.0, z: 0.6}, orientation: {w: 1.0}}}"
```

### Contact demo
Push into the box, then change stiffness while in contact:
```bash
ros2 topic pub -r 10 /lbr/cartesian_impedance_controller/target_frame geometry_msgs/msg/PoseStamped \
"{header: {frame_id: lbr_link_0}, pose: {position: {x: 0.385, y: -0.55, z: 0.55}, orientation: {w: 1.0}}}"
```

---

## Controller evaluation (tracking plots)

With the simulation running:
```bash
cd src/controller_evaluation
python3 traj_sin.py     
```
The executed trajectory lags/attenuates the commanded sinusoid — the expected
compliant tracking of an impedance controller.

---

## Credits

- KUKA hardware interface, descriptions, bringup: **lbr-stack**
  (https://github.com/lbr-stack/lbr_fri_ros2_stack).
