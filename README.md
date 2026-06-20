# pearch_mission

ROS Noetic package for UAV branch perching. Receives pixel error and ToF distance from the Raspberry Pi over UDP and drives the UAV through three stages: alignment, forward approach, and perch engagement.

Runs inside a Docker container (`uav_ros_stack_binary`) on a ground station connected to the same LAN as the Pi.

---

## What It Does

`udp_receiver_node.py` listens for JSON packets on UDP port 5005 and republishes the fields as ROS topics:

| ROS Topic | Type | Description |
|-----------|------|-------------|
| `/perch/error_x` | `Float32` | Horizontal pixel error (branch vs frame center) |
| `/perch/error_y` | `Float32` | Vertical pixel error |
| `/perch/tof` | `Float32` | ToF distance in mm (`-1.0` if unavailable) |

`perch_controller_node.py` subscribes to those topics plus odometry and drives the UAV through a state machine:

```
ALIGNING → APPROACHING → PERCHING → DONE
```

---

## State Machine

### ALIGNING
Corrects lateral (y) and vertical (z) position using pixel error until `combined_error < err_thresh` (default 10 px).

- Camera `error_x` → UAV `y` correction
- Camera `error_y` → UAV `z` correction
- Forward (`x`) is held constant

### APPROACHING
Drives forward in `x` by a fixed step each second until ToF `< tof_thresh` (default 200 mm).

- Drifts back to ALIGNING if error exceeds `2 × err_thresh`

### PERCHING
Publishes a two-leg trajectory:
1. Move forward to perch point (`x + tof_m`), drop `perch_drop` m below it
2. Climb `perch_climb` m above the perch point to engage the gripper hook

Then transitions to DONE.

---

## Stage Enable Flags

At the top of `perch_controller_node.py`:

```python
ENABLED_STAGES = {
    ALIGNING:    True,
    APPROACHING: True,
    PERCHING:    True,
}
```

Set any stage to `False` to skip it. The state machine jumps to the next enabled stage automatically.

---

## UDP Packet Format

JSON received from `UDP_client/client/udp_client.py` on the Pi:

```json
{
  "error_x":   -12.4,
  "error_y":    3.1,
  "tof":        480.0,
  "timestamp":  1748000000.0
}
```

Parsed by `udp_packet.py → PerchPacket.from_json()`.

---

## Directory Structure

```
pearch_mission/
├── CMakeLists.txt
├── package.xml
├── udp_packet.py              # PerchPacket dataclass — shared by receiver and tests
├── launch/
│   └── perch_mission.launch   # Starts udp_receiver + perch_controller
├── scripts/
│   ├── udp_receiver_node.py   # UDP → /perch/* ROS topics
│   ├── perch_controller_node.py  # State machine: ALIGNING → APPROACHING → PERCHING
│   ├── pearch_manual_node.py  # Manual trajectory test node
│   └── plot_perch_bag.py      # Post-run bag analysis and plotting
└── plot/                      # Saved bag plots
```

---

## Running

```bash
# Start the container
docker start uav_ros_stack_binary
docker exec -it uav_ros_stack_binary bash

# Inside container
source /root/uav_ws/devel/setup.bash
roslaunch pearch_mission perch_mission.launch
```

Or start nodes individually:

```bash
rosrun pearch_mission udp_receiver_node.py
rosrun pearch_mission perch_controller_node.py
```

---

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gain` | `0.005` | m per error-unit for alignment corrections |
| `z_step` | `0.005` | Forward step size during approach (clamped to `min_step`) |
| `err_thresh` | `10.0` | Combined pixel error threshold for "aligned" |
| `tof_thresh` | `200.0` | ToF distance (mm) to trigger perch |
| `rate` | `30.0` | Control loop Hz |
| `min_step` | `0.10` | Minimum tracker step size (m) — smaller moves are ignored |
| `max_step` | `0.15` | Maximum step per command |
| `cmd_rate` | `1.0` | Committed setpoints per second |
| `perch_drop` | `0.40` | m below perch point before climbing |
| `perch_climb` | `0.60` | m above perch point to engage gripper |
| `odom_topic` | `/red/vrpn_client/estimated_odometry` | Odometry source (OptiTrack) |

---

## Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `/red/tracker/input_pose` | `PoseStamped` | Single setpoint (ALIGNING / APPROACHING) |
| `/red/tracker/input_trajectory` | `MultiDOFJointTrajectory` | Two-leg perch trajectory |

---

## Platform

- ROS Noetic inside Docker (`uav_ros_stack_binary`)
- Odometry from OptiTrack via VRPN
- Flight controller: Pixhawk + MAVROS
