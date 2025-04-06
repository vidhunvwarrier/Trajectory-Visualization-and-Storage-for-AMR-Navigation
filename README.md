# Trajectory Visualization and Storage for AMR Navigation

This ROS 2 workspace contains two packages:

1. **custom_service** – Defines a custom service for requesting trajectory saves.
2. **trajectory_analyser** – Contains nodes for saving and visualizing robot trajectories.

---

## Package Structure

### trajectory_analyser
```
trajectory_analyser/
├── src/
│   ├── trajectory_saver.cpp
│   └── trajectory_viewer_node.cpp
├── CMakeLists.txt
└── package.xml
```

### custom_service
```
custom_service/
├── srv/
│   └── TrajectorySaver.srv
├── CMakeLists.txt
└── package.xml
```

---

## Custom Service
Defines a custom service to receive a filename and duration for saving the trajectory.

**Service Definition (`TrajectorySaver.srv`)**
```srv
string filename
float64 duration
---
bool success
```
- `filename`: Name of the YAML file to save the trajectory.
- `duration`: Time (in seconds) from the current moment to save data going backward.

The service is named `TrajectorySaver` and resides in the `custom_service` package.

---

## 🗏 Trajectory Analyser
This package includes two nodes:

### Trajectory Saver Node
**Purpose**: Subscribes to `/odom`, records pose data, and saves it to a YAML file upon service request.

#### How to Start the Node
```bash
ros2 run trajectory_analyser trajectory_saver
```

#### Service Request Example
```bash
ros2 service call /trajectory/saver custom_service/srv/TrajectorySaver "{filename: 'run1.yaml', duration: 10.0}"
```
- Saves the robot's trajectory from the **last 10 seconds**.
- Stores it in `~/logged_path/run1.yaml`.

#### Saved File Format (YAML)
```yaml
path:
  - 1.2, 2.3
  - 1.4, 2.6
```
Each data point represents an `(x, y)` coordinate from the trajectory.

---

###  Trajectory Viewer Node
**Purpose**: Reads a saved YAML trajectory file and publishes the path as a `MarkerArray` for visualization in RViz.

#### Visualizing the Path
```bash
ros2 run trajectory_analyser trajectory_viewer_node --ros-args -p filename:=run1.yaml
```

Optional topic override:
```bash
--ros-args -p marker_topic:=/custom_marker_topic
```

#### RViz Instructions
1. Launch RViz:
   ```bash
   rviz2
   ```
2. Set **Fixed Frame** to `odom`
3. Add a **MarkerArray** display
4. Set the topic to `/marker/data` or your custom topic

---

## Pseudocode

### Trajectory Saver Node
#### Startup
- Subscribe to `/odom`
- Create service `/trajectory/saver`

#### On Receiving Odometry
- Extract `(x, y)` from message
- Get current time
- Store as `PoseData` in `pos_xy_`

#### On Service Call (filename, duration)
- If `filename` is empty or "none", use default `"path.yaml"`
- Get current time
- For each `PoseData`:
  - If `(now - time) <= duration`, add `(x, y)` to `selected_xy`
- Call `file_saver(filename, selected_xy)`
- Set `response.success = true`

#### File Saver Function
- Set folder to `~/logged_path/`
- If it doesn't exist, create it
- Use YAML emitter to write:
```yaml
path:
  - "x1,y1"
  - "x2,y2"
```
- Save the file to `~/logged_path/<filename>`

---

### Trajectory Viewer Node
#### Startup
- Declare and retrieve parameters:
  - `filename`: name of the YAML file
  - `marker_topic`: topic to publish marker data
- If `filename` is empty, return error
- Create publisher for `MarkerArray`
- Load YAML file and parse all positions
- Start timer to call `publish_data()` every second

#### `read_file()`
- Build full path: `~/logged_path/<filename>`
- If file does not exist, return
- Load YAML
- For each item in `path`:
  - Parse string to extract `x`, `y`
  - Create `geometry_msgs::msg::Point(x, y, 0)`
  - Append to `points_`

#### `publish_data()`
- Create a `LINE_STRIP` Marker with:
  - `frame_id = "odom"`
  - Red color `(1, 0, 0, 1)`
  - `scale.x = 0.05`
- Assign all points to the marker
- Add marker to `MarkerArray`
- Publish to `marker_topic`

---

## Build Instructions
From your ROS 2 workspace root:
```bash
colcon build --packages-select custom_service trajectory_analyser
source install/setup.bash
```

---

## Video Demo
[▶ Watch the demo](screen_rec.mp4)
