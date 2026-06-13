# Inspire Hand MediaPipe Teleoperation

<video src="media/inspire_hand_mediapipe_teleoperation.mp4" width="720" controls muted></video>

This package provides a ROS 2 teleoperation layer for Unitree G1 Inspire RH56DFX hands.
Simulation uses RViz plus `ros2_control` mock hardware with the same controller action names as the real G1 hand controllers.

## Interfaces

Actions:

- `/right_hand_controller/follow_joint_trajectory`
- `/left_hand_controller/follow_joint_trajectory`

Topics:

- `/image_raw`: camera input
- `/mediapipe/hand/landmarks`: MediaPipe hand landmarks
- `/mediapipe/debug_image`: MediaPipe debug overlay image
- `/mediapipe/markers`: MediaPipe RViz markers
- `/inspire_hand/target_joint_states`: target Inspire hand joint commands
- `/joint_states`: simulated or real controller joint states

Each hand has six commanded joints

```text
thumb_yaw thumb_pitch index middle ring pinky
```

The command convention is `0 = open` and `100 = closed`.
The tests include synthetic open-palm and closed-fist landmark fixtures to check that this direction is not inverted.

## Setup

Install ROS 2 Jazzy and workspace tools first.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-vcstool python3-pip

mkdir -p ~/g1_ws/src
cd ~/g1_ws/src
git clone https://github.com/Michi-Tsubaki/inspire_tutorials.git inspire_tutorials
vcs import < inspire_tutorials/inspire_tutorials.repos
rosdep update
rosdep install -iqry --from-paths . --ignore-src

# MediaPipe itself is not installed by rosdep or colcon.
# Keep ROS' apt-provided numpy/OpenCV in use; do not let pip replace them.
python3 -m pip install --user --break-system-packages --no-deps mediapipe absl-py flatbuffers sounddevice cffi pycparser

# Required by real.launch.py and sim.launch.py for hand landmarks.
mkdir -p ~/g1_ws/src/PME26Elvis/mediapipe_ros2_suite/src/mediapipe_ros2_node/models
curl -L \
  -o ~/g1_ws/src/PME26Elvis/mediapipe_ros2_suite/src/mediapipe_ros2_node/models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task

cd ~/g1_ws
colcon build --symlink-install --packages-up-to inspire_tutorials
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

On ROS 2 Jazzy / Ubuntu 24.04, Python is marked as externally managed.
The `--break-system-packages` flag is required for a user-site pip install.
The `--no-deps` flag is intentional: `mediapipe` declares PyPI `numpy` and `opencv-contrib-python` dependencies, but this workspace should use the ROS/apt packages installed by `rosdep` (`python3-numpy`, `python3-opencv`, and `cv_bridge`).
Installing the full PyPI dependency set can shadow those packages and break ROS image conversion.


`inspire_tutorials.repos` imports

- `g1_ros` and its Unitree dependencies, for the real G1 controllers.
- `mediapipe_ros2_suite`, for MediaPipe ROS 2 hand landmarks and debug images.
The vendored assets keep the upstream license file.

## MediaPipe Models

`mediapipe_ros2_suite` expects MediaPipe task files. Place these files in either `mediapipe_ros2_node/models` before building, or put them in any directory and set `MP_MODELS_DIR` before launching.

Required for this package:
```text
hand_landmarker.task
```

Install the required hand model before building:
```bash
mkdir -p ~/g1_ws/src/PME26Elvis/mediapipe_ros2_suite/src/mediapipe_ros2_node/models
curl -L \
  -o ~/g1_ws/src/PME26Elvis/mediapipe_ros2_suite/src/mediapipe_ros2_node/models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
```

If the workspace was already built before adding the model, rebuild the asset package so the default lookup path under `install/` contains the file:
```bash
cd ~/g1_ws
colcon build --symlink-install --packages-select mediapipe_ros2_node
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Optional:
```text
gesture_recognizer.task
pose_landmarker.task
face_landmarker.task
```

Download the model files from the official MediaPipe model pages and keep their upstream license terms. If you use `MP_MODELS_DIR`:

```bash
export MP_MODELS_DIR=$HOME/mediapipe_models
```

## Real Hardware

This workflow is ROS 2 only. Keep each command group below running in its own terminal.

Replace `<network interface>` with the NIC connected to the G1, for example `enp0s31f6`.

- Terminal 1: start the Inspire hand service on the robot

```bash
ssh unitree@192.168.123.164  # default password: 123
cd ~/dfx_inspire_service/build
sudo ./inspire_g1 -k -u
```

- Terminal 2: bring up the G1 as a ROS 2 robot

```bash
source /opt/ros/jazzy/setup.bash
source ~/g1_ws/install/setup.bash
ros2 launch g1_bringup g1_bringup.launch.py \
  network_interface:=<network interface> \
  hand_type:=inspire_dfq \
  use_rviz:=false
```

- Terminal 3: activate the two hand controllers

```bash
source /opt/ros/jazzy/setup.bash
source ~/g1_ws/install/setup.bash
ros2 control set_controller_state right_hand_controller active
ros2 control set_controller_state left_hand_controller active
```

- Terminal 4: start MediaPipe hand teleoperation

```bash
source /opt/ros/jazzy/setup.bash
source ~/g1_ws/install/setup.bash
ros2 launch inspire_tutorials real.launch.py hand:=both
```

You can command only one side from Terminal 4

```bash
ros2 launch inspire_tutorials real.launch.py hand:=right
ros2 launch inspire_tutorials real.launch.py hand:=left
```

For real hardware, `allow_missing_controllers:=true` by default. If `hand:=both` is requested but only one controller action exists, the teleoperation node warns and uses the available side.
Set `allow_missing_controllers:=false` when you want launch to fail unless every selected controller is available.

If MediaPipe handedness appears swapped because of camera mirroring, use
```bash
ros2 launch inspire_tutorials real.launch.py hand:=both mirror_handedness:=true
```


## Simulation

Simulation starts `usb_cam`, MediaPipe, RViz, `robot_state_publisher`, `ros2_control_node`, `joint_state_broadcaster`, and the selected hand trajectory controller(s).

```bash
source /opt/ros/jazzy/setup.bash
source ~/g1_ws/install/setup.bash
ros2 launch inspire_tutorials sim.launch.py hand:=both
```

Side selection is the same as real hardware

```bash
ros2 launch inspire_tutorials sim.launch.py hand:=right
ros2 launch inspire_tutorials sim.launch.py hand:=left
```

### Simulation with the sample MP4

The sample video is installed from `test_data/test.mp4`.
To publish it on `/image_raw`, disable `usb_cam` and enable the video-file publisher

```bash
source /opt/ros/jazzy/setup.bash
source ~/g1_ws/install/setup.bash
ros2 launch inspire_tutorials sim.launch.py \
  hand:=both \
  use_camera:=false \
  use_video_file:=true \
  video_playback_speed:=0.5 \
  video_loop:=true
```

`test_data/test.mp4` is a 30 fps video. With `video_playback_speed:=0.5`, the publisher sends every frame at 15 Hz and restarts from the first frame at EOF.
Use `video_file:=/path/to/other.mp4` to replay a different local video.

RViz shows

- The Inspire hand robot model.
- `/mediapipe/debug_image`.
- `/mediapipe/markers`.

You can also inspect the generated target commands

```bash
ros2 topic echo /inspire_hand/target_joint_states
```

## Manual 0..100 Command Bridge

```bash
ros2 run inspire_tutorials inspire_hand_command --hand right --preset open
ros2 run inspire_tutorials inspire_hand_command --hand right --preset close
ros2 run inspire_tutorials inspire_hand_command --hand both --values 10 30 100 100 100 100
```

Presets are compatible with the JSK G1 hand examples

- `open`
- `close`
- `stop_grasp`
- `prepare_grasp`
- `start_grasp`
- `default_grasp`

The raw value order is

```text
thumb_yaw thumb_pitch index middle ring pinky
```

## Tests

Run package tests

```bash
source /opt/ros/jazzy/setup.bash
source ~/g1_ws/install/setup.bash
colcon test --packages-select inspire_tutorials --event-handlers console_direct+
colcon test-result --verbose
```

The tests do not require a camera, a robot, RViz, or MediaPipe model files. They cover

- Open-palm vs closed-fist synthetic landmark conversion.
- 0..100 command conversion.
- Left/right/both hand selection.
- Simulator URDF generation from vendored URDF and mesh assets.
- Launch/package interface contracts.
