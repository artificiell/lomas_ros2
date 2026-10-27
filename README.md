# lomas_ros2

ROS 2 bridge for the LOMAS project.

This workspace lets ROS 2 control a CNC machine and read out the devices around
it. The CNC machine is operated through ROS 2 commands and reports its status
back as ROS 2 messages. Alongside it, a set of devices (environmental sensors,
cameras and a smart plug for the grow light) interface their readings and
controls through ROS 2 as well. On top of this, the workspace provides nodes for
scripted demo motion, automated image-collection scans and sensor data logging.

![Figure 1: LOMAS testbed architecture](lomas_images/lomas_testbed_setup.png)

*Figure 1. The LOMAS testbed is distributed over several Raspberry Pis. Sensor
nodes (top) publish readings to the data-collection Pi (`rp50`), which also
drives image scans of the main controller (`rp0`). `rp0` in turn operates the
machine, the grow-light plug and the top camera.*

## Architecture overview

The system is a set of small ROS 2 nodes spread over five Raspberry Pis (see
Figure 1). Each node owns one piece of hardware or one task, and the nodes
communicate only through ROS 2 topics and services over the local network. No
node needs to know which Pi another node runs on, so a part can be moved to a
different Pi without changing the code.

The architecture has three layers:

1. **Sensing** (`lomas_sensors`, on `rpz1`, `rpz2` and `rpz4`). Each Pi Zero 2W
   reads one sensor, either a soil-moisture sensor through an MCP3002 ADC or
   the BME680 air sensor, and publishes its values as `std_msgs/Float32`
   topics under `/lomas/soil/…` and `/lomas/air/…`.
2. **Control** (`lomas_bridge`, on `rp0`). `rp0` is the only host that talks
   to the hardware that acts on the testbed:
   - The `machine` node is the sole owner of the GRBL serial connection. It
     accepts `MachineCommand` messages, tracks the real motion state of the
     machine and publishes it as `MachineStatus`. The `store` node keeps the
     last known position across restarts.
   - The `light` node switches the grow light through the Tapo smart plug.
   - The `demo` node moves the machine along a preset path or follows commands
     from a web control.
   - The top camera, started by `rp0`, offers an image service.
3. **Data collection** (`lomas_bridge` and `lomas_sensors`, on `rp50`). Both
   collection nodes run on the Pi with the SSD:
   - The image scan publishes waypoints to the machine on `rp0`, waits for the
     machine's status to confirm arrival, requests an image from the top
     camera and stores it locally.
   - The sensor logger subscribes to all sensor topics and writes hourly
     averages to CSV files.

`lomas_interfaces` defines the shared messages (`MachineCommand` and
`MachineStatus`), and `lomas_bringup` contains the launch files that start
each part with the right topic names. The side camera (`camera_side.launch.py`)
is available in the workspace but is not part of Figure 1.

## Packages

| Package | Build type | Purpose |
|---|---|---|
| [`lomas_interfaces`](./lomas_interfaces) | `ament_cmake` | Custom messages: `MachineCommand`, `MachineStatus` |
| [`lomas_bridge`](./lomas_bridge) | `ament_python` | Machine (GRBL) bridge, position store, light plug, demo motion, image-scan collection, command converter |
| [`lomas_sensors`](./lomas_sensors) | `ament_python` | BME680 air sensor, MCP3002 soil-moisture sensor, hourly sensor data logger |
| [`lomas_bringup`](./lomas_bringup) | `ament_python` | Launch files that start the nodes above with the right topic names |

Also in the repo: [`lomas_test.py`](./lomas_test.py), a standalone G-code sender
for testing the serial connection to the machine without ROS.

## Requirements

**ROS 2 packages**

- `rclpy`, `std_msgs`, `std_srvs`, `rcl_interfaces`
- `cv_bridge` (and OpenCV) for `lomas_bridge/collection`
- [`camera_ros`](https://github.com/christianrauch/camera_ros) for the side camera
- [`rpi_cams_ros2`](https://github.com/artificiell/rpi_cams_ros2) for the top camera (provides `rpi_gs_cam`, whose `camera.launch.py` is included by `camera_top.launch.py`, and the `cam_interfaces/srv/Image` image service used by `lomas_bridge/collection`). Clone it into the same workspace, see [Installation](#installation).

**Python packages**

- `pyserial` (machine bridge)
- `tapo` (light plug)
- `gpiozero` (MCP3002 soil sensor)
- `adafruit-circuit-python-bme680` and `board` (BME680 air sensor)

**Hardware**

- A GRBL-based machine on a serial port (default `/dev/ttyACM0`, 115200 baud)
- TP-Link Tapo P100 smart plug (light)
- Adafruit BME680 on I²C
- Soil-moisture sensor(s) via an MCP3002 ADC (SPI)
- Raspberry Pi cameras (a global-shutter camera for the top view, a `camera_ros`-compatible camera for the side view)

**ROS 2 distribution:** developed, tested and verified on ROS 2 Jazzy. It should also work with later distributions.

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/artificiell/lomas_ros2.git
git clone https://github.com/artificiell/rpi_cams_ros2.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Testbed setup

The LOMAS testbed is distributed over several Raspberry Pis, all running
Ubuntu Server 24.04 LTS and ROS 2 Jazzy. Each has a unique hostname and one or
more dedicated tasks. The nodes talk to each other over ROS 2 (DDS) on the
local network, so all machines need to be on the same network and use the same
`ROS_DOMAIN_ID`.

| Host | Hardware | Role | Runs |
|---|---|---|---|
| `rp0` | Raspberry Pi 4 | Main LOMAS controller: machine, light, demo motion and top camera | `ros2 launch lomas_bringup machine_with_demo.launch.py` |
| `rp50` | Raspberry Pi 5, 500 GB SSD | All data collection (images and sensor logs) | `ros2 run lomas_bridge collection`<br>`ros2 run lomas_sensors collection` |
| `rpz1` | Raspberry Pi Zero 2W | Left soil-moisture sensor | `ros2 launch lomas_bringup soil_sensor_left.launch.py` |
| `rpz2` | Raspberry Pi Zero 2W | Right soil-moisture sensor | `ros2 launch lomas_bringup soil_sensor_right.launch.py` |
| `rpz4` | Raspberry Pi Zero 2W | Air sensor (temperature, humidity, pressure) | `ros2 launch lomas_bringup air_sensor.launch.py` |

The data flow between the hosts is shown in Figure 1 at the top of this README.

## Quick start

The commands below run the individual parts on one machine. On the real
testbed, each Raspberry Pi runs only its own part, as listed above.

Machine, position store and light plug only:

```bash
ros2 launch lomas_bringup machine_minimal.launch.py
```

Same, plus the demo node and the top camera:

```bash
ros2 launch lomas_bringup machine_with_demo.launch.py
```

Sensors and cameras:

```bash
ros2 launch lomas_bringup air_sensor.launch.py
ros2 launch lomas_bringup soil_sensor_left.launch.py
ros2 launch lomas_bringup soil_sensor_right.launch.py
ros2 launch lomas_bringup camera_side.launch.py
ros2 launch lomas_bringup camera_top.launch.py
```

Data collection (run after the machine and top camera are up):

```bash
ros2 run lomas_bridge collection --ros-args -p service_name:=/lomas/top/camera/image   # image scan over the machine workspace
ros2 run lomas_sensors collection    # hourly sensor averages to CSV
```

> **Credentials:** the Tapo plug username, password and IP address are
> parameters of the `light` node. Pass them at launch time or from a
> parameters file kept out of version control; don't commit real credentials.

## Launch files (`lomas_bringup`)

| Launch file | Starts | Notes |
|---|---|---|
| `machine_minimal.launch.py` | `machine`, `store`, `light` | Arg: `motor_speed` (default `100`). `machine` is respawned after 3 s if it dies. |
| `machine_with_demo.launch.py` | `machine`, `store`, `light`, `demo`, top camera | Args: `motor_speed`, `service_name` |
| `camera_top.launch.py` | Includes `rpi_gs_cam/camera.launch.py` | Args: `sharpness`, `denoise`, `shutter`, `gain`, `ev`, `awb`, `brightness`, `contrast`, `service_name` (default `/lomas/top/camera/image`) |
| `camera_side.launch.py` | `camera_ros/camera_node` | Remapped to `/lomas/side/camera/{info,image,image/compressed}` |
| `air_sensor.launch.py` | `bme680` | Remapped to `/lomas/air/*` |
| `soil_sensor_left.launch.py` | `mcp3002` | Args: `auto_calibration`, `sensor_channel`, `time_period`. Publishes `/lomas/soil/moisture/left` |
| `soil_sensor_right.launch.py` | `mcp3002` | Same args. Publishes `/lomas/soil/moisture/right` |

Each soil sensor runs on its own Raspberry Pi (see [Testbed setup](#testbed-setup)),
so both launch files can use the default `sensor_channel` of `0`.

## Interfaces (`lomas_interfaces`)

**`MachineCommand`**

| Field | Type | Description |
|---|---|---|
| `x`, `y`, `z` | `float32` | Target position in machine units |
| `stop` | `bool` | If true, the machine is sent home (`G28`) instead of to `x/y/z` |

**`MachineStatus`**

| Field | Type | Description |
|---|---|---|
| `is_synced` | `bool` | False if the last exchange with the machine couldn't be confirmed (status may be stale) |
| `machine_moving` | `bool` | True from move acceptance until the machine reports `Idle` |
| `error_nr` | `uint8` | `0` ok, `1` serial error/timeout (or stalled move), `2` out-of-range command rejected, `98` connection/reinit failure |
| `interval` | `uint16` | Cultivation interval |
| `x`, `y`, `z` | `float32` | Current position (with offset applied) |

## Nodes (`lomas_bridge`)

Topic names below are the nodes' defaults; the launch files remap the machine
topics to `/lomas/machine/cmd` and `/lomas/machine/status`.

### `machine` (`machine_bridge_node`)

Bridge to the GRBL machine over serial. Commands are non-blocking: a move is
accepted into GRBL's buffer, and a polling timer tracks actual completion
(`Idle`), stalls and alarms. On a GRBL reset or alarm it reinitialises and
restores the position offset.

- Subscribes: `/machine/cmd` (`MachineCommand`)
- Publishes: `/machine/status` (`MachineStatus`)
- Accepted ranges: `0 ≤ x ≤ 24`, `0 ≤ y ≤ 24`, `z ≥ 0`; anything else is rejected with `error_nr = 2`
- Position is persisted through the `store` node so it survives restarts

| Parameter | Default | Description |
|---|---|---|
| `sim_mode` | `false` | Simulate the serial port |
| `port` | `/dev/ttyACM0` | Serial device |
| `baud` | `115200` | Baud rate |
| `interval` | `120` | Cultivation interval (published in status) |
| `speed` | `100` | Feed rate |
| `homing_timeout` | `30.0` | Seconds before a homing move is flagged as stalled |
| `move_timeout` | `5.0` | Seconds before a normal move is flagged as stalled |
| `status_poll_period` | `0.25` | Seconds between position polls |
| `position_store_node` | `machine_position_store_node` | Node holding x/y/z across restarts |
| `position_save_period` | `1.0` | Minimum seconds between position-store writes |

### `store` (`machine_position_store_node`)

Holds the last known machine position as ROS parameters `x`, `y`, `z`. Run it
alongside `machine`.

### `light` (`light_plug_node`)

Controls a Tapo P100 smart plug (the grow light) and reconnects if the plug
stops responding.

- Subscribes: `/lomas/light` (`std_msgs/Bool`)
- Service: `/lomas/light` (`std_srvs/SetBool`)
- Parameters: `username`, `password`, `ip_address`

### `demo` (`lomas_demo_node`)

Drives the machine through a preset path and accepts normalised commands from a
web control. Each step is confirmed against `MachineStatus` before the next is
sent.

- Subscribes: `machine/start`, `machine/stop` (`Bool`); `lomas/control/cmd` (`String`, JSON `{"x": 0..1, "y": 0..1}`); `lomas/machine/status`
- Publishes: `lomas/machine/cmd`; `lomas/control/state` (`String`, normalised position as JSON)
- Parameters: `machine_max_x` / `machine_max_y` (`24.0`), `position_tolerance` (`0.1`)

### `collection` (`collection_scan_node`)

Runs a column-major grid scan of the workspace: moves to each waypoint,
waits for the machine to confirm arrival, lets it settle, then requests an
image from the camera service and saves it as a PNG. Failed grabs are retried,
and a stop message aborts the scan and sends the machine home.

- Subscribes: `lomas/machine/stop` (`Bool`), `lomas/machine/status`
- Publishes: `lomas/machine/cmd`, `lomas/control/state`
- Calls: camera service `service_name` (`cam_interfaces/srv/Image`)
- Output: `~/data/images/<YYYY-MM-DD>/<service>_x###_y###_<timestamp>.png`

| Parameter | Default | Description |
|---|---|---|
| `machine_max_x`, `machine_max_y` | `24.0` | Workspace limits |
| `service_name` | `camera/image` | Camera service to call (use `/lomas/top/camera/image` with the bringup launch files) |
| `x_start`, `y_start` | `2.0` | First waypoint |
| `x_step`, `y_step` | `4.0`, `5.0` | Grid spacing |
| `position_tolerance` | `0.1` | Distance counted as "reached" |
| `init_delay_sec` | `5.0` | Wait after the initial home before scanning |
| `settle_delay_sec` | `2.0` | Wait after reaching a normal waypoint |
| `column_delay_sec` | `10.0` | Wait after reaching the first waypoint of a new column |
| `end_delay_sec` | `5.0` | Wait after the last waypoint |
| `quality` | `80` | JPEG quality (1–99) to request compression; `0` or ≥100 for raw |
| `max_grab_retries` | `1` | Retries for a failed image grab |
| `grab_retry_delay_sec` | `2.0` | Delay before retrying |
| `grab_timeout_sec` | `10.0` | Treat a grab as failed after this long |

### `converter` (`command_converter_node`)

Converts JSON strings into `MachineCommand` messages.

- Subscribes: `/converter/cmd` (`String`, e.g. `{"x": 5, "y": 5, "z": 0, "stop": false}`)
- Publishes: `/machine/cmd`

## Nodes (`lomas_sensors`)

### `bme680` (`adafruit_circuit_node`)

Reads temperature, humidity and pressure from a BME680 over I²C at 1 Hz.

- Publishes (`std_msgs/Float32`): `/air/temperature`, `/air/humidity`, `/air/pressure`

### `mcp3002` (`soil_moisture_node`)

Reads a soil-moisture sensor through an MCP3002 ADC and publishes a value
where higher means wetter. With calibration enabled, the dry baseline is
estimated from the lowest readings and the signal is min-max normalised.

- Publishes: `/soil/moisture` (`std_msgs/Float32`)

| Parameter | Default | Description |
|---|---|---|
| `calibration` | `false` | Auto-calibrate (the sensor must see its driest and wettest conditions) |
| `channel` | `0` | MCP3002 channel |
| `period` | `1.0` | Seconds between readings |

### `collection` (`data_collection_node`)

Subscribes to the sensor topics (`/lomas/air/{humidity,pressure,temperature}`
and `/lomas/soil/moisture/{left,right}`), averages them per hour
(Europe/Stockholm time) and appends the results to
`~/data/sensors/<YYYY-MM-DD>.csv` with columns
`date, hour, topic, average, samples`.

## Topic overview (with the bringup launch files)

| Topic | Type | Source |
|---|---|---|
| `/lomas/machine/cmd` | `lomas_interfaces/MachineCommand` | `demo`, `collection` → `machine` |
| `/lomas/machine/status` | `lomas_interfaces/MachineStatus` | `machine` |
| `/lomas/machine/start`, `/lomas/machine/stop` | `std_msgs/Bool` | UI / external → `demo`, `collection` |
| `/lomas/control/cmd`, `/lomas/control/state` | `std_msgs/String` (JSON) | web control ↔ `demo` |
| `/lomas/light` | `std_msgs/Bool` + `SetBool` service | → `light` |
| `/lomas/air/{temperature,humidity,pressure}` | `std_msgs/Float32` | `bme680` |
| `/lomas/soil/moisture/{left,right}` | `std_msgs/Float32` | `mcp3002` |
| `/lomas/side/camera/{info,image,image/compressed}` | camera topics | `camera_ros` |
| `/lomas/top/camera/image` (service) | `cam_interfaces/srv/Image` | `rpi_gs_cam` → called by `collection` |

## Testing the machine without ROS

`lomas_test.py` opens the serial port, sets a work-zero, moves to a test
position and homes:

```bash
python3 lomas_test.py --port /dev/ttyACM0 --baud 115200 --delay 1
```

## License

MIT, see [LICENSE](./LICENSE).

Maintainer: Andreas Persson (<andreas.persson@oru.se>)
