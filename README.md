# copter

Autonomous waypoint-following flight controller for the [Clover](https://github.com/CopterExpress/clover) drone platform. The drone reads a list of waypoints from a file, navigates to each one in sequence, avoids obstacles detected by the front camera, and drops cargo at designated points using an electromagnet.

## How it works

- **`flight.py`** — main flight loop: arms the drone, follows waypoints from `points.txt`, calls the obstacle-avoidance and navigation helpers, and controls the electromagnet via GPIO
- **`planner.py`** — navigation math (bearing calculation) and waypoint file reader
- **`video_detect.py`** — OpenCV obstacle detection using the front camera; identifies colored obstacles and estimates distance from bounding-box width

## Stack

- Python 3 on Raspberry Pi
- [ROS Noetic/Melodic](https://www.ros.org/) — rospy, mavros, std_srvs
- [Clover SDK](https://github.com/CopterExpress/clover) — `navigate`, `get_telemetry`, `set_position`
- OpenCV, NumPy, RPi.GPIO

## Waypoints format

Edit `points.txt` to define the flight plan. Each line is one waypoint:

```
x y z drop_cargo
```

- `x`, `y`, `z` — target position in the `aruco_map` frame (metres)
- `drop_cargo` — `1` to land briefly and release cargo at this point, `0` to pass through

Example:
```
1.5 2.0 1.5 0
4.0 1.5 1.0 1
```

Update `end_num` in `flight.py` to match the number of waypoints.

## Setup

Install ROS and the Clover SDK according to [the official guide](https://clover.coex.tech/en/), then:

```bash
pip install -r requirements.txt
```

## Running

```bash
python flight.py
```

The drone takes off immediately on launch. Make sure the ArUco map is visible and the drone is on a flat surface before starting.
