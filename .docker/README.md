# Docker GUI Forwarding

Ensure docker is installed before continuing: [https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/)

## On Linux
```sh
host +local:root
```

```sh
docker run --env DISPLAY=$DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--network=host \
-it ghcr.io/reazon-research/openarm:v0.3 \
/bin/bash
```

Open the MuJoCo sim at
[https://thomasonzhou.github.io/mujoco_anywhere/](https://thomasonzhou.github.io/mujoco_anywhere/)

Launch an example with MoveIt2:
```sh
. ~/ros2_ws/install/setup.bash && \
ros2 launch -d openarm_bimanual_moveit_config demo.launch.py hardware_type:=sim
```

### To build the latest image (v0.3)
```sh
docker build --no-cache -t ghcr.io/reazon-research/openarm:v0.3 .
```

## On MacOS and Windows

Please note that this method runs at a much lower frequency than ideal conditions, meaning the commands may not be fast enough for stable operation. 

However, it is a good way to visualize the necessity of high frequency communication when using torque control.

### MacOS Notes

It may be necessary to enable Rosetta.

### Windows Notes

Use Git Bash to run the shell script.

In the current directory, run:
```sh
./run-novnc.sh
```

Visit the localhost URL to open a GUI into Ubuntu.

To stop the container, run 
```sh
docker compose down
```


Open the MuJoCo sim (inside or outside the GUI)
[https://thomasonzhou.github.io/mujoco_anywhere/](https://thomasonzhou.github.io/mujoco_anywhere/)


Double-click on Terminator and launch an example with MoveIt2:
```sh
. ~/ros2_ws/install/setup.bash && \
ros2 launch -d openarm_bimanual_moveit_config demo.launch.py hardware_type:=sim
```

## Debugging

Open Developer Tools in the browser. When the MuJoCo OpenArm is connected, state and command data should be visible in the Console.

If a connection is not established, try restarting either the browser or relaunch the ROS2 process.