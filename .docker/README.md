# Docker GUI Forwarding

On Linux:
```sh
host +local:root
```

```sh
docker run --env DISPLAY=$DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--network=host \
-it thchzh/ros2:openarm-humble \
/bin/bash
```

Open the MuJoCo sim at
[https://thomasonzhou.github.io/mujoco_anywhere/](https://thomasonzhou.github.io/mujoco_anywhere/)

```sh
. ~/ros2_ws/install/setup.bash && \
ros2 launch openarm_bimanual_moveit_config demo.launch.py hardware_type:=mujoco
```

# To build the latest image (v0.3)
```sh
docker build --no-cache -t thchzh/ros2:openarm-humble .
```
