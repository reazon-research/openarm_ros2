# ROS2 packages for OpenArm robots

- openarm_bimanual_description: urdf with pedestal torso and arm on each side
- openarm_description: urdf with gripper actuator
- openarm_moveit_config: motion planning with [moveit2](https://github.com/moveit/moveit2)

## Description Packages

Each link has a visual mesh and a collision mesh, as shown in the figures below:
  
<img width="412" alt="visual meshes of openarm_bimanual_description urdf in rviz2" src="https://github.com/user-attachments/assets/9020efc3-69bc-420d-93a1-305885925638" />
<img width="383" alt="collision meshes of openarm_bimanual_description urdf in rviz2" src="https://github.com/user-attachments/assets/6f62184e-ccea-4859-9364-7c7d1b8def86" />

### TODO: 
- [ ] Add results from true inertia tests to URDF

## MoveIt2 Support

https://github.com/user-attachments/assets/a0f962e5-6150-49ce-b18e-9914bcb322ef

### TODO:
- [ ] ROS 2 control packages (separate branch)

Tested with:
- [x] Rolling
- [x] Jazzy
- [x] Humble


## License

All packages of `openarm_ros2` are licensed under the [BSD-3-Clause](https://opensource.org/license/bsd-3-clause).
