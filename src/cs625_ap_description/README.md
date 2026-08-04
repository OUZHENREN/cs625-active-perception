# cs625_ap_description

This package is an application-layer extension of the CS625 description. It
does not copy or modify the vendor robot model.

`urdf/cs625_active_perception.urdf.xacro` includes the reusable
`cs_robot` macro from `eli_cs_robot_description`, preserves the senior
MoveIt-compatible `my_end_effector_link`, adds a conventional `tool0` alias,
and appends the configurable Eye-in-Hand chain:

```text
flange -> my_end_effector_link -> tool0
       -> camera_mount_link -> camera_link
       -> camera_depth_optical_frame
```

The vendor description package must be available in the underlay. Camera
mount geometry and the optical-frame rotation are xacro parameters; camera
intrinsics and driver topics belong to the sim/real profile, not this package.
