# auto exposure controller for ros2
ROS2 node that subscribes to camera meta information and publishes
control messages for exposure time and gain. This package works in
conjunction with the
[flir_spinnaker_ros2](https://github.com/berndpfrommer/flir_spinnaker_ros2)
package and allows to externally control the auto-exposure which is
useful in situations where two cameras have to maintain consistent
exposure settings.


## How to build

Make sure you have your ROS2 environment sourced, for example:
```
source /opt/ros/galactic/setup.bash
```

Create a workspace (``exposure_control_ros2_ws``), clone this repo,
and use ``vcs`` (ubuntu package ``python3-vcstool`` to pull in the
remaining dependencies:

```
mkdir -p ~/exposure_control_ros2_ws/src
cd ~/exposure_control_ros2_ws
git clone https://github.com/berndpfrommer/exposure_control_ros2 src/exposure_control_ros2
cd src
vcs import < exposure_control_ros2/exposure_control_ros2.repos
cd ..
```

Build the package:
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. install/setup.bash
```

You should now be able to launch the example node:
```
ros2 launch exposure_control_ros2 exposure_control.launch.py

```
Also have a look at the ``stereo_synced.launch.py`` file in the ``launch``
directory of the
[flir_spinnaker_ros2](https://github.com/berndpfrommer/flir_spinnaker_ros2)
camera driver.

## Parameters

- ``max_gain``: (default 10) maximum allowed camera gain (in db)
- ``gain_priority``: (default false) if true, first vary gain, then vary exposure time.
- ``brightness_target``: (default 120) average target brightness (0..255)
- ``brightness_tolerance``: (default 5) brightness threshold before sending control command
- ``min_exposure_time``: (default 1000) minimum exposure time (microseconds)
- ``max_exposure_time``: (default 1000000) max exposure time (microseconds)

## Topics

- ``meta``: camera image metadata to which the node subscribes
- ``control``: topic on which the control messages will be published

## License

This software is issued under the Apache License Version 2.0.
