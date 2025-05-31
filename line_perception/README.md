# line_perception

This package is to allow the lines of the IGVC course to be able to be added to the Costmap within Nav2 so they can be navigated around.

## How it works

The main `line_perception` node subscribes to the raw color image from the depth camera and uses OpenCV to apply an ROI/mask, then runs the image through a basic pipeline to essentially further mask the image for anything white.
We combine this mask with the raw depth image from the depth camera, then use the `depth_image_proc` node to convert the "filtered" depth image back to a point cloud.
The point cloud is then added to the global costmap as a voxel layer within Nav2.

## How to run it

First, make sure the dependencies are installed:

```
cd ros2_ws
rosdep install --from-paths src -y --ignore-src
```

Build and source the workspace:

```
colcon build --symlink-install
. install/setup.bash
```

The node (and it's dependency) can then be launched:

```
ros2 launch line_perception lines.launch.py
```