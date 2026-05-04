# Particle Filter tf node
This code implements a tf converter to properly provide `/tf` data when using [Particle Filter](https://github.com/CPS-TUWien/particle_filter).

In the original version (currently with `publish_tf = True`), particle filter provided transformation `/map → /laser` which is unfortunately not compatible with `/odom → /laser`.

The `pf_tf_node` obtains the particle filter transformation from the Odometry message (`/odom` by default) and computes the proper transformation to create `/map → /odom → /laser` tree.

Usage:
```sh
ros2 run pf_tf_node pf_tf_node --ros-args -r /odom:=/pf/pose/odom
```