# RQT ROS 2 Lifecycle Manager Plugin

Simple RQT plugin to manage the lifecycle of ROS 2 nodes.

Currently, has minimal functionality to `configure`, `activate`, `deactivate`, `cleanup` and `shutdown` nodes.

## Installation
```bash
$ mkdir -p ~/colcon_rqt_ws/src
$ cd ~/colcon_rqt_ws/src
$ git clone https://github.com/ipa-vsp/rqt_lifecycle_manager.git
$ cd ~/colcon_rqt_ws
$ colcon build
$ source install/local_setup.bash
```

## Usage
Run: 
```bash
$ ros2 run rqt_lifecycle_manager rqt_lifecycle_manager --force-discover
```
Once the GUI is loaded and press `Refresh` to get the list of available lifecycle nodes.

### Todo
- [ ] Improve UI
- [ ] More lifecycle states and transitions support
- [ ] Get available transitions from the node
- [ ] Fix trasitioning issues in UI display. 
