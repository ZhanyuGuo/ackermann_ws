# Ackermann

## 01. Quick Start within 3 minutes

```bash
git clone https://github.com/ZhanyuGuo/ackermann_ws.git
cd ackermann_ws/
catkin_make
source devel/setup.bash
roslaunch racecar_gazebo racecar_keyop.launch
```

NOTE: You can disable the visualization of laser in **line 65** of `racecar_description/urdf/racecar.gazebo`.

## 02. Standard Process

### Preliminary

1. Download the repository.
    ```bash
    git clone https://github.com/ZhanyuGuo/ackermann_ws.git
    ```

2. Compile the code.
    ```bash
    cd ackermann_ws/
    catkin_make
    ```

**Remember to source the environment before running the following code.**

```bash
source devel/setup.bash
```

or you can add it into your `~/.bashrc` or `~/.zshrc`.

### Use keyboard to control the car

Launch the racecar by

```bash
roslaunch racecar_gazebo racecar_keyop.launch
```

focus on the small tk window and use `WASD` to control the car.

### Mapping

Launch the racecar by

```bash
roslaunch racecar_gazebo racecar_mapping.launch
```

control the car and save map by

```bash
roscd racecar_gazebo/
cd map/
./save.bash
```

and then you can find the map in the same directory.

### Navigation

Launch the racecar by

```bash
roslaunch racecar_gazebo racecar_navigation.launch
```

use the 2D Nav Goal to set the goal.

## 03. Acknowledgment

This repository is built on [mit-racecar](https://github.com/mit-racecar).
