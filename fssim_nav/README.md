# TDR-SDC's Software Stack

Wall Following robot implementation of [f1tenth_simulator](https://github.com/f1tenth/f1tenth_simulator)

## I. Bulding code

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/TDR-SDC/fssim_nav
cd ..
catkin_make
```

## II. Launch the simulator:

Run the two sets of commands in different terminals:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fssim_nav simulator.launch
```

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fssim_nav control.launch
```
