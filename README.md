# OCS2 Humanoid MPC

This repository contains a Nonlinear Model Predictove Controller (NMPC) for humanoid loco-manipulation control. It contains a library of 
functionality and extends OCS2 to enable motion planning for humanoids. 

It contains off the shelf functionality to runt he following MPC configurations.

### Centroidal MPC
The centroidal MPC optimizes over the whole-body kinematics and the center off mass dynamics, with a choice to either use a single rigid 
body model or the full centroidal dynamics. It's functionality is contained in `humanoid_centroidal_mpc`.

### Whole-Body Dynamics MPC
The whole-body dynamics MPC optimized over the contact forces and joint accelerations with the option to compute the joint torques for 
each step planned accross the horizon. It's functionality is contained in `humanoid_wb_mpc`.

### Robot Examples

The project supports the following robot examples:

- Unitree G1
- 1X Neo (Comming soon)

## Get Started

### Setup Workspace

Create a colcon workspace and clone the repository into the src folder:

```bash
mkdir -p humanoid_mpc_ws/src && cd humanoid_mpc_ws/src
git clone https://github.com/manumerous/ocs2-humanoid-mpc.git
```

Then initialize all submodules using:

```bash
cd ocs2-humanoid-mpc
git submodule update --init --recursive
```

### Install Dependencies

Install all aptitude dependencies using:

```bash
xargs -a dependencies.txt sudo apt install
```

Then install ros2 jazzy as specified in
the [installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

As a final step:

```bash
sudo apt install ros-jazzy-ament-cmake-clang-format ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro ros-jazzy-mcap-vendor ros-jazzy-interactive-markers
```

### Building the MPC 

```bash
make build-all
```

## Running the examples

On the top level folder run: 

```make launch-g1-dummy-sim```

A window with Rviz will appear, On the first run the auto differentiation libraries will be generated. This might take up to 5-10 min depending on your system. Once done the robot appears and you can control it via the gamepad. 

## Acknowledgements

This work was developed by the company [1X Technologies](https://www.1x.tech/). I would like to thank everyone that contributed in code, discussion or suggestions to the success of this project. 

In particular I would like to acknowledge the support from Michael Purcell, Jesper Smith, Simon Zimmermann, Joel Filho, Varit (Ohm) Vichathorn, Armin Nurkanovic, Charles Khazoom, Farbod Farshidian, Eric Jang, Bernt Børnich and everyone at 1X Technologies.