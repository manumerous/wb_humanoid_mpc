# Whole-Body Humanoid MPC

This repository contains a Whole-Body Nonlinear Model Predictove Controller (NMPC) for humanoid loco-manipulation control. It contains a library of 
functionality and extends OCS2 to enable motion planning for humanoids. 

**Velocity and Base Height Control via Joystick:**
![Screencast2024-12-16180254-ezgif com-video-to-gif-converter(1)(3)](https://github.com/user-attachments/assets/a032477b-2e70-41b0-90d3-9539e1a4b723)

It contains the following MPC fromulations to be applied to any humanoid robot. 

### Centroidal Dynamics MPC
The centroidal MPC optimizes over the **whole-body kinematics** and the center off mass dynamics, with a choice to either use a single rigid 
body model or the full centroidal dynamics. It's functionality is contained in `humanoid_centroidal_mpc`.

### Whole-Body Dynamics MPC
The **whole-body dynamics** MPC optimized over the contact forces and joint accelerations with the option to compute the joint torques for 
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
git clone https://github.com/1x-technologies/wb-humanoid-mpc.git
```

Then initialize all submodules using:

```bash
cd wb-humanoid-mpc
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
Once you run the NMPC a window with Rviz will appear for visualization. The first time you start the MPC for a certain robot model the auto differentiation code will be generated which might take up to 5-15 min depending on your system. Once done the robot appears and you can control it via an xbox gamepad or the controls in the terminal. 

On the top level folder run:

For the **Centroidal Dynamics MPC**

```
make launch-g1-dummy-sim
```

For the **Whole-Body Dynamics MPC**

```
make launch-wb-g1-dummy-sim
```
## Citing Whole-Body Humanoid MPC
To cite the Whole-Body Humanoid MPC in your academic research, please consider citing the following web BibTeX entry:

```
@misc{wholebodyhumanoidmpcweb,
   author = {Manuel Yves Galliker},
   title = {Whole-body Humanoid MPC: Realtime Physics-Based Procedural Loco-Manipulation Planning and Control},
   howpublished = {https://github.com/1x-technologies/wb_humanoid_mpc},
   year = {2024}
}
```

## Acknowledgements
This project was developed at [1X Technologies](https://www.1x.tech/) and is primarily authored and maintained by [Manuel Yves Galliker](https://github.com/manumerous).

Further acknoledgment for their contributions, insights, discussion and support goes to Michael Purcell, Jesper Smith, Simon Zimmermann, Joel Filho, Paal Arthur Schjelderup Thorseth, Varit (Ohm) Vichathorn, Armin Nurkanovic, Charles Khazoom, Farbod Farshidian, Eric Jang, Bernt Børnich and everyone at 1X Technologies.
