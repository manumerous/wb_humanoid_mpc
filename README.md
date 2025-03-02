# Whole-Body Humanoid MPC

This repository contains a Whole-Body Nonlinear Model Predictive Controller (NMPC) for humanoid loco-manipulation control. This approach enables to directly optimize through the **full-order torque-level dynamics in realtime** to generate a wide range of humanoid behaviors building up on an [updated version of ocs2](https://github.com/manumerous/ocs2_ros2)

**Interactive Velocity and Base Height Control via Joystick:**
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

### Setup Colcon Workspace

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
The project supports both Dockerized workspaces (recommended) or a local installation for developing and running the humanoid MPC. 

<details>
<summary>Dockerized Workspace</summary>

We provide a [Dockerfile](https://github.com/manumerous/wb_humanoid_mpc/blob/main/docker/Dockerfile) to enable running and devloping the project from a containerized environment. Check out the [devcontainer.json](https://github.com/manumerous/wb_humanoid_mpc/blob/main/.devcontainer/devcontainer.json) for the arguments that must be supplied to the `docker build` and `docker run` commands. 

For working in **Visual Studio Code**, we recommend to install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension. Then, with the root of this repository as the root of your VS Code workspace, enter `Ctrl + Shift + P` and select `Dev Containers: Rebuild and Reopen in Container` at the top of the screen. VS Code will then automatically handle calling the `docker build` and `docker run` commands for you and will reopen the window at the root of the containerized workspace. Once this step is completed, you are ready to [build and run the code](https://github.com/manumerous/wb_humanoid_mpc/tree/main?tab=readme-ov-file#building-the-mpc).

</details>

<details>
<summary>Install Dependencies Locally</summary>

Make sure you have **ros2** installed on your system as e.g specified for jazzy in
the [installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

Then install all dependencies using:

```bash
envsubst < dependencies.txt | xargs sudo apt install -y
```
</details>

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

#### Interactive Robot Control
Command a desired base velocity and root link height via **Robot Base Controller GUI** and **XBox Controller Joystick**. For the joystick it is easiest to directly connect via USB. Otherwise you need to install the required bluetooth Xbox controller drivers on your linux system. The GUI application automatically scanns for Joysticks and indicates whether one is connected. 

![robot_remote_control](https://github.com/user-attachments/assets/779be1da-97a1-4d0c-8f9b-b9d2df88384f)


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

Further acknowledgement for their contributions, insights, discussion and support goes to Michael Purcell, Jesper Smith, Simon Zimmermann, Joel Filho, Paal Arthur Schjelderup Thorseth, Varit (Ohm) Vichathorn, Sjur Grønnevik Wroldsen, Armin Nurkanovic, Charles Khazoom, Farbod Farshidian, Eric Jang, Bernt Børnich and everyone at 1X Technologies.
