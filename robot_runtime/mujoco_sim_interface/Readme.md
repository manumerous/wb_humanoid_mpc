# Neo MuJoCo Interface

## Installation

To set up MuJoCo build and install it from source according to the following [installation guide](https://mujoco.readthedocs.io/en/latest/programming/#building-mujoco-from-source).

## Add to your bashrc

Add the mujoc installation path with the correct version to your bashrc. 
This could for example look like this: 
```
export MUJOCO_PATH=/opt/mujoco-3.1.3/
```


## Generate new MuJoCo model files

In a local downloaded release of mujoco run the simulation by copying over the urdf and modifying it as follows:

To the urdf add inside the robot tag: 

```
<mujoco>
    <compiler 
    meshdir="../meshes/" 
    discardvisual="true"
    convexhull="true" />

</mujoco>

<link name="world_link">
<!-- The world link typically does not have visual or collision properties 
    as it's an abstract reference frame. -->
</link>
```

and 

```
<!-- Define a free-floating joint -->
<joint name="world_joint" type="floating">
    <!-- Connect the world link to the original base link of your robot -->
    <parent link="world_link"/>
    <child link="link_pelvis"/> <!-- Replace with your robot's actual base link name -->
    <!-- The origin of a floating joint is typically at the robot's base frame -->
    <origin xyz="0 0 5" rpy="0 0 0"/>
</joint>
```

Then launch the sim in the bin folder using: 

```
./simulate ../neo_alpha1_5_description/urdf/neo_alpha1_5.urdf
```

or similar. 

Then in the window that opens klick export xml. 

Then modify the xml by adding the extra actuator and collision entries as required. Here take a look at the MuJoCo files that were already created. 