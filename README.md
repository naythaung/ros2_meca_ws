# Meca500 MoveIt learning and test-rig template

This branch extends Solène’s existing Meca500 ROS/RViz testing environment with MoveIt 2 planning, collision geometry and laptop-only trajectory execution. It uses the existing robot, camera assembly, breadboard and phantom as a working example.

**The purpose is to learn and establish a reusable template for a different test rig.** The current CAD assembly is a stepping stone. The intended next stage is to import the new Onshape rig, define its tool frame and obstacles, then write repeatable commands that move the robot to specified positions and orientations.

Development branch: `naythan-joint-gui`. Working environment: **Ubuntu 26.04, ROS 2 Lyrical**, on a Dell XPS 13. The interactive workflow was demonstrated locally in September 2026. A clean-machine installation has not yet been independently tested; package versions are not pinned.

## Current milestone

- Meca500 and the existing rig load in RViz.
- The `meca_arm` group uses KDL inverse kinematics and OMPL/RRTConnect planning.
- Collision geometry covers the arm, phantom, breadboard and camera/adapter assembly.
- Specific intentional assembly contacts are excluded through the SRDF.
- Planned paths receive velocity/acceleration-based timing.
- `ros2_control` mock hardware follows trajectories, and joint-state feedback updates RViz and MoveIt.
- Subsequent plans can start from the previous executed destination.

This is idealised motion simulation, not a dynamics/contact simulation. Success means the configured planning and mock-control pipeline accepted and followed a trajectory; it does not validate physical forces, sensor accuracy or real hardware safety. RRTConnect does not guarantee the shortest or fastest path.

## How the system fits together

| Component | Responsibility |
|---|---|
| Xacro / URDF | Robot links, joints, meshes, transforms and control interfaces |
| SRDF | Planning group, named joint states and allowed collision pairs |
| MoveIt / `move_group` | Kinematics, collision checking, path planning and execution requests |
| OMPL / RRTConnect | Searches for a feasible path through joint configurations |
| Time parameterisation | Adds timing using joint velocity and acceleration limits |
| Trajectory controller | Follows the timed joint trajectory |
| `mock_components/GenericSystem` | Mirrors commanded positions into simulated joint states |
| Joint-state broadcaster | Publishes `/joint_states` |
| Robot-state publisher | Converts joint states into `/tf` and `/tf_static` transforms |
| RViz | Displays geometry, editable goals, previews and current robot state |

`/robot_description` carries the model. `/display_planned_path` carries previews. MoveIt sends execution requests through `/meca_velocity_controller/follow_joint_trajectory`, a ROS action with feedback and a completion result. `/trajectory_execution_event` carries control events such as stopping execution.

Despite its inherited name, **`meca_velocity_controller` uses position command interfaces in this mock configuration**.

## Installation and dependencies

### 1. ROS environment

Install ROS 2 Lyrical using the [official Ubuntu installation instructions](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html). The commands below assume the ROS apt repository is configured and ROS is installed at `/opt/ros/lyrical`.

Use a normal system-Python terminal for the core simulation. An activated unrelated virtual environment can prevent ROS Python modules from being found.

```bash
sudo apt update
sudo apt install git build-essential python3-colcon-common-extensions \
  python3-rosdep python3-yaml \
  ros-lyrical-desktop ros-lyrical-moveit \
  ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
  ros-lyrical-xacro ros-lyrical-joint-state-publisher-gui
```

These are the intended apt dependencies for this branch, not a frozen list of the developer's installed versions. Check Lyrical package availability if apt reports a missing package; do not substitute another ROS distribution's binaries.

The MoveIt meta-package supplies the planning, kinematics, visualisation and controller integration stack. The control packages supply controller manager, mock hardware, joint-state broadcaster and joint trajectory controller.

### 2. Clone and resolve workspace dependencies

```bash
mkdir -p ~/Documents
cd ~/Documents
git clone --branch naythan-joint-gui https://github.com/naythaung/ros2_meca_ws.git
cd ros2_meca_ws
source /opt/ros/lyrical/setup.bash
```

Initialise rosdep once per machine, if it has not already been initialised:

```bash
sudo rosdep init
```

Then:

```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro lyrical -r -y
```

Some inherited `package.xml` files are incomplete. The explicit packages above cover core functionality missing from their declarations; rosdep alone is not yet a complete installation guarantee.

### 3. Python packages: core versus optional tools

**No separate pip installation is needed for the core MoveIt/mock-hardware launch.** Its Python launch file imports `yaml`, supplied by `python3-yaml`; ROS Python modules come from ROS packages.

The inherited camera, analysis and hardware scripts have additional dependencies:

| Dependency | Used for | Installation |
|---|---|---|
| NumPy, SciPy | Transform calculations and numerical processing | `python3-numpy python3-scipy` through apt |
| OpenCV, Matplotlib | Camera images, depth previews and dataset plots | `python3-opencv python3-matplotlib` through apt |
| `cv_bridge`, `rosbag2_py` | ROS image conversion and bag processing | `ros-lyrical-cv-bridge ros-lyrical-rosbag2-py` through apt |
| `mecademicpy` | Inherited physical-robot driver/scripts | pip, optional |
| `trimesh`, `rtree` | Mesh ray casting for mock ground-truth generation | pip, optional; `rtree` supports the ray-query backend |

For those optional workflows:

```bash
sudo apt install python3-venv python3-numpy python3-scipy \
  python3-opencv python3-matplotlib \
  ros-lyrical-cv-bridge ros-lyrical-rosbag2-py
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install mecademicpy trimesh rtree
touch .venv/COLCON_IGNORE
```

This is a dependency inventory inferred from the checked-in imports, not a claim that every optional script has been tested on Lyrical. No pip version lock or reproducible Onshape-export toolchain is currently recorded. Do not install ROS itself using pip. Leave the environment with `deactivate` before returning to the standard simulation workflow.

## Build and launch

Run from the workspace root, **not from `src`**:

```bash
cd ~/Documents/ros2_meca_ws
source /opt/ros/lyrical/setup.bash
colcon build
source install/setup.bash
ros2 launch meca500_moveit_config move_group.launch.py
```

`colcon build` builds all discovered workspace packages incrementally. `source` makes the ROS installation and then this workspace available in the current shell. The launch file starts RViz, MoveIt, robot-state publisher, controller manager and controller spawner. Keep the terminal running; Ctrl+C stops the launch.

After saving an edit, stop the launch, rebuild, source and relaunch. If nothing changed, skip the build. Do not run a separate joint-state GUI alongside mock execution: the controller should be the source of joint feedback.

The developer's optional `meca_sim` shell function automates these commands. Editing `~/.bashrc` is not required for other users. At the documentation snapshot, `scripts/meca_sim.bash` is not present on this remote branch; the commands above are the supported entry point.

### RViz configuration and high-DPI workaround

The launch file asks RViz to load `config/scene.rviz`. **That file is not present in the inspected remote snapshot**, even though a layout was saved locally during development. Until it is committed, recreate the layout if RViz opens without it:

1. Enable the Displays panel and add the MoveIt `MotionPlanning` display.
2. Show/add the MotionPlanning panel if its controls are hidden.
3. Set Fixed Frame to `world` and Planning Group to `meca_arm`.
4. For a separate RobotModel display, select Topic as the description source and `/robot_description` as its topic.
5. Save with File → Save Config As to `src/meca500_moveit_config/config/scene.rviz`.
6. Rebuild the configuration package. The next launch reads the installed copy.

Keep the source filename stable and use Git commits for history. RViz layouts store panels, display settings and the view, not a durable saved execution or robot pose. Saving directly into `install/` will not update the source file.

The RViz node explicitly sets:

```python
additional_env={"QT_ENABLE_HIGHDPI_SCALING": "0"}
```

This preserves the high-DPI scaling workaround used on the development laptop. It is an RViz/Qt display setting, not a change to robot dimensions, units or planning. Other Qt/Wayland overrides tried during earlier setup are not all part of this launch file.

## First obstacle-avoidance test

1. Leave **Use Cartesian Path unchecked** for OMPL obstacle avoidance.
2. Choose a reachable start configuration clear of the phantom and execute it.
3. Set Start State to `<current>`.
4. Place a reachable, collision-free goal across the phantom.
5. Press **Plan** and inspect the whole arm and camera assembly from multiple angles.
6. Press **Execute** to run that inspected plan. **Plan and Execute** calculates a fresh plan.

A useful test requires an obstacle interaction, rather than a movement entirely clear of the rig. A preview alone is not evidence of successful execution. Expected terminal messages include `Goal reached, success!` and `Solution was found and executed.`

For easier inspection, reduce velocity and acceleration scaling, and hide Query Start/Goal State overlays after planning. Scaling changes timing, not the geometric objective. Cartesian planning follows the specified tool path and can stop at an obstruction; it does not automatically generate a detour. A large upward route can be a valid RRTConnect result without being optimal.

## Important files

Paths below are relative to the workspace root. Edit source files, not generated copies.

| File or directory | Purpose |
|---|---|
| `src/meca500_scene_description/urdf/scene.urdf.xacro` | Current complete rig: world/scene frames, robot placement, camera mounts, phantom, breadboard, collision meshes and mock control interfaces |
| `src/meca500_scene_description/meshes/` | Existing rig STL assets; visual and collision geometry reference these |
| `src/meca500_description/urdf/meca500_macro.xacro` | Reusable arm links/joints included by the scene |
| `src/meca500_description/urdf/meca500.urdf.xacro` | Inherited standalone arm wrapper; invokes the real-hardware control macro |
| `src/meca500_description/urdf/meca500.ros2_control.xacro` | Inherited physical hardware plugin/IP and velocity interfaces; not the mock interface used by the scene launch |
| `src/meca500_moveit_config/launch/move_group.launch.py` | Loads descriptions/configuration and starts the complete mock execution stack |
| `src/meca500_moveit_config/config/meca500.srdf` | `meca_arm` chain (`base_link` to `link_6`), home state and exact collision exclusions |
| `src/meca500_moveit_config/config/kinematics.yaml` | KDL IK solver and solver settings |
| `src/meca500_moveit_config/config/ompl_planning.yaml` | RRTConnect configuration plus request/response adapters |
| `src/meca500_moveit_config/config/joint_limits.yaml` | Joint speed/acceleration limits and default scaling; angles use radians |
| `src/meca500_moveit_config/config/moveit_controllers.yaml` | Tells MoveIt which FollowJointTrajectory action to contact |
| `src/meca500_moveit_config/config/mock_controllers.yaml` | Configures controller manager, broadcaster and position trajectory controller |
| `src/meca500_moveit_config/config/scene.rviz` | Intended saved layout path; still needs to be added to the remote snapshot |
| `src/meca500_scene_description/launch/view_scene.launch.py` | Scene visualisation workflow; not the complete MoveIt execution launch |
| `src/meca500_scene_description/launch/headless_scene.launch.py` | Scene transforms without the RViz interface |
| Each package's `package.xml` | Declares dependencies and package metadata |
| Each package's `CMakeLists.txt` or `setup.py` | Defines build/install rules; scene/MoveIt configs must be installed for launch-time lookup |
| `src/meca500_driver/` | Inherited Mecademic connection, joint feedback and sweep/capture routines |
| `src/camera_bridge/` | Inherited camera stream to ROS image bridge |
| `src/data_collection/` | Capture and mock ground-truth generation tools |
| `src/compute_*.py`, `src/*footprint*.py`, `src/check_camera_aim.py` | Inherited geometry, transform and footprint analysis helpers |
| `src/extract_video.py`, `src/generate_depth_previews.py`, `src/inspect_dataset.py` | Inherited bag/image/dataset processing helpers |
| `src/onshape_export/` | Existing exported CAD/URDF material; not an automatic import pipeline for a new rig |
| `start_system.sh`, `update_scene.sh`, `run_ground_truth_batch.sh` | Legacy workflows with old workspace paths; some use Jazzy and physical devices. Do not use as the Lyrical mock launcher |

Some inherited driver scripts connect to, activate and home the physical robot. They are distinct from the mock launch. Preserve original authorship/licensing and review these workflows with the rig owner before physical use.

## What was changed to reach this stage

1. Adapted MoveIt planning-pipeline configuration for the working Lyrical setup, including OMPL plugin loading and response adapters.
2. Matched the SRDF robot name to `meca500_scene`.
3. Added collision geometry for the phantom, breadboard and camera assembly. Visual geometry alone is not collision geometry; mesh scales and origins must agree.
4. Added exact SRDF exclusions for intentional contacts: flange/camera assembly contacts, camera assembly internal contacts, base/breadboard and breadboard/phantom. Other pairs remain checked.
5. Supplied `robot_description_planning` to **both MoveIt and RViz**. Without RViz's acceleration limits, Cartesian geometry could reach 100% but timestamp generation failed.
6. Added mock hardware, a running trajectory controller and joint-state broadcaster. A controller address in YAML alone did not start a controller, causing `Action client not connected` errors.
7. Preserved the Qt high-DPI workaround and configured a stable RViz layout path.

The phantom is currently a fixed link within the scene URDF. Its contacts are checked as part of that model subject to the allowed collision matrix. A separately published planning-scene `CollisionObject` is not implemented here; it is an option for movable/reconfigurable obstacles later.

## Applying the template to the new rig

1. Import the new CAD assets and verify units, axes and assembly transforms. Existing millimetre STLs use `0.001` scale; inspect each new export rather than assuming its units.
2. Reuse the arm model and planning setup where appropriate, replacing the surrounding scene and payload geometry.
3. Add appropriate collision shapes and re-evaluate every intentional-contact exclusion. Do not copy old exceptions blindly.
4. Define a tool centre point (TCP) for the laser/probe/sensor, with a fixed transform from the flange. Current requests target `link_6`; sensor positioning needs the correct working frame.
5. Establish named joint configurations and target poses in an explicit reference frame. Specify position in metres and orientation as a valid rotation/quaternion.
6. Implement a ROS node using a MoveIt interface to set a target, plan, check the result and execute against mock hardware. Publishing joint states alone does not request collision-aware motion.
7. Split the task into approach, measurement/sweep and retreat. Add orientation/distance constraints and Cartesian waypoints where measurement requires them.
8. Integrate sensor/actuation messages and synchronisation once their actual interfaces are known.

**Repeatable scripted pose commands are the next milestone, not an implemented feature of this branch.** Existing SDK sweep code is not proof of MoveIt collision-aware scripted execution. Record success, path clearance and task behaviour in simulation before considering the physical rig and its operating procedures.

## Troubleshooting and remaining cleanup

| Symptom | Check |
|---|---|
| Missing robot in RViz | Full launch running; `/robot_description` selected; Fixed Frame `world`; inspect display Status |
| Cartesian path 100%, timing fails | Joint acceleration limits present and passed to RViz as well as `move_group` |
| Preview works, execution fails | Controller action connection and `ros2 control list_controllers`; both controllers should be active |
| Permanently red assembly | Inspect exact collision pairs and whether they are intentional; also check for real overlap or misplaced geometry |
| Strange detour | Compare fixed start/goal plans, target orientation, full payload geometry and planner behaviour |
| New edits absent | Save source files, rebuild from workspace root, source workspace, restart nodes |
| Layout missing | Add the source `scene.rviz`, rebuild and commit it |

Known repository cleanup still to do:

- Add the saved RViz layout and optional launcher script to the remote branch.
- Complete runtime dependencies and placeholder metadata in package manifests.
- Remove tracked generated `src/build` and `src/install` content through a reviewed cleanup; these directories currently have `COLCON_IGNORE` markers. Root-only ignore rules do not cover all nested build outputs.
- Ignore local `.venv`/`.venv-1` environments and add `COLCON_IGNORE` inside them if present.
- Audit inherited absolute paths, old distro references and hardware assumptions before reusing optional tools.
- Record tested package versions and verify setup on a fresh Ubuntu 26.04/Lyrical machine.
- Add repeatable scripted targets and an explicit sensor TCP for the next rig.

Further reading: [MoveIt RViz tutorial](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html), [ros2_control mock hardware](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html), [OMPL planners](https://ompl.kavrakilab.org/planners.html). Match examples to the installed version; Rolling documentation may differ from Lyrical.
