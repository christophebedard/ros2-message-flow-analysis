# ROS 2 message flow analysis experiments

ROS 2 message flow analysis experiments using [`ros2_tracing`](https://gitlab.com/ros-tracing/ros2_tracing) and [Eclipse Trace Compass](https://www.eclipse.org/tracecompass/).
<!-- TODO
This is part of the [ROS 2 message flow paper](https://arxiv.org/abs/TODO).
If you use or refer to this method or this repository, please cite:
<!-- TODO replace with early access/published version when available ->
* C. Bédard, P.-Y. Lajoie, G. Beltrame, and M. Dagenais, "TODO," *arXiv preprint arXiv:22xy.01234*, 2022.

BibTeX:

```bibtex
@article{bedard2022messageflow,
  title={TODO},
  author={B{\'e}dard, Christophe and Lajoie, Pierre-Yves and Beltrame, Giovanni and Dagenais, Michel},
  journal={arXiv preprint arXiv:22xy.01234},
  year={2022}
}
```
-->

## Relevant pepositories

* `ros2_tracing`: tracing instrumentation and launch tools for ROS 2
    * [repository](https://gitlab.com/ros-tracing/ros2_tracing)
    * branch: [`message-link-instrumentation`](https://github.com/christophebedard/ros2_tracing/tree/message-link-instrumentation)
* Trace Compass Incubator
    * [website](https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/Trace-Compass-Incubator.html#Trace_Compass_Incubator)
    * branch: [`ros2-message-flow`](https://github.com/christophebedard/tracecompass-incubator/tree/ros2-message-flow)
* DDS implementations
    * Fast DDS
        * [repository](https://github.com/eProsima/Fast-DDS)
        * branch: [`instrumentation-lttng`](https://github.com/christophebedard/Fast-DDS/tree/instrumentation-lttng)
    * Cyclone DDS
        * [repository](https://github.com/eclipse-cyclonedds/cyclonedds)
        * branch: [`instrumentation-lttng`](https://github.com/christophebedard/cyclonedds/tree/instrumentation-lttng)
* Experimentation-related
    * Message flow test cases
        * [repository](https://github.com/christophebedard/ros2-message-flow-test-cases)
    * Autoware reference system
        * [repository](https://github.com/ros-realtime/reference-system)
        * branch: [`message-link-instrumentation`](https://github.com/christophebedard/reference-system/tree/message-link-instrumentation)

## Experiments

For all systems:
1. Setup system to build ROS 2 and enable tracing
    * https://docs.ros.org/en/rolling/Installation/Ubuntu-Development-Setup.html
    * https://gitlab.com/ros-tracing/ros2_tracing
        * The LTTng kernel tracer will be required for some experiments ([examples](#examples) and [experiment 1](#autoware-reference-system))

### Examples

See https://github.com/christophebedard/ros2-message-flow-test-cases.

1. For each of the 2 systems
    1. Make sure that the LTTng kernel tracer is installed
        * https://gitlab.com/ros-tracing/ros2_tracing#building
    1. Setup code workspaces and build
        ```sh
        ./exp-1_setup_workspace.sh
        ```
        * We use the same workspace as [experiment 1](#autoware-reference-system)
1. Run examples
    * First run the single-system examples on the system of your choice
        <!-- ./examples_run.sh -->
        ```sh
        source exp-1_ws/install/setup.bash
        ros2 launch ros2_message_flow_testcases examples/example-2_trivial.launch.py
        ros2 launch ros2_message_flow_testcases examples/example-3_periodic_async.launch.py
        ros2 launch ros2_message_flow_testcases examples/example-4_partial_sync.launch.py
        ```
    * Then run the distributed example over 2 systems
        * On system 1
            <!-- ./examples_run.sh 1 -->
            ```sh
            source exp-1_ws/install/setup.bash
            ros2 launch ros2_message_flow_testcases examples/example-1_transport_1.launch.py
            ```
        * On system 2
            <!-- ./examples_run.sh 2 -->
            ```sh
            source exp-1_ws/install/setup.bash
            ros2 launch ros2_message_flow_testcases examples/example-1_transport_2.launch.py
            ```
        * The order does not really matter
    * Data will be written to `examples/trace-example-*-YYYYMMDDTHHMMSS`

### Autoware reference system

In this experiment, we run and trace the [Autoware reference system proposed by the ROS 2 Real-Time Working Group](https://github.com/ros-realtime/reference-system).
We first run it in a single process on a single system, and then distributed it over multiple processes over 2 systems.

1. For each of the 2 systems
    1. Make sure that the LTTng kernel tracer is installed
        * https://gitlab.com/ros-tracing/ros2_tracing#building
    1. Setup code workspaces and build
        ```sh
        ./exp-1_setup_workspace.sh
        ```
        * this creates a workspace and builds it in release mode
        * the workspace includes all of ROS 2 from source, as well as some additional repos and specific branches for some of the ROS 2 repos (see [`reference_system.repos`](./reference_system.repos))
1. Run experiment
    * On a single system
        <!-- ./exp-1_run.sh -->
        ```sh
        source exp-1_ws/install/setup.bash
        ros2 launch experiment-1/reference_system.launch.py
        ```
    * Distributed over 2 systems
        * On system 1
            <!-- ./exp-1_run.sh 1 -->
            ```sh
            source exp-1_ws/install/setup.bash
            ros2 launch experiment-1/reference_system_1.launch.py
            ```
            ```sh
            ```
        * On system 2
            <!-- ./exp-1_run.sh 2 -->
            ```sh
            source exp-1_ws/install/setup.bash
            ros2 launch experiment-1/reference_system_2.launch.py
            ```
        * The order does not really matter
    * Experiment data will be written to `experiment-1/trace-reference-system*-YYYYMMDDTHHMMSS`
1. Analyze the traces
    * See [*Analysis*](#analysis)

### RTAB-Map

In this experiment, we distribute and run [RTAB-Map](https://github.com/introlab/rtabmap) over 2 systems and trace it.
We have 4 components: the camera driver node, the odometry node, the RTAB-Map node, and rviz.
These can be split up into two separate groups, one for each system.

1. For each of the 2 systems
    1. Setup [`rtabmap_ros`](https://github.com/introlab/rtabmap_ros) using the [`ros2` branch](https://github.com/introlab/rtabmap_ros/tree/ros2)
        * Follow the build instructions
    1. Prepare camera and driver
        * We use an Intel RealSense D400, so we use the `realsense_d400.launch.py` launch file
1. Synchronize system clocks
    1. Using NTP or PTP
1. Modify launch files
    1. Add `Trace` action to existing launch files to trace the system when executing them: `realsense_d400.launch.py` and `rtabmap_kitti.launch.py`
        ```py
        # ...
        from tracetools_launch.action import Trace
        # ...
        return LaunchDescription([
            # Tracing
            Trace(
                session_name='rtabmap-kitti',
                events_ust=[
                    'ros2:*',
                    'dds:*',
                ],
                events_kernel=[],
            ),
            # ...
        ])
        # ...
        ```
1. Run experiment
    * On system 1
        ```sh
        ros2 launch rtabmap_ros realsense_d400.launch.py
        ```
    * On system 2
        ```sh
        ros2 launch rtabmap_ros rtabmap_kitti.launch.py
        ```
    * The `rtabmap_kitti.launch.py` launch file can be modified to launch the `*_odometry` node and the `rtabmap` node separately
        * The `*_odometry` node can then be run on system 1
    * Launch rviz on system 2 for visualization
    * Experiment data will be written to `~/.ros/tracing/rtabmap-kitti` on each system
1. Analyze the traces
    * See [*Analysis*](#analysis)

## Analysis

1. Set up [Eclipse Trace Compass](https://www.eclipse.org/tracecompass/)
    * Currently, to use the analysis features described in the paper, Trace Compass needs to be built from source
    * Follow the instructions on the [*Development Environment Setup* page](https://wiki.eclipse.org/Trace_Compass/Development_Environment_Setup) up to *Run (or Debug) the plugins*
        * Clone & build both Trace Compass and the Trace Compass Incubator
        * Use this branch for the Incubator: [`ros2-message-flow`](https://github.com/christophebedard/tracecompass-incubator/tree/ros2-message-flow)
1. Run Trace Compass
    * See the [*Run (or Debug) the plugins* section](https://wiki.eclipse.org/Trace_Compass/Development_Environment_Setup#Run_.28or_Debug.29_the_plugins)
    * See the [*Trace Compass User Guide*](https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/User-Guide.html) for a full user guide
1. Import trace and visualize
    1. Under *File*, click on *Import...*
    1. Select the root directory of the trace (`system-YYYYMMDDTHHMMSS/`)
        * See the experiment instructions for the path to the trace directory
    1. Then make sure the trace directory is selected in the filesystem tree view
    1. Click on *Finish*
    1. (See also the [user guide](https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/Trace-Compass-Main-Features.html#Importing_Traces_to_the_Project))
1. Create experiment (i.e., an aggregation of multiple traces)
    1. In the tree view on the left, under *Traces*, select both traces
    1. Then right click, and, under *Open As Experiment...*, select *ROS 2 Experiment*
    1. (See also the [user guide](https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/Trace-Compass-Main-Features.html#Creating_an_Experiment))
1. (for Autoware reference system experiement) Synchronize traces
    * See [*Synchronize traces in Trace Compass*](https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/Trace-synchronization.html#Synchronize_traces_in_Trace_Compass)
1. Open *Messages* view
    * This shows timer & subscription callbacks as well as message publications and receptions over time for each node.
    * Arrows also provide links between subscription or timer callbacks and message publications, as well as between message publications and the resulting subscription callback(s).
1. Navigate and inspect the trace
    * Basic controls:
        * `Ctrl` and mouse wheel up/down to zoom in/out
        * `Shift` and mouse wheel up/dowm to move left/right
1. Run *Message Flow* analysis
    1. Click on a segment in the *Messages* view, i.e., message publication or timer/subscription callback instance
    1. Click on the *Follow this element* button in the top right of the view (hover over buttons to see their description)
    1. The analysis will run and should not take much time (less than 5-10 seconds)
1. Open *Message Flow* view to view the analysis results
    1. Press `Ctrl+3` and enter *Message Flow*
    1. In the results below, click on *Message Flow (incubator) (ROS 2)*
    1. The *Message Flow* view should open

## Useful commands

* For running experiments on a separate system
    * Copy experiment directories from remote to local
        ```sh
        scp -P $PORT -r $USER@server:/home/$USER/ros2-message-flow-analysis/examples/trace-example-* .
        ```
