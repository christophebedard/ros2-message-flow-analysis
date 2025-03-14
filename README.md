# ROS 2 message flow analysis experiments

ROS 2 message flow analysis experiments using [`ros2_tracing`](https://github.com/ros2/ros2_tracing) and [Eclipse Trace Compass](https://eclipse.dev/tracecompass/).

This is part of the [ROS 2 message flow paper](https://arxiv.org/abs/2204.10208).
If you use or refer to this method or this repository, please cite:
* C. Bédard, P.-Y. Lajoie, G. Beltrame, and M. Dagenais, "Message Flow Analysis with Complex Causal Links for Distributed ROS 2 Systems," *Robotics and Autonomous Systems*, vol. 161, p. 104361, 2023.

BibTeX:

```bibtex
@article{bedard2023messageflow,
  title={Message flow analysis with complex causal links for distributed {ROS} 2 systems},
  author={B{\'e}dard, Christophe and Lajoie, Pierre-Yves and Beltrame, Giovanni and Dagenais, Michel},
  journal={Robotics and Autonomous Systems},
  year={2023},
  volume={161},
  pages={104361},
  doi={10.1016/j.robot.2022.104361}
}
```

## Relevant repositories

* `ros2_tracing`: tracing instrumentation and launch tools for ROS 2
    * [repository](https://github.com/ros2/ros2_tracing)
    * branch: [`message-link-instrumentation`](https://github.com/christophebedard/ros2_tracing/tree/message-link-instrumentation)
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
    1. Set up system to build ROS 2 from source. This was originally tested with ROS 2 Humble on Ubuntu 22.04: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
    1. Install LTTng, including the kernel tracer: https://github.com/ros2/ros2_tracing/tree/humble#building
        * Make sure that you have lttng-ust >= 2.13.3
            * Otherwise, there's a build issue related to lttng-ust's `ctf_sequence_hex`, see [LTTng bug #1355](https://bugs.lttng.org/issues/1355)
            * Check with:
                ```sh
                dpkg -l | grep liblttng-ust-dev
                ```
            * If the version installed on your system is lower than 2.13.3, [install a newer version (>= 2.13.3) using the LTTng stable PPA](https://lttng.org/docs/v2.13/#doc-ubuntu-ppa)
        * The LTTng kernel tracer will be required for some experiments ([examples](#examples) and [experiment 1](#autoware-reference-system))

### Examples

See https://github.com/christophebedard/ros2-message-flow-test-cases.

1. For each of the 2 systems
    1. Make sure that the LTTng kernel tracer is installed
        * https://github.com/ros2/ros2_tracing/tree/humble#building
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
We first run it in a single process on a single system, and then distribute it over multiple processes over 2 systems.

1. For each of the 2 systems
    1. Make sure that the LTTng kernel tracer is installed
        * https://github.com/ros2/ros2_tracing/tree/humble#building
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
        * On system 2
            <!-- ./exp-1_run.sh 2 -->
            ```sh
            source exp-1_ws/install/setup.bash
            ros2 launch experiment-1/reference_system_2.launch.py
            ```
        * The order does not really matter
    * Variant: launch the same system again, but with `reference_system_1b.launch.py` for system 1, which uses a multi-threaded executor for one of the most critical processes
        * On system 1
            <!-- ./exp-1_run.sh 1 -->
            ```sh
            source exp-1_ws/install/setup.bash
            ros2 launch experiment-1/reference_system_1b.launch.py
            ```
        * On system 2
            <!-- ./exp-1_run.sh 2 -->
            ```sh
            source exp-1_ws/install/setup.bash
            ros2 launch experiment-1/reference_system_2.launch.py
            ```
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
    1. Add `Trace` action to existing launch files to trace the system when executing them: `realsense_d400.launch.py` and `rtabmap.launch.py`
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
        ros2 launch rtabmap_ros rtabmap.launch.py
        ```
    * The `rtabmap.launch.py` launch file can be modified to launch the `*_odometry` node and the `rtabmap` node separately
        * The `*_odometry` node can then be run on system 1
    * Launch rviz on system 2 for visualization
    * Experiment data will be written to `~/.ros/tracing/rtabmap-kitti` on each system
1. Analyze the traces
    * See [*Analysis*](#analysis)

### Overhead

In this experiment, we evaluate the end-to-end latency for a typical system when tracing is disabled and when it is enabled.
The end-to-end latency difference is the overhead.

1. For each of the 2 systems
    1. Workspace with tracing
        ```sh
        ./exp-1_setup_workspace.sh
        ```
        * We use the same workspace as [experiment 1](#autoware-reference-system)
    1. Workspace without any tracepoints or instrumentation
        ```sh
        ./exp-3_setup_workspace.sh
        ```
1. Run experiment
    1. First with tracing
        ```sh
        source exp-1_ws/install/setup.bash
        ros2 launch overhead/end_to_end_tracing.launch.py
        ```
        * Latency data will be written to `latencies_tracing_*.txt`
    1. Then without tracing
        ```sh
        source exp-3_ws/install/setup.bash
        ros2 launch overhead/end_to_end_no-tracing.launch.py
        ```
        * Latency data will be written to `latencies_no-tracing_*.txt`
1. Plot results
    * Providing the names of the two files
        ```sh
        cd overhead/
        python3 plot_latencies.py latencies_no-tracing_*.txt latencies_tracing_*.txt
        ```
    * A plot will be displayed and exported to a file

## Analysis

1. Download [Eclipse Trace Compass](https://eclipse.dev/tracecompass/)
    * Install ROS 2 features from the [Trace Compass Incubator](https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/Trace-Compass-Incubator.html#Trace_Compass_Incubator):
        * Open Trace Compass, click on *Help*, then *Install New Software...*
        * Enter the following update site URL: `https://download.eclipse.org/tracecompass.incubator/master/repository/`
        * Under *Trace Types*, select *Trace Compass ROS 2 (Incubation)*
        * Click *Next* twice, then accept the license terms, and click *Finish*
        * When prompted, restart Trace Compass
    * Or use the provided Dockerfile:
        ```sh
        docker build --tag tc-incubator tc-incubator/
        docker run --net=host -e DISPLAY -v ~/.ros/tracing:/root/.ros/tracing -v ~/.tracecompass:/root/.tracecompass tc-incubator
        ```
1. Run Trace Compass
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
1. (for Autoware reference system experiment) Synchronize traces
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
