# ROS2 Driver for VISIOSCAN

The document describes the usage of the driver for BEA VISIOSCAN laser scanner.

The driver is based upon the [Boost Asio library](http://www.boost.org)

## Supported LIDAR

| Lidar Model            |
| ---------------------- |
| Visioscan RD           |

Visit following website for more details about Visioscan laser scanner: <https://asia.beasensors.com/en/product/lzr-visioscan-rd/>

## Tested environment

The driver is only tested within a VirtualBox VM installing environment below:

OS: [Ubuntu 22.04.5 LTS (Jammy Jellyfish)](https://www.releases.ubuntu.com/22.04/)

ROS2: [Humble Hawksbill](https://docs.ros.org/en/humble/)

## Usage with ROS2

### Create a ROS2 workspace

1. For example, choose the directory name `ros2_ws`, for "development workspace" :

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   mkdir visioscan
   ```

2. Copy & Paste project files into `/ros2_ws/src/visioscan` directory
   

### Parameters

The parameters for configuring laser scanner are listed as below. You need to change the parameters in the file `/config/params.yaml` corresponding to the configuration in the laser scanner.

| Parameter       | Description |
| --------------- | ----------- |
| frame_id        | Frame ID for LaserScan msg                                 |
| topic_id        | Topic ID for the publisher                                 |
| laser_ip        | IP address of laser scanner                                |
| laser_port      | Ethernet port number of laser scannner                     |
| scan_frequency  | Frequency of laser scanner                                 |
| laser_direction | Indicate if laser scanner is normal mounted or upside down |

### Compile & install visioscan package

1. Build visioscan package
   From the root of your workspace `ros2_ws`, you can now build visioscan package using the command:

   ```bash
   cd ~/ros2_ws/
   colcon build --packages-select visioscan
   ```

2. Package environment setup
    
    ```bash
    source ./install/setup.bash
    ```

    Note: Add permanent workspace environment variables.
    It's convenient if the ROS2 environment variables are automatically added to your bash session every time a new terminal is launched:

    ```bash
    echo "source ros2_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

### Run visioscan

You can run `visioscan` node with following steps, or make a [Quick start](#quick-start)

#### Run visioscan node

   ```bash
   ros2 run visioscan visioscan_node --ros-args --params-file ~/ros2_ws/src/visioscan/config/params.yaml
   ```

#### Run RViz

You can run `RViz` package in ROS2 to visualize the point cloud. Open a new terminal and run the following command:

   ```bash
   ros2 run rviz2 rviz2
   ```
   Note: Please make sure that the frame ID and scan topic ID are corresponding to that you set in the code or the config file.


### Quick start

After compiled and installed the package, run the following command to make a quick start:

    ```bash
    ros2 launch visioscan visioscan_launch.py
    ```

This starts the `RViz` and the `visioscan `driver and you should see the cloud point output of the laser scanner.

