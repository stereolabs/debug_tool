
# 🔧 Debugging Workflows Using the ZED ROS Wrapper
This tutorial explains how to use the proposed debugging workflow with the ZED ROS package. Two main approaches are supported, depending on your debugging and evaluation needs:

## Introduction

### Workflow 1: Using SVO Files for SDK Parameter Optimization and Feature Testing

This workflow is ideal when testing new features or tuning SDK parameters using pre-recorded data.

#### **SVO Playback as Live Data:**
Stereolabs' SVO files (recorded stereo video sequences) are used as input for the ZED ROS wrapper. The SDK processes them just like live camera input, publishing topics such as depth, images, object detection results, and localization.

**Visualization:**

Output topics can be visualized using tools like RViz, just as you would with real-time data.

**SVO Controller Tool:**
Stereolabs provides a controller to manage playback of SVO files:

- Pause, resume, step forward/backward

- Adjust playback speed

- Re-run sequences with different SDK parameters (e.g., object detection models, depth settings) for performance comparison

#### **Rosbag Topic Synchronization:**

A synchronization node is available to align the replayed SVO data with other recorded data  based on timestamp and frame rate. This ensures consistent analysis across sensors.

> Note: Currently, the sync node is limited to the synchronization of one rosbag topic that is of similar frame rate as the SDK topics. (e.g. LiDAR data).

### Workflow 2: Using Standard ROS Bag Recording and Replay

This method follows the standard ROS debugging approach and is better suited for full-system evaluation and cross-topic analysis.

**Full System Capture:**
Record all relevant topics (camera, LiDAR, localization, control, etc.) into a ROS bag file during operation.

**Data Replay & Visualization:**
Replay the bag using Foxglove or RViz. You can:

- Visualize synchronized data across multiple sensors

- Pause and resume playback

- Step through data frame by frame
---

## ⚙️ **Installation**  

### **1️⃣ Install the ZED SDK and ROS 2 Wrapper**  

Ensure you have the latest [ZED SDK](https://www.stereolabs.com/en-fr/developers/release), then install the ZED ROS 2 Wrapper:  

```bash
# Create and navigate to your ROS 2 workspace
mkdir -p ~/ros2_ws/src/
# Move to the `src` folder of the ROS 2 Workspace and pull the ROS2 Wrapper
cd ~/ros2_ws/src/ 
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
``` 

> If you already have the ZED ROS2 Wrapper installed, pull from the latest `master` branch to update it.

---

### **2️⃣ Install and Build the current ROS2 Package**  

- Copy the `sync_data` repository in your ROS2 workspace src folder.
- Compile the ROS2 Workspace.

```bash
cd ~/ros2_ws/
sudo apt update

# Install dependencies
python3 -m pip install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

# Set up environment variables
echo "source $(pwd)/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## WORKFLOW 1 

### Replay data with SVO 

#### 🔴 Step 1 : Recording Data

1️⃣ **Launch the ZED ROS 2 Wrapper:**  
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
```

2️⃣  **Start recording an SVO file:**  

In a new terminal: 

```bash
ros2 service call /zed/zed_node/start_svo_rec zed_msgs/srv/StartSvoRec "{svo_filename: '/path/to/svo/file/file.svo2', compression_mode: <choose between 0 and 4>}
```
> **Note:** By default, the SVO file is saved as `zed.svo2` in the current directory. To change this, use the `svo_filename` parameter. 
> **Note:** Compression mode by default will be 0 if not assigned, which is LOSSLESS mode (no compression). SVOs in this mode can be quite heavy. We recommend H264 (compression mode 1) or H265 (compression mode 2) to have optimal sized SVOs. 
> **Alternative:** You can also record an SVO directly using **ZED Explorer**, skipping the need to run the ROS 2 Wrapper.  

3️⃣  **Stop the SVO recording:**  

```bash
  ros2 service call /zed/zed_node/stop_svo_rec std_srvs/srv/Trigger
```

#### Step 2 : Replay the SVO 

- Launch the wrapper with the recorded SVO file and RVIZ (set up rviz such that it matches the Wrapper topics you want to see replayed):  

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model> svo_path:=<path/to/svo.svo2>
```
> Note: keep in mind that data can be replayed with different parameters and modify ZED wrapper config files to compare with different parameters

- Launch the SVO replay controller node: 

```bash
ros2 run debug_tool control_svo_node
```

Keyboard keys to control the svo replay : 

- "Space" key : Pause/Resume the SVO.
- "Right Arrow" key : Move to the next frame (only when SVO is paused).
- "Left Arrow" key : Move to the previous frame (only when SVO ispaused).
- "Up Arrow" key: Increase the SVO replay rate.
- "Down Arrow" key:  Decrease the SVO replay rate.

### Sync Rosbag data with SVO 

#### **Step 1: Recording Data**  

> **Important:** SVO and ROSbag must be recorded **simultaneously** to ensure timestamps match.  

##### **🔴 Start Recording**  

1️⃣ **Launch the ZED ROS 2 Wrapper:**  

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
```

2️⃣ **Launch additional ROS 2 nodes**.  

3️⃣ **Start recording an SVO file:**  

In a new terminal: 

```bash
ros2 service call /zed/zed_node/start_svo_rec zed_msgs/srv/StartSvoRec "{svo_filename: '/path/to/svo/file/file.svo2', compression_mode: <choose between 0 and 4>}
```
> **Note:** By default, the SVO file is saved as `zed.svo2` in the current directory. To change this, use the `svo_filename` parameter. 
> **Note:** Compression mode by default will be 0 if not assigned, which is LOSSLESS mode (no compression). SVOs in this mode can be quite heavy. We recommend H264 (compression mode 1) or H265 (compression mode 2) to have optimal sized SVOs. 
> **Alternative:** You can also record an SVO directly using **ZED Explorer**, skipping the need to run the ROS 2 Wrapper.  

4️⃣ **Start recording a ROSbag:**  

In a new terminal, record the requested topics from the additional ROS2 nodes: 

```bash
ros2 bag record <topics_to_record>
```

##### **🛑 Stop Recording**  

- **Stop the SVO recording:**  
```bash
  ros2 service call /zed/zed_node/stop_svo_rec std_srvs/srv/Trigger
```

- **Stop the ROSbag recording:** *(Press CTRL+C in the terminal running ros2 bag record)*  

Your **SVO** and **ROSbag** files are now saved and ready for replay!  

---

#### **Step 2: Replaying Data**  

##### **1️⃣ Configure and launch the sync_data Node**  

The sync_data node synchronizes SVO playback with the ROSbag. It requires:  
- bag_path: The **full path** to the recorded ROSbag file  
- yaml_file: A **YAML configuration file** listing the ZED ROS2 Wrapper topics to synchronize with the ROS bag topics. 

Example **YAML file (topics.yaml)**:  
```yaml
sync_node:
  zed_wrapper_topics:
    - name: "/zed/zed_node/point_cloud/cloud_registered"
      type: "sensor_msgs/msg/PointCloud2"
    - name: "/zed/zed_node/left/image_rect_color"
      type: "sensor_msgs/msg/Image"
  rosbag_topics:
    - name: "/ldlidar_node/scan"
      type: "sensor_msgs/msg/LaserScan"
```

> **Modify the file** to include the ZED ROS2 wrapper topics you want to synchronize, as well as the rosbag topics. Only the rosbag topics mentioned are replayed from the rosbag. 

> The node currently supports ROS2 `sensor_msgs`, `std_msgs`, `geometry_msgs` and `zed_msgs`. To support additional messages types, make sure to import them in the Python `Sync Data` node. 

Launch the sync_node Node: 

```bash
ros2 run debug_tool sync_node --ros-args -p bag_path:=<path/to/rosbag.db3> -p yaml_file:=<path/to/topics.yaml>
```
---

Launch the sync_node Node in DEBUG Node : 

```bash
ros2 run debug_tool sync_node --ros-args -p bag_path:=<path/to/rosbag.db3> -p yaml_file:=<path/to/topics.yaml> --log-level DEBUG

```
---

Keyboard keys to control the svo replay : 

- "Space" key : Pause/Resume the SVO.
- "Right Arrow" key : Move to the next frame (only when SVO is paused).

##### **2️⃣ Configure and Launch the ZED ROS 2 Wrapper for SVO Playback**  

Modify the **common_stereo.yaml** configuration file:  
- ✅ Set `use_svo_timestamps` to **true**  
- ✅ Set `svo_realtime` to **false**  
- ✅ Set `svo.replay_rate` to **1.0**   
- ✅ Adjust any other camera parameters as needed  

Then, launch the wrapper with the recorded SVO file:  

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model> svo_path:=<path/to/svo.svo2>
```
---

##### ** 3️⃣ Visualize the Synchronized Data**  

The sync_data node republishes topics using this format:  
`/sync/<original_topic_name>`.

For example, `/zed/zed_node/point_cloud/cloud_registered` becomes:  
`/sync/zed/zed_node/point_cloud/cloud_registered`

Use **RViz** or other visualization tools to inspect the synchronized data.  

## WORKFLOW 2

### Installation

We recommend the mcap rosbag plugin because it can easily replayed in Foxglove. Install the plugin : 

```bash
sudo apt install ros-ROS_VERSIOn-rosbag2-storage-mcap
```

Check documentation about this recording mode [here](https://github.com/ros2/rosbag2/tree/rolling/rosbag2_storage_mcap).

Install Foxglove by following these [instructions](https://foxglove.dev/download). 

### Steps

Launch your robotic full stack and use rosbag record tool to record the data you want. 
Add the desired topics in a text file, for example : (make sure to change the topics names based on the node namespace)

```json
/zed/zed_node/left/camera_info
/zed/zed_node/left/image_rect_color/compressed
/zed/zed_node/point_cloud/cloud_registered
/zed/zed_node/status/health
/zed/zed_node/obj_det/objects
.... + other topics
```

In a new terminal, run: 

```bash
ros2 bag record -s mcap $(< path/to/txt/file/file.txt)
```

The mcap recorded file can be direcly open in Foxglove. Modify the Foxglove panels to display the needed topics for debug. 

