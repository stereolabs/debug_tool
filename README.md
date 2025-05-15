<img src=".assets/stereolabs-icon.png" width="150">

# 🎬 ZED ROS 2 Data Replay Tools: Debug and Optimize Your Robotics Applications with SVO/Rosbag

This tutorial provides a comprehensive guide to using the **ZED ROS 2 wrapper** to record, replay, and synchronize sensor data offline using various formats like **SVO** and **rosbag**. These tools are essential for developers working with the ZED stereo cameras in a ROS 2 environment, particularly when building, testing, or debugging complex robotics applications. They allow you to replicate scenarios offline, without needing live hardware or real-time replay. This leads to faster iteration, reproducibility, and deeper insights into system behavior.

🔁 **Tool for Replaying SVOs**

`svo_control_node`: Enables precise control over the playback of SVO files using the ZED ROS 2 wrapper. Features include pausing/resuming, stepping forward/backward, and adjusting playback speed.

🔄 **Tools for Synchronizing SVO and ROSBags**

`sync_node`: Allows synchronized playback between SVO files and external navigation modules topics provided from a rosbag. Features include pausing/resuming, stepping forward on the data while maintaining synchronization.

`SVO to Rosbag Conversion Workflow`: Offers a method to convert ZED-recorded .svo files into standard ROS 2 bag files and merge it with another rosbag containaing external navigation modules topics. 

## ⚙️ **Installation**  

### **1️⃣ Install the ZED SDK and ROS 2 Wrapper**  

-  Install the latest [ZED SDK](https://www.stereolabs.com/en-fr/developers/release) version.
-  Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
-  Install the ZED ROS 2 Wrapper:  

```bash
# Create and navigate to your ROS 2 workspace
mkdir -p ~/ros2_ws/src/
# Move to the `src` folder of the ROS 2 Workspace and pull the ROS2 Wrapper / interfaces / examples / replay data packages
cd ~/ros2_ws/src/ 
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
git clone https://github.com/stereolabs/zed-ros2-examples.git
git clone https://github.com/stereolabs/zed-ros2-interfaces.git
git clone https://github.com/stereolabs/ros2_replay_data
``` 

> 💡 If you already have the ZED ROS2 Wrapper installed, pull from the latest `master` branch to update it.

---
### **2️⃣ Build the current ROS2 Packages**  

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

## 🔴 Recording Data 

### Record SVO files with the ZED Ros Wrapper 

> An SVO file is a proprietary video format used by the ZED stereo camera to record:
> - Synchronized stereo video (left and right images)
> - Inertial Measurement Unit (IMU) data (if enabled)

✅ **Main advantages of the SVO**

- **Low load** during recording (file size and CPU usage). No need to collect heavy data like pointclouds and stereo images, those can be later processed from the ZED wrapper during replay.
- **SDK parameters optimization** during replay, allowing users to run the same recorded sequences multiple times while adjusting ZED SDK parameters. This makes it easy to fine-tune performance by experimenting with settings like depth mode, resolution, or tracking—without re-recording data.

🚫 **SVO Limitations**

- **Record and replay SDK data only**, users cannot record the data from external ROS navigation modules in this format. 

#### Recording Instructions

1️⃣ **Launch the ZED ROS 2 Wrapper:**  
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
```

2️⃣  **Start recording an SVO file:**  

In a new terminal: 

```bash
ros2 service call /zed/zed_node/start_svo_rec zed_msgs/srv/StartSvoRec "{svo_filename: '/path/to/svo/file/file.svo2', compression_mode: <choose between 0 and 4>}
```
> 💡 **Note:** By default, the SVO file is saved as `zed.svo2` in the current directory. To change this, use the `svo_filename` parameter.
>  Compression mode by default will be 0 if not assigned, which is the H265 LOSSY compression mode, similar to __ZED_Explorer__.
>  If you are using namespaces to run your node, adapt the command with the corresponding service name that includes the namespace.

3️⃣  **Stop the SVO recording:**  

```bash
  ros2 service call /zed/zed_node/stop_svo_rec std_srvs/srv/Trigger
```

### Record Rosbag Files

>Rosbag files are the commonly used files within the ROS framework to replay data and debug complex robotics system. They record topics from your stack modules and allow to replay them and visualize them using __RVIZ__ or __Foxglove__.

✅ **Main advantages of the Rosbag**

- **Record and replay the full robotic stack** Possibility to record all ROS topics from all navigation modules, including standard ROS messages and proprietary ones.

🚫 **Rosbag Limitations**

- **Heavy recording load**, depending on the topics selected to record (ie: sensor data like images and pointclouds are quite heavy for example).
- **Not possible to modify parameters** during the replay for optimization.

#### Recording Instructions

1️⃣ **Launch your full Robotics Stack**  

For this step, You can launch your typical ROS launcher that will include all your robotics modules topics: sensor data (Zed Wrapper, Lidars ...), perception, navigation and localization modules. 

2️⃣ **Record node topics as a Rosbag file**

Add the desired topics you wish to record in a text file and use the `ros2 bag record` tool to capture the rosbag: 

```bash
ros2 bag record -s $(< path/to/txt/file/topics_to_record.txt)
```

🔥 **Tips for recording rosbags efficiently and reduce overall recording load:**

> - Record only topics you need.
> - Split large bags with the `--max-bag-size` or `--max-bag-duration` parameters.
> - For better performances, used compressed topics for images and pointclouds when possible. The ZED ROS wrapper provides such topics.
> - Reduce frame rate for the images and pointclouds (e.g from 30 fps to 10 fps) to reduce rosbag loads.Reduce publishing rates of other topics when a fast publishing rate (>10 Hz) is not necessary.

### 🚀 Recommended Recording Workflow

To get the most out of both SVO and rosbag formats—minimizing recording overhead while maximizing replay flexibility—we recommend to simultaneously record an SVO and a rosbag:

- Launch your full navigation stack, including the ZED ROS 2 wrapper and any external navigation modules (e.g. SLAM, localization, planners).
- Record the SVO using the ZED wrapper with H.265 lossy compression (as described above) with minimal impact on system performance.
- Record a rosbag that excludes ZED SDK topics (e.g. point clouds, raw images), and instead focuses on external navigation modules to reduce bandwidth and disk usage during runtime.

## ▶️ Replaying Data

### Replay the SVO with the ZED ROS Wrapper (__svo_control_node__)

#### What to use it for ?

- Replaying specific recorded sequences in ROS with different SDK parameters to optimize zed modules (depth, positional tracking, object detection).
- Test new AI models on the recorded sequence to check for improvements (ie: object detection).
- Inspect carefully a scene to detect potential issues.
- Output topics can be reused as input of other nodes to check their behavior.
  
#### How to use it

Modify the **common_stereo.yaml** configuration file:  
- ✅ Set `use_svo_timestamps` to **true**  
- ✅ Set `svo_realtime` to **false**  
- ✅ Set `svo.replay_rate` to **1.0**   
- ✅ Adjust any other camera parameters as needed 

Then launch the node: 

```bash 
ros2 launch ros2_replay_data replay_svo_demo.launch.py namespace:=<namespace> camera_model:=<camera_model> camera_name:=<camera_name> svo_path:=<path/to/file.svos2> svo_replay_rate:=<rate> svo_replay_rate_increment:=<increment> 
```

An example would be: 

```bash 
ros2 launch ros2_replay_data replay_svo_demo.launch.py namespace:='replay_svo' camera_model:='zedx' camera_name:='zed_front' svo_path:='record.svo2' svo_replay_rate:=1.0 svo_replay_rate_increment:=0.1
```
Parameters :
- `replay_rate`: the rate at which the svo is being replayed. 
- `replay_rate_increment`: when users chose to increase/decrease the replay rate on the keyboard, the rate is modified by one increment. Rates are comprised in the range [0.1, 5.0].

#### Launcher workflow 

The launchers starts replaying the SVO with the ZED ROS Wrapper using the specified SDK module parameters. Rviz is also instantiated with the possibility to directly view the data displayed. The svo controller node uses the keyboard to trigger specific actions : 

- "Space" key : Pause/Resume the SVO.
- "Right Arrow" key : Move to the next frame (only when SVO is paused).
- "Left Arrow" key : Move to the previous frame (only when SVO ispaused).
- "Up Arrow" key: Increase the SVO replay rate by one increment.
- "Down Arrow" key:  Decrease the SVO replay rate by one increment.

Initial `replay_rate` and `replay_rate_increment` are specified in the launcher. Adapt the Rviz panel to match the relevant topics you want to see displayed. If the namespace has been changed, topic names will need to be changed too. 

#### Demo 

The package is provided by default with a demo launcher of a person walking in a room. To launch it: 

```bash 
ros2 launch ros2_replay_data replay_svo_demo.launch.py
```

You should see the following RVIZ panel displayed on screen:

<img src=".assets/replay_svo.gif">

The example displays the current SVO RBG image, depth image and pointcloud as well as a bounding box for obstacle detection. The SVO can be paused by pressing 'space' with the keyboard. Then, users can explore it frame by frame or rewind on previous frames to check the data output (left/right arrows). The replay rate can be increased/decreased (up/down) arrow. Users can adapt the demo launcher, example and rviz panels to fit their usecase and the parameters/topics they wish to see replayed and displayed from the ZED wrapper.

### Synchronize the SVO with Rosbag topics

#### What is it used for ? 

- Replaying specific recorded sequences with different SDK parameters to optimize modules (depth, positional tracking, object detection) **synchronized with external recorded data** (e.g other sensors, other robotics module messages,  proprietary ros messages...)
- Test new AI models on the recorded sequence to check for improvements
- Inspect carefully a scene to detect potential issues.
- Topics can be reused as input of other nodes to check their behavior.

#### How to use it ? 

1️⃣  Record an Svo and rosbag simultaneously as per our recommended recording instructions (very important that they be recorded with the same timestamps to be able to synchronize).

2️⃣ Configure the yaml file to add the topics you want to be synchronized: 
 
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

3️⃣ Configure and Launch the ZED ROS 2 Wrapper for SVO Playback

Modify the **common_stereo.yaml** configuration file:  
- ✅ Set `use_svo_timestamps` to **true**  
- ✅ Set `svo_realtime` to **false**  
- ✅ Set `svo.replay_rate` to **1.0**   
- ✅ Adjust any other camera parameters as needed  

Then launch the demo launcher : 
```bash 
ros2 launch ros2_replay_data sync_node_demo.launch.py namespace:=<namespace> camera_model:=<camera_model> camera_name:=<camera_name> bag_path:=<path/to/rosbag_folder> svo_path:=<path/to/file.svos2> synchronized_topic_config_file:=<path/to/file.yaml> sync_queue_size:=<size> sync_slop:=<slop>
```

An example would be: 

```bash 
ros2 launch ros2_replay_data sync_node_demo.launch.py namespace:='sync_node' camera_model:='zedx' camera_name:='zed_front' bag_path:='rosbag_record/' svo_path:='record.svo2' synchronized_topic_config_file:='topics.yaml' sync_queue_size:=2000 sync_slop:=0.3 
```

> A Rosbag is generally composed of a folder containing a metadata.yaml file and the topics databse (.db3) file. The node takes as input the folder path only, allowing to replay the topics using the metadata configurations (particularly the topics QoS profiles).
> At the moment, rosbag recorded with the `mcap` compression mode are not supported by the sync Node.

Parameters :
- `sync_queue_size`: The number of messages to store for each input topic. Helps the synchronizer match messages by keeping recent history. Larger queue_size = more chance to find a match, but uses more memory.
- `slop`: The maximum time difference (in seconds) allowed between messages to be considered "synchronized." Use larger slop when sensors are not hardware-synchronized or when network delays are involved (0.2/0.3 are quite common).

#### Launcher workflow 

The launchers starts replaying the SVO with the ZED ROS Wrapper using the specified SDK module parameters, as well as the specified rosbag topics. Rviz is also instantiated with the possibility to directly view the data displayed. The keyboard can be used to trigger some actions : 
 
Keyboard keys to control the svo replay : 

- "Space" key : Pause/Resume the SVO.
- "Right Arrow" key : Move to the next frame (only when SVO is paused).

>Adapt the Rviz panel to match the relevant topics you want to see displayed. If the namespace has been changed, topic names will need to be changed too. 
> To connect data from external sensors publishing in other frames, you might need to set up a static transform between the camera frame and the sensor frame to see it synchronized in rviz. 
> The `sync_node` republishes topics using this format:  
`/sync/<original_topic_name>`.

For example, `/zed/zed_node/point_cloud/cloud_registered` becomes:  `/sync/zed/zed_node/point_cloud/cloud_registered`

#### Demo 

The package is provided by default with a demo launcher of a person walking in a room. To launch it : 

```bash 
ros2 launch ros2_replay_data sync_node_demo.launch.py 
```

<img src=".assets/sync_svo_rosbag.gif">

The example displays the current camera pointcloud extracted from the SVO file. The rosbag contains the data from an external sensor source (Lidar) that is shown synchronized with the camera. The SVO can be paused by pressing 'space' with the keyboard. Then, users can explore it messages by messages (right arrows). Users can adapt the demo launcher, example and rviz panels to fit their usecase and the parameters/topics they wish to see replayed and displayed from the ZED wrapper.

#### 🚫 Limitations

⚠️ This synchronization workflow works best with a limited number of topics that share similar time frames. It is ideal for quick optimization of ZED SDK parameters in combination with external sensor data (e.g., IMU, odometry, localization).

For larger-scale setups involving full robotics stacks and broader topic coverage, we recommend an alternative approach:
➡️ Convert the SVO to a rosbag, then merge it with the external data rosbag that was recorded simultaneously. Check the instructions below.

### Convert SVO to Rosbag and merge it with the external data rosbag

1️⃣ Record an Svo and rosbag simultaneously as per our recommended recording instructions (very important that they be recorded with the same timestamps to be able to synchronize).

2️⃣ Configure and Launch the ZED ROS 2 Wrapper for SVO Playback

- ✅ Set `use_svo_timestamps` to **true**  
- ✅ Set `svo_realtime` to **false**  
- ✅ Set `svo.publish_svo_clock` to **true** (allow to use the wrapper as new /clock simulated time for ROS)   
- ✅ Adjust any other camera parameters as needed

Replay the Svo with the ZED wrapper: 

```bash 
ros2 launch zed_wrapper zed_camera.launch.py namespace:=<namespace> camera_model:=<camera_model> camera_name:=<camera_name> svo_path:=<path/to/file.svos2> publish_svo_clock:=true 
```

3️⃣ Create a rosbag file that records the Zed SDK topics. Open a new terminal: 

```bash
ros2 bag record -s $(< path/to/txt/file/topics_to_record.txt) --use-sim-time
```
> Stop the rosbag record when the SVO is done replaying.

4️⃣  Merge the newly created rosbag with the rosbag containing external navigation modules topics

```bash
ros2 bag convert -i /path/to/rosbag1 -i /path/to/rosbag2 -o out.yaml
```
with `out.yaml` being: 

```yaml
output_bags:
- uri: merged_bag
  all_topics: true
  all_services: true
```
5️⃣ Replay the merged bag 

```bash
ros2 bag play merged_bag
```
Use __Rviz__ or __Foxglove__ for data visualization. Both rosbags data should be replaying with full synchronization. If you wish to replay the same sequence with different SDK parameters, you need to create a new Rosbag from the SVO and merge it with the rosbag containing the external navigation modules data.







