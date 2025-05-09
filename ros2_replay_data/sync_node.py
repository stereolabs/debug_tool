import rclpy
from rclpy.node import Node
import message_filters
import yaml
from sensor_msgs.msg import *
from diagnostic_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from zed_msgs.msg import *
from zed_msgs.srv import *
from std_srvs.srv import Trigger
from rosidl_runtime_py.utilities import get_message
import sys
import os
import time
import threading
from pynput import keyboard
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from time import sleep, time
from threading import Thread

class DataSynchronizer(Node):
    def __init__(self):
        super().__init__('data_synchronizer')

        ## This sync node aims at synchronizing topics taken from a svo file and a rosbag. Users set up the topics they wish to sync together in a yaml file. 
        ## The rosbag is replayed directly from the node. The SVO is replayed in the ZED ROS wrapper with any camera parameters. Synchronized topics are published by the sync node. 
        ## SVOs are replayed in non realtime, and the replay rate in the wrapper is adjusted 
        ## At any time, users can press the keyboard "space" key to pause the replay of SVO/ ROS BAG and use the right arrow key to advance frame by frame while keeping the rosbag and svo topics synchronized.

        ################################### CLASS PARAMETERS #########################################
        # Get parameters (bag path and launcher param file)
        self.bag_path = self.declare_parameter("bag_path", "").get_parameter_value().string_value
        self.yaml_file = self.declare_parameter("yaml_file", "").get_parameter_value().string_value
        self.namespace = self.declare_parameter("namespace", "").get_parameter_value().string_value

        if self.namespace == "":
            self.namespace = 'zed/zed_node'
        else:
            self.namespace = self.namespace+'/zed'

        if not self.bag_path or not self.yaml_file:
            self.get_logger().error("Missing parameters: bag_path or yaml_file")
            sys.exit(1)


        self.get_logger().info(f"Opening ROS2 bag: {self.bag_path}")
        self.get_logger().info(f"Opening yaml file: {self.yaml_file}")
        self.get_logger().info(f"Node namespace: {self.namespace}")

        #####################################TOPIC SUBSCRIBERS/PUBLISHERS#############################

        # Initialize publishers and subscribers
        self.sync_subscribers = []
        self.rosbag_topic_publishers = {}
        self.sync_topic_publishers = {}

        ############################################################################################

        ################################### SVO REPLAY #############################################

        # Load topics from ZED wrapper to synchronize (list of topics present in the yaml config file)
        self.load_wrapper_topics(self.yaml_file)

        #Listen to SVO status message 
        self.svo_status_sub = self.create_subscription(SvoStatus,'/'+self.namespace+'/status/svo',self.svo_callback,10)
        self.svo_current_frame = 0
        self.svo_current_timestamp = 0.0
        self.svo_started = False
        
        #Service to modify SVO replay rate
        self.cli = self.create_client(SetParameters, '/'+self.namespace+'/set_parameters')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for'+ '/'+self.namespace+'/set_parameters service...')

        #Service to pause the SVO
        self.svo_pause_client = self.create_client(Trigger, '/'+self.namespace+'/toggle_svo_pause')
        while not self.svo_pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service'+ '/'+self.namespace+'/toggle_svo_pause...')
        self.pause_request = Trigger.Request()

        #Service to set frame position on SVO
        self.set_svo_frame_position_client = self.create_client(SetSvoFrame, '/'+self.namespace+'/set_svo_frame')
        while not self.set_svo_frame_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service'+ '/'+self.namespace+'/set_svo_frame...')
        self.frame_request = SetSvoFrame.Request()

        # SVO Replay Rate parameters
        self.rate = 1.0
        self.error_integral = 0.0
        self.kp = 0.002  # Proportional gain
        self.ki = 0.0001  # Integral gain

        ############################################################################################


        ################################### ROSBAG REPLAY ##########################################

        # Create rosbag reader
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)
        self.rosbag_current_timestamp = 0.0 

        # Load topics from ROS Bag to synchronize (list of topics present in the yaml config file)
        self.load_rosbag_topics(self.yaml_file)

        # Get all topic types
        self.topic_types = {topic.name: topic.type for topic in self.reader.get_all_topics_and_types()}

        #Only read topics that are requested 
        topics = list(self.rosbag_topic_publishers.keys())
        self.reader.set_filter(rosbag2_py._storage.StorageFilter(topics=topics))

        # Initialize pause flag
        self.paused = False  

        # Initialize rosbag rate
        self.rosbag_rate = 1.0 
        self.reset_time = False

         # Launch rosbag playback in a background thread
        self.playback_thread = Thread(target=self.play, daemon=True)
        self.playback_thread.start()

        ############################################################################################

         ################################### DATA SYNC ###############################################
  
        # Synchronize messages with ApproximateTimeSynchronizer
        if self.sync_subscribers:
            self.sync = message_filters.ApproximateTimeSynchronizer(self.sync_subscribers, 2000, 0.02, allow_headerless=True)
            self.sync.registerCallback(self.synchronized_callback)

       ############################################################################################

       ################################### KEYBOARD THREAD ########################################

        
        # Start a separate thread to listen for keyboard input
        self.keyboard_thread = threading.Thread(target=self.listen_for_keypress, daemon=True)
        self.keyboard_thread.start()

        ############################################################################################


    def svo_callback(self, msg):
        """Gives information about the current SVO status. The SVO timestamp is compared with the rosbag timestamp all the time to ensure they are not diverging too much. 
        If the svo is much faster than the rosbag, its replay rate is reduced by adjusting a dynamic ROS wrapper parameter. If the svo is much slower than the rosbag replay rate, its replay rate is increased.""" 
 
        if self.svo_started is False:
            self.svo_started = True
            self.reset_time = True
        
        self.svo_current_frame = msg.frame_id
        self.svo_current_timestamp = msg.frame_ts* 1e-9
        self.get_logger().info(f"SVO current frame : {self.svo_current_frame}")
        self.get_logger().info(f"SVO current timestamp : {self.svo_current_timestamp}")
        self.get_logger().info(f"ROSBAG current timestamp : {self.rosbag_current_timestamp}")
        if self.paused:
            ## if the ROSbag/ SVO replyay is paused, no need to adjust the rate
            return

        if self.svo_current_timestamp == 0.0 or self.rosbag_current_timestamp == 0.0:
            return

        ## Calculating difference between both ROSbag / SVO timestamps
        ## The new rate is adjusted based on a PI controller logic whose parameters can be tuned. 
        error = self.rosbag_current_timestamp - self.svo_current_timestamp
        self.get_logger().info(f"Time difference between SVO and rosbag : {error}")
        if abs(error)<0.25:
            self.rate = 1
        adjustment = self.kp * error 
        self.rate += adjustment
        ## Cap rate between [0.1,5]
        self.rate = max(0.1, min(5, self.rate))

        param = Parameter(name='svo.replay_rate', value=self.rate)
        req = SetParameters.Request()
        req.parameters = [param.to_parameter_msg()]

        

        ## As the new rate is calculated, it is sent to the wrapper.

        ##call set SVO replay rate
        try:
          self.rate_service_call = self.cli.call_async(req)
          response = self.rate_service_call.result()
          self.get_logger().debug(f"SVO rate service call")
        except Exception as e:
          self.get_logger().error(f'Service call failed: {e}')
        


    def listen_for_keypress(self):
        """Listen for spacebar press to toggle pause"""
        def on_press(key):
            sleep(0.5)
            if key == keyboard.Key.space:
                self.paused = not self.paused
                state = "Paused" if self.paused else "Resumed"
                self.get_logger().info(f"[KEYBOARD] {state}")
                if state == "Resumed":
                     self.reset_time = True
                ## Pause SVO 
                try:
                  self.pause_service_call = self.svo_pause_client.call_async(self.pause_request)
                  response = self.pause_service_call.result()
                  self.get_logger().debug(f"SVO Toggle pause service call")
                except Exception as e:
                  self.get_logger().error(f'Service call failed: {e}')
            elif key == keyboard.Key.right:
                ## When the SVO is paused, it is possibl to advance frame by frame when the right arrow key button is pressed.
                self.advance()
                

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    def timer_callback(self):
        """Replay messages from the rosbag"""
        if not self.reader.has_next():
            self.get_logger().info("Rosbag replay complete. Shutting down node.")
            rclpy.shutdown()
            sys.exit(0)  

        while self.reader.has_next():
            if self.paused:
                return
            self.get_logger().debug(f'Replaying bag: {self.bag_path}')
            topic, data, t = self.reader.read_next()
            if topic in self.topic_types:
                try:
                    msg_type = self.topic_types[topic]
                    msg_class = get_message(msg_type)
                    msg = deserialize_message(data, msg_class)
                    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                      stamp = msg.header.stamp
                      self.rosbag_current_timestamp = stamp.sec + stamp.nanosec * 1e-9
                    if topic in self.rosbag_topic_publishers:
                        self.rosbag_topic_publishers[topic].publish(msg)

                except Exception as e:
                    self.get_logger().error(f'Error deserializing message for {topic}: {str(e)}')

            break

    def advance(self): 
        """In pause mode, advance on the rosbag message by message. As the bag timestamp increases, ensure the SVO timestamp remains synchronized by advancing on the SVO position"""
        if not self.paused:
           self.get_logger().info("Rosbag needs to be paused in order for the advance frame feature to work.")
           return
        
        if self.reader.has_next():
            topic, data, t = self.reader.read_next()
            if topic in self.topic_types:
                try:
                    msg_type = self.topic_types[topic]
                    msg_class = get_message(msg_type)
                    msg = deserialize_message(data, msg_class)
                    self.rosbag_current_timestamp = t * 1e-9

                    if topic in self.rosbag_topic_publishers:
                        self.rosbag_topic_publishers[topic].publish(msg)
                        self.get_logger().info("publishing next rosbag message")
                except Exception as e:
                    self.get_logger().error(f'Error deserializing message for {topic}: {str(e)}')
            
        if self.svo_current_timestamp < self.rosbag_current_timestamp: 
            #svo is late so advance svo
            try:
                self.frame_request.frame_id = self.svo_current_frame +1
                self.frame_service_call = self.set_svo_frame_position_client.call_async(self.frame_request)
                self.get_logger().info(f"SVO frame position service call")
                self.svo_current_frame = self.svo_current_frame+1
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')


    def synchronized_callback(self, *msgs):
        """Publish synchronized messages"""
        if not msgs:
            return

        self.get_logger().debug(f"Publishing synchronized messages at timestamp {msgs[0].header.stamp.sec}.{msgs[0].header.stamp.nanosec}")

        for topic, msg in zip(self.sync_topic_publishers.keys(), msgs):
            self.sync_topic_publishers[topic].publish(msg)


    def load_wrapper_topics(self, yaml_file):
        """ Load topics from YAML file """
        try:
            with open(yaml_file, "r") as file:
                data = yaml.safe_load(file)
                topic_list = data.get("sync_node", {}).get("zed_wrapper_topics", [])
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file {yaml_file}: {str(e)}")
            return 

        if len(topic_list) == 0:
            self.get_logger().error(f"No ZED Wrapper topics found to synchronize, exiting...")
            sys.exit(0) 
            return

        # Dynamically create subscribers and publishers
        for topic in topic_list:
            try:
                msg_type = get_message(topic["type"])
                subscriber = message_filters.Subscriber(self, msg_type, topic["name"])
                self.sync_subscribers.append(subscriber)
                publisher = self.create_publisher(msg_type, "/sync"+topic["name"], 10)
                self.sync_topic_publishers[topic["name"]] = publisher

                self.get_logger().info(f"Created publisher for {topic['name']} [{topic['type']}]")

            except Exception as e:
                self.get_logger().error(f"Failed to create publisher for {topic['name']}: {str(e)}")
        
        return

    def load_rosbag_topics(self, yaml_file):
        """ Load rosbag topics from YAML file """
        try:
            with open(yaml_file, "r") as file:
                data = yaml.safe_load(file)
                topic_list = data.get("sync_node", {}).get("rosbag_topics", [])
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file {yaml_file}: {str(e)}")
            return 

        if len(topic_list) == 0:
            self.get_logger().error(f"No ROSbag topics found to synchronize, exiting...")
            sys.exit(0)
            return 

        # Dynamically create subscribers and publishers
        for topic in topic_list:
            try:
                msg_type = get_message(topic["type"])
                subscriber = message_filters.Subscriber(self, msg_type, "/bag"+topic["name"])
                self.sync_subscribers.append(subscriber)
                publisher = self.create_publisher(msg_type, topic["name"], 10)
                self.sync_topic_publishers[topic["name"]] = publisher
                publisher = self.create_publisher(msg_type, "/bag"+topic["name"], 10)
                self.rosbag_topic_publishers[topic["name"]] = publisher
                self.get_logger().info(f"Created publisher for {topic['name']} [{topic['type']}]")

            except Exception as e:
                self.get_logger().error(f"Failed to create publisher for {topic['name']}: {str(e)}")
        
        return

    def play(self):
        first_msg_time = None
        base_time = time()

        while self.reader.has_next():
            if self.svo_started is False:
                self.get_logger().info(f'Waiting for svo replay to start...')
                continue

            if self.paused:
                continue 

            topic, data, t = self.reader.read_next()
            # Time control
            if first_msg_time is None or self.reset_time is True:
                print("reset time")
                first_msg_time = t
                base_time = time()
                self.reset_time = False
            else:
                elapsed_ros_time = (t - first_msg_time) / 1e9
                self.get_logger().info(f'elapsed ros time: {elapsed_ros_time}')
                target_wall_time = base_time + elapsed_ros_time / self.rosbag_rate
                self.get_logger().info(f'target_wall time: {target_wall_time}')
                self.get_logger().info(f'curent time: {time()}')
                sleep(max(0, target_wall_time - time()))
            #Publish data
            if topic in self.topic_types:
                try:
                    msg_type = self.topic_types[topic]
                    msg_class = get_message(msg_type)
                    msg = deserialize_message(data, msg_class)
                    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                      stamp = msg.header.stamp
                      self.rosbag_current_timestamp = stamp.sec + stamp.nanosec * 1e-9
                    if topic in self.rosbag_topic_publishers:
                        self.rosbag_topic_publishers[topic].publish(msg)

                except Exception as e:
                    self.get_logger().error(f'Error deserializing message for {topic}: {str(e)}')

        if not self.reader.has_next():
            self.get_logger().info("Rosbag replay complete. Shutting down node.")
            return

    
def main(args=None):
    rclpy.init(args=args)
    
    try:
        sync_node = DataSynchronizer()
        rclpy.spin(sync_node)
    except KeyboardInterrupt:
        sync_node.get_logger().info("Shutting down...")
    finally:
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
