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

class SVOController(Node):
    def __init__(self):
        super().__init__('control_svo')

        ## This sync node aims at synchronizing topics taken from a svo file and a rosbag. Users set up the topics they wish to sync together in a yaml file. 
        ## The rosbag is replayed directly from the node. The SVO is replayed in the ZED ROS wrapper with any camera parameters. Synchronized topics are published by the sync node. 
        ## SVOs are replayed in non realtime, and the replay rate in the wrapper is adjusted 
        ## At any time, users can press the keyboard "space" key to pause the replay of SVO/ ROS BAG and use the right arrow key to advance frame by frame while keeping the rosbag and svo topics synchronized.

        ################################### CLASS PARAMETERS #########################################
        
        ############################################################################################

        ################################### SVO REPLAY #############################################


        #Listen to SVO status message 
        self.svo_status_sub = self.create_subscription(SvoStatus,'/zed/zed_node/status/svo',self.svo_callback,10)
        self.svo_current_frame = 0
        self.svo_current_timestamp = 0.0
        self.rate = 1.0
        self.rate_increment = 0.5
        self.paused = False
        
        #Service to modify SVO replay rate
        self.cli = self.create_client(SetParameters, '/zed/zed_node/set_parameters')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /zed/zed_node/set_parameters service...')

        #Service to pause the SVO
        self.svo_pause_client = self.create_client(Trigger, '/zed/zed_node/toggle_svo_pause')
        while not self.svo_pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /zed/zed_node/toggle_svo_pause...')
        self.pause_request = Trigger.Request()

        #Service to set frame position on SVO
        self.set_svo_frame_position_client = self.create_client(SetSvoFrame, '/zed/zed_node/set_svo_frame')
        while not self.set_svo_frame_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /zed/zed_node/set_svo_frame...')
        self.frame_request = SetSvoFrame.Request()

        ############################################################################################

         
       ################################### KEYBOARD THREAD ########################################

        
        # Start a separate thread to listen for keyboard input
        self.keyboard_thread = threading.Thread(target=self.listen_for_keypress, daemon=True)
        self.keyboard_thread.start()

       ############################################################################################


    def svo_callback(self, msg):
        """Gives information about the current SVO status. The SVO timestamp is compared with the rosbag timestamp all the time to ensure they are not diverging too much. 
        If the svo is much faster than the rosbag, its replay rate is reduced by adjusting a dynamic ROS wrapper parameter. If the svo is much slower than the rosbag replay rate, its replay rate is increased.""" 
        
        self.svo_current_frame = msg.frame_id
        self.svo_current_timestamp = msg.frame_ts* 1e-9
        self.get_logger().debug(f"SVO current frame : {self.svo_current_frame}")
        self.get_logger().debug(f"SVO current timestamp : {self.svo_current_timestamp}")

    
    def listen_for_keypress(self):
        """Listen for spacebar press to toggle pause"""
        def on_press(key):
            if key == keyboard.Key.space:
                self.paused = not self.paused
                state = "Paused" if self.paused else "Resumed"
                self.get_logger().info(f"[KEYBOARD] {state}")
                ## Pause SVO 
                try:
                  self.pause_service_call = self.svo_pause_client.call_async(self.pause_request)
                  response = self.pause_service_call.result()
                  self.get_logger().debug(f"SVO Toggle pause service call")
                except Exception as e:
                  self.get_logger().error(f'Service call failed: {e}')
            elif key == keyboard.Key.left and self.paused:
                #move svo to the previous frame
                self.get_logger().info(f"[KEYBOARD] Moving to the previous frame")
                try:
                    self.frame_request.frame_id = self.svo_current_frame -1
                    self.frame_service_call = self.set_svo_frame_position_client.call_async(self.frame_request)
                    self.get_logger().info(f"SVO frame position service call")
                    self.svo_current_frame = self.svo_current_frame+1
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
            elif key == keyboard.Key.right and self.paused:
                #move svo to the next frame
                self.get_logger().info(f"[KEYBOARD] Moving to the next frame")
                try:
                    self.frame_request.frame_id = self.svo_current_frame +1
                    self.frame_service_call = self.set_svo_frame_position_client.call_async(self.frame_request)
                    self.get_logger().info(f"SVO frame position service call")
                    self.svo_current_frame = self.svo_current_frame+1
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
            elif key == keyboard.Key.up:
                #move svo to the next frame
                self.rate = self.rate + self.rate_increment
                self.rate = max(0.1, min(5, self.rate))
                self.get_logger().info(f"[KEYBOARD] Increase SVO replay rate to: {self.rate}")
                ##call set SVO replay rate
                try:
                   
                    param = Parameter(name='svo.replay_rate', value=self.rate)
                    req = SetParameters.Request()
                    req.parameters = [param.to_parameter_msg()]
                    self.rate_service_call = self.cli.call_async(req)
                    response = self.rate_service_call.result()
                    self.get_logger().debug(f"SVO rate service call")
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')

            elif key == keyboard.Key.down:
                #move svo to the next frame
                self.rate = self.rate - self.rate_increment
                self.rate = max(0.1, min(5, self.rate))
                self.get_logger().info(f"[KEYBOARD] Decrease SVO replay rate to: {self.rate}")
                ##call set SVO replay rate
                try:
                    
                    param = Parameter(name='svo.replay_rate', value=self.rate)
                    req = SetParameters.Request()
                    req.parameters = [param.to_parameter_msg()]
                    self.rate_service_call = self.cli.call_async(req)
                    response = self.rate_service_call.result()
                    self.get_logger().debug(f"SVO rate service call")
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
                

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    
def main(args=None):
    rclpy.init(args=args)
    try:
        svo_control_node = SVOController()
        rclpy.spin(svo_control_node)
    except KeyboardInterrupt:
        svo_control_node.get_logger().info("Shutting down...")
    finally:
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
