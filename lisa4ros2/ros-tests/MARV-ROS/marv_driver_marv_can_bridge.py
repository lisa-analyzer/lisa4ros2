#!/usr/bin/env python3
''' 
# ------------------------------------- #
# MARV CAN Bridge node            #
# By Viktor Lindstrom and               #
# Noel Danielsson, Spring 2021          #
# Chalmers University of Technology     #
# ------------------------------------- #
# The MARV CAN Bridge acts as a bridge between the low level system (can messages) and high level system (ros messages).
# It uses the marv.dbc file to encode and decode the messages in order to get the right format and arbitration id.
# Each message/topic that should be bridged needs to be specified in this node. 
# Messages from ROS to CAN are ROS Subscriptions that calls callbacks, encodes and sends the CAN message. See INCOMMING ROS MESSAGES -> OUTGOING CAN MESSAGES
# Messages from CAN to ROS are decoded and then passed through "incomming_CAN_message_callback" to a ROS Publish. See INCOMMING CAN MESSAGES -> OUTGOING ROS MESSAGES
'''

# System imports
import os
import time
import can
import cantools

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Messages
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import UInt64
from std_msgs.msg import UInt8
from marv_msgs.msg import CmdSteering
from marv_msgs.msg import ScenarioConfig
from marv_msgs.msg import Notification
from marv_msgs.msg import Status
from marv_msgs.msg import Log1NCU
from marv_msgs.msg import Log1PDU
from marv_msgs.msg import Log1TCU
from marv_msgs.msg import Log2TCU

class CAN_Listener(can.Listener):
    def __init__(self, bridge_node):
        self.bridge_node = bridge_node

    def on_message_received(self,message):
        self.bridge_node.incomming_CAN_message_callback(message)

class MARV_CAN_Bridge(Node):

    ##################################### INIT #############################################################
    def __init__(self):
        # Setup parameters, variables, publishers, subscribers and timers
        super().__init__('marv_can_bridge')

        self.get_logger().info("Initializing...")

        os.chdir(os.path.dirname(__file__))
        #print(os.getcwd())
        time.sleep(1.0)
        
        # Subscribe to various topics
        self.subscription = self.create_subscription(Int8, '/marv/sys/ctrl/scenario_state', self.stateScenario_callback, 10)
        self.subscription = self.create_subscription(Int8, '/marv/sys/ctrl/sound_buzzer', self.soundBuzzerACU_callback, 10)
        self.subscription = self.create_subscription(CmdSteering, '/marv/sys/ctrl/cmd_steering', self.cmdSteeringACU_callback, 10)
        self.subscription = self.create_subscription(Bool, '/marv/sys/status/logging_state', self.loggingState_callback, 10)
        self.subscription = self.create_subscription(Int8, '/marv/sys/status/logging_marker', self.loggingMarker_callback, 10)
        self.subscription = self.create_subscription(ScenarioConfig, '/marv/sys/ctrl/scenario_config', self.scenarioConfig_callback, 100)
        self.subscription = self.create_subscription(UInt64, 'marv/sys/ctrl/scenario_sys_time', self.scenarioSysTime_callback, 10)
        self.subscription = self.create_subscription(Notification, '/marv/sys/ctrl/formatted_notification_message', self.notification_message_callback, 20)
        self.subscription = self.create_subscription(Status, '/marv/sys/status/heartbeat_acu', self.heartbeatACU_callback, 10)
        self.subscription = self.create_subscription(Int8, '/marv/nav/sbg_ekf_status', self.sbg_ekf_status_callback ,10)
        self.subscription = self.create_subscription(Int8, '/marv/nav/sbg_ins_status', self.sbg_ins_status_callabck ,10)
        self.subscription = self.create_subscription(Int8, '/marv/sys/status/radar_state', self.radarState_callback ,10)
        self.subscription
        
        # Publishes the power management topic
        self.marv_sys_ctrl_acu_power_publisher_ = self.create_publisher(Int8, '/marv/sys/ctrl/acu_power', 10)
        # Publishes the log start or stop command
        self.marv_sys_ctrl_log_data_publisher_ = self.create_publisher(Bool, '/marv/sys/ctrl/log_data', 10)
        # Publishes the scenario configuration state
        self.marv_sys_status_scenario_configuration_publisher_ = self.create_publisher(Bool, '/marv/sys/ctrl/scenario_config_state', 10)
        # Publishes the 12V auto state
        self.marv_sys_status_12V_Auto_publisher_ = self.create_publisher(Bool, '/marv/sys/status/state_12V_auto', 10)
        # Publishes start requests
        self.marv_sys_status_start_scenario_publisher_ = self.create_publisher(UInt8, '/marv/sys/status/start_scenario', 10)
        # Publishes scenario handler reset command
        self.marv_sys_reset_scenario_handler_publisher_ = self.create_publisher(Bool, '/marv/sys/ctrl/reset_scenario_handler', 10)
        # Heartbeat PDU
        self.marv_sys_status_heartbeat_pdu_publisher_ = self.create_publisher(Status, '/marv/sys/status/heartbeat_pdu', 10)
        # Heartbeat RCU
        self.marv_sys_status_heartbeat_rcu_publisher_ = self.create_publisher(Status, '/marv/sys/status/heartbeat_rcu', 10)
        # Heartbeat TCU
        self.marv_sys_status_heartbeat_tcu_publisher_ = self.create_publisher(Status, '/marv/sys/status/heartbeat_tcu', 10)
        # Heartbeat NCU
        self.marv_sys_status_heartbeat_ncu_publisher_ = self.create_publisher(Status, '/marv/sys/status/heartbeat_ncu', 10)
        # Heartbeat UCU
        self.marv_sys_status_heartbeat_ucu_publisher_ = self.create_publisher(Status, '/marv/sys/status/heartbeat_ucu', 10)
        # Heartbeat OCU
        self.marv_sys_status_heartbeat_ocu_publisher_ = self.create_publisher(Status, '/marv/sys/status/heartbeat_ocu', 10)
        # Log1NCU
        self.marv_sys_log_1NCU_publisher_ = self.create_publisher(Log1NCU, '/marv/sys/log/log1_ncu', 10)
        # Log1PDU
        self.marv_sys_log_1PDU_publisher_ = self.create_publisher(Log1PDU, '/marv/sys/log/log1_pdu', 10)
        # Log1TCU
        self.marv_sys_log_1TCU_publisher_ = self.create_publisher(Log1TCU, '/marv/sys/log/log1_tcu', 10)
        # Log2TCU
        self.marv_sys_log_2TCU_publisher_ = self.create_publisher(Log2TCU, '/marv/sys/log/log2_tcu', 10)
        # Publishes the radar enable/disable command
        self.marv_sys_radar_enable_publisher_ = self.create_publisher(Bool, '/marv/sys/ctrl/radar_enable', 10)

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)

        # Debug ignore list of IDs
        self.ignore_list = [-1]
        self.get_logger().info("Debug message IDs in the ignore list: " + str(self.ignore_list))

        # CAN
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
        #self.db = cantools.database.load_file('../../../../share/marv_driver/marv.dbc')
        self.db = cantools.database.load_file('../resource/marv.dbc')
        # List of all IDs in the can database
        self.frame_id_list = [msg.frame_id for msg in self.db.messages]
        self.get_logger().info("Message IDs in the data base: " + str(self.frame_id_list))
        # Set up listener
        self.a_listener = CAN_Listener(self)
        notifier = can.Notifier(self.bus, [self.a_listener]) 

        self.get_logger().info("Running...")

    ########################################################################################################

    ##################################### INCOMMING CAN MESSAGES -> OUTGOING ROS MESSAGES ##################

    def incomming_CAN_message_callback(self,message):
        if message.arbitration_id in self.frame_id_list and not message.arbitration_id in self.ignore_list:
            decoded_message = self.db.decode_message(message.arbitration_id, message.data)
            print
            # ACU Power
            if message.arbitration_id == self.db.get_message_by_name("powerACU").frame_id:
                #print("here:" + str(message.data))
                #print(decoded_message['powerACU_option'])
                if decoded_message['powerACU_option'] == 'reboot': # Reboot
                    ros_msg = Int8()
                    ros_msg.data = 1 # Reboot
                    self.marv_sys_ctrl_acu_power_publisher_.publish(ros_msg)
                    self.get_logger().info("powerACU_option: reboot")
                elif decoded_message['powerACU_option'] == 'shutdown': # Shutdown
                    ros_msg = Int8()
                    ros_msg.data = 2 # Shutdown
                    self.marv_sys_ctrl_acu_power_publisher_.publish(ros_msg)
                    self.get_logger().info("powerACU_option:shutdown")

            # Log Data
            elif message.arbitration_id == self.db.get_message_by_name("logData").frame_id:
                if decoded_message['logData_option'] == 'true': # Start logging data
                    ros_msg = Bool()
                    ros_msg.data = True
                    self.marv_sys_ctrl_log_data_publisher_.publish(ros_msg)
                    self.get_logger().info("logData_option: true")
                elif decoded_message['logData_option'] == 'false': # Start logging data
                    ros_msg = Bool()
                    ros_msg.data = False
                    self.marv_sys_ctrl_log_data_publisher_.publish(ros_msg)
                    self.get_logger().info("logData_option: false")
            
            # State 12V Auto
            elif message.arbitration_id == self.db.get_message_by_name("status12V_Auto").frame_id:
                if decoded_message['status12V_Auto_state'] == 'on': # 12V Auto on
                    ros_msg = Bool()
                    ros_msg.data = True
                    self.marv_sys_status_12V_Auto_publisher_.publish(ros_msg)
                    #self.get_logger().info("status12V_Auto_state: on")
                elif decoded_message['status12V_Auto_state'] == 'off': # 12V Auto off
                    ros_msg = Bool()
                    ros_msg.data = False
                    self.marv_sys_status_12V_Auto_publisher_.publish(ros_msg)
                    #self.get_logger().info("status12V_Auto_state: off")

            # State 12V Auto temp!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            elif message.arbitration_id == self.db.get_message_by_name("status12V_Auto_temp").frame_id:
                if decoded_message['status12V_Auto_temp_state'] == 'on': # 12V Auto on
                    ros_msg = Bool()
                    ros_msg.data = True
                    self.marv_sys_status_12V_Auto_publisher_.publish(ros_msg)
                    #self.get_logger().info("status12V_Auto_state: on")
                elif decoded_message['status12V_Auto_temp_state'] == 'off': # 12V Auto off
                    ros_msg = Bool()
                    ros_msg.data = False
                    self.marv_sys_status_12V_Auto_publisher_.publish(ros_msg)
                    #self.get_logger().info("status12V_Auto_state: off")

            # Start Scenario
            elif message.arbitration_id == self.db.get_message_by_name("startScenario").frame_id:
                ros_msg = UInt8()
                ros_msg.data = decoded_message['startScenario_ID']
                self.marv_sys_status_start_scenario_publisher_.publish(ros_msg)
                self.get_logger().info("startScenario_ID: " + str(ros_msg.data))

            # Reset Scenario Handler
            elif message.arbitration_id == self.db.get_message_by_name("resetScenarioHandler").frame_id:
                ros_msg = Bool()
                ros_msg.data = decoded_message['resetScenarioHandler_command']
                self.marv_sys_reset_scenario_handler_publisher_.publish(ros_msg)
                self.get_logger().info("resetScenarioHandler_command: " + str(ros_msg.data))

            # HeartbeatPDU
            elif message.arbitration_id == self.db.get_message_by_name("heartbeatPDU").frame_id:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data, decode_choices=False)  
                ros_msg = Status()
                ros_msg.status = int(decoded_message['heartbeatPDU_Status'])
                ros_msg.error_code = int(decoded_message['heartbeatPDU_Error_code'])
                ros_msg.source = int(decoded_message['heartbeatPDU_Source'])
                self.marv_sys_status_heartbeat_pdu_publisher_.publish(ros_msg)
                #self.get_logger().info("heartbeatPDU: " + str(ros_msg.status))

            # HeartbeatRCU
            elif message.arbitration_id == self.db.get_message_by_name("heartbeatRCU").frame_id:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data, decode_choices=False)
                ros_msg = Status()
                ros_msg.status = int(decoded_message['heartbeatRCU_Status'])
                self.marv_sys_status_heartbeat_rcu_publisher_.publish(ros_msg)
                #self.get_logger().info("heartbeatRDU: " + str(ros_msg.status))

            # HeartbeatTCU
            elif message.arbitration_id == self.db.get_message_by_name("heartbeatTCU").frame_id:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data, decode_choices=False)
                ros_msg = Status()
                ros_msg.status = int(decoded_message['heartbeatTCU_Status'])
                ros_msg.error_code = int(decoded_message['heartbeatTCU_Error_code'])
                ros_msg.source = int(decoded_message['heartbeatTCU_Source'])
                self.marv_sys_status_heartbeat_tcu_publisher_.publish(ros_msg)
                #self.get_logger().info("heartbeatTCU: " + str(ros_msg.status))

            # HeartbeatNCU
            elif message.arbitration_id == self.db.get_message_by_name("heartbeatNCU").frame_id:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data, decode_choices=False)
                ros_msg = Status()
                ros_msg.status = int(decoded_message['heartbeatNCU_Status'])
                ros_msg.error_code = int(decoded_message['heartbeatNCU_Error_code'])
                ros_msg.source = int(decoded_message['heartbeatNCU_Source'])
                self.marv_sys_status_heartbeat_ncu_publisher_.publish(ros_msg)
                #self.get_logger().info("heartbeatNCU: " + str(ros_msg.status))

            # HeartbeatUCU
            elif message.arbitration_id == self.db.get_message_by_name("heartbeatUCU").frame_id:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data, decode_choices=False)
                ros_msg = Status()
                ros_msg.status = int(decoded_message['heartbeatUCU_Status'])
                ros_msg.error_code = int(decoded_message['heartbeatUCU_Error_code'])
                self.marv_sys_status_heartbeat_ucu_publisher_.publish(ros_msg)
                #self.get_logger().info("heartbeatUCU: " + str(ros_msg.status))

            # HeartbeatOCU
            elif message.arbitration_id == self.db.get_message_by_name("heartbeatOCU").frame_id:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data, decode_choices=False)
                ros_msg = Status()
                ros_msg.status = int(decoded_message['heartbeatOCU_Status'])
                ros_msg.error_code = int(decoded_message['heartbeatOCU_Error_code'])
                self.marv_sys_status_heartbeat_ocu_publisher_.publish(ros_msg)
                #self.get_logger().info("heartbeatOCU: " + str(ros_msg.status))

            # Log1NCU
            elif message.arbitration_id == self.db.get_message_by_name("log1NCU").frame_id:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data, decode_choices=False)
                ros_msg = Log1NCU()
                ros_msg.angle = float(decoded_message['log1NCU_Angle'])
                ros_msg.angle_ref = float(decoded_message['log1NCU_AngleRef'])
                ros_msg.filt_vel = float(decoded_message['log1NCU_FiltVel'])
                ros_msg.drv_vel = float(decoded_message['log1NCU_DrvVel'])
                self.marv_sys_log_1NCU_publisher_.publish(ros_msg)

            # Log1PDU
            elif message.arbitration_id == self.db.get_message_by_name("log1PDU").frame_id:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data, decode_choices=False)
                ros_msg = Log1PDU()
                ros_msg.db_ext_current = float(decoded_message['log1PDU_DbExtCurrent'])
                ros_msg.db_int_current = float(decoded_message['log1PDU_DbIntCurrent'])
                ros_msg.acu_current = float(decoded_message['log1PDU_AcuCurrent'])
                ros_msg.db_int_auto_current = float(decoded_message['log1PDU_DbIntAutoCurrent'])
                ros_msg.db_ext_auto_current = float(decoded_message['log1PDU_DbExtAutoCurrent'])
                ros_msg.int_temp = float(decoded_message['log1PDU_IntTemp'])
                self.marv_sys_log_1PDU_publisher_.publish(ros_msg)

            # Log1TCU
            elif message.arbitration_id == self.db.get_message_by_name("log1TCU").frame_id:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data, decode_choices=False)
                ros_msg = Log1TCU()
                ros_msg.aps_raw_out = float(decoded_message['log1TCU_ApsRawOut'])
                ros_msg.rps_raw_out = float(decoded_message['log1TCU_RpsRawOut'])
                ros_msg.fc_raw_out = float(decoded_message['log1TCU_FcRawOut'])
                ros_msg.aps_out = float(decoded_message['log1TCU_ApsOut'])
                ros_msg.rps_out = float(decoded_message['log1TCU_RpsOut'])
                ros_msg.fc_out = float(decoded_message['log1TCU_FcOut'])
                ros_msg.status = bool(decoded_message['log1TCU_Status'])
                self.marv_sys_log_1TCU_publisher_.publish(ros_msg)

            # Log2TCU
            elif message.arbitration_id == self.db.get_message_by_name("log2TCU").frame_id:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data, decode_choices=False)
                ros_msg = Log2TCU()
                ros_msg.aps_raw_in = float(decoded_message['log2TCU_ApsRawIn'])
                ros_msg.rps_raw_in = float(decoded_message['log2TCU_RpsRawIn'])
                ros_msg.fc_raw_in = float(decoded_message['log2TCU_FcRawIn'])
                ros_msg.aps_in = float(decoded_message['log2TCU_ApsIn'])
                ros_msg.rps_in = float(decoded_message['log2TCU_RpsIn'])
                ros_msg.fc_in = float(decoded_message['log2TCU_FcIn'])
                ros_msg.status = bool(decoded_message['log2TCU_Status'])
                self.marv_sys_log_2TCU_publisher_.publish(ros_msg)

            # Publishes the radar enable/disable command
            elif message.arbitration_id == self.db.get_message_by_name("radarControl").frame_id:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data, decode_choices=False)
                ros_msg = Bool()
                ros_msg.data = bool(decoded_message['radarControl_state'])
                self.marv_sys_radar_enable_publisher_.publish(ros_msg)

            '''
            # Scenario Config State
            elif message.arbitration_id == self.db.get_message_by_name("scenarioConfigState").frame_id:
                if decoded_message['scenarioConfigState_option'] == 'true': # The scenario configuration is set
                    ros_msg = Bool()
                    ros_msg.data = True
                    self.marv_sys_ctrl_log_data_publisher_.publish(ros_msg)
                    print("scenarioConfigState_option: true")
                elif decoded_message['scenarioConfigState_option'] == 'false': # The scenario configuration is not set
                    ros_msg = Bool()
                    ros_msg.data = False
                    self.marv_sys_ctrl_log_data_publisher_.publish(ros_msg)
                    print("scenarioConfigState_option: false")'''

            # TCU Throttle status
            #if message.arbitration_id == self.db.get_message_by_name("throttleStatus").frame_id:
            #if message.arbitration_id == self.db.get_message_by_name("steeringStatus").frame_id:

    ########################################################################################################

    ##################################### INCOMMING ROS MESSAGES -> OUTGOING CAN MESSAGES ##################

    def stateScenario_callback(self,ros_message):
        can_message = self.db.get_message_by_name('stateScenario')
        encoded_message = can_message.encode({'stateScenario_state': ros_message.data})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    def soundBuzzerACU_callback(self,ros_message):
        can_message = self.db.get_message_by_name('soundBuzzerACU')
        encoded_message = can_message.encode({'soundBuzzerACU_Signal': ros_message.data})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    def cmdSteeringACU_callback(self,ros_message):
        can_message = self.db.get_message_by_name('cmdSteeringACU')
        encoded_message = can_message.encode({'cmdSteeringACU_APS': ros_message.aps, 'cmdSteeringACU_RPS': ros_message.rps, 'cmdSteeringACU_Angle': ros_message.angle})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    def loggingState_callback(self,ros_message):
        can_message = self.db.get_message_by_name('loggingState')
        encoded_message = can_message.encode({'loggingState_state': ros_message.data})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    def loggingMarker_callback(self,ros_message):
        can_message = self.db.get_message_by_name('loggingMarker')
        encoded_message = can_message.encode({'loggingMarker_value': ros_message.data})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    def scenarioSysTime_callback(self,ros_message):
        can_message = self.db.get_message_by_name('scenarioSystemTime')
        encoded_message = can_message.encode({'scenarioSystemTime_value': ros_message.data})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    def scenarioConfig_callback(self,ros_message):
        can_message = self.db.get_message_by_name('scenarioConfig')
        encoded_message = can_message.encode({'scenarioConfig_scenario_id': ros_message.scenario_id,'scenarioConfig_location': ros_message.location, 'scenarioConfig_data_char0': ros_message.data[0].item(), 'scenarioConfig_data_char1': ros_message.data[1].item(), 'scenarioConfig_data_char2': ros_message.data[2].item(), 'scenarioConfig_data_char3': ros_message.data[3].item(), 'scenarioConfig_data_char4': ros_message.data[4].item(), 'scenarioConfig_data_char5': ros_message.data[5].item(), 'scenarioConfig_data_char6': ros_message.data[6].item()})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    def notification_message_callback(self,ros_message):
        can_message = self.db.get_message_by_name('notificationMessage')
        encoded_message = can_message.encode({'notificationMessage_location': ros_message.location, 'notificationMessage_data_char0': ros_message.data[0].item(), 'notificationMessage_data_char1': ros_message.data[1].item(), 'notificationMessage_data_char2': ros_message.data[2].item(), 'notificationMessage_data_char3': ros_message.data[3].item(), 'notificationMessage_data_char4': ros_message.data[4].item(), 'notificationMessage_data_char5': ros_message.data[5].item(), 'notificationMessage_data_char6': ros_message.data[6].item()})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    def heartbeatACU_callback(self,ros_message):
        can_message = self.db.get_message_by_name('heartbeatACU')
        encoded_message = can_message.encode({'heartbeatACU_Status': ros_message.status, 'heartbeatACU_Error_code': ros_message.error_code})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    def sbg_ekf_status_callback(self,ros_message):
        can_message = self.db.get_message_by_name('ekfStatus')
        encoded_message = can_message.encode({'ekfStatus_Value': ros_message.data})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    def sbg_ins_status_callabck(self,ros_message):
        can_message = self.db.get_message_by_name('insStatus')
        encoded_message = can_message.encode({'insStatus_Value': ros_message.data})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    def radarState_callback(self,ros_message):
        can_message = self.db.get_message_by_name('radarState')
        encoded_message = can_message.encode({'radarState_state': ros_message.data})
        self.bus.send(can.Message(arbitration_id=can_message.frame_id, data=encoded_message, is_extended_id=False))

    ########################################################################################################

    ##################################### OTHER FUNCTIONS ##################################################

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    marv_can_bridge = MARV_CAN_Bridge()

    rclpy.spin(marv_can_bridge)

    marv_can_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
