#!/usr/bin/env python3
#Node sending the string to the can_tx node according to the defined sequence.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import struct
import time
import os

DLC = 8

class frame_tx(Node): # Node Initialisation

    def __init__(self):
        super().__init__("frame_tx")
        self.get_logger().info("frame_tx node running...")

        self.frame_pub = self.create_publisher(String, "/frame_topic", 10)

        os.system("clear") # To clear the terminal when running the sequence again.

        time.sleep(2)

        self.sequence()
        
    #//////////////////////////////////////////////////////////////////////////////////////////////////

    def sequence(self): # sequence definition:
        # sequence/logic
        time.sleep(0.1) #necessary, otherwise the subscriber node will miss the first published message.

        self.home(257, 1.2) # 257 for 0x101
        self.home(258, 3.4) # 258 for 0x102

        print("sequence complete.")

    #/////////////////////////////////////////////////////////////////////////////////////////////////
    
    def send_string(self, data_str): # publishing the string on the frame_topic
        frame = String()
        frame.data = data_str
        self.frame_pub.publish(frame)

    # /////////////////////////////////////////////////////////////////////////////////////////////////

    def convert(self, f): # Conversion function (Float to 8 bit unsigned int)
        byte_arr = bytearray(struct.pack("f", f))
        return byte_arr

    #/////////////////////////////////////////////////////////////////////////////////////////////////
   
    def home(self, can_id=0, pos=0): # Frame Description
        param_id = 1
        dlc=DLC

        if (pos >= 0 and pos <= 300):
            val_arr = self.convert(pos)
            print("Device ID: %s" % can_id)
            print("Parameter ID: %s" % param_id)
            print("DLC: %s" % dlc)
            print(["0x%02x" %b for b in val_arr])

            data = str()
            data = (str(can_id) + " " + str(param_id)+ " " + str(dlc) + " " + str(val_arr[0]) + " " + str(val_arr[1]) + " " + str(val_arr[2]) + " " + str(val_arr[3]) )
            print("Data string: %s\n" %data)
            self.send_string(data)
        
        else:
            print("Entered value is larger than the maximum defined value.\n")


    def position(self, can_id=0, pos=0):
        param_id = 2
        dlc=DLC

        if (pos >= 0 and pos <= 300):
            val_arr = self.convert(pos)

            print("Device ID: %s" % can_id)
            print("Parameter ID: %s" % param_id)
            print("DLC: %s" % dlc)
            print(["0x%02x" %b for b in val_arr])
            
            data = str()
            data = (str(can_id) + " " + str(param_id)+ " " + str(dlc) + " " + str(val_arr[0]) + " " + str(val_arr[1]) + " " + str(val_arr[2]) + " " + str(val_arr[3]) )
            print("Data string: %s\n" %data)
            self.send_string(data)
        
        else:
            print("Entered value is larger than the maximum defined value.\n")


    def speed(self, can_id=0, pos=0):
        param_id = 3
        dlc=DLC

        if (pos >= 0 and pos <= 300):
            val_arr = self.convert(pos)

            print("Device ID: %s" % can_id)
            print("Parameter ID: %s" % param_id)
            print("DLC: %s" % dlc)
            print(["0x%02x" %b for b in val_arr])
            
            data = str()
            data = (str(can_id) + " " + str(param_id)+ " " + str(dlc) + " " + str(val_arr[0]) + " " + str(val_arr[1]) + " " + str(val_arr[2]) + " " + str(val_arr[3]) )
            print("Data string: %s\n" %data)
            self.send_string(data)
        
        else:
            print("Entered value is larger than the maximum defined value.\n")


    def acceleration(self, can_id=0, pos=0):
        param_id = 4
        dlc=DLC

        if (pos >= 0 and pos <= 300):
            val_arr = self.convert(pos)

            print("Device ID: %s" % can_id)
            print("Parameter ID: %s" % param_id)
            print("DLC: %s" % dlc)
            print(["0x%02x" %b for b in val_arr])
            
            data = str()
            data = (str(can_id) + " " + str(param_id)+ " " + str(dlc) + " " + str(val_arr[0]) + " " + str(val_arr[1]) + " " + str(val_arr[2]) + " " + str(val_arr[3]) )
            print("Data string: %s\n" %data)
            self.send_string(data)
        
        else:
            print("Entered value is larger than the maximum defined value.\n")

#/////////////////////////////////////////////////////////////////////////////////////////////////



def main(args=None):
    rclpy.init(args=args)
    node = frame_tx()
    print("spinning the node once now.")
    #rclpy.spin_once(node)  # spin is only used when you want to keep the node alive so that it can capture all the msg received on the subscribed topic and process the callbacks.
    print("control back to the main program.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()