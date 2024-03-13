#!/usr/bin/env -S HOME=${HOME} ${HOME}/.virtualenvs/cv/bin/python
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time


class Dynamixel:
    def __init__(self, com, baud):
        self.str_comport = com
        self.str_baudrate = baud
    def connect(self):
        self.serialDevice = serial.Serial(port = self.str_comport, baudrate = self.str_baudrate, timeout=0)

    def setReadMotorPacket(self,deviceID,Offset,Length):
        readPacket = [0xFF, 0xFF, deviceID, 0x04, 0x02, Offset, Length]
        checkSumOrdList = readPacket[2:]
        checkSumOrdListSum = sum(checkSumOrdList)
        computedCheckSum = ( ~(checkSumOrdListSum%256) ) % 256
        readPacket.append(computedCheckSum)
        self.serialDevice.write(readPacket)
        #print(readPacket)

    def getMotorQueryResponse( self, deviceID, Length ):

        queryData = 0
        responsePacketSize = Length + 6
        # responsePacket = readAllData(serialDevice)
        responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())

        if len(responsePacket) == responsePacketSize:

            # print("responsePacket=", responsePacket)

            responseID = responsePacket[2]
            errorByte = responsePacket[4]

            ### python 3
            if responseID == deviceID and errorByte == 0:
                if Length == 2:
                    queryData = responsePacket[5] + 256 * responsePacket[6]
                elif Length == 1:
                    queryData = responsePacket[5]
                    # print "Return data:", queryData
            else:
                print("Error response:", responseID, errorByte)

            responsePacketStatue = True

        else:
            responsePacketStatue = False

        # print("queryData=", queryData)
        return queryData, responsePacketStatue
    def get(self,deviceID, address, Length):

            for i in range(0,5):
                self.setReadMotorPacket(deviceID, address, Length)
                time.sleep(0.02)
                data, status = self.getMotorQueryResponse(deviceID, Length)

                if status == True:
                    break
                else:
                    print("motor ID " + str(deviceID) + "  no response " + str(i))

            return data

    def getMotorPosition(self,id):
            data = self.get(id,36,2)
            return data
    def rxPacketConversion( self,value ):
            if value < 1024 and value >= 0:
                    hiByte = int(value/256)
                    loByte = value%256
            else:
                    print("rxPacketConversion: value out of range", value)
            return loByte, hiByte
    def setMotorSpeed(self, deviceID, speed):
        (speedL, speedH) = self.rxPacketConversion(speed)
        Packet = [0xFF, 0xFF, deviceID, 0x05, 0x03, 0x20, speedL, speedH]
        checkSumOrdList = Packet[2:]
        checkSumOrdListSum = sum(checkSumOrdList)
        computedCheckSum = (~(checkSumOrdListSum % 256)) % 256
        Packet.append(computedCheckSum)
        self.serialDevice.write(Packet)

motor_id = 1
baudrate = 57600
device_name = "/dev/ttyUSB0"
speed = 3

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        self.publisher_ = self.create_publisher(Float32, '/chessboard/encoder', 10)
        timer_period = 1 / 60  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.motor = Dynamixel(device_name, baudrate)
        self.motor.connect()
        self.motor.setMotorSpeed(motor_id, speed)
    def timer_callback(self):
        pos = self.motor.getMotorPosition(motor_id)
        pos = float(pos) / 4096 # Scale to (0, 1)
        f = Float32()
        f.data = pos * 2 * math.pi
        self.publisher_.publish(f)  # Publish encoder

def main():
    rclpy.init()
    encoder_service = EncoderPublisher()
    rclpy.spin(encoder_service)
    encoder_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()