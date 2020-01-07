#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import os, sys
import rospy
import serial
import time
import threading
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String,Int8


class SerialPort:
    def __init__(self,port,buand):
        self.port=serial.Serial(port,buand,timeout=0.05)
        if not self.port.isOpen():
            self.port.open()
        #rospy.on_shutdown(self.shutdown)
        self.send_thread = threading.Thread(target=self.send_data)
        self.send_thread.setDaemon(True)
        self.read_thread = threading.Thread(target=self.read_data)
        self.read_thread.setDaemon(True)
        #self.__ros_pub_gyro = rospy.Publisher('gyro', Float32, queue_size=1)
        #rospy.init_node('gyro', anonymous=True)

    def port_open(self):
        if not self.port.isOpen():
            self.port.open()

    def port_close(self):
        self.port.close()

    def send_data(self):
        #while True:
            #list = 'ffff0109032a000c000041007b'
            list = 'ffff0109032afc080000410083'
            hexer = list.decode("hex")
            number = self.port.write(hexer)
            #rospy.sleep(2)
            #list = 'ffff0104023802be'
            #hexer = list.decode("hex")
            #number = self.port.write(hexer)
            #rospy.sleep(2)

            #return number
            # print("xxxxxxxxxxx")
            # print("xxxxxxxxxxx")
            # rospy.loginfo("xxxxxxxxxxx")
            # rospy.logdebug("xxxx")
            #rospy.sleep(0.005)


    def read_data(self):

        while True:
            #rospy.sleep(0.1)
            data = self.port.readline() 
            #print (data)               
            hLen = len(data)
            if len(data) == 6:
                #print("no erro")
                continue	    
            #for i in range(hLen):
                #hvol = ord(data[i])
                #print(ord(data[i])) 
	    if len(data) == 8:
                    print("no erro")
                    #continue
	            #hLen = len(data)
                    #for i in range(hLen):
                      #hvol = ord(data[i])
                      #print(ord(data[i])) 
                    weizhi=ord(data[6]) *256+ord(data[5])
                    print("位置数据：%d" % weizhi ) 
            rospy.sleep(0.01)
            #print("00000000000000000\n")
            # rospy.loginfo("00000000000000000")
            #rospy.sleep(0.1)
            #print(time.ctime())



    def read_data2(self):
        global num
        Pitch_head1 = 0
        Pitch_middle1 = 0
        Pitch_last1 = 0
        Roll_head1 = 0
        Roll_middle1 = 0
        Roll_last1 = 0
        Heading_head1 = 0
        Heading_middle1 = 0
        Heading_last1 = 0

        num = 0

        while True:

            # n=n+1
            # print(n)
            #n = self.port.inWaiting()

            #data = self.port.readline(14)
            data = self.port.readline(14)
            #data = self.port.readall

            # data = self.port.read(14)
            # self.port.flushInput()
            #self.message+=data

            # if num < 5:
            #     num = num+1
            #     continue

            if len(data) == 0:
                print("no data")
                continue
            # print(data)
            # rospy.sleep(0.1)
            # continue
            # if ord(data[0]) !='119':
            #    continue
            result = ''
            hLen = len(data)

            for i in range(hLen):
                hvol = ord(data[i])
                # print(ord(data[0]))
                hhex = '%02x' % hvol
                result += hhex + ' '
                if i == 4:
                    Pitch_head1 = hhex
                elif i == 5:
                    Pitch_middle1 = hhex
                elif i == 6:
                    Pitch_last1 = hhex
                elif i == 7:
                    Roll_head1 = hhex
                elif i == 8:
                    Roll_middle1 = hhex
                elif i == 9:
                    Roll_last1 = hhex
                elif i == 10:
                    Heading_head1 = hhex
                elif i == 11:
                    Heading_middle1 = hhex
                elif i == 12:
                    Heading_last1 = hhex
                else:
                    continue

            print(result)
            Pitch_head = int(Pitch_head1)
            Pitch_middle = int(Pitch_middle1)
            Pitch_last = int(Pitch_last1)
            #print('Pitch:', Pitch_head, Pitch_middle, Pitch_last)

            Roll_head = int(Roll_head1)
            Roll_middle = int(Roll_middle1)
            Roll_last = int(Roll_last1)
            #print('Roll:', Roll_head, Roll_middle, Roll_last)

            Heading_head = int(Heading_head1)
            Heading_middle = int(Heading_middle1)
            Heading_last = int(Heading_last1)
            #print('Heading:', Heading_head, Heading_middle, Heading_last)

            if Pitch_head > 9:
                Pitch = - ((Pitch_head - 10) * 100 + Pitch_middle + Pitch_last * 0.01)
            else:
                Pitch = Pitch_head * 100 + Pitch_middle + Pitch_last * 0.01

            print('Pitch: %f' % Pitch)
            if Roll_head > 9:
                Roll = -((Roll_head - 10) * 100 + Roll_middle + Roll_last * 0.01)
            else:
                Roll = Roll_head * 100 + Roll_middle + Roll_last * 0.01
            print('Roll: %f' % Roll)
            if Heading_head > 9:
                Heading = - ((Heading_head - 10) * 100 + Heading_middle + Heading_last * 0.01)
            else:
                Heading = Heading_head * 100 + Heading_middle + Heading_last * 0.01
            print('Heading: %f' % Heading)
            print(time.ctime())
            #self.__ros_pub_gyro.publish(Heading)
            print(Heading)
            # rate = rospy.Rate(50)
            #rate.sleep()

    def shutdown(self):
        self.alive = False
        if self.port.isOpen():
            self.port.close()
    #
    # def Gyro_Data_Pub(self):
    #     # self.__ros_pub_gyro = rospy.Publisher('gyro',Float32, queue_size=3)
    #     # rospy.init_node('gyro', anonymous= True)
    #     # rate = rospy.Rate(50)
    #     while not rospy.is_shutdown():
    #         self.__ros_pub_gyro.publish(self.read_data.Heading)
    #         print(self.read_data.Heading)




if __name__ =='__main__':
    Serial = SerialPort('/dev/ttyUSB1', 115200)
    try:
        # Serial.Gyro_Data_Pub()
        Serial.send_thread.start()
        #Serial.read_thread.start()
        #Serial.send_thread.join()
        #Serial.read_thread.join()
        #while not rospy.is_shutdown():
          #  print("main thread")
            #rospy.sleep(0.1)
        rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo("Gripper init has failed")
        pass

