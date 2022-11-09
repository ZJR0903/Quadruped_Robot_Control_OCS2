#!/usr/bin/env python3
#coding=utf-8
import rospy
import serial
import time
from std_msgs.msg import Int16MultiArray

 
if __name__=="__main__":
    rospy.init_node('foot_sensor_detect')
    t0 = time.time()
    f = open('foot-record.txt',mode='w')
    ser = serial.Serial(port='/dev/ttyUSB1', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=2,rtscts=True,dsrdtr=True)
    pub = rospy.Publisher('/foot/sensor', Int16MultiArray, queue_size=10)
    strdata  = ""  
    initdata = [-100,-100,-100,-100]
    initcount = 0
    curdata = [0,0,0,0]
    while not rospy.is_shutdown():
        try:
            # pubdata = Int16MultiArray() 
            # pubdata.data = [1,1,1,1]     
            # pub.publish(pubdata)
            tmpmsg = ser.read(1).decode("utf-8")
            if(tmpmsg == ":"):
                strdata = ""
            elif(tmpmsg == "\n"):
                datalist = strdata.split(",")
                if(len(datalist)!=5):
                    continue
                pubdata = Int16MultiArray()
                for i in range(4):
                    res = 100*int(datalist[i])/0x800000
                    curdata[i] = int((res-initdata[i])*10)/10
                    if(initcount < 100):
                        initdata[i] = res if res>initdata[i] else initdata[i]
                        pubdata.data.append(0)
                    else:
                        sinmsg = 1 if res>(0.1+initdata[i]) else 0
                        pubdata.data.append(sinmsg)
                pub.publish(pubdata)
                print(initcount,pubdata.data,curdata)      
                f.write(str(int((time.time()-t0)*1000))+":"+str(pubdata.data)+"|"+ str(curdata)+"\n")
                initcount += 1
            else:
                strdata += tmpmsg
        except Exception as e:
            print(e)
            pass
    f.close()
