#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float32MultiArray

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCOut   

import roslib
import rospy
import tf
import argparse

class savePressureCVS():
    def __init__(self):
    
                
        rospy.init_node('savePressureCVS', anonymous=True)
        rospy.Subscriber("/mavros/rc/out", RCOut, self.getRCOut)
        rospy.Subscriber("/pressureBMP388_array", Float32MultiArray, self.getPressure)
        rate = rospy.Rate(50) 
        
        self.file = open('pressure_test.csv','w')
        self.file.write('Time(s), RCOut(x8), pressure(x5) \n')
        
        sc = 0
        sp = 0
        
        rospy.sleep(2)
        self.t0 = rospy.get_time()
        
        while not rospy.is_shutdown():
            hello_str = "saving to cvs running for  %s" % (rospy.get_time()-self.t0)
            rospy.loginfo(hello_str)
            self.saveDataCVS()
            rate.sleep()


    def getRCOut(self,data):
        self.rc = data.channels
        #rospy.loginfo(str(self.rc))

    def getPressure(self,data):
        self.pressure = data.data

    def saveDataCVS(self):
        self.file.write("%5.5f, " % (rospy.get_time()-self.t0)  )
        self.file.write(", ".join(str(elem) for elem in self.rc) )
        self.file.write(", ".join(str(elem) for elem in self.pressure) + "\n")
        #self.file.write('%5.3f, 5.3f, 5.3f, 5.3f, 5.3f,5.3f,5.3f,5.3f,' % (self.rc))
        #self.file.write('%5.3f \n' % (self.data))

    def shutdown(self):
        self.file.close()
        rospy.loginfo("Stop savePressureCVS")
        rospy.sleep(1)

        
        
if __name__ == '__main__':
    try:
        gotoop = savePressureCVS()
    except rospy.ROSInterruptException:
        pass
