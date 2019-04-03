#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
import argparse

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

# Usage: use this node to publish a path topic from a pose. 
# For example: python pose2path --reference '/vrpn_client_node/bintel/pose'

class pose2path():
    def __init__(self):
        # Parse Input
        parser = argparse.ArgumentParser()
        parser.add_argument("reference", help="name of the optitrack frame used for reference")
        
        args = parser.parse_args()
        self.reference = args.reference

        # Init Node
        rospy.init_node('path_node')
        rospy.Subscriber(self.reference, PoseStamped, self.poseCallback)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.path = Path()
      
        self.start = 0

        rospy.spin()

    def processFeedback_init(self,feedback):
        p = feedback.pose.position
        #print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

    def poseCallback(self, data):
        self.path.header = data.header
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)
        if self.start == 0:
            self.start = 1
            self.start_interactive_marker(data)

  

    def start_interactive_marker(self,initPose):
        # create an interactive marker server on the topic namespace simple_marker
        server = InteractiveMarkerServer("simple_marker")
        
        # create an interactive marker for our server
        int_marker_end = InteractiveMarker()
        int_marker_end.header.frame_id = "world"
        int_marker_end.name = "my_marker"
        int_marker_end.description = "Simple 1-DOF Control"

        # create a grey box marker
        self.box_marker = Marker()

        self.box_marker.type = Marker.CUBE
        self.box_marker.scale.x = 0.2
        self.box_marker.scale.y = 0.2
        self.box_marker.scale.z = 0.2
        self.box_marker.color.r = 0.0
        self.box_marker.color.g = 0.5
        self.box_marker.color.b = 0.5
        self.box_marker.color.a = 1.0
        self.box_marker.pose.position = initPose.pose.position

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( self.box_marker )

        # add the control to the interactive marker
        int_marker_end.controls.append( box_control )

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        self.rotate_control = InteractiveMarkerControl()
        self.rotate_control.name = "move_x"
        self.rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        # add the control to the interactive marker
        int_marker_end.controls.append(self.rotate_control);

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        server.insert(int_marker_end, self.processFeedback_init)

        # 'commit' changes and send to ll clients
        server.applyChanges()  


if __name__ == '__main__':
    try:
        opR = pose2path()
    except rospy.ROSInterruptException:
        pass
