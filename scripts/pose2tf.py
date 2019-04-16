
import argparse
import sys
import yaml

import rospy
import tf2_ros as tf2
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped

class 
    def pose_to_tf(self, pose, frame_name, parent_frame, time=None):
        """
        Generate a TF from a given pose, frame, and parent.
        """
        assert pose is not None, 'Cannot have None for pose.'
        tf = TransformStamped()
        tf.child_frame_id = frame_name
        if time is None:
            time = rospy.Time.now()
        tf.header.stamp = time
        tf.header.frame_id = parent_frame

        tf.transform.translation = pose.position
        tf.transform.rotation = pose.orientation

        return tf

    def spin(self):
        """
        Publish transforms at the requested rate.
        """
        r = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            self.publish_transforms()
            r.sleep()
    def run(argv):
        rospy.init_node('pose_to_tf_rebroadcaster', anonymous=True)

        # Parse initial pose: can be None or list of elements.
        parser = argparse.ArgumentParser(
            description='Simple Pose->TF rebroadcaster.')
        parser.add_argument('--config_file', required=True,
                            help='YAML configuration file',
                            )

        # Parse arguments and convert into StampedPose.
        args = parser.parse_args(rospy.myargv(argv))
        config = load_config(args.config_file)
        print 'Have configuration: {}'.format(config)

        transform_rebroadcaster = PoseToTFRebroadcaster(config)
        rospy.loginfo('Pose to TF rebroadcaster is now running...')

        transform_rebroadcaster.spin()
        rospy.loginfo('Pose to TF rebroadcaster has finished.')

if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)