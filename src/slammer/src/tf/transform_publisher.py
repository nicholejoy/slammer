#!/usr/bin/python

import rospy

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
import tf

class TransformPublisher:
    def __init__(self):
        rospy.init_node('transform_publisher')
        self.pose = [0.0,0.0,0.0]
        self.rate = rospy.get_param('~rate', 50)
        self.tf_bcaster = tf.TransformBroadcaster()

        self.current_time = rospy.Time.now()


    def pose_callback(self, msg):
        self.pose = msg.data

    def actual_twist_callback(self, msg):
        self.twist_cmd = msg


    def broadcast_tf(self, pose):
        self.tf_bcaster.sendTransform((0.0, 0, 0.0), tf.transformations.quaternion_from_euler(0, 0, 0),
                                      self.current_time, "imu_link", "chassis")

        self.tf_bcaster.sendTransform((0.04, 0, 0.075), tf.transformations.quaternion_from_euler(0, 0, 0),
                                      self.current_time, "laser_link", "chassis")

        self.tf_bcaster.sendTransform((0.04, -0.1035, 0), tf.transformations.quaternion_from_euler(0, 0, 0),
                                      self.current_time, "right_wheel", "chassis")

        self.tf_bcaster.sendTransform((0.04, 0.1035, 0), tf.transformations.quaternion_from_euler(0, 0, 0),
                                    self.current_time, "left_wheel", "chassis")

        self.tf_bcaster.sendTransform((0.04, -0.06, -0.02), tf.transformations.quaternion_from_euler(0, 0, 0),
                                      self.current_time, "rear_right_wheel", "chassis")

        self.tf_bcaster.sendTransform((0.04, 0.06, -0.02), tf.transformations.quaternion_from_euler(0, 0, 0),
                                    self.current_time, "rear_left_wheel", "chassis")


    def spin(self):
        rospy.loginfo("Started transform_publisher")
        rate = rospy.Rate(self.rate)
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.broadcast_tf(self.pose)
            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stopped transform_publisher")
        rospy.sleep(1)

def main():
  transform_publisher = TransformPublisher()
  transform_publisher.spin()

if __name__ == '__main__':
  main()
