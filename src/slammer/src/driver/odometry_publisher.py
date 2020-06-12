#!/usr/bin/python
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
import tf

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher')
        self.radius = rospy.get_param('~radius', 0.05)
        self.wheel_sep = rospy.get_param('~wheel_separation', 0.203)
        self.rate = rospy.get_param('~rate', 10)

        self.frame_id = rospy.get_param('~frame_id','odom_wheel')
        self.child_frame_id = rospy.get_param('~child_frame_id','chassis')

        self.left_enc_rad = [0.0, 0.0]
        self.right_enc_rad = [0.0, 0.0]
        self.time_delta = 1.0 / self.rate
        self.joint_state_sub = rospy.Subscriber('/slammer/joint_states', JointState, self.joint_states_callback)
        self.time_delta = 1.0 /self.rate
        self.pose = {'x': 0.0, 'y': 0.0, 'angle': 0.0}

        self.left_m = 0.0
        self.right_m = 0.0
        self.avg_lin_dist = 0.0
        self.avg_ang_dist = 0.0
        self.linear_vel = 0.0
        self.anguler_vel = 0.0
        self.odom_pub = rospy.Publisher('/slammer/odom', Odometry, queue_size=10)
        #self.actual_twist_pub = rospy.Publisher('/slammer/actual_cmd_vel', Twist, queue_size=10)
        self.slammer_pos_pub = rospy.Publisher('/slammer/slammer_pose', Float64MultiArray, queue_size=10)
        self.twist_cmd = Twist()
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.frame_id
        self.odom_msg.child_frame_id = self.child_frame_id

        self.odom_msg.pose.covariance[0] = 0.000000000000000000000001
        self.odom_msg.pose.covariance[7] = 0.000000000000000000000001
        self.odom_msg.pose.covariance[14] = 1000000.0
        self.odom_msg.pose.covariance[21] = 1000000.0
        self.odom_msg.pose.covariance[28] = 1000000.0
        self.odom_msg.pose.covariance[35] = 0.03
        self.odom_msg.twist.covariance = self.odom_msg.pose.covariance
        self.odom_quat = Quaternion()
        self.pose_msg = Float64MultiArray()

    def joint_states_callback(self, msg):
        self.left_enc_rad[0] = self.left_enc_rad[1]
        self.right_enc_rad[0] = self.right_enc_rad[1]

        self.left_enc_rad[1] = msg.position[0]
        self.right_enc_rad[1] = msg.position[1]

    def distance(self, enc_rad):
        meters = (enc_rad[1] - enc_rad[0]) * self.radius
        return meters

    def exact_integration_pose(self):
        self.pose['x'] = self.pose['x'] + self.avg_lin_dist / self.avg_ang_dist * (
                math.sin(self.pose['angle'] + self.avg_ang_dist) - math.sin(self.pose['angle']))
        self.pose['y'] = self.pose['y'] - self.avg_lin_dist / self.avg_ang_dist * (
                math.cos(self.pose['angle'] + self.avg_ang_dist) - math.cos(self.pose['angle']))
        self.pose['angle'] = self.pose['angle'] + self.avg_ang_dist

    def runge_kutta_pose(self):
        self.pose['x'] = self.pose['x'] + self.avg_lin_dist * math.cos(self.pose['angle'] + self.avg_ang_dist / 2)
        self.pose['y'] = self.pose['y'] + self.avg_lin_dist * math.sin(self.pose['angle'] + self.avg_ang_dist / 2)

        self.pose['angle'] = self.pose['angle'] + self.avg_ang_dist



    def update_twist(self, linear, angular):
        self.twist_cmd.linear.x = linear
        self.twist_cmd.angular.z = angular


    def publish_odometry(self):

        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.pose['angle'])
        self.odom_msg.pose.pose = Pose(Point(self.pose['x'], self.pose['y'], 0.0), Quaternion(*self.odom_quat))
        self.odom_msg.twist.twist = self.twist_cmd
        self.odom_pub.publish(self.odom_msg)


    def publish_pose(self):
        self.pose_msg.data = [val for val in self.pose.values()]
        self.slammer_pos_pub.publish(self.pose_msg)


    def spin(self):
        rospy.loginfo("Started pose_publisher")
        rate = rospy.Rate(self.rate)
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.left_m = self.distance(self.left_enc_rad)
            self.right_m = self.distance(self.right_enc_rad)
            self.avg_lin_dist = (self.left_m + self.right_m) / 2
            self.avg_ang_dist = (self.right_m - self.left_m) / self.wheel_sep
            self.linear_vel = self.avg_lin_dist / self.time_delta
            self.anguler_vel = self.avg_ang_dist / self.time_delta

            if self.anguler_vel > 0.0000001:
                self.exact_integration_pose()
            else:
                self.runge_kutta_pose()
            self.update_twist(self.linear_vel, self.anguler_vel)
            self.publish_odometry()
            self.publish_pose()


            rate.sleep()
        rospy.spin()


    def shutdown(self):
        rospy.loginfo("Stopped odometry_publisher")
        self.update_twist(0, 0)
        self.publish_odometry()
        self.publish_pose()
        rospy.sleep(1)


def main():
    odom_publisher = OdometryPublisher()
    odom_publisher.spin()


if __name__ == '__main__':
    main()
