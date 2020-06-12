#!/usr/bin/python
import rospy


from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class VelocityPublisher:
    def __init__(self):
        rospy.init_node('velocity_publisher')

        self.wheel_sep = rospy.get_param('~wheel_separation', 0.203)
        self.radius = rospy.get_param('~radius', 0.05)

        self.rate = rospy.get_param('~rate', 10)
        self.timeout = 1.0/self.rate

        self.time_prev_update = rospy.Time.now()
        self.forward_vel_setpnt= 0;
        self.rotational_vel_setpnt = 0;

        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.twist_cmd_callback)
        self.left_angular_vel_setpnt_pub = rospy.Publisher('/slammer/leftWheel_vel_controller/command', Float64, queue_size=10)
        self.right_angular_vel_setpnt_pub = rospy.Publisher('/slammer/rightWheel_vel_controller/command', Float64, queue_size=10)


    def twist_cmd_callback(self,msg):
        self.forward_vel_setpnt = msg.linear.x
        self.rotational_vel_setpnt = msg.angular.z
        self.time_prev_update = rospy.Time.now()
 
    def spin(self):
        rospy.loginfo("Started velocity_publisher")
        rate = rospy.Rate(self.rate)
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            time_diff = (rospy.Time.now() - self.time_prev_update).to_sec()
            if time_diff < self.timeout:

                right_tangent = (2*self.forward_vel_setpnt + self.rotational_vel_setpnt*self.wheel_sep) /2
                left_tangent = (2*self.forward_vel_setpnt - self.rotational_vel_setpnt*self.wheel_sep) /2

                right_angular = right_tangent / self.radius
                left_angular = left_tangent / self.radius

                self.right_angular_vel_setpnt_pub.publish(right_angular)
                self.left_angular_vel_setpnt_pub.publish(left_angular)

            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stopped velocity_publisher")

        self.right_angular_vel_setpnt_pub.publish(0)
        self.left_angular_vel_setpnt_pub.publish(0)

        rospy.sleep(1)




def main():
    velocity_publisher = VelocityPublisher()
    velocity_publisher.spin()

if __name__ == '__main__':
    main()
