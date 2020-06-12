#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu

class ImuPublisher:
    def __init__(self):
        rospy.init_node('imu_publisher')
        self.imu_raw_sub = rospy.Subscriber('/slammer/imu_raw', Imu, self.imu_raw_callback)
        self.imu_data_pub = rospy.Publisher('/slammer/imu_raw_cov', Imu, queue_size = 10)
        self.imu_msg = Imu()
        self.rate = rospy.get_param('~rate', 10)

    def imu_raw_callback(self, msg):
        self.imu_msg = msg

    def spin(self):
        rospy.loginfo("Started imu_publisher")
        rate = rospy.Rate(self.rate)

        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            self.imu_msg.angular_velocity_covariance = (1000000000000000.0, 1000000000000000.0, 1000000000000000.0,
                                                        1000000000000000.0, 1000000000000000.0, 1000000000000000.0,
                                                        1000000000000000.0, 1000000000000000.0, 1000000000000000.0)
            self.imu_msg.linear_acceleration_covariance = (1000000000000000.0, 1000000000000000.0, 1000000000000000.0,
                                                           1000000000000000.0, 1000000000000000.0, 1000000000000000.0,
                                                           1000000000000000.0, 1000000000000000.0, 1000000000000000.0)
            self.imu_data_pub.publish(self.imu_msg)
            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stopped imu_publisher")

        rospy.sleep(1)


def main():
    imu_publisher = ImuPublisher()
    imu_publisher.spin()


if __name__ == '__main__':
    main()
