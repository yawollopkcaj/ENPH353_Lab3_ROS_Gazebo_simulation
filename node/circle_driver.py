#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('circle_driver', anonymous=True)

    # Parameters
    linear_speed = 0 # meters per second
    angular_speed = 1 # radians per second
    topic = rospy.get_param('~topic', '/cmd_vel')
    hz = rospy.get_param('~rate_hz', 20)

    pub = rospy.Publisher(topic, Twist, queue_size=10)
    rate = rospy.Rate(hz)

    # Prep constant command
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    
    rospy.loginfo("Starting circle driving with linear speed: %.2f m/s and angular speed: %.2f rad/s", linear_speed, angular_speed)

    # Stop on shutdown
    def on_shutdown():
        stop = Twist()
        pub.publish(stop)
        rospy.sleep(0.05)
    rospy.on_shutdown(on_shutdown)

    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    main()