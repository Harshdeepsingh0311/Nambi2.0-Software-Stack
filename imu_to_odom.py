import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class ImuOdometry:
    def __init__(self):
        #rospy.init_node('imu_odometry')
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        
        self.position = [0, 0, 0]
        self.velocity = [0, 0, 0]
        self.orientation = [0, 0, 0]
        self.angular_velocity = [0, 0, 0]
        
    def imu_callback(self, msg):
        self.linear_acceleration = msg.linear_acceleration
        self.angular_velocity = msg.angular_velocity
        
        # integrate linear acceleration to estimate velocity
        self.velocity[0] += self.linear_acceleration.x * 0.01
        self.velocity[1] += self.linear_acceleration.y * 0.01
        self.velocity[2] += self.linear_acceleration.z * 0.01
        
        # integrate angular velocity to estimate orientation
        self.orientation[0] += self.angular_velocity.x * 0.01
        self.orientation[1] += self.angular_velocity.y * 0.01
        self.orientation[2] += self.angular_velocity.z * 0.01
        
        # integrate velocity to estimate position
        self.position[0] += self.velocity[0] * 0.01
        self.position[1] += self.velocity[1] * 0.01
        self.position[2] += self.velocity[2] * 0.01
        
        # convert orientation to quaternion
        quat = quaternion_from_euler(self.orientation[0], self.orientation[1], self.orientation[2])
        
        # publish odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        self.odom_pub.publish(odom)
        print("Published")

if __name__ == '__main__':
    rospy.init_node('imu_odometry')
    ImuOdometry()
    rospy.spin()

