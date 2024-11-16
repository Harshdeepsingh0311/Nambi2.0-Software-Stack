#! /usr/bin/env python

import rospy
import sys
import tf2_ros
import geonav_transform.geonav_conversions as gc
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray

ROOT_LINK = "root_link"


class gps_tester:
    def __init__(self):
        rospy.init_node("gps_test_node")
        rospy.loginfo("gps_test_node init")
        self.olat, self.olon = 19.13079235, 72.91834925  # 49.8999997, 8.90000001
        self.q = Quaternion(0, 0, 0, 1)
        self.br = tf2_ros.TransformBroadcaster()
        self.marker_array = MarkerArray()
        self.marker_count = 1
        self.marker_array_pub = rospy.Publisher(
            "/detected_marker", MarkerArray, queue_size=10
        )
        self.marker_array.markers.append(
            make_arrow_marker(ps=Pose(), id=0, color=(0, 0.3, 0.5))
        )
        self.marker_array_pub.publish(self.marker_array)
        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.wait_for_message("/imu", Imu, timeout=5)
        rospy.wait_for_message("/fix", NavSatFix, timeout=5)

    def update_init_latlon(self, lat, lon):
        self.olat = lat
        self.olon = lon

    def gps_callback(self, data):
        lat, lon = data.latitude, data.longitude
        y, x = gc.ll2xy(lat, lon, self.olat, self.olon)
        self.update_pose(x, -y)

    def xy2gps(self, x, y):
        return gc.xy2ll(x, y, self.olat, self.olon)

    def gps2xy(self, lat, lon):
        return gc.ll2xy(lat, lon, self.olat, self.olon)

    def imu_callback(self, data):
        self.q = data.orientation

    def update_pose(self, x, y):
        for m in self.marker_array.markers:
            if m.id == 0:
                m.pose = Pose(Point(x, y, 0), self.q)
                self.marker_array_pub.publish(self.marker_array)
                t = TransformStamped()

                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = ROOT_LINK
                t.transform.translation = Point(x, y, 0)
                t.transform.rotation = self.q
                self.br.sendTransform(t)
                return

    def add_arrow(
        self, pos_x, pos_y, q=None, color=(0.2, 0.7, 0.2)
    ):  # color = (r,g,b), in [0,1]
        marker = make_arrow_marker(
            Pose(Point(pos_x, pos_y, 0), recast_quaternion(q)), self.marker_count, color
        )

        self.marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.marker_array_pub.publish(self.marker_array)

        self.marker_count += 1


def make_arrow_marker(ps, id, color=(0.2, 0.5, 1.0)):
    # make a visualization marker array for the occupancy grid
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    m.ns = "arrows"
    m.id = id
    m.type = Marker.ARROW
    m.pose = ps
    m.scale = Point(1, 0.1, 0.1)
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = 1

    return m


def recast_quaternion(quaternion):
    if quaternion is None:
        q = Quaternion(0, 0, 0, 1)
    elif isinstance(quaternion, list) or isinstance(quaternion, tuple):
        q = Quaternion(*quaternion)
    elif isinstance(quaternion, Quaternion):
        q = quaternion
    else:
        print("Quaternion in incorrect format")
        q = Quaternion(0, 0, 0, 1)
    return q


if __name__ == "__main__":
    try:
        my_tester = gps_tester()
        print("my_tester created")
        my_tester.add_arrow(10, 0)
        ps = my_tester.gps2xy(*my_tester.xy2gps(10, 0))
        my_tester.add_arrow(*ps, color=(1, 0, 0))
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Exiting... ")
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise
