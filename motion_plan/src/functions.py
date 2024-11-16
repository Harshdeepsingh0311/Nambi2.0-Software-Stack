#! /usr/bin/env python
import rospy
import rospkg
import numpy as np
import math
from numpy.linalg import norm
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf import transformations
from tf.transformations import quaternion_multiply
import tf
from nav_msgs.msg import OccupancyGrid
from pcl_arrow_detect import ArrowDetector
from numpy import nan

path = rospkg.RosPack().get_path("motion_plan")
MARKERS_MAX = 50
ROOT_LINK = "root_link"


class client:
    def __init__(self):
        rospy.init_node("goal_client_node")
        rospy.loginfo("goal_client_node init")

        # define a client for to send goal requests to the move_base server through a SimpleActionClient
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # wait for the action server to come up
        while (
            not self.ac.wait_for_server(rospy.Duration.from_sec(2.0))
            and not rospy.is_shutdown()
        ):
            rospy.loginfo("Waiting for the move_base action server to come up")
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            "map", ROOT_LINK, rospy.Time(0), rospy.Duration(10.0)
        )
        self.mapData = OccupancyGrid()  # map
        self.frame = None
        self.lidar_data = None
        rospy.Subscriber("/map", OccupancyGrid, self.mapCallBack)
        rospy.Subscriber("/scan_filtered", LaserScan, self.lidar_callback)
        self.arrow_detector = ArrowDetector()
        print("ArrowDetector Launched")
        self.marker_array_pub = rospy.Publisher(
            "/detected_arrow", MarkerArray, queue_size=10
        )
        self.marker_array = MarkerArray()
        self.marker_count = 0
        self.completed_list = []
        self.last_good_location = self.bot_to_map(0, 0, (0, 0, 0, 1))
        rospy.Rate(5).sleep()  #
        # rospy.spin()

    def cone_detect(self):
        print(self.arrow_detector.cone_detect())
        found,val,cone_distance = self.arrow_detector.cone_detect()
        if cone_distance==0 or cone_distance==nan:
            q=q_from_vector3D(val)
            return found,q,cone_distance #sending quaternion
        return found,val,cone_distance #sending pos

    def arrow_detect(self, far=True):
        # returns Found(0/1), position(x,y,z), theta(degrees; rover forward=0)
        return self.arrow_detector.arrow_detect(far=far, visualize=False)

    def mapCallBack(self, data):
        self.mapData = data

    # TODO change to ps, q format for all
    def bot_to_map(self, pos_x, pos_y, q, frame=ROOT_LINK, timestamp=None):
        ps = PoseStamped()
        new_ps = PoseStamped()

        # set up the frame parameters
        ps.header.frame_id = frame

        if timestamp == None:
            timestamp = rospy.Time.now()
        ps.header.stamp = timestamp
        ps.pose.position = Point(pos_x, pos_y, 0)
        ps.pose.orientation = recast_quaternion(q)

        success = False
        while not rospy.is_shutdown() and success == False:
            try:
                new_ps = self.listener.transformPose("map", ps)
                success = True
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                success = False
        new_ps.pose.orientation.x = 0  # TODO Check if advisable to do so
        new_ps.pose.orientation.y = 0
        return new_ps.pose.position.x, new_ps.pose.position.y, new_ps.pose.orientation

    def send_goal(self, xGoal, yGoal, q=None, frame="map"):  # frame=ROOT_LINK
        # relative to the bot location
        # quaternion is a 4-tuple/list-x,y,z,w or Quaternion
        goal = MoveBaseGoal()

        if frame == ROOT_LINK:
            xGoal, yGoal, q = self.bot_to_map(xGoal, yGoal, q)
            frame = "map"
        # set up the frame parameters
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
        goal.target_pose.pose.orientation = recast_quaternion(q)

        rospy.loginfo(
            "Sending goal location - [" + str(xGoal) + ", " + str(yGoal) + "] .."
        )
        self.add_arrow(xGoal, yGoal, q)
        self.ac.send_goal(goal)

    def move_to_goal(self, xGoal, yGoal, q=None, frame="map"):  # frame=ROOT_LINK
        if frame == ROOT_LINK:
            xGoal, yGoal, q = self.bot_to_map(xGoal, yGoal, q)
            frame = "map"
        self.send_goal(xGoal, yGoal, q, frame)

        self.ac.wait_for_result(rospy.Duration(60))

        if self.ac.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False

    def move_to_off_goal(self, xGoal, yGoal, q=None, frame="map", off_dist=0.5,ahead=0.75):
        goal_initial=self.find_off_goal(xGoal, yGoal, q=q, frame=frame, offset=(-0.0, off_dist, 0, 0))
        goal_final=just_ahead(*goal_initial,off_dist=ahead)
        #print('move to off goal',goal_initial,goal_final)
        return self.move_to_goal(*goal_final)

    def find_off_goal(self, xGoal, yGoal, q=None, frame="map", offset=(0, 0, 0, 0)):
        if frame == ROOT_LINK:
            xGoal, yGoal, q = self.bot_to_map(xGoal, yGoal, q)
            frame = "map"
        q = uncast_quaternion(q)
        x0, y0, _ = self.bot_to_map(0, 0, (0, 0, 0, 1))  # Bot location in map
        offset = quaternion_multiply(q, offset)
        offset = quaternion_multiply(offset, transformations.quaternion_inverse(q))
        x1, y1 = xGoal + offset[0], yGoal + offset[1]
        cell1 = get_cell_status(self.mapData, [x1, y1])
        x2, y2 = xGoal - offset[0], yGoal - offset[1]
        cell2 = get_cell_status(self.mapData, [x2, y2])
        if cell1 == 0:
            if cell2 == 0:
                x, y = (
                    [x1, y1]
                    if (norm([x1 - x0, y1 - y0]) < norm([x2 - x0, y2 - y0]))
                    else [x2, y2]
                )
            else:
                x, y = x1, y1
        else:
            if cell2 == 0:
                x, y = x2, y2
            else:
                x, y = (
                    [x1, y1]
                    if (norm([x1 - x0, y1 - y0]) < norm([x2 - x0, y2 - y0]))
                    else [x2, y2]
                )
        # rospy.loginfo(str(["x1",x1,"y1",y1,":",cell1,", x2",x2,"y2",y2,":",cell2]))
        return x, y, q

    # def move_towards_goal(self,xGoal,yGoal, frame="map", move_dist=1.5):
    #       if frame == "map":
    #             xGoal,yGoal,_ = self.map_to_bot(xGoal,yGoal,None)
    #             frame = ROOT_LINK
    #       q = uncast_quaternion(q)
    #       offset = transformations.quaternion_multiply(q, (-off_dist,0,0,0))
    #       offset = transformations.quaternion_multiply(offset, transformations.quaternion_inverse(q))
    #       return self.move_to_goal(xGoal+offset[0],yGoal+offset[1],q, frame)

    def add_to_completed(self, pos_x, pos_y, q):
        self.completed_list.append([pos_x, pos_y, q])
        self.last_good_location = self.find_off_goal(
            pos_x, pos_y, q=q, frame="map", offset=(0, 1, 0, 0)
        )

    def is_complete(self, pos_x, pos_y, q):
        for ps in self.completed_list:
            if (
                norm([ps[0] - pos_x, ps[1] - pos_y]) < 0.7 and diff(q, ps[2]) < 5
            ):  # Replace 5 by 0.2 after adding orientation specific things
                return True
        return False

    def cancel_goal(self):
        self.ac.cancel_goal()
        rospy.loginfo("Goal cancelled")

    def recovery(self, far=True):
        rospy.loginfo("Initiating recovery")
        found, pos, orient, timestamp = self.arrow_detect(far)
        j = 0
        while found == False and j < 3:
            x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
            q = uncast_quaternion(q)
            q = quaternion_multiply(q, (0, 0, np.sin(0.2), np.cos(0.2)))
            self.move_to_goal(x, y, q)
            found, pos, orient, timestamp = self.arrow_detect(far)
            j += 1
        j = 0
        while found == False and j < 6:
            x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
            q = uncast_quaternion(q)
            q = quaternion_multiply(q, (0, 0, np.sin(-0.2), np.cos(-0.2)))
            self.move_to_goal(x, y, q)
            found, pos, orient, timestamp = self.arrow_detect(far)
            j += 1
        if found:
            orient2 = orient + 90 if orient < 0 else orient - 90
            q = (
                0,
                0,
                np.sin(np.pi * orient2 / (2 * 180)),
                np.cos(np.pi * orient2 / (2 * 180)),
            )
            posx, posy, q = self.bot_to_map(pos[0], pos[1], q, timestamp=timestamp)  # map frame
        if found == False or pos is None or self.is_complete(posx, posy, q):
            rospy.loginfo("Failed. Moving to last known good location")
            self.move_to_goal(*self.last_good_location)
            return False, None, None, timestamp
        else:
            return found, pos, orient, timestamp

    def add_arrow(
        self, pos_x, pos_y, q, color=(0.2, 0.5, 1.0), pos_z=0
    ):  # color = (r,g,b), in [0,1]
        marker = make_arrow_marker(
            Pose(Point(pos_x, pos_y, pos_z), recast_quaternion(q)),
            self.marker_count,
            color,
        )

        # We add the new marker to the MarkerArray, removing the oldest
        # marker from it when necessary
        if self.marker_count > MARKERS_MAX:
            self.marker_array.markers.pop(0)

        self.marker_array.markers.append(marker)

        # Renumber the marker IDs
        id = 0
        for m in self.marker_array.markers:
            m.id = id
            id += 1

        # Publish the MarkerArray
        self.marker_array_pub.publish(self.marker_array)

        self.marker_count += 1

    def lidar_callback(self, data):
        self.lidar_data = data

    def find_obs_lidar(
        self, theta, error=30, max_width=0.5, max_depth=0.4, sensitivity=0.2
    ):
        if theta is None:
            return None, None
        # Returns posx,posy for the obstacle found around theta with width less than max_width, depth less than given depth
        obs_list = []  # obstacle list
        theta_idx = int((theta + 90) * 4)
        prev_val = 10
        curr_obs_start = None
        min_diff = error
        arrow = None
        for i in range(theta_idx - error, theta_idx + error):
            if i < 0:
                continue
            if i >= 720:
                break
            if abs(self.lidar_data.ranges[i] - prev_val) < sensitivity:
                if curr_obs_start is None:
                    curr_obs_start = i
            else:
                if curr_obs_start is not None:
                    obs_list.append((curr_obs_start, i))
                    curr_obs_start = None
            prev_val = self.lidar_data.ranges[i]
        for obs in obs_list:
            # Check for width and depth
            width = (
                self.lidar_data.ranges[int(np.average(obs))]
                * self.lidar_data.angle_increment
                * (obs[1] - obs[0])
            )
            depth = max(self.lidar_data.ranges[obs[0] : obs[1]]) - min(
                self.lidar_data.ranges[obs[0] : obs[1]]
            )
            if width > max_width or depth > max_depth:
                continue
            else:
                if min_diff > abs(theta_idx - np.average(obs)):
                    min_diff = abs(theta_idx - np.average(obs))
                    arrow = obs
        rospy.loginfo(str(obs_list))
        if arrow is None:
            return None, None
        r, theta = (
            self.lidar_data.ranges[int(np.average(arrow))],
            np.average(arrow) / 4 - 90,
        )
        return self.bot_to_map(
            r * np.cos(np.pi * theta / 180),
            r * np.sin(np.pi * theta / 180),
            (0, 0, 0, 1),
        )[:2]


def recast_quaternion(quaternion):
    if quaternion is None:
        q = Quaternion(0, 0, 0, 1)
    elif (
        isinstance(quaternion, list)
        or isinstance(quaternion, tuple)
        or isinstance(quaternion, np.ndarray)
    ):
        q = Quaternion(*quaternion)
    elif isinstance(quaternion, Quaternion):
        q = quaternion
    else:
        print("Quaternion in incorrect format: ", type(quaternion))
        q = Quaternion(0, 0, 0, 1)
    return q


def uncast_quaternion(quaternion):
    if quaternion is None:
        q = (0, 0, 0, 1)
    elif (
        isinstance(quaternion, list)
        or isinstance(quaternion, tuple)
        or isinstance(quaternion, np.ndarray)
    ):
        q = tuple(quaternion)
    elif isinstance(quaternion, Quaternion):
        q = quaternion
        q = (q.x, q.y, q.z, q.w)  # lazy+readable code
    else:
        print("Quaternion in incorrect format: ", type(quaternion))
        q = (0, 0, 0, 1)
    return q


def q_from_vector3D(point):
    # http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    q = Quaternion()
    # calculating the half-way vector.
    u = [1, 0, 0]
    norm = np.linalg.norm(point)
    v = np.asarray(point) / norm
    if np.all(u == v):
        q.w = 1
        q.x = 0
        q.y = 0
        q.z = 0
    elif np.all(u == -v):
        q.w = 0
        q.x = 0
        q.y = 0
        q.z = 1
    else:
        half = [u[0] + v[0], u[1] + v[1], u[2] + v[2]]
        q.w = np.dot(u, half)
        temp = np.cross(u, half)
        q.x = temp[0]
        q.y = temp[1]
        q.z = temp[2]
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm == 0:
        norm = 1
    q.x /= norm
    q.y /= norm
    q.z /= norm
    q.w /= norm
    return q


def diff(q1, q2):  # accepts tuples
    q1 = uncast_quaternion(q1)
    q2 = uncast_quaternion(q2)
    q1_inv = transformations.quaternion_inverse(q1)
    diff_q = quaternion_multiply(q2, q1_inv)
    return abs(
        transformations.euler_from_quaternion(diff_q)[2]
    )  # return yaw between two angles (quaternions)


def get_cell_status(mapData, pt):
    # returns grid value at point "pt"- shape:(2)
    # map data:  100 occupied      -1 unknown       0 free
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data

    index = (np.floor((pt[1] - Xstarty) / resolution) * width) + (
        np.floor((pt[0] - Xstartx) / resolution)
    )
    if np.isnan(index) == True:
        print("Error: index is NaN, check : ", resolution, pt, width, Xstartx, Xstarty)
        return 100

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100


def just_ahead(pos_x, pos_y, q, off_dist=0.5):
    q = recast_quaternion(q)
    offset = quaternion_multiply((q.x, q.y, q.z, q.w), (off_dist, 0, 0, 0))
    offset = quaternion_multiply(
        offset, transformations.quaternion_inverse((q.x, q.y, q.z, q.w))
    )
    x, y = pos_x + offset[0], pos_y + offset[1]
    return x, y, q


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

def main():
    sin45 = 1 / np.sqrt(2)
    my_client = client()
    my_client.move_to_off_goal(4, 0, q=(0, 0, sin45, sin45))  # 4,0
    my_client.move_to_off_goal(4, 6)  # 4,6
    my_client.move_to_off_goal(8, 6, q=(0, 0, sin45, -sin45))  # 8,6
    my_client.move_to_off_goal(8, 2)  # 8,2
    my_client.move_to_off_goal(
        12, 2, q=(0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8))
    )  # 12,2
    my_client.move_to_off_goal(
        12, 6, q=(0, 0, sin45, sin45)
    )  # 12,6#check for error, if any
    my_client.move_to_off_goal(8, 10, q=(0, 0, 1, 0))  # 8,10
    my_client.move_to_goal(-2, 10, q=(0, 0, 1, 0))  # -2,10

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Closing")
