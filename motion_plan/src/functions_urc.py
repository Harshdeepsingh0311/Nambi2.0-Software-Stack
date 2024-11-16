#! /usr/bin/env python

import rospy
import rospkg
import numpy as np
from numpy.linalg import norm
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from tf.transformations import quaternion_multiply
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, LaserScan
from tf import transformations
import tf
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import geonav_transform.geonav_conversions as gc

# from process_ar_tags import *
import cv2, copy

path = rospkg.RosPack().get_path("motion_plan")
MARKERS_MAX = 50
ROOT_LINK = "root_link"
CALIB_MTX = np.array(
    [
        [1.01496820e03, 0.00000000e00, 2.97246399e02],
        [0.00000000e00, 1.03049834e03, 1.74905232e02],
        [0.00000000e00, 0.00000000e00, 1.00000000e00],
    ]
)
DIST = np.array([[0.20890098, 0.13254024, -0.03460789, -0.00425681, 0.81484297]])
DICTIONARY = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


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
        self.mapData = OccupancyGrid()
        self.olat, self.olon = 19.13079235, 72.91834925  # 49.8999997, 8.90000001

        rospy.Subscriber("/map", OccupancyGrid, self.mapCallBack)
        rospy.wait_for_message("/mrt/camera/color/image_raw", Image, timeout=5)
        rospy.Subscriber("/mrt/camera/color/image_raw", Image, self.cam_callback)
        rospy.Subscriber("/mrt/laser/scan", LaserScan, self.lidar_callback)
        self.lidar_data = None
        self.marker_array_pub = rospy.Publisher(
            "/detected_marker", MarkerArray, queue_size=10
        )
        self.marker_array = MarkerArray()
        self.marker_count = 0
        self.completed_list = []
        rospy.Rate(5).sleep()  #
        # rospy.spin()

    def xy2gps(self, x, y):
        return gc.xy2ll(x, y, self.olat, self.olon)

    def gps2xy(self, lat, lon):
        return gc.ll2xy(lat, lon, self.olat, self.olon)

    def mapCallBack(self, data):
        self.mapData = data

    def bot_to_map(self, pos_x, pos_y, q):
        ps = PoseStamped()

        # set up the frame parameters
        ps.header.frame_id = ROOT_LINK
        ps.header.stamp = rospy.Time.now()

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
        rospy.loginfo("send goal")

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

    def send_goal_gps(self, Lat, Lon, q=None, frame="map"):
        x, y = self.gps2xy(Lat, Lon)
        self.send_goal(x, y, q, frame)

    def move_to_goal(self, xGoal, yGoal, q=None, frame="map"):  # frame=ROOT_LINK
        self.send_goal(xGoal, yGoal, q, frame)

        self.ac.wait_for_result(rospy.Duration(60))

        if self.ac.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False

    def move_to_goal_gps(self, Lat, Lon, q=None, frame="map"):
        x, y = self.gps2xy(Lat, Lon)
        return self.move_to_goal(x, y, q, frame)

    def move_to_off_goal(self, xGoal, yGoal, q=None, frame="map", off_dist=1.5):
        return self.move_to_goal(
            *self.find_off_goal(
                xGoal, yGoal, q=q, frame=frame, offset=(0, off_dist, 0, 0)
            ),
            frame=frame
        )

    def move_to_off_goal_gps(self, Lat, Lon, q=None, frame="map", off_dist=1.5):
        x, y = self.gps2xy(Lat, Lon)
        rospy.loginfo("off goal" + str(x) + str(y))
        return self.move_to_goal(
            *self.find_off_goal(x, y, q=q, frame=frame, offset=(0, off_dist, 0, 0)),
            frame=frame
        )

    def find_off_goal(self, xGoal, yGoal, q=None, frame="map", offset=(0, 0, 0, 0)):
        rospy.loginfo("find off goal")
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

    def find_off_goal_gps(self, Lat, Lon, q=None, frame="map"):
        x, y = self.gps2xy(Lat, Lon)
        return self.find_off_goal(x, y, q, frame)

    def add_to_completed(self, pos_x, pos_y, q):
        self.completed_list.append([pos_x, pos_y, q])

    def is_complete(self, pos_x, pos_y, q):
        for ps in self.completed_list:
            if (
                np.sqrt((ps[0] - pos_x) ** 2 + (ps[1] - pos_y) ** 2) < 0.7
                and diff(q, ps[2]) < 5
            ):  # Replace 5 by 0.2 after adding orientation specific things
                return True
        return False

    def cancel_goal(self):
        self.ac.cancel_goal()
        rospy.loginfo("Goal cancelled")

    def add_arrow(
        self, pos_x, pos_y, q, color=(0.2, 0.5, 1.0)
    ):  # color = (r,g,b), in [0,1]
        marker = make_arrow_marker(
            Pose(Point(pos_x, pos_y, 0), recast_quaternion(q)), self.marker_count, color
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

    def cam_callback(self, data):
        # Used to convert between ROS and OpenCV images
        br = CvBridge()

        # Output debugging information to the terminal
        # rospy.loginfo("receiving video frame")

        # Convert ROS Image message to OpenCV image
        current_frame = br.imgmsg_to_cv2(data)
        self.frame = current_frame

    def ar_detect(self):
        if self.frame is None:
            return False, None
        image = self.frame.copy()
        h, w = image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            CALIB_MTX, DIST, (w, h), 1, (w, h)
        )
        dst = cv2.undistort(image, CALIB_MTX, DIST, None, newcameramtx)
        img_copy = copy.deepcopy(dst)
        # convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # thresh = 100
        # converting image to black and white to make the process robust
        # im_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]
        # cv2.imshow("in", im_bw)
        im_bw = gray
        # Parameters for the detectors
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.minMarkerPerimeterRate = 0.2  # default: 0.05
        # return values: corners, Tag ID array (nonetype), rejected candidates for tags
        corners, ids, rejects = cv2.aruco.detectMarkers(
            im_bw, DICTIONARY, parameters=parameters
        )
        # TODO(Ashwin,Harsh): Use Camera Calibration
        # corners, ids, rejects = cv2.aruco.detectMarkers(im_bw, DICTIONARY, parameters=parameters,cameraMatrix=cameraMatrix)
        # drawing markers
        img = cv2.aruco.drawDetectedMarkers(img_copy, corners, ids)
        if len(corners) > 0:
            # print the coordinates (Can use the returned values)
            corners = np.array(corners).reshape(-1, 2)
            # theta = -(np.average(corners, axis=1)[0]/(np.shape(img)[0]) - 0.5)*45*2
            found = len(corners) / 4
            theta = []
            for ar in range(found):
                centroid_x = np.mean([i[1] for i in corners])
                theta.append(-40 + (centroid_x * 80) / width)
            # print("Detected: ", theta)
            return found, theta
        return False, None

    def arrow_detect(self):
        # Arrow detection
        img = self.frame.copy()
        found = False
        theta = None
        orient = None
        direction = None
        contours, _ = cv2.findContours(
            preprocess(img), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )[-2:]
        # cv2.imshow("Image", preprocess(img))
        # cv2.waitKey(0)
        for cnt in contours:
            if (
                cv2.contourArea(cnt) < 150
            ):  # Increase to make it robust to false positive
                continue
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
            hull = cv2.convexHull(approx, returnPoints=False)
            sides = len(hull)

            if (sides == 5 or sides == 4) and sides + 2 == len(approx):
                arrow_tip = find_tip(approx[:, 0, :], hull.squeeze())
                rect, dirct = find_tail_rect(approx[:, 0, :], hull.squeeze())
                if arrow_tip and rect is not None:
                    cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
                    cv2.circle(img, arrow_tip, 3, (0, 0, 255), cv2.FILLED)
                    # cv2.polylines(img, [rect],  True, (0, 0, 255), 2)
                    arrow_tail = tuple(
                        np.average([rect[0], rect[3]], axis=0).astype(int)
                    )
                    if (
                        arrow_tail[0] - arrow_tip[0] == 0
                    ):  # escape possible runtime error
                        continue
                    print(
                        "tip-tail tan angle: ",
                        abs(
                            float(arrow_tail[1] - arrow_tip[1])
                            / (arrow_tail[0] - arrow_tip[0])
                        ),
                    )
                    # Check that tan of angle of the arrow in the image from horizontal is less than 0.2(we are expecting nearly horizontal arrows)(atan(0.2) = 11.31)
                    if (
                        abs(
                            float(arrow_tail[1] - arrow_tip[1])
                            / (arrow_tail[0] - arrow_tip[0])
                        )
                        > 0.2
                    ):
                        continue  # Discard it, not a horizontal arrow
                    # cv2.circle(img, arrow_tail, 3, (0, 0, 255), cv2.FILLED)
                    # cv2.circle(img, tuple(np.average([arrow_tail, arrow_tip], axis=0).astype(int)), 3, (0, 0, 255), cv2.FILLED)#arrow centre
                    theta = (
                        -(
                            np.average([arrow_tail[0], arrow_tip[0]])
                            / (np.shape(img)[0])
                            - 0.5
                        )
                        * 45
                        * 2
                    )  # linear estimate, assuming camera horizontal range from -45 to 45
                    direction = dirct  # TODO multiple arrow case
                    found = True
                    rospy.loginfo("arrow_x_img: " + str(np.average(rect, axis=0)[0]))
        if direction is not None:  # TODO: Improve upon this naive orientation
            if direction == 1:  # Right
                orient = -90
            elif direction == 0:  # Left
                orient = 90
            else:
                rospy.loginfo(
                    "error: direction not found and not None, " + str(direction)
                )
                found = False
        if found:
            # global path #motion_plan pkg dir
            rospy.loginfo(str([found, theta, orient]))
        # return found, theta, orient   #theta and orient wrt forward direction, in degree
        # cv2.imwrite(path + "/src/arrow_frames/Arrow_detection@t="+str(rospy.Time.now())+".png", img)
        # cv2.imwrite(path + "/src/arrow_frames/og@t="+str(rospy.Time.now())+".png", self.frame)
        return found, theta, orient

    def lidar_callback(self, data):
        self.lidar_data = data

    def find_obs_lidar(
        self, theta, error=30, max_width=0.5, max_depth=0.4, sensitivity=0.2
    ):
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
        return r * np.cos(np.pi * theta / 180), r * np.sin(np.pi * theta / 180)


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


def uncast_quaternion(quaternion):
    if quaternion is None:
        q = (0, 0, 0, 1)
    elif isinstance(quaternion, list) or isinstance(quaternion, tuple):
        q = quaternion
    elif isinstance(quaternion, Quaternion):
        q = quaternion
        q = (q.x, q.y, q.z, q.w)  # lazy+readable code
    else:
        print("Quaternion in incorrect format")
        q = (0, 0, 0, 1)
    return q


def diff(q1, q2):  # accepts tuples
    q1 = uncast_quaternion(q1)
    q2 = uncast_quaternion(q2)
    q1_inv = tf.transformations.quaternion_inverse(q1)
    diff_q = tf.transformations.quaternion_multiply(q2, q1_inv)
    return abs(
        tf.transformations.euler_from_quaternion(diff_q)[2]
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
    offset = transformations.quaternion_multiply(
        (q.x, q.y, q.z, q.w), (off_dist, 0, 0, 0)
    )
    offset = transformations.quaternion_multiply(
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


# ____________________________________ Image Processing


def preprocess(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, img_thres = cv2.threshold(img_gray, 70, 255, cv2.THRESH_TOZERO)
    img_blur = cv2.GaussianBlur(img_thres, (5, 5), 1)
    img_canny = cv2.Canny(img_blur, 50, 50)
    kernel = np.ones((3, 3))
    img_dilate = cv2.dilate(img_canny, kernel, iterations=1)
    img_erode = cv2.erode(img_dilate, kernel, iterations=1)
    return img_erode


def find_tip(points, convex_hull):
    length = len(points)
    indices = np.setdiff1d(range(length), convex_hull)
    # print(indices, "convex_hull:",convex_hull,"points:", points)
    for i in range(2):
        j = indices[i] + 2
        # if j > length - 1:
        #    j = length - j
        if np.all(points[j % length] == points[indices[i - 1] - 2]):
            return tuple(points[j % length])


def find_tail_rect(points, convex_hull):
    length = len(points)
    indices = np.setdiff1d(range(length), convex_hull)
    direction = None
    for i in range(2):
        j = indices[i] + 2
        if j > length - 1:
            j = length - j
        if np.all(points[j] == points[indices[i - 1] - 2]):
            # rospy.loginfo( "diff: "+ str( abs(abs(points[(indices[i-1]+1)%length]- points[indices[i-1]]) - abs(points[indices[i]]- points[indices[i]-1]))/abs(points[(indices[i-1]+1)%length]- points[indices[i-1]]) ))
            if np.all(
                abs(
                    abs(points[(indices[i - 1] + 1) % length] - points[indices[i - 1]])
                    - abs(points[indices[i]] - points[indices[i] - 1])
                )
                < 5
            ):  # Check if tails is nearly a rectangle#TODO change 5 to something relative to area
                if points[indices[i] - 1][0] < points[indices[i]][0]:
                    rospy.loginfo("Right")
                    direction = 1  # TODO : Add respective rect pts in order
                else:
                    rospy.loginfo("Left")
                    direction = 0
                return (
                    np.array(
                        (
                            points[(indices[i - 1] + 1) % length],
                            points[indices[i - 1]],
                            points[indices[i]],
                            points[indices[i] - 1],
                        )
                    ),
                    direction,
                )
    return None, None


def arrow_test():
    img = cv2.imread("/home/khush/Downloads/frame5.jpg")

    contours, _ = cv2.findContours(
        preprocess(img), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )[-2:]
    cv2.imshow("Image", preprocess(img))
    cv2.waitKey(0)
    for cnt in contours:
        if cv2.contourArea(cnt) < 180:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
        hull = cv2.convexHull(approx, returnPoints=False)
        sides = len(hull)

        if 6 > sides > 3 and sides + 2 == len(approx):
            arrow_tip = find_tip(approx[:, 0, :], hull.squeeze())
            if arrow_tip:
                cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
                cv2.circle(img, arrow_tip, 3, (0, 0, 255), cv2.FILLED)
            rect, _ = find_tail_rect(approx[:, 0, :], hull.squeeze())
            if rect is not None:
                # cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
                cv2.polylines(img, [rect], True, (0, 0, 255), 2)
    cv2.imshow("Image", img)
    cv2.waitKey(0)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Closing")
