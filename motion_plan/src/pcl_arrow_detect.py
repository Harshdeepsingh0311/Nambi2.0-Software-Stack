#!/usr/bin/env python

import numpy as np
import rospy
import rospkg
import cv2
import open3d as o3d
from image_geometry import PinholeCameraModel
from ctypes import *  # convert float to uint32
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from scipy.spatial import cKDTree
import ros_numpy
import message_filters
from numpy import nan
import ConeDetection.detect as cd

OFFSET = 0.9
HORZ_OFFSET = 0.5
COLOR_OFFSET = 0
CANNY_THRES = 250
CONE_THRESH=0.7

path = rospkg.RosPack().get_path("motion_plan")

class ArrowDetector:

    def __init__(self, ros_cloud="/mrt/camera/depth/color/points",
                 ros_image="/mrt/camera/color/image_raw",
                 depth_topic='/mrt/camera/aligned_depth_to_color/image_raw',
                 info_topic="/mrt/camera/color/camera_info", visualize=False):
        self.br = CvBridge()
        self.detector = cd.Detector()
        self.visualize=visualize
        self.pcd = None
        self.lagging_pcd = None
        self.lagging_stamp = None
        self.lagging_depth = None
        self.info_sub = rospy.Subscriber(
            info_topic, CameraInfo, self.info_callback
        )
        rospy.wait_for_message(info_topic, CameraInfo, timeout=5)

        image_sub = message_filters.Subscriber(ros_image, Image)
        #rospy.Subscriber(ros_image, Image, self.cam_callback)
        rospy.wait_for_message(ros_image, Image, timeout=5)
        if self.visualize:
            cloud_sub = message_filters.Subscriber(ros_cloud, PointCloud2)
        #rospy.Subscriber(ros_cloud, PointCloud2, self.pc_callback)
            rospy.wait_for_message(ros_cloud, PointCloud2, timeout=10)
        depthim_sub=message_filters.Subscriber(depth_topic,Image)
        #rospy.Subscriber(depth_topic, Image, self.depth_callback)
        rospy.wait_for_message(depth_topic, Image, timeout=5)
        if self.visualize:
            sync_sub = message_filters.ApproximateTimeSynchronizer([image_sub,depthim_sub,cloud_sub], 15, 0.5)
            sync_sub.registerCallback(self.depth_cam_callback_cloud)
        else:
            sync_sub = message_filters.ApproximateTimeSynchronizer([image_sub,depthim_sub], 10, 0.1)
            sync_sub.registerCallback(self.depth_cam_callback)
        # save for unregistering

        #rospy.sleep(1)
        rospy.Rate(5).sleep()
        # rospy.spin()


    def depth_cam_callback_cloud(self, img_data,depth_im,cloud_data):
        current_frame = self.br.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        self.frame = current_frame
        self.depth_im=self.br.imgmsg_to_cv2(depth_im)
        #resized_frame=cv2.resize(self.frame, self.depth_im.shape)
        #self.timestamp = cloud_data.header.stamp
        self.timestamp=img_data.header.stamp
        depth_o3d = o3d.geometry.Image(self.depth_im.astype(np.float32))
        #color_o3d = o3d.geometry.Image(resized_frame.astype(np.uint8))
        intrinsics = o3d.camera.PinholeCameraIntrinsic(self.frame.shape[0],self.frame.shape[1],(self.K[0]), (self.K[4]), (self.K[2]), (self.K[5]))
        #rgbd_im = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d,depth_o3d,convert_rgb_to_intensity=False)
        self.pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_o3d, intrinsics )


        #self.numpy_pcd = np.asarray(self.pcd.points)

        # print('pcd height: ',cloud_data.height, 'width: ', cloud_data.width )
        self.roscloud = cloud_data
        self.pc = convertCloudFromRosToOpen3d(cloud_data)
        #self.pcd_width = cloud_data.width

        # For unordered, height = 1


    def depth_cam_callback(self, img_data,depth_im):
        current_frame = self.br.imgmsg_to_cv2(img_data)
        self.frame = current_frame
        self.depth_im=self.br.imgmsg_to_cv2(depth_im)
        #resized_frame=cv2.resize(self.frame, self.depth_im.shape)
        #self.timestamp = cloud_data.header.stamp
        self.timestamp=img_data.header.stamp
        depth_o3d = o3d.geometry.Image(self.depth_im.astype(np.float32))
        #color_o3d = o3d.geometry.Image(resized_frame.astype(np.uint8))
        intrinsics = o3d.camera.PinholeCameraIntrinsic(self.frame.shape[0],self.frame.shape[1],(self.K[0]), (self.K[4]), (self.K[2]), (self.K[5]))
        #rgbd_im = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d,depth_o3d,convert_rgb_to_intensity=False)
        self.pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_o3d, intrinsics )


        #self.numpy_pcd = np.asarray(self.pcd.points)

        # print('pcd height: ',cloud_data.height, 'width: ', cloud_data.width )
        #self.roscloud = cloud_data
        #self.pc = convertCloudFromRosToOpen3d(cloud_data)
        #self.pcd_width = cloud_data.width

        # For unordered, height = 1

    def get_depth(self, x, y):
        # print(f"x: {x}, y: {y}")
        # numpy_pcd = ros_numpy.point_cloud2.get_xyz_points(ros_numpy.point_cloud2.pointcloud2_to_array(self.roscloud), remove_nans=True, dtype=np.float32)
        # points = numpy_pcd[:,0:2]
        # tree = cKDTree(points)
        # idx = tree.query((x,y))[1]
        # depth = numpy_pcd[idx,2]
        # pt = numpy_pcd[idx]
        # print(f"\npcl: {pt}")
        #return pt#depth#next(gen)[0]
        #print(self.depth_im.shape,self.frame.shape)
        depth = self.lagging_depth[int(y)][int(x)]
        #print(self.lagging_depth[int(y//2)-1:int(y//2)+2][int(x//2)-1:int(x//2)+2])

        return depth/1000

    def info_callback(self, data):
        self.model = PinholeCameraModel()
        self.model.fromCameraInfo(data)
        self.K = data.K
        self.info_sub.unregister()  # Only subscribe once

    def pixel_to_3d(self, x, y,cone=False):
        """Converts an image pixel into 3D point from Pointcloud

        :returns: np.ndarray
        """
        # print(f'getting {x}, {y} pixel coordinates')
        # print(depth)
        ray = self.model.projectPixelTo3dRay(
            (x, y)
        )  # get 3d ray of unit length through desired pixel
        ray_z = [
            el / ray[2] for el in ray
        ]  # normalize the ray so its Z-component equals 1.0
        # print(ray_z)
        #pcl = self.get_depth(ray_z[0],ray_z[1])
        # pcl2 = self.get_depth(ray[0],ray[1])
        #depth = pcl[2]
        depth=self.get_depth(x,y)
        pt = [
            el * depth for el in ray_z
        ]  # multiply the ray by the depth; its Z-component should now equal the depth value
        # ray_z[2]=depth

        # print(f"3D pt: {pt}, ray_z: {ray_z}")
        # return self.old_pixel_to_3d(x,y),pcl
        # print("reconstructed pixel:", self.model.project3dToPixel((pt[0], pt[1], pt[2])),' actual pixel ',(x,y))
        # assert math.dist(self.model.project3dToPixel((pt[0], pt[1], pt[2])), [x,y]) < 5
        # return pt, ray_z
        if cone:
            return ray,pt
        return pt

    def old_pixel_to_3d(self, x ,y):
        print(f'getting {x}, {y} pixel coordinates')
        gen = pc2.read_points(self.roscloud, field_names="z", skip_nans=True, uvs=[(x,y)])
        depth = next(gen)[0]
        print("depth: ",depth)
        ray = self.model.projectPixelTo3dRay(
            (x, y)
        )  # get 3d ray of unit length through desired pixel
        ray_z = [
            el / ray[2] for el in ray
        ]
        pt = [
            el * depth for el in ray_z
        ]  # multiply the ray by the depth; its Z-component should now equal the depth value
        print(f"old fn: {pt}")
        return pt

    def get_orientation(self, corners, z=None, visualize=False):
        # print("corners", corners)
        if z is None:
            z = np.median(np.asarray(pcd.points), axis=0)[2]  # get median z TODO
        pcd = self.crop_pcd(corners[:4], z=z)
        # TODO add this to completed?
        # ps.append(downpcd.get_centre())

        if visualize:
            lines = [[0, 1], [1, 2], [2, 3], [3, 0]]
            colors = [[1, 0, 0] for i in range(len(lines))]
            line_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(corners),
                lines=o3d.utility.Vector2iVector(lines),
            )
            line_set.colors = o3d.utility.Vector3dVector(colors)
            pcd.paint_uniform_color([1.0, 0, 0])
            o3d.visualization.draw_geometries(
                [pcd, self.pc, line_set],
                zoom=0.1,
                front=[-0.016, -0.22, -1.0],
                lookat=[0.27, 0.4, 2.3],
                up=[0.0048, -1.0, 0.22],
            )

        # downpcd = pcd.voxel_down_sample(voxel_size=0.05)
        # theta.append(-40 + (centroid_x*80)/width)
        # downpcd.estimate_normals(
        #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        # o3d.visualization.draw_geometries([downpcd],
        #                                   zoom=0.3412,
        #                                   front=[0.4257, -0.2125, -0.8795],
        #                                   lookat=[2.6172, 2.0475, 1.532],
        #                                   up=[-0.0694, -0.9768, 0.2024],
        #                                   point_show_normal=True)

        try:
            plane_model, _ = pcd.segment_plane(
                distance_threshold=0.01, ransac_n=3, num_iterations=500
            )
        except:
            return 0
        [a, b, c, d] = plane_model
        a, b, c, d = (-a, -b, -c, -d) if d > 0 else (a, b, c, d)
        # print("abcd:", a,b,c,d)
        # print(np.arcsin(-a))
        theta = np.arcsin(-a)

        return theta

    def crop_pcd(self, corners, z):
        """Accepts 3D points and returns a cropped pcd

        :corners: shape (n,3)
        """
        corners = np.array(corners)

        # Convert the corners array to have type float64
        bounding_polygon = corners.astype("float64")

        # Create a SelectionPolygonVolume
        vol = o3d.visualization.SelectionPolygonVolume()

        # You need to specify what axis to orient the polygon to.
        # I choose the "Z" axis. I made the max value the maximum Z of
        # the polygon vertices and the min value the minimum Z of the
        # polygon vertices.
        vol.orthogonal_axis = "Z"
        vol.axis_max = max(z,np.max(bounding_polygon[:, 2])) + 0.2
        vol.axis_min = min(z,np.min(bounding_polygon[:, 2])) - 0.2

        # Set all the Z values to 0 (they aren't needed since we specified what they
        # should be using just vol.axis_max and vol.axis_min).
        bounding_polygon[:, 2] = 0

        # Convert the np.array to a Vector3dVector
        vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)

        # Crop the point cloud using the Vector3dVector
        cropped_pcd = vol.crop_point_cloud(self.lagging_pcd)

        # Get a nice looking bounding box to display around the newly cropped point cloud
        # (This part is optional and just for display purposes)
        # bounding_box = cropped_pcd.get_axis_aligned_bounding_box()
        # bounding_box.color = (1, 0, 0)

        # Draw the newly cropped PCD and bounding box
        # o3d.visualization.draw_geometries([cropped_pcd, bounding_box],
        #                                   zoom=2,
        #                                   front=[5, -2, 0.5],
        #                                   lookat=[7.67473496, -3.24231903,  0.3062945],
        #                                   up=[1.0, 0.0, 0.0])
        return cropped_pcd
    def cone_detect(self,visualize=False):

        img=self.frame.copy()
        xyxy, bb_img, conf = self.detector.run_on_img(img)
        print(conf)
        if len(conf) == 0:
            return False, None, None
        idx=np.argmax(conf)
        cone_bb=xyxy[idx]
        if conf[idx]>CONE_THRESH:
            found=True
            print("Cone Detected")
        else:
            return False,None,None
        im_x=(cone_bb[0]+cone_bb[2])/2
        im_y=(cone_bb[1]+cone_bb[3])/2
        ray,pos_tup=self.pixel_to_3d(im_x,im_y,cone=True)
        x,y,z=pos_tup
        cone_distance=z
        x,y,z=z,-x,y
        pos=(x,y)
        if visualize:
            corners=[[im_x-5,im_y-5,cone_distance],[im_x-5,im_y+5,cone_distance],[im_x+5,im_y-5,cone_distance],[im_x+5,im_y+5,cone_distance],
            [cone_bb[0],cone_bb[1],cone_distance],[cone_bb[0],cone_bb[3],cone_distance],[cone_bb[2],cone_bb[3],cone_distance],[cone_bb[2],cone_bb[1],cone_distance]]
            lines = [[0, 1], [1, 2], [2, 3], [3, 0],[4,5],[5,6],[6,7],[7,4]]
            colors = [[1, 0, 0] for i in range(len(lines))]
            line_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(corners),
                lines=o3d.utility.Vector2iVector(lines),
            )
            line_set.colors = o3d.utility.Vector3dVector(colors)
            self.lagging_pcd.paint_uniform_color([1.0, 0, 0])
            o3d.visualization.draw_geometries(
                [self.lagging_pcd, self.pc, line_set],
                zoom=0.1,
                front=[-0.016, -0.22, -1.0],
                lookat=[0.27, 0.4, 2.3],
                up=[0.0048, -1.0, 0.22],
            )
            cv2.imshow('Cone',bb_img)
            cv2.waitKey(0)
        if cone_distance==0 or cone_distance==nan:
            return found,ray,cone_distance
        return found,pos,cone_distance


    def arrow_detect(self, far=True, visualize=False):
        # Arrow detection
        img = self.frame.copy()
        #self.lagging_pcd = o3d.geometry.PointCloud(self.pcd)
        self.lagging_pcd=self.pcd
        self.lagging_depth = self.depth_im
        self.lagging_stamp = self.timestamp
        # print("img dim: ", img.shape)
        orig_img = img.copy()
        found = False
        orient = None
        pos = None
        direction = None
        bounding_box = None
        contours, _ = cv2.findContours(
            preprocess(img), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )[-2:]
        if visualize:
            cv2.imshow("Image", preprocess(img))
            cv2.waitKey(0)
        # template = cv2.imread("arrow.jpeg")
        max_cnt_area = -1
        for cnt in contours:
            cnt_area = cv2.contourArea(cnt)
            # filtering using area
            if cnt_area < 60:
                continue
            #print(cnt_area)

            # filtering using color of arrow
            arrow_mask = np.zeros(img.shape[:2], np.uint8)
            cv2.drawContours(arrow_mask, [cnt], -1, 255, -1)
            cnt_mean = np.array(cv2.mean(img, mask=arrow_mask)[:3])
            norm_mean = cnt_mean/np.linalg.norm(cnt_mean)
            unit_vec = np.array([1, 1, 1])/np.sqrt(3)
            # print("cosine dist: ", np.dot(norm_mean, unit_vec))
            if np.dot(norm_mean, unit_vec) < OFFSET or np.mean(cnt_mean)>255/2-COLOR_OFFSET:
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
            hull = cv2.convexHull(approx, returnPoints=False)
            sides = len(hull)

            if (sides == 5 or sides == 4) and sides + 2 == len(approx):
                arrow_tip, _ = find_tip(approx[:, 0, :], hull.squeeze())
                rect, dirct = find_tail_rect(approx[:, 0, :], hull.squeeze())
                if arrow_tip and rect is not None:
                    # cv2.polylines(img, [rect],  True, (0, 0, 255), 2)
                    arrow_tail = tuple(
                        np.average([rect[0], rect[3]], axis=0).astype(int)
                    )
                    if (
                        arrow_tail[0] - arrow_tip[0] == 0
                    ):  # to avoid division by 0 in next step
                        continue
                    # print(
                    #     "tip-tail tan angle: ",
                    #     abs(
                    #         float(arrow_tail[1] - arrow_tip[1])
                    #         / (arrow_tail[0] - arrow_tip[0])
                    #     ),
                    # )
                    # Check that tan of angle of the arrow in the image from horizontal is less than 0.2(we are expecting nearly horizontal arrows)(atan(0.2) = 11.31)
                    if (
                        abs(
                            float(arrow_tail[1] - arrow_tip[1])
                            / (arrow_tail[0] - arrow_tip[0])
                        )
                        > HORZ_OFFSET
                    ):
                        continue  # Discard it, not a horizontal arrow
                    ##cv2.circle(img, arrow_tail, 3, (0, 0, 255), cv2.FILLED)
                    ##cv2.circle(img, tuple(np.average([arrow_tail, arrow_tip], axis=0).astype(int)), 3, (0, 0, 255), cv2.FILLED)#arrow centre
                    # theta = -(np.average([arrow_tail[0], arrow_tip[0]])/(np.shape(img)[0]) - 0.5)*45*2#linear estimate, assuming camera horizontal range from -45 to 45
                    ##theta not needed using pcd
                    # In case of multiple arrows, max area cnt is taken
                    if max_cnt_area < cnt_area:
                        tmp_bb = cv2.boundingRect(cnt)
                        x, y, w, h = tmp_bb
                        # filtering using color of background
                        background_mask = np.zeros(img.shape[:2], np.uint8)
                        bb_cnt = np.array([np.array([[x+w, y]]), np.array([[x+w, y+h]]), np.array([[x, y+h]]), np.array([[x, y]])])
                        cv2.drawContours(background_mask, [bb_cnt], -1, 255, -1)
                        background_mask = background_mask - arrow_mask
                        if visualize:
                            cv2.imshow("Image Bg 1", background_mask)
                            cv2.waitKey(0)

                        cnt_mean = np.array(cv2.mean(img, mask=background_mask)[:3])
                        norm_mean = cnt_mean/np.linalg.norm(cnt_mean)
                        unit_vec = np.array([1, 1, 1])/np.sqrt(3)
                        # print("cosine dist2: ", np.dot(norm_mean, unit_vec))
                        if np.dot(norm_mean, unit_vec) < OFFSET or np.mean(cnt_mean) < 255/2 + COLOR_OFFSET:
                            continue
                        direction = dirct
                        max_cnt_area = cnt_area
                        bounding_box = tmp_bb

                    found = True

                    # print(bounding_box)
                    cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
                    cv2.drawContours(img, [approx], -1, (0, 150, 155), 2)
                    cv2.circle(img, arrow_tip, 3, (0, 0, 255), cv2.FILLED)
                    print("arrow_x_img: " + str(np.average(rect, axis=0)[0]))

        # if direction is not None and far == False:
        #     new_img = orig_img[
        #      int(bounding_box[1])-10: int(bounding_box[1]+bounding_box[3]+10),
        #      int(bounding_box[0])-10:int(bounding_box[0]+bounding_box[2]+10)]
        #     train_pts = get_arrow_arr(new_img, False)
        #     if train_pts is None:
        #         rospy.loginfo("not found in close up")
        #         return False, None, None
        #     new_train_pts = []
        #     for i, pt in enumerate(train_pts):
        #         new_pt = [pt[0] + int(bounding_box[0])-10, pt[1] + int(bounding_box[1])-10]
        #         new_train_pts.append(new_pt)
        #     train_pts = np.array(new_train_pts)
        #     new_img = orig_img.copy()
        #     query_pts = np.array([[663, 197],
        #                    [476, 326],
        #                    [474, 234],
        #                    [ 31, 232],
        #                    [ 30, 162],
        #                    [473, 162],
        #                    [476,  69]])#get_arrow_arr(template, False)
        #     matrix, mask = cv2.findHomography(query_pts, train_pts, 0, 5.0)
        #     mat_inv = np.linalg.inv(matrix)
        #     h,w = 416, 686#template.shape
        #     pts = np.float32([ [10,10],[10,h-10],[w-10,h-10],[w-10,10] ]).reshape(-1,1,2)# + [[320, 223]]
        #     # print(pts)
        #     dst = cv2.perspectiveTransform(pts, matrix)
        #     homography = cv2.polylines(new_img, [np.int32(dst)], True, (255, 0, 0), 3)
        #     cam_mat = np.array([[480.0, 0, 400],
        #                         [0, 465.0, 400],
        #                         [0, 0, 1]])
        #     axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        #        [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])*50
        #     # axis = np.float32([[1,0,0], [0,1,0], [0,0,-1]]).reshape(-1,3)/10
        #     axes_img = new_img.copy()
        #     ret,rvecs, tvecs = cv2.solvePnP(np.c_[query_pts, np.zeros(7)].astype(np.float32), train_pts.astype(np.float32), cam_mat, 0)
        #     r_mtx,_ = cv2.Rodrigues(rvecs)
        #     pm = cam_mat.dot(np.c_[r_mtx, tvecs])
        #     ea = cv2.decomposeProjectionMatrix(pm)[-1]
        #     print(ea)#euler angles
        #     imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, cam_mat.astype(np.float32), 0)
        #     axes_img = draw(axes_img,train_pts[2:],imgpts)
        #     img = axes_img
        #     # cv2.imshow('axes img',axes_img)
        #     # k = cv2.waitKey(0) & 0xFF
        #     orient = ea[1]
        #     # if abs(abs(ea[0]) - 180) < 30:
        #     #   orient = -orient
        #     # cv2.imshow("Homography", homography)
        #     # cv2.waitKey(0)

        if direction is not None:
            x, y, w, h = bounding_box
            if far:
                delta=1
            else:
                delta=2
            corners = [
                self.pixel_to_3d(im_x, im_y)
                for im_x, im_y in [(x+delta, y+delta), (x + w-delta, y+delta), (x + w-delta, y + h-delta), (x+delta, y + h-delta)]
            ]
            temp=self.lagging_depth.copy()
            temp=cv2.cvtColor(temp,cv2.COLOR_GRAY2RGB)
            cv2.rectangle(temp,(x,y),((x+w),(y+h)),color=(0, 0, 255),thickness=3)
            if visualize:
                cv2.imshow('depth corners',temp)
                cv2.waitKey(0)
            im_x, im_y = x, y
            X, Y, Z = [],[],[]
            for i in np.random.randint(w/2,size=3):
                for j in np.random.randint(h/2,size=3):
                    x, y, z = self.pixel_to_3d(int(im_x + i + w/4), int(im_y + j + h/4))
                    X.append(x)
                    Y.append(y)
                    Z.append(z)
            x, y, z = np.nanmedian(X), np.nanmedian(Y), np.nanmedian(Z)
            x, y, z = z, -x, y
            # print("x,y,z: ", x, y, z)
            pos = x, y, z
            # print("Point",points)
            print("Corners 3d",corners)

            if far == False:
                orient = self.get_orientation(corners,z=z, visualize=visualize) * 180 / np.pi
                # print(orient)
        if far == True:
            orient = 0
        if direction is not None:
            if direction == 1:  # Right
                orient = orient - 90
            elif direction == 0:  # Left
                orient = 90 + orient
            else:
                print("error: direction not found and not None, " + str(direction))
                found = False
        ret_stamp=None
        if found:
            # global path #motion_plan pkg dir
            rospy.loginfo(str(["arrow found!", pos, orient]))
            ret_stamp=self.lagging_stamp
        if visualize:
            cv2.imshow("frame", img)
            cv2.waitKey(0)
        # return found, theta, orient   #theta and orient wrt forward direction, in degree
        # cv2.imwrite(path + "/src/arrow_frames/Arrow_detection@t="+str(rospy.Time.now())+".png", img)
        # cv2.imwrite(path + "/src/arrow_frames/og@t="+str(rospy.Time.now())+".png", self.frame)
        return found, pos, orient, ret_stamp




### open3d - ROS pointcloud conversion
### code from https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/blob/master/lib_cloud_conversion_between_Open3D_and_ROS.py
### for more info check the above repo

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + [
    PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1)
]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00FF0000) >> 16,
    (rgb_uint32 & 0x0000FF00) >> 8,
    (rgb_uint32 & 0x000000FF),
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points = np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors:  # XYZ only
        fields = FIELDS_XYZ
        cloud_data = points
    else:  # XYZ + RGB
        fields = FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors) * 255)  # nx3 matrix
        colors = colors[:, 0] * BIT_MOVE_16 + colors[:, 1] * BIT_MOVE_8 + colors[:, 2]
        cloud_data = np.c_[points, colors]

    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)


def convertCloudFromRosToOpen3d(ros_cloud):
    # Get cloud data from ros_cloud
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(
        pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names)
    )

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

        # Get xyz
        xyz = [
            (x, y, z) for x, y, z, rgb in cloud_data
        ]  # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if (
            type(cloud_data[0][IDX_RGB_IN_FIELD]) == float
        ):  # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb) / 255.0)
    else:
        xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud



# ____________________________________ Image Processing


def preprocess(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_equalized = cv2.equalizeHist(img_gray)

    _, img_thres = cv2.threshold(img_equalized, 70, 255, cv2.THRESH_TOZERO)
    # img_blur = cv2.GaussianBlur(img_thres, (5, 5), 1)
    # img_blur = cv2.bilateralFilter(img_equalized, 5, 75, 75)
    kernel = np.ones((3, 3))
    # img_erode = cv2.erode(img_equalized, kernel, iterations=1)
    # img_dilate = cv2.dilate(img_erode, kernel, iterations=1)
    img_canny = cv2.Canny(img_thres, CANNY_THRES, CANNY_THRES)

    # cv2.imshow("Img equalized", img_equalized)
    # cv2.imshow("Img blur", img_blur)
    # cv2.imshow("Erode", img_erode)
    # cv2.imshow("Dilate", img_dilate)
    # cv2.imshow("Canny", img_canny)
    # cv2.imshow("Thres", img_thres)
    # cv2.waitKey(0)
    return img_canny

def find_tip(points, convex_hull):
    length = len(points)
    indices = np.setdiff1d(range(length), convex_hull)
    # print(indices, "convex_hull:",convex_hull,"points:", points)
    for i in range(2):
        j = indices[i] + 2
        # if j > length - 1:
        #    j = length - j
        if np.all(points[j % length] == points[indices[i - 1] - 2]):
            return tuple(points[j % length]), j % length
    return None, None


def find_tail_rect(points, convex_hull):
    length = len(points)
    indices = np.setdiff1d(range(length), convex_hull)
    direction = None
    for i in range(2):
        j = (indices[i] + 2) % length
        # if j > length - 1:
        #     j = length - j
        if np.all(points[j] == points[indices[i - 1] - 2]):
            sides = []  # length of sides of the tail rectangle
            prev_pt = points[(indices[i - 1] + 1) % length]
            for pt in (
                points[indices[i] - 1],
                points[indices[i]],
                points[indices[i - 1]],
                points[(indices[i - 1] + 1) % length],
            ):
                sides.append(np.linalg.norm(pt - prev_pt))
                prev_pt = pt

            if (
                abs(sides[0] - sides[2]) / float(max(sides[2], sides[0])) < 0.5
                and abs(sides[1] - sides[3]) / float(max(sides[1], sides[3])) < 0.15
            ):
                # if np.all(abs(abs(points[(indices[i-1]+1)%length]- points[indices[i-1]]) - abs(points[indices[i]]- points[indices[i]-1])) < 5):#Check if tails is nearly a rectangle#TODO change 5 to something relative to area
                if points[indices[i] - 1][0] < points[indices[i]][0]:
                    print("Right")
                    direction = 1  # TODO : Add respective rect pts in order
                else:
                    print("Left")
                    direction = 0
                if points[indices[i - 1]][1] < points[indices[i]][1]:
                    # print("here")
                    return (
                        np.array(
                            (
                                points[indices[i] - 1],
                                points[indices[i]],
                                points[indices[i - 1]],
                                points[(indices[i - 1] + 1) % length],
                            )
                        ),
                        direction,
                    )
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


def correct_corners(points, corners):
    new_points = []
    for n, pt in enumerate(points):
        err = (
            5 if not n in [3, 4] else 0
        )  # int(2*np.linalg.norm(points[3]-points[4])/5)
        if err == 0:
            new_points.append(pt)
            continue
        new_pt = corners[np.argmin([np.linalg.norm(corner - pt) for corner in corners])]
        # print(np.linalg.norm(new_pt - pt))
        new_pt = new_pt if np.linalg.norm(new_pt - pt) < err else pt
        new_points.append(new_pt)
    return np.array(new_points)


# def draw(img, corners, imgpts):
#     corner = tuple(corners[0].ravel())
#     img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 3)
#     img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 3)
#     img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 3)
#     return img
def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1, 2)
    # draw ground floor in green
    img = cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)
    # draw pillars in blue color
    for i, j in zip(range(4), range(4, 8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)
    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 3)
    return img


def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result, rot_mat


def get_arrow_arr(img, debug=True):
    if debug:
        cv2.imshow("Image", img)
        cv2.waitKey(0)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, img_thres = cv2.threshold(img_gray, 120, 255, cv2.THRESH_OTSU)
    img_blur = cv2.GaussianBlur(img_thres, (5, 5), 1)
    img = cv2.bilateralFilter(img_thres, 5, 75, 75)
    contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    if debug:
        cv2.imshow("Image", img)
        cv2.waitKey(0)
    for cnt in contours:
        if cv2.contourArea(cnt) < 200:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
        hull = cv2.convexHull(approx, returnPoints=False)
        sides = len(hull)

        if (sides == 5 or sides == 4) and sides + 2 == len(approx):
            if debug:
                img_tmp = img.copy()
                # cv2.drawContours(img_tmp, [cnt], -1, (0, 25, 0), 1)
                cv2.drawContours(img_tmp, [approx], -1, (100), 1)
                cv2.imshow("contour", img_tmp)
                cv2.waitKey(0)
            arrow_tip, tip_idx = find_tip(approx[:, 0, :], hull.squeeze())
            if arrow_tip is None:
                continue
            points = np.roll(approx[:, 0, :], -tip_idx)
            if points[1][1] < arrow_tip[1]:
                points = np.flipud(np.roll(points, -1, axis=0))  # for uniformity
            # print(np.uint8(np.average(points, axis=0)))
            # img_inv = cv2.bitwise_not(img)
            h, w = img.shape[:2]
            mask1 = np.zeros((h + 2, w + 2), np.uint8)
            ret, _, mask1, _ = cv2.floodFill(
                cv2.erode(img.copy(), np.ones((3, 3), np.uint8)),
                mask1,
                tuple(np.uint8(np.average(points, axis=0))),
                255,
                flags=cv2.FLOODFILL_MASK_ONLY,
            )  # line 27
            # cv2.imshow("mask",mask1*200)
            # print(mask1.shape, img.shape)
            mask1 = mask1[1:-1, 1:-1]
            # mask_inv=cv2.bitwise_not(mask1)
            # masked_img = cv2.bitwise_and(img, img, mask=mask1)
            # cv2.imshow("masked",masked_img)
            # cv2.waitKey()
            # print(mask1.shape, img.shape)

            corners = cv2.goodFeaturesToTrack(img, 25, 0.0001, 10, mask=mask1).reshape(
                -1, 2
            )
            corners2 = [[-1], [-1], [-1], [-1]]
            max_vals = [-1e5, -1e5, -1e5, -1e5]  # x+y, x-y, y-x, -y-x
            lim = int(np.floor(2 * np.linalg.norm(points[3] - points[4]) / 3))
            lim = min(lim, 10)
            direction = (points[0] - points[1])[0] > 0  # left = 0, right = 1
            for i in range(-lim, lim):
                for j in range(-lim, lim):
                    x, y = points[3] + [i, j]
                    if img[y, x] == 255 or mask1[y, x] == 0:
                        continue
                    for k, fn in enumerate(
                        [
                            lambda x, y: x + y,
                            lambda x, y: x - y,
                            lambda x, y: y - x,
                            lambda x, y: -x - y,
                        ]
                    ):
                        if fn(x, y) > max_vals[k]:
                            max_vals[k] = fn(x, y)
                            corners2[k] = x, y
            # print(mask1[points[3][1]-9:points[3][1]+9, points[3][0]-9:points[3][0]+9])
            points[3] = (
                corners2[2] if direction else corners2[0]
            )  # corners2[np.argmin([np.linalg.norm(corner- points[3]) for corner in corners2])]
            corners2 = [[-1], [-1], [-1], [-1]]
            max_vals = [-1e5, -1e5, -1e5, -1e5]  # x+y, x-y, y-x, -y-x
            for i in range(-lim, lim):
                for j in range(-lim, lim):
                    x, y = points[4] + [i, j]

                    if img[y, x] == 255 or mask1[y, x] == 0:
                        continue
                    for k, fn in enumerate(
                        [
                            lambda x, y: x + y,
                            lambda x, y: x - y,
                            lambda x, y: y - x,
                            lambda x, y: -x - y,
                        ]
                    ):
                        if fn(x, y) > max_vals[k]:
                            max_vals[k] = fn(x, y)
                            corners2[k] = x, y
            # print(mask1[points[3][1]-9:points[3][1]+9, points[3][0]-9:points[3][0]+9])
            points[4] = (
                corners2[3] if direction else corners2[1]
            )  # corners2[np.argmin([np.linalg.norm(corner- points[4]) for corner in corners2])]

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            corners = cv2.cornerSubPix(
                img, np.float32(corners), (3, 3), (-1, -1), criteria
            )
            # corners = centroids
            # print(len(corners))
            corners = np.uint8(corners)
            if debug:
                img_tmp = img.copy()
                for corner in corners:
                    cv2.circle(img_tmp, tuple(corner), 3, (125), cv2.FILLED)
                cv2.imshow("corners", img_tmp)
                cv2.waitKey(0)
            points = correct_corners(points, corners)
            if debug:
                img_tmp = img.copy()
                for n, i in enumerate(points):
                    cv2.circle(img_tmp, tuple(i), 3, (125), cv2.FILLED)
                cv2.imshow(str(n) + "th point", img_tmp)
                cv2.waitKey(0)

            return points


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
        rospy.init_node('pcd_checker')
        checker = ArrowDetector(visualize=True)
        rate = rospy.Rate(2)
        rospy.wait_for_message("/mrt/camera/color/image_raw", Image, timeout=10)
        rospy.wait_for_message("/mrt/camera/depth/color/points", PointCloud2, timeout=10)
        rospy.sleep(5)
        while not rospy.is_shutdown():
            found, pos, orient, timestamp = checker.arrow_detect(far=False, visualize=True)
            print("Found: ", found)
            found,val,cone_dist=checker.cone_detect(visualize=True)
            rate.sleep()
    except rospy.ROSInterruptException:
        print("Closing")

