#!/usr/bin/env python

import sys, rospkg, os
import importlib

path = rospkg.RosPack().get_path("motion_plan")
sys.path.append(path + "/src")
import functions

# from functions import *
import rospy
from cv2 import destroyAllWindows

my_client = functions.client()


def main_node():

    rate = rospy.Rate(5)
    prev_x, prev_y, prev_q = my_client.bot_to_map(
        0, 0, (0, 0, 0, 1)
    )  # prev always in map frame
    olat, olon = my_client.olat, my_client.olon

    # = (0,0,0,1)#TODO change prev here to initial bot location which may not be 0
    # my_client.move_to_goal(0.5,0)
    # x,y,q =  1,0,(0,0,np.sin(np.pi/8), np.cos(np.pi/8))
    # print(x,y,q)
    # my_client.move_to_off_goal(*my_client.bot_to_map(x,y,q))
    # print(x,y,q)
    i = 1  # iteration for moving just ahead of the previous goal

    gps_goal_lat, gps_goal_lon = my_client.xy2gps(0, 8)
    gps_goals(False, gps_goal_lat, gps_goal_lon)
    gps_goal_lat, gps_goal_lon = my_client.xy2gps(-10, -3)
    gps_goals(False, gps_goal_lat, gps_goal_lon)


def gps_goals(high_res, lat, lon):
    if high_res:
        success = my_client.move_to_off_goal_gps(lat, lon)
        if success:
            rospy.loginfo("Reached Post")
        else:
            rospy.loginfo("Failed")

    else:
        while not rospy.is_shutdown():
            # AR Tag Detection
            (
                found,
                theta,
            ) = (
                my_client.ar_detect()
            )  # theta and orient wrt forward direction, in degree
            # single post
            if found == 1:
                posx, posy = my_client.find_obs_lidar(theta[0])
                if posx is None:
                    rospy.loginfo(
                        "AR Tag detected but not found in LIDAR. Check width/error/arrow detection"
                    )
                    found = False

                else:
                    q = (
                        0,
                        0,
                        0,
                        1,
                    )  # (0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
                    # TODO Add a check if arrow found is nearly in the direction of the previous arrow(or add a warning if it is)
                    posx, posy, q = my_client.bot_to_map(posx, posy, q)  # map frame
                    # rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))

                    # move to final positions using AR Tag
                    success = my_client.move_to_goal(
                        my_client.find_off_goal(
                            posx, posy, q=(0, 0, 0, 1), frame="map", offset=(1, 0, 0, 0)
                        )
                    )
                    if success == True:
                        # my_client.add_arrow(*my_client.bot_to_map(posx, posy, q), color=(0,1,1))
                        # prev_x, prev_y, prev_q = posx, posy, q#map frame
                        # my_client.add_arrow(prev_x, prev_y, prev_q, (1,0,1))
                        # my_client.add_to_completed(posx, posy, q)
                        rospy.loginfo("Reached Post")
                        # Flash Green LED
                        return True
                    else:
                        rospy.loginfo("Failed goal: " + str((posx, posy, q)))
                        return False  # TODO

            elif found == 2:
                # Gate post
                rospy.loginfo("Gate Post Traversal")
                posx1, posy1 = my_client.find_obs_lidar(theta[0])
                posx2, posy2 = my_client.find_obs_lidar(theta[1])
                if posx1 is None or posx2 is None:
                    rospy.loginfo(
                        "AR Tag detected but not found in LIDAR. Check width/error/arrow detection"
                    )
                    found = False

                else:
                    q = (
                        0,
                        0,
                        0,
                        1,
                    )  # (0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
                    # TODO Add a check if arrow found is nearly in the direction of the previous arrow(or add a warning if it is)
                    posx1, posy1, q = my_client.bot_to_map(posx1, posy1, q)  # map frame
                    posx2, posy2, q = my_client.bot_to_map(posx2, posy2, q)  # map frame

                    assert abs(norm([posx1 - posx2, posy1 - posy2]) - 2) <= 0.5

                    # rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))

                    # move to final positions using AR Tag
                    posx = (posx1 + posx2) / 2.0
                    posy = (posy1 + posy2) / 2.0

                    success = my_client.move_to_goal(posx, posy, q=q, frame="map")
                    if success == True:
                        # my_client.add_arrow(*my_client.bot_to_map(posx, posy, q), color=(0,1,1))
                        # prev_x, prev_y, prev_q = posx, posy, q#map frame
                        # my_client.add_arrow(prev_x, prev_y, prev_q, (1,0,1))
                        # my_client.add_to_completed(posx, posy, q)
                        rospy.loginfo("Reached Post")
                        return True
                        # Flash Green LED
                    else:
                        rospy.loginfo("Failed goal: " + str((posx, posy, q)))
                        return False

            else:
                my_client.send_goal_gps(lat, lon, frame="map")
                rospy.sleep(1)  # Sleep for 1-2s and let the bot move towards the goal

    # while not rospy.is_shutdown():
    #     if len(my_client.completed_list) == 5:
    #         rospy.loginfo("End Goal found")
    #         success = my_client.move_to_goal(12,6,q=(0,0,1/np.sqrt(2),1/np.sqrt(2)))
    #         if success:
    #             rospy.loginfo("Completed all goals")
    #         else:
    #             rospy.loginfo("Failed")
    #         break
    #     found, theta, orient = my_client.arrow_detect()#theta and orient wrt forward direction, in degree
    #     if found:
    #         posx,posy = my_client.find_obs_lidar(theta)
    #         if posx is None:
    #             rospy.loginfo("Arrow detected but not found in LIDAR. Check width/error/arrow detection")
    #             found = False

    #         else:
    #             q=(0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
    #             #TODO Add a check if arrow found is nearly in the direction of the previous arrow(or add a warning if it is)
    #             posx, posy, q = my_client.bot_to_map(posx, posy, q)#map frame
    #             if my_client.is_complete(posx,posy,q):
    #                 rospy.loginfo("Already visited recently found Goal: " + str([posx,posy]))
    #             else:
    #                 i = 1
    #                 #rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))
    #                 my_client.add_arrow(posx, posy, q, color=(0,1,0))#Add Rviz arrow marker, map frame
    #                 success = my_client.move_to_off_goal(posx,posy, q = q, frame = "map", off_dist = 1)
    #                 if success == True:
    #                     #my_client.add_arrow(*my_client.bot_to_map(posx, posy, q), color=(0,1,1))
    #                     prev_x, prev_y, prev_q = posx, posy, q#map frame
    #                     #my_client.add_arrow(prev_x, prev_y, prev_q, (1,0,1))
    #                     my_client.add_to_completed(posx, posy, q)
    #                 else:
    #                     rospy.loginfo("Failed goal: " + str((posx, posy, q)))
    #     if not found:
    #         nearby_goal = just_ahead(prev_x,prev_y, prev_q, off_dist= 0.5 + 0.6*i)
    #         my_client.send_goal(*nearby_goal,frame="map")
    #         rospy.sleep(1)#Sleep for 1-2s and let the bot move towards the goal
    #         i+=1
    #     rate.sleep()

    # Close down the video stream when done


if __name__ == "__main__":
    try:
        main_node()
    except rospy.ROSInterruptException:
        print("Exiting... ")
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise
