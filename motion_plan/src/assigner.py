#!/usr/bin/env python

from functions import *
import rospy
from cv2 import destroyAllWindows
import sys
from numpy import nan

def main_node():

    my_client = client()
    rate = rospy.Rate(5)
    prev_x, prev_y, prev_q = my_client.bot_to_map(
        0, 0, (0, 0, 0, 1)
    )  # prev always in map frame
    i = 1  # iteration for moving just ahead of the previous goal

    while not rospy.is_shutdown():
        if i > 8:
            found, pos, orient = my_client.recovery(far=True)
            i = 0
        if len(my_client.completed_list) == 5:
            rospy.loginfo("End Goal found")
            success = my_client.move_to_goal(
                12, 6, q=(0, 0, 1 / np.sqrt(2), 1 / np.sqrt(2))
            )
            if success:
                rospy.loginfo("Completed all goals")
            else:
                rospy.loginfo("Failed")
            break
        count = 0
        found, pos, orient, timestamp = my_client.arrow_detect(far=True)

        while not found and count < 15:
            count += 1
            found, pos, orient, timestamp = my_client.arrow_detect(far=True)
        if found:
            # TODO reduce conversions
            orient = orient + 90 if orient < 0 else orient - 90
            q = (
                0,
                0,
                np.sin(np.pi * orient / (2 * 180)),
                np.cos(np.pi * orient / (2 * 180)),
            )
            posx, posy, q = my_client.bot_to_map(
                pos[0], pos[1], q, timestamp=timestamp, frame="mrt/camera_link"
            )  # map frame
            if my_client.is_complete(posx, posy, q):
                rospy.loginfo(
                    "Already visited recently found Goal: " + str([posx, posy])
                )
                found = False
            else:
                # rotate q by 90 degrees to make it point upwards
                q_right = (0, -np.sqrt(0.5), 0, np.sqrt(0.5))
                q_right = quaternion_multiply(uncast_quaternion(q), q_right)
                my_client.add_arrow(
                    posx, posy, q_right, color=(1, 0, 0)
                )  # Add Rviz arrow marker, map frame
                i = 1
                # rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))
                x, y, q_p = my_client.bot_to_map(0, 0, (0, 0, 0, 1))  # bot location
                success = my_client.send_goal(
                    *my_client.find_off_goal(posx, posy, q=q_p, offset=(-1.75, 0, 0, 0)),
                    frame="map"
                )
                rospy.sleep(1)
                dist = norm([x - posx, y - posy])
                while (
                    success == False and dist > 1.85
                ):  # keep checking if we are moving correctly
                    dist = norm([x - posx, y - posy])
                    success = my_client.send_goal(
                        *my_client.find_off_goal(
                            posx, posy, q=q_p, offset=(-1.75, 0, 0, 0)
                        ),
                        frame="map"
                    )
                    rospy.sleep(1)
                    found, pos, orient, timestamp = my_client.arrow_detect(far=dist > 3)
                    if found == False or posx is None:
                        found, pos, orient, timestamp = my_client.recovery(far=dist > 3)
                    if found == False:
                        break
                    orient = orient + 90 if orient < 0 else orient - 90
                    q_p = (
                        0,
                        0,
                        np.sin(np.pi * orient / (2 * 180)),
                        np.cos(np.pi * orient / (2 * 180)),
                    )
                    x, y, q_p = my_client.bot_to_map(
                        0, 0, q_p
                    )  # bot location, with perpendicular to arrow goal
                    dist = norm([x - posx, y - posy])
                    if my_client.is_complete(posx, posy, q_p):
                        rospy.loginfo(
                            "Already visited recently found Goal: " + str([posx, posy])
                        )
                        found = False
                        break
                if found == True:
                    # my_client.cancel_goal()
                    # TODO change to 20
                    rospy.sleep(10)
                    success = my_client.move_to_goal(
                        *my_client.find_off_goal(
                            posx, posy, q=q_p, offset=(-1.5, 0, 0, 0)
                        ),
                        frame="map"
                    )

                    found, pos, orient, timestamp = my_client.arrow_detect(far=False)
                    if found == False or pos is None:
                        found, pos, orient, timestamp = my_client.recovery(far=False)
                    if found == False:
                        continue
                    q = (
                        0,
                        0,
                        np.sin(np.pi * orient / (2 * 180)),
                        np.cos(np.pi * orient / (2 * 180)),
                    )
                    posx, posy, q = my_client.bot_to_map(
                        pos[0], pos[1], q, timestamp=timestamp, frame="mrt/camera_link"
                    )
                    # posx,posy, q = my_client.bot_to_map(0, 0, q)
                    my_client.add_arrow(
                        posx, posy, q, color=(0, 1, 0), pos_z=0.48
                    )  # Add Rviz arrow marker, map frame
                    success = my_client.move_to_off_goal(
                        posx, posy, q=q, frame="map", off_dist=1, ahead=0.75
                    )
                    if success == True:
                        # my_client.add_arrow(*my_client.bot_to_map(posx, posy,
                        # q, frame="mrt/camera_link"), color=(0,1,1))
                        prev_x, prev_y, prev_q = posx, posy, q  # map frame
                        # my_client.add_arrow(prev_x, prev_y, prev_q, (1,0,1))
                        my_client.add_to_completed(posx, posy, q)
                        #rospy.sleep(10)
                    else:
                        rospy.loginfo("Failed goal: " + str((posx, posy, q)))
        if not found:
            nearby_goal = my_client.move_to_off_goal(prev_x, prev_y, prev_q,off_dist=0.3, ahead=0.7 + 0.7 * i)
            my_client.send_goal(*nearby_goal, frame="map")
            rospy.sleep(6.2)  # Sleep for 1-2s and let the bot move towards the goal
            i += 1

            found_cone,val_cone,distance_cone=my_client.cone_detect()
            curr_x,curr_y,q_cone=my_client.bot_to_map(0, 0, (0, 0, 0, 1),frame='mrt/camera_link')
            if found_cone:
                if distance_cone==0 or distance_cone==nan :
                    my_client.send_goal(*just_ahead(curr_x,curr_y,val_cone,off_dist=0.25),frame="map")
                if distance_cone>1:
                    posx,posy,q = my_client.bot_to_map(val_cone[0],val_cone[1],q=None,frame='mrt/camera_link')
                    a,b,c=my_client.find_off_goal(posx, posy, q=None, offset=(-1, 0, 0, 0))
                    q_right = (0, -np.sqrt(0.5), 0, np.sqrt(0.5))
                    q_right = quaternion_multiply(uncast_quaternion(c), q_right)
                    my_client.add_arrow(posx, posy, q_right, color=(1, 1, 0))
                    my_client.send_goal(a,b,c,frame="map")
                    break
                else:
                    my_client.cancel_goal()
                    print('Completed Task')
                    break
    # rate.sleep()

    # Close down the video stream when done
    destroyAllWindows()


if __name__ == "__main__":
    try:
        main_node()
    except rospy.ROSInterruptException:
        print("Exiting... ")
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise
