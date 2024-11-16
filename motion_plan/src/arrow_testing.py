import numpy as np
import cv2
import os, glob, time
import argparse
import rospy


OFFSET = 0.8
HORZ_OFFSET = 0.5
COLOR_OFFSET = 0



def preprocess(img, adaptive=False):
    
    
    if adaptive:
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_thres = cv2.adaptiveThreshold(img_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,25,2)
        kernel = np.ones((3, 3))
        
        if adaptive:
            img_blur = cv2.medianBlur(img_thres, 5)
            # cv2.imshow("Image Blur", img_blur)

            img_erode = cv2.erode(img_blur, kernel, iterations=1)
            # cv2.imshow("Image Erode", img_erode)

            img_dilate = cv2.dilate(img_erode, kernel, iterations=1)
            # cv2.imshow("Image Dilate", img_dilate)
        else:
            img_dilate = img_thres
        img_canny = cv2.Canny(img_dilate, 200, 200)
        # cv2.imshow("Image Canny", img_canny)

        # cv2.waitKey(0)

        return img_canny
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_equalized = cv2.equalizeHist(img_gray)

    _, img_thres = cv2.threshold(img_equalized, 70, 255, cv2.THRESH_TOZERO)
    # img_blur = cv2.GaussianBlur(img_thres, (5, 5), 1)
    # img_blur = cv2.bilateralFilter(img_equalized, 5, 75, 75)
    kernel = np.ones((3, 3))
    # img_erode = cv2.erode(img_equalized, kernel, iterations=1)
    # img_dilate = cv2.dilate(img_erode, kernel, iterations=1)
    img_canny = cv2.Canny(img_thres, 250, 250)

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
    #print(indices, "convex_hull:",convex_hull,"points:", points)
    for i in range(2):
        j = indices[i] + 2
        #if j > length - 1:
        #    j = length - j
        if np.all(points[j%length] == points[indices[i - 1] - 2]):
            return tuple(points[j%length]), j%length
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
            # print(sides)
            # print(abs(sides[0]-sides[2])/float(sides[2]))
            # print(abs(sides[1]-sides[3])/float(sides[1]))
            # print( "diff: "+ str( abs(abs(points[(indices[i-1]+1)%length]- points[indices[i-1]]) - abs(points[indices[i]]- points[indices[i]-1])) ))#/abs(points[(indices[i-1]+1)%length]- points[indices[i-1]])
            # print( "diff: "+ str( abs(abs(points[(indices[i-1]+1)%length]- points[indices[i-1]]) - abs(points[indices[i]]- points[indices[i]-1]))/abs((points[(indices[i-1]+1)%length]- points[indices[i]]).astype(np.float32)) ))#

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
        err = 5 if not n in [3,4] else 0#int(2*np.linalg.norm(points[3]-points[4])/5)
        if err == 0:
            new_points.append(pt)
            continue
        new_pt = corners[np.argmin([np.linalg.norm(corner- pt) for corner in corners])]
        # print(np.linalg.norm(new_pt - pt))
        new_pt = new_pt if np.linalg.norm(new_pt - pt) < err else pt
        new_points.append(new_pt)
    return np.array(new_points)

def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
    # draw ground floor in green
    img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)
    return img

def rotate_image(image, angle):
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
  return result, rot_mat

def arrow_detect(frame):
    # Arrow detection
    img = frame.copy()
    #self.lagging_pcd = o3d.geometry.PointCloud(self.pcd)

    orig_img = img.copy()
    found = False
    orient = None
    pos = None
    direction = None
    bounding_box = None
    contours, _ = cv2.findContours(
        preprocess(img), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )[-2:]
    # cv2.imshow("Image", preprocess(img))
    # cv2.waitKey(0)
    # template = cv2.imread("arrow.jpeg")
    max_cnt_area = -1
    # all_cnt_mask = np.zeros(img.shape[:2], np.uint8)
    for i, cnt in enumerate(contours):
        cnt_area = cv2.contourArea(cnt)
        # filtering using area
        if cnt_area < 60:
            continue

        # filtering using color of arrow
        arrow_mask = np.zeros(img.shape[:2], np.uint8)
        cv2.drawContours(arrow_mask, [cnt], -1, 255, -1 )
        cnt_mean = np.array(cv2.mean(img, mask=arrow_mask)[:3])

        cv2.imshow("Arrow mask", arrow_mask)
        cv2.waitKey(0)
        mag_mean = np.linalg.norm(cnt_mean)
        norm_mean = cnt_mean/mag_mean
        unit_vec = np.array([1, 1, 1])/np.sqrt(3)
        if np.dot(norm_mean, unit_vec) < OFFSET or np.mean(cnt_mean)>255/2-COLOR_OFFSET:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
        hull = cv2.convexHull(approx, returnPoints=False)
        sides = len(hull)

        if (sides == 5 or sides == 4) and sides + 2 == len(approx):
            # cv2.imshow("arrow mask", arrow_mask)
            # cv2.waitKey(0)
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
                    # cv2.imshow("Image Bg 1", background_mask)
                    # cv2.waitKey(0)

                    cnt_mean = np.array(cv2.mean(img, mask=background_mask)[:3])
                    mag_mean = np.linalg.norm(cnt_mean)
                    norm_mean = cnt_mean/mag_mean
                    unit_vec = np.array([1, 1, 1])/np.sqrt(3)
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

    return found, pos, orient


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arrow detection')
    parser.add_argument('--dir_detect', type=bool, default=True)
    parser.add_argument('--webcam', type=bool, default=False)


    print("Starting arrow detection script")
    ''''''
    args = parser.parse_args()
    if args.dir_detect:
        images = glob.glob('*.jpg') + glob.glob('*.jpeg') + glob.glob("*.png")
        num=0
        num2 = 0
        for fname in images:
            sample_img=cv2.imread(fname)
            found, _, _ = arrow_detect(sample_img)
            print("Found :", found)
            # if direction == 1:
            #     direction = 'Right'
            # else:
            #     direction = 'Left'
            # if found:
            #     num+=1
            #     print("Arrows detected:", num)
            #     output = cv2.putText(output, direction + ", angle:{:.2f}".format(orient[0]), (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
            #                     1, (255, 0, 0), 2, cv2.LINE_AA)
            #     cv2.imshow("Result", output)
            #     cv2.waitKey(0)
            # else:
            #     print("Not found")
            #     num2 += 1
            #     print("Not found in :", num2)

    
    #Uncomment what you need
    if args.webcam:
        time_max = 0
        time_sum = 0
        n_detected = 0
        capture = cv2.VideoCapture(0)
        while True:
            ret_val, frame = capture.read()
            if ret_val == False:
                print("image/video error")
                time.sleep(1)
                continue
            start = time.time()
            found, _, _ = arrow_detect(frame)
            end = time.time()
            if found == True:
                time_sum += end-start
                n_detected += 1
                if end-start > time_max:
                    time_max = end - start
            if direction == 1:
                direction = 'Right'
            else:
                direction = 'Left'
            # font
            font = cv2.FONT_HERSHEY_SIMPLEX

            # org
            org = (50, 50)

            # fontScale
            fontScale = 1

            # Blue color in BGR
            color = (255, 0, 0)

            # Line thickness of 2 px
            thickness = 2

            output = cv2.putText(output, direction + " \n"+ str(orient), org, font,
                                fontScale, color, thickness, cv2.LINE_AA)

            cv2.imshow("Arrow", output)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        if n_detected > 0:
            print("Time taken(avg): ",time_sum/n_detected)
            print("Time taken(max): ",time_max)