import numpy as np
import cv2
import os

# This can be used to test the arrow detection without ROS


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
            # print( "diff: "+ str( abs(abs(points[(indices[i-1]+1)%length]- points[indices[i-1]]) - abs(points[indices[i]]- points[indices[i]-1]))/abs(points[(indices[i-1]+1)%length]- points[indices[i-1]]) ))
            if np.all(
                abs(
                    abs(points[(indices[i - 1] + 1) % length] - points[indices[i - 1]])
                    - abs(points[indices[i]] - points[indices[i] - 1])
                )
                < 5
            ):  # Check if tails is nearly a rectangle#TODO change 5 to something relative to area
                if points[indices[i] - 1][0] < points[indices[i]][0]:
                    print("Right")
                    direction = 1  # TODO : Add respective rect pts in order
                else:
                    print("Left")
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


def arrow_detect(img):
    # Arrow detection
    # img = self.frame.copy()
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
        if cv2.contourArea(cnt) < 200:
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
                cv2.drawContours(img, [approx], -1, (0, 150, 155), 2)
                cv2.circle(img, arrow_tip, 3, (0, 0, 255), cv2.FILLED)
                # cv2.polylines(img, [rect],  True, (0, 0, 255), 2)
                arrow_tail = tuple(np.average([rect[0], rect[3]], axis=0).astype(int))
                if arrow_tail[0] - arrow_tip[0] == 0:
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
                        np.average([arrow_tail[0], arrow_tip[0]]) / (np.shape(img)[0])
                        - 0.5
                    )
                    * 45
                    * 2
                )  # linear estimate, assuming camera horizontal range from -45 to 45
                direction = dirct  # TODO multiple arrow case
                found = True
                print("arrow_x_img: " + str(np.average(rect, axis=0)[0]))
    if direction is not None:  # TODO: Improve upon this naive orientation
        if direction == 1:  # Right
            orient = -90
        elif direction == 0:  # Left
            orient = 90
        else:
            print("error: direction not found and not None, " + str(direction))
            found = False
    return found, theta, orient, direction, img


if __name__ == "__main__":
    print("Starting arrow detection script")
    """
    sample_img=cv2.imread('sample_image.jpg')
    found, theta, orient, direction, output = detect_arrow(sample_img)
    cv2.imshow("Result", output)
    cv2.waitKey(0)
    """
    # Uncomment what you need

    capture = cv2.VideoCapture(0)
    while True:
        ret_val, frame = capture.read()
        found, theta, orient, direction, output = arrow_detect(frame)
        if found == False:
            continue
        if direction == 1:
            direction = "Right"
        else:
            direction = "Left"
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

        output = cv2.putText(
            output, direction, org, font, fontScale, color, thickness, cv2.LINE_AA
        )

        cv2.imshow("Arrow", output)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        pass
