# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import numpy as np
import cv2 as cv
import os


def detect_arrow(frame, template, check=False):
    (tH, tW) = template.shape[:2]
    found = None
    direction = None
    template = cv.cvtColor(template, cv.COLOR_BGR2GRAY)
    ret1, template_bin = cv.threshold(template, 120, 255, cv.THRESH_BINARY)

    frame_grayscale = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret2, frame_img_bin = cv.threshold(frame_grayscale, 120, 255, cv.THRESH_BINARY)

    # detect edges in the resized, grayscale image and apply template
    # matching to find the template in the image
    for tem_scale in np.linspace(0.2, 1.0, 10)[::-1]:
        r_tem = tem_scale

        template_resize = cv.resize(
            template_bin,
            dsize=(
                int(template_bin.shape[1] * tem_scale),
                int(template_bin.shape[0] * tem_scale),
            ),
            interpolation=cv.INTER_LINEAR,
        )
        if frame.shape[0] < tH * tem_scale or frame.shape[1] < tW * tem_scale:
            continue

        result = cv.matchTemplate(frame_img_bin, template_resize, cv.TM_SQDIFF)
        (minVal, _, minLoc, _) = cv.minMaxLoc(result)
        minVal = minVal / (template_resize.size)

        # check to see if the iteration should be visualized
        # draw a bounding box around the detected region
        """
        clone = np.dstack([edged, edged, edged])
        cv.rectangle(clone, (maxLoc[0], maxLoc[1]),
                     (maxLoc[0] + int(tW * tem_scale), maxLoc[1] + int(tH * tem_scale)), (0, 0, 255), 2)
        cv.imshow("checking", clone)
        cv.waitKey(0)
        """
        if found is None or minVal < found[0]:
            found = (minVal, minLoc, r_tem)
            direction = 1

    for tem_scale in np.linspace(0.2, 1.0, 10)[::-1]:
        r_tem = tem_scale

        template_resize = cv.resize(
            template_bin,
            dsize=(
                int(template_bin.shape[1] * tem_scale),
                int(template_bin.shape[0] * tem_scale),
            ),
            interpolation=cv.INTER_LINEAR,
        )
        if frame.shape[0] < tH * tem_scale or frame.shape[1] < tW * tem_scale:
            continue
        template_inversion = template_resize[:, ::-1]

        result = cv.matchTemplate(frame_img_bin, template_inversion, cv.TM_SQDIFF)
        (minVal, _, minLoc, _) = cv.minMaxLoc(result)
        minVal = minVal / (template_resize.size)
        # check to see if the iteration should be visualized
        # draw a bounding box around the detected region
        """
        clone = np.dstack([edged, edged, edged])
        cv.rectangle(clone, (maxLoc[0], maxLoc[1]),
                     (maxLoc[0] + int(tW * tem_scale), maxLoc[1] + int(tH * tem_scale)), (0, 0, 255), 2)
        """

        if found is None or minVal < found[0]:

            found = (minVal, minLoc, r_tem)
            direction = -1

    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio

    (_, minLoc, r_tem) = found

    (startX, startY) = (int(minLoc[0]), int(minLoc[1]))
    (endX, endY) = (int(minLoc[0] + tW * r_tem), int(minLoc[1] + tH * r_tem))
    # draw a bounding box around the detected result and display the image
    cv.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)

    return frame, direction


if __name__ == "__main__":
    print("Starting arrow detection script")
    """
    sample_img=cv.imread('sample_image.jpg')
    template = cv.imread('template.png')
    #output,direction = detect_arrow(sample_img, template)
    #cv.imshow("Result", output)
    #cv.waitKey(0)
    """
    template = cv.imread("/home/khush/Downloads/arrow.jpeg")
    cv.imshow("Result", template)
    cv.waitKey(0)
    capture = cv.VideoCapture(0)
    while True:
        ret_val, frame = capture.read()
        output, direction = detect_arrow(frame, template)
        if direction == 1:
            direction = "Right"
        else:
            direction = "Left"
        # font
        font = cv.FONT_HERSHEY_SIMPLEX

        # org
        org = (50, 50)

        # fontScale
        fontScale = 1

        # Blue color in BGR
        color = (255, 0, 0)

        # Line thickness of 2 px
        thickness = 2

        output = cv.putText(
            output, direction, org, font, fontScale, color, thickness, cv.LINE_AA
        )

        cv.imshow("Arrow", output)
        if cv.waitKey(1) & 0xFF == ord("q"):
            break
