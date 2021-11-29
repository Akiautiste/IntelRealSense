import cv2
import numpy as np

from matplotlib import pyplot as plt

import threading

imgdd = cv2.imread('./1_6_RGB_Color.png', 0)
imgg = cv2.imread('./1_6_depth_Depth_Depth.png', 0)
imgd = cv2.Canny(imgdd, 100, 200)
img = cv2.Canny(imgg, 100, 200)


def parm_adjust_th():
    cv2.namedWindow('Pose_Adjuster')
    cv2.createTrackbar('pos_x', 'Pose_Adjuster', 159, 200 * 2, callback)
    cv2.createTrackbar('pos_y', 'Pose_Adjuster', 80, 200 * 2, callback)
    cv2.createTrackbar('scale', 'Pose_Adjuster', 186, 200, callback)

    while (True):
        pos_x = (cv2.getTrackbarPos('pos_x', 'Pose_Adjuster') - 200)
        pos_y = (cv2.getTrackbarPos('pos_y', 'Pose_Adjuster') - 200)
        scale = (cv2.getTrackbarPos('scale', 'Pose_Adjuster'))

        # print(img.shape)
        # print(imgd.shape[0], imgd.shape[1])

        # global trans
        trans = [pos_x, pos_y, scale]
        print("===>>> Trans : ", trans)
        scale_percent = trans[2]  # percent of original size
        width = int(imgd.shape[1] * scale_percent / 100)
        height = int(imgd.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(imgd, dim, interpolation=cv2.INTER_AREA)
        # print(resized.shape)
        # print(img.shape)
        # print("===>>> Trans : ", trans)
        for u in range(min(len(resized[:, 0]), img.shape[0])):
            for v in range(min(len(resized[0, :] - 1), img.shape[1])):
                # print(u, v, img.shape[0], img.shape[1])
                if resized[u, v] != 0 and trans[1] + u < img.shape[0] and trans[0] + v < img.shape[1]:
                    img[trans[1] + u, trans[0] + v] = resized[u, v]
        # print(trans[1], trans[1]+ imgd.shape[0], trans[0], trans[0]+imgd.shape[1])

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        cv2.imshow('Pose_Adjuster', img)
        cv2.imwrite("cnny_align.png", img)
    cv2.destroyAllWindows()


def callback(object):
    pass


if __name__ == '__main__':
    parm_adjust_th()

