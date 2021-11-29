import cv2
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    color_image = cv2.imread("./0.png")
    depth_image = np.load("./0.npy")

    cv2.imshow("color", color_image)

    # 读取到的深度信息/1000 为真实的深度信息，单位为m
    # truth_depth = depth_image[x, y]/1000
    # 如果深度信息为0, 则说明没有获取到
    plt.imshow(depth_image.astype(np.int), "gray")
    plt.show()
    cv2.waitKey()
    plt.imshow(depth_image.astype(np.int), "gray")
    plt.show()
    cv2.waitKey()
