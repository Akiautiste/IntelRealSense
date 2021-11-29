# coding=utf-8
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os

# 0:使用相机
# 1:使用API录制好的rosbag
USE_ROS_BAG = 1
# 0:彩色图像对齐到深度图;
# 1:深度图对齐到彩色图像
ALIGN_WAY = 1




def Align_version(frames, align, show_pic=0):
    # 对齐版本
    aligned_frames = align.process(frames)
    depth_frame_aligned = aligned_frames.get_depth_frame()
    print(np.asanyarray(depth_frame_aligned.get_data()))
    color_frame_aligned = aligned_frames.get_color_frame()
    # if not depth_frame_aligned or not color_frame_aligned:
    #     continue
    color_image_aligned = np.asanyarray(color_frame_aligned.get_data())
    if USE_ROS_BAG:
        color_image_aligned = cv2.cvtColor(color_image_aligned, cv2.COLOR_BGR2RGB)
    depth_image_aligned = np.asanyarray(depth_frame_aligned.get_data())

    depth_colormap_aligned = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_aligned, alpha=0.05), cv2.COLORMAP_JET)
    images_aligned = np.hstack((color_image_aligned, depth_colormap_aligned))
    if show_pic:
        # for i in range(100):
        #     data = pipeline.wait_for_frames()
        #     depth = data.get_depth_frame()
        #     color = data.get_color_frame()
        # dprofile = depth.get_profile()
        # cprofile = color.get_profile()
        # cvsprofile = rs.video_stream_profile(cprofile)
        # dvsprofile = rs.video_stream_profile(dprofile)
        # color_intrin = cvsprofile.get_intrinsics()
        # print(color_intrin)
        # depth_intrin = dvsprofile.get_intrinsics()
        # print(depth_intrin)
        # depth_image = np.asanyarray(depth.get_data())
        # color_image = np.asanyarray(color.get_data())
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_JET)
        # t = time.time()
        # tname = str(t)[5:10]
        # file_name =os.path
        # cv2.imwrite(f"\Apple_Aligned\color{F}.png", color_image_aligned)
        # cv2.imwrite(f"\Apple_Aligned\depth{F}.png", depth_image_aligned)
        # cv2.imwrite(f"\Apple_Aligned\depth_colorMAP{F}.png", depth_colormap_aligned)
        cv2.imshow(f'{F}_aligned_images', images_aligned)
    #     # Convert images to numpy arrays


    return color_image_aligned, depth_image_aligned, depth_colormap_aligned


def Unalign_version(frames, show_pic=0):
    # 未对齐版本
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not USE_ROS_BAG:
        left_frame = frames.get_infrared_frame(1)
        right_frame = frames.get_infrared_frame(2)
        left_image = np.asanyarray(left_frame.get_data())
        right_image = np.asanyarray(right_frame.get_data())
        if show_pic:
            cv2.imshow('left_images', left_image)
            cv2.imshow('right_images', right_image)
    # if not depth_frame or not color_frame:
    #     continue
    color_image = np.asanyarray(color_frame.get_data())
    # print("color:", color_image.shape)
    depth_image = np.asanyarray(depth_frame.get_data())
    # print("depth:", depth_image.shape)

    # 相机API录制的大小rosbag的rgb图像与depth图像不一致，用resize调整到一样大
    if USE_ROS_BAG:
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        if ALIGN_WAY:  # 深度图对齐到彩色图像
            depth_image = cv2.resize(depth_image, (color_image.shape[1], color_image.shape[0]))
        else:  # 彩色图像对齐到深度图
            color_image = cv2.resize(color_image, (depth_image.shape[1], depth_image.shape[0]))
    # 上色
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_JET)
    # Stack both images horizontally
    images = np.hstack((color_image, depth_colormap))
    if show_pic:
        cv2.imshow(f'{F}_images', images)
    return color_image, depth_image, depth_colormap


if __name__ == "__main__":
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    file_name = os.listdir("C:\\Apple_Data\\BAG\\aks")
    for F in file_name:

        if USE_ROS_BAG:
            config.enable_device_from_file(f"C:\\Apple_Data\\BAG\\aks\\{F}")  # 这是打开相机API录制的视频
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # 10、15或者30可选,20或者25会报错，其他帧率未尝试
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            # 左右双目
            config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
            config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

        if ALIGN_WAY:
            way = rs.stream.color
        else:
            way = rs.stream.depth
        align = rs.align(way)
        profile = pipeline.start(config)    #开始串流

        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("scale:", depth_scale)
        # 深度比例系数为： 0.0010000000474974513

        try:
            while True:
                frames = pipeline.wait_for_frames()
                color_image_aligned, depth_image_aligned, depth_colormap_aligned = Align_version(frames, align, show_pic=1)
                color_image, depth_image, depth_colormap = Unalign_version(frames, show_pic=1)
                cv2.imwrite(f"color{}.png", color_image_aligned)
                cv2.imwrite(f"depth{F}.png", depth_image_aligned)
                cv2.imwrite(f"depth_colorMAP{F}.png", depth_colormap_aligned)
                # time.sleep(10)
                # cv2.destroyAllWindows()
                print(depth_image_aligned*depth_scale)
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
        finally:
            # Stop streaming
            pipeline.stop()