# coding=utf-8
import os
import shutil
import cv2
import numpy as np
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)  #10、15或者30可选,20或者25会报错，其他帧率未尝试
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 15)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align = rs.align(rs.stream.color)


def init_dir(dirname):
    '''
    如果文件夹不存在就创建，如果文件存在就清空！
    :param filepath:需要创建的文件夹路径
    :return:
    '''
    if not os.path.exists(dirname):
        os.mkdir(dirname)
    else:
        shutil.rmtree(dirname)
        os.mkdir(dirname)

root = 'E:\\data\\'
dir = 'cv_office\\'
frame_id = 1

try:
    init_dir(root + dir + "depth\\")
    init_dir(root + dir + "rgb\\")

    while True:
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)

        # Get aligned frames
        depth_frame = frames.get_depth_frame()  # depth_frame is a 640x480 depth image
        if not depth_frame:
            continue
        depth_frame = np.asanyarray(depth_frame.get_data())
        # 将深度图转化为伪彩色图方便观看
        depth_frame_toRGB = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imshow('depth', depth_frame_toRGB)

        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_frame = np.asanyarray(color_frame.get_data())
        cv2.imshow('color', color_frame)

        cv2.imwrite(root + dir + "depth\\" + str(frame_id) +  ".png", depth_frame)
        cv2.imwrite(root + dir + "rgb\\" + str(frame_id) +  ".png", color_frame)

    # # left　frames
    #     left_frame = frames.get_infrared_frame(1)
    #     if not left_frame:
    #         continue
    #     left_frame = np.asanyarray(left_frame.get_data())
    #     cv2.imshow('3 left_frame', left_frame)
    #
    # # right frames
    #     right_frame = frames.get_infrared_frame(2)
    #     if not right_frame:
    #         continue
    #     right_frame = np.asanyarray(right_frame.get_data())
    #     cv2.imshow('4 right_frame', right_frame)

        if cv2.waitKey(1) == 27:
            # 如果按下ESC则关闭窗口（ESC的ascii码为27），同时跳出循环
            cv2.destroyAllWindows()
            break
        print("process frameId: " + str(frame_id))
        frame_id += 1
finally:
    # Stop streaming
    pipeline.stop()

