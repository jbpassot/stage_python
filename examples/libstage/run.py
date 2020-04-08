import stagesim
import logging
import threading
import time
import cv2

def imshow_scaled(window_name, img, scaling=3):
    '''Convenience function to show cv images on high-res monitors'''
    cv2.imshow(window_name, cv2.resize(img, None, None, scaling, scaling, cv2.INTER_NEAREST))


if __name__ == "__main__":


    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,
                        datefmt="%H:%M:%S")

    sim = stagesim.StageSimulator("/home/jb/projects/stage4/Stage/worlds/benchmark/hospital_2.world");

    while (True):
        # print (sim.get_info())
        odom = sim.get_odom()
        robot_state = sim.get_robot_state()
        lidar_data = sim.get_scan_data()
        depth_data = sim.get_depth_data()

        print (robot_state)

        if depth_data["timestamp_us"] > 0:
            width = depth_data["width"]
            height = depth_data["height"]
            camera = depth_data["data"].reshape((height,width)) / 8.
            camera = camera[::-1,...]
            imshow_scaled("camera", camera, scaling=1)
            cv2.waitKey(100)
