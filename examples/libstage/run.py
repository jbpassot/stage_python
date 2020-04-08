import stagesim
import logging
import threading
import time
import cv2
import numpy as np

def imshow_scaled(window_name, img, scaling=3):
    '''Convenience function to show cv images on high-res monitors'''
    cv2.imshow(window_name, cv2.resize(img, None, None, scaling, scaling, cv2.INTER_NEAREST))


def send_command(sim, key):
    left_wheel_desired_velocity = 0.
    right_wheel_desired_velocity = 0.
    send_motor_command = True

    if key == 'w': # Move Forward for 50 ms:
        left_wheel_desired_velocity = 0.5
        right_wheel_desired_velocity = 0.5
    elif key == 's': # Move Backward for 50 ms:
        left_wheel_desired_velocity = -0.5
        right_wheel_desired_velocity = -0.5
    elif key == 'a': # Move Left for 50 ms:
        left_wheel_desired_velocity = -0.5
        right_wheel_desired_velocity = 0.5
    elif key == 'd': # Move Backward for 50 ms:
        left_wheel_desired_velocity = 0.5
        right_wheel_desired_velocity = -0.5
    else:
        send_motor_command = False
    if send_motor_command:
        #print ('Sending Motor Command')
        sim.send_command(left_wheel_desired_velocity, right_wheel_desired_velocity)



if __name__ == "__main__":


    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,
                        datefmt="%H:%M:%S")

    sim = stagesim.StageSimulator("/home/jb/projects/stage4/Stage/worlds/benchmark/hospital_2.world");

    key = 0
    free_simulation = True

    while (True):
        # print (sim.get_info())
        odom = sim.get_odom()
        robot_state = sim.get_robot_state()
        lidar_data = sim.get_scan_data()
        depth_data = sim.get_depth_data()

        # print robot state here
        # print (robot_state)

        if depth_data["timestamp_us"] > 0:
            width = depth_data["width"]
            height = depth_data["height"]
            camera = depth_data["data"].reshape((height,width)) / 8.
            camera = camera[::-1,...]
            imshow_scaled("camera", camera, scaling=1)
        else:
            imshow_scaled("camera", np.zeros((300, 300)), scaling=1)

        key = chr(key%256)

        if key == 'r':
            free_simulation = not free_simulation
            if free_simulation:
                print "*"*50
                print "RELEASING THE SIMULATION"
                sim.release_simulation()
                print "*"*50
            else:
                print "*"*50
                print "LOCKING THE SIMULATION"
                print "*"*50

        if not free_simulation:
            sim.step_simulation(50)


        send_command(sim, key)

        key = cv2.waitKey(1)
