import stagesim
import logging
import threading
import time
import cv2
import numpy as np

def imshow_scaled(window_name, img, scaling=3):
    '''Convenience function to show cv images on high-res monitors'''
    cv2.imshow(window_name, cv2.resize(img, None, None, scaling, scaling, cv2.INTER_NEAREST))

global left_wheel_desired_velocity
global right_wheel_desired_velocity


def send_command(sim, key):
    global left_wheel_desired_velocity
    global right_wheel_desired_velocity
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
        left_wheel_desired_velocity *= 0.9
        right_wheel_desired_velocity *= 0.9
        if left_wheel_desired_velocity < 0.2: left_wheel_desired_velocity=0.
        if right_wheel_desired_velocity < 0.2: right_wheel_desired_velocity=0.

    sim.send_command(left_wheel_desired_velocity, right_wheel_desired_velocity)


def toggle_lock_simulation(lock, sim):
    if lock:
        print ("Locking Simulation Environment (Stepping must be controlled by calling step_simulation)")
        sim.lock_simulation()
    else:
        print ("Unlocking Simulation Environment (Stepping is controlled within the simulator")
        sim.release_simulation()


if __name__ == "__main__":


    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,
                        datefmt="%H:%M:%S")
    key = 0
    lock_simulation = False
    run_async = True

    sim = stagesim.StageSimulator("/home/jb/projects/stage4/Stage/worlds/hospital.world");
    """
    world = "hospital.world" 
    dir_path = os.path.dirname(os.path.realpath(__file__))
    sim = stagesim.StageSimulator(os.path.join(dir_path, 'worlds', world));
    """


    left_wheel_desired_velocity = 0.
    right_wheel_desired_velocity = 0.

    while (True):
        # print (sim.get_info())
        # odom = sim.get_odom()
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
            lock_simulation = not lock_simulation
            toggle_lock_simulation(lock_simulation, sim)

        if lock_simulation and run_async:
            send_command(sim, key)
            sim.step_simulation_async(50)  # Triggers the simulation to move forward by x ms, asynchronous
            key = cv2.waitKey(1)
        elif lock_simulation:  #  Implied: and not run_async:
            send_command(sim, key)
            sim.step_simulation_sync(50)  # Triggers the simulation to move forward by x ms, SYNCHRONOUSLY
            key = cv2.waitKey(1)
        else:
            key = cv2.waitKey(1)
