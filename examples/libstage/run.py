import stagesim
import logging
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
        sim.start_simulation()
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
    send_motor_command = False

    sim = stagesim.StageSimulator("/home/jb/projects/stage4/Stage/worlds/hospital.world");
    """
    world = "hospital.world" 
    dir_path = os.path.dirname(os.path.realpath(__file__))
    sim = stagesim.StageSimulator(os.path.join(dir_path, 'worlds', world));
    """

    left_wheel_desired_velocity = 0.
    right_wheel_desired_velocity = 0.

    last_robot_state_timestamp_us = -1
    last_depth_data_timestamp_us = -1
    last_lidar_data_timestamp_us = -1
    last_rgb_data_timestamp_us = -1


    while (True):
        # print (sim.get_info())
        # odom = sim.get_odom()
        if sim.get_robot_state_timestamp_us() > last_robot_state_timestamp_us: robot_state = sim.get_robot_state()
        if sim.get_scan_data_timestamp_us() > last_lidar_data_timestamp_us: lidar_data = sim.get_scan_data()
        if sim.get_depth_data_timestamp_us() > last_depth_data_timestamp_us: depth_data = sim.get_depth_data()
        if sim.get_rgb_data_timestamp_us() > last_rgb_data_timestamp_us: rgb_data = sim.get_rgb_data()

        last_robot_state_timestamp_us = sim.get_robot_state_timestamp_us()
        last_lidar_data_timestamp_us = sim.get_scan_data_timestamp_us()
        last_depth_data_timestamp_us = sim.get_depth_data_timestamp_us()
        last_rgb_data_timestamp_us =  sim.get_rgb_data_timestamp_us()

        homing_data = sim.get_home_marker()

        if depth_data["timestamp_us"] > 0:
            width = depth_data["width"]
            height = depth_data["height"]
            camera = depth_data["data"].reshape((height,width)) / 8.
            camera = camera[::-1,...]
            imshow_scaled("camera", camera, scaling=1)
        else:
            imshow_scaled("camera", np.zeros((300, 300)), scaling=1)


        if rgb_data["timestamp_us"] > 0:
            width = rgb_data["width"]
            height = rgb_data["height"]
            camera = rgb_data["data"].reshape((height,width, 4))
            camera = camera[::-1,...]
            imshow_scaled("rgb camera", camera, scaling=1)
        else:
            imshow_scaled("rgb camera", np.zeros((300, 300)), scaling=1)

        key = chr(key%256)


        """
        Switching from different options
        """
        if key == '1': # Locking the simulation, i.e. stepping is controlled within this script
            lock_simulation = not lock_simulation
            toggle_lock_simulation(lock_simulation, sim)
        if key == '2':
            # Sync or Async, only valid when locking the simulation
            # If Async, step_simulation will return without waiting for the simulation to finish stepping
            # but will pause automatically after stepping
            if not lock_simulation:
                lock_simulation = not lock_simulation
                toggle_lock_simulation(lock_simulation, sim)
            run_async = not run_async
            if (run_async): print("Running Async")
            else: print("Running Sync")
        if key == '3':
            # Control sending motor comamand from the script itself
            # If True, we can send motor command from the OpenCV window (WSAD keys)
            send_motor_command = not send_motor_command
            if send_motor_command: print("Sending Motor Command From OpenCV Window: ENABLED")
            else: print("Sending Motor Command From OpenCV Window: DISABLED")


        if lock_simulation and run_async:
            if send_motor_command: send_command(sim, key)
            sim.step_simulation_async(50)  # Triggers the simulation to move forward by x ms, asynchronous
            key = cv2.waitKey(1)
        elif lock_simulation and not run_async:
            if send_motor_command: send_command(sim, key)
            sim.step_simulation_sync(50, True)  # Triggers the simulation to move forward by x ms, SYNCHRONOUSLY
            key = cv2.waitKey(1)
        else:
            key = cv2.waitKey(1)
