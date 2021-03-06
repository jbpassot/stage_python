/**
 * @file pyblind_stage.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Original work Copyright 2020 JB Passot
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Original authors: JB Passot
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <sstream>
#include <iostream>
#include <cmath>

#include <chrono>
#include "stage.hh"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

namespace pybind11 {
    template <typename T>
    using safe_array = typename pybind11::array_t<T, pybind11::array::c_style>;
}

namespace py = pybind11;
using namespace pybind11::literals;
using namespace std::chrono;
using time_stamp = std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds>;

class DifferentialDriveCommand{
public:
    Stg::usec_t timestamp_us;
    Stg::usec_t clock_us;
    double traction_left_wheel_speed;
    double traction_right_wheel_speed;

    DifferentialDriveCommand(): clock_us(0), timestamp_us(0), traction_left_wheel_speed(0.), traction_right_wheel_speed(0.)
    {}
};

class DifferentialDriveState
{
public:
    Stg::usec_t timestamp_us;
    Stg::usec_t clock_us;
    double traction_left_distance_mm;
    double traction_right_distance_mm;
    double heading_angle_rad;
    double angular_velocity_rad_per_sec;
    double wheel_distance;

    DifferentialDriveState() : clock_us(0), wheel_distance(0.413), timestamp_us(0), traction_left_distance_mm(0.),
        traction_right_distance_mm(0.), heading_angle_rad(0.), angular_velocity_rad_per_sec(0.)
    {}
};


class TricycleDriveCommand{
public:
    Stg::usec_t timestamp_us;
    Stg::usec_t clock_us;
    double linear_velocity;
    double steering_angle;

    TricycleDriveCommand(): clock_us(0), timestamp_us(0), linear_velocity(0.), steering_angle(0.)
    {}
};

class TricycleDriveState
{
public:

    Stg::usec_t timestamp_us;
    Stg::usec_t clock_us;
    uint64_t traction_encoder;
    double traction_distance_mm;
    double steering_angle;
    double heading_angle_rad;
    double angular_velocity_rad_per_sec;
    int tick_per_meter;

    TricycleDriveState() : clock_us(0), timestamp_us(0), traction_encoder(0), traction_distance_mm(0.), steering_angle(0.),
        heading_angle_rad(0.), angular_velocity_rad_per_sec(0.), tick_per_meter(1000)
    {}
};


class LidarState{
public:
    Stg::usec_t timestamp_us;
    Stg::usec_t clock_us;
    //Stg::Pose pose;
    //Stg::Bounds range;
    double fov;
    //double angle_noise; //< variance for ranger angle
    //double range_noise; //< variance for range readings
    //double range_noise_const; //< variance for constant noise (not depending on range)
    unsigned int sample_count;

    std::vector<double> ranges;
    std::vector<double> intensities;
    std::vector<double> bearings;

    LidarState():
    //pose(0, 0, 0, 0), range(0.0, 5.0),
    fov(0.1), clock_us(0), timestamp_us(0),
    sample_count(1), ranges(), intensities(), bearings()
    {}
};

class CameraState{
public:
    Stg::usec_t timestamp_us;
    Stg::usec_t clock_us;
    std::vector<float> depth_data;
    std::vector<unsigned char> rgb_data;
    int camera_width = 0;
    int camera_height = 0;

    bool enabled;

    CameraState():
            clock_us(0), depth_data(),rgb_data(),camera_width(0),camera_height(0), timestamp_us(0), enabled(true)
    {}
};


class FiducialState{
public:
    Stg::usec_t timestamp_us;
    Stg::usec_t clock_us;
    bool detected;
    double bearing;
    double range;
    double heading;


    FiducialState():
            clock_us(0), timestamp_us(0), detected(false),bearing(0.), range(0.), heading(0.)
    {}
};



class Robot {
public:
  Stg::ModelPosition *position;
  Stg::ModelRanger *ranger;
  Stg::ModelCamera *camera;
  Stg::ModelFiducial *fiducial;
  Stg::ModelBumper *bumpers;
};

class Logic {
  Logic() {}
  Logic(const Logic &) {}
public:
  static int Callback(Stg::World *world, void *userarg)
  {
    Logic *lg = reinterpret_cast<Logic *>(userarg);

    lg->Tick(world);

    // never remove this call-back
    return 0;
  }

    static int MoveCallback(Stg::World *world, void *userarg)
    {

        PRINT_ERR("In move Callback\n");
        return 0;
    }


    explicit Logic(unsigned int popsize, std::string world_file):
    population_size(popsize), robots(new Robot[population_size]), buffer_motor_command_us(200000), world_file(world_file)
  {
  }

  Stg::Velocity get_odometry_data(){
    return velocity;
  }

   Stg::ModelCamera *  get_camera(){
      return robots[0].camera;
  }

  void connect(Stg::World *world){
        int idx = 0;
        // the robots' models are named r0 .. r1999
        std::stringstream name;
        name << "r" << idx;

        // get the robot's model and subscribe to it
        Stg::ModelPosition *posmod =
          reinterpret_cast<Stg::ModelPosition *>(world->GetModel(name.str()));
        assert(posmod != 0);


        drive_mode = posmod->GetDriveMode();
        differential_drive_robot_state.wheel_distance = posmod->wheeldistance;
        robots[idx].position = posmod;
        robots[idx].position->Subscribe();
        robots[idx].fiducial = (Stg::ModelFiducial *)robots[idx].position->GetUnusedModelOfType("fiducial");
        robots[idx].fiducial ->AddCallback(Stg::Model::CB_UPDATE, (Stg::model_callback_t)FiducialUpdate, &fiducial_state);
        robots[idx].fiducial ->Subscribe();
        
        robots[idx].bumpers = reinterpret_cast<Stg::ModelBumper *>(robots[idx].position->GetChild("bumper:0"));
          if (robots[idx].bumpers){
              robots[idx].bumpers->Subscribe();
          }
        
        // get the robot's camera model and subscribe to it
        Stg::ModelCamera *cammod =
                reinterpret_cast<Stg::ModelCamera *>(robots[idx].position->GetChild("camera:0"));
        robots[idx].camera = cammod;
        if (cammod){
            camera_state.camera_width = robots[idx].camera->getWidth();
            camera_state.camera_height = robots[idx].camera->getHeight();
            robots[idx].camera->Subscribe();
            _camera_enable = true;
            //robots[idx].camera->enable_camera(false);
        }
        // get the robot's ranger model and subscribe to it
        Stg::ModelRanger *rngmod =
          reinterpret_cast<Stg::ModelRanger *>(robots[idx].position->GetChild("ranger:0"));
        assert(rngmod != 0);
        robots[idx].ranger = rngmod;
        robots[idx].ranger->Subscribe();
        
        Stg::WorldGui *world_gui = dynamic_cast<Stg::WorldGui *>(world);
        
        // register with the world
        world->AddUpdateCallback(Logic::Callback, reinterpret_cast<void *>(this));
  }

    static int FiducialUpdate(Stg::ModelFiducial *mod, FiducialState *fiducial)
    {
        std::vector<Stg::ModelFiducial::Fiducial> &fids = mod->GetFiducials();

        fiducial->detected = false;
        for (unsigned int i = 0; i < fids.size(); i++) {
            // printf( "fiducial %d is %d at %.2f m %.2f radians\n",
            //	  i, f->id, f->range, f->bearing );

            if (fids[i].id == 9) // I see a charging station
            {
                fiducial->detected = true;
                fiducial->range = fids[i].range;
                fiducial->heading = fids[i].geom.a;
                fiducial->bearing = fids[i].bearing;
                break;
            }
        }

        return 0; // run again
    }

  ~Logic() { delete[] robots; }
  void Tick(Stg::World * world) {
      bool debug = false;
      Stg::WorldGui *world_gui = dynamic_cast<Stg::WorldGui *>(world);


      //Read status of the robot
      velocity = robots[0].position->GetVelocity();
      state = robots[0].position->GetState();

      odom_pose = robots[0].position->GetPose();
      global_pose = robots[0].position->GetGlobalPose();

      // Filling scan state
      lidar_state.timestamp_us = robots[0].ranger->last_update;
      //lidar_state.clock_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      lidar_state.clock_us = robots[0].ranger->last_clock_update;
      lidar_state.ranges = robots[0].ranger->GetSensors()[0].ranges;
      lidar_state.intensities = robots[0].ranger->GetSensors()[0].intensities;
      lidar_state.bearings = robots[0].ranger->GetSensors()[0].bearings;
      lidar_state.fov = robots[0].ranger->GetSensors()[0].fov;
      lidar_state.sample_count = robots[0].ranger->GetSensors()[0].sample_count;

      if (_camera_enable != camera_state.enabled && robots[0].camera != NULL){
          _camera_enable = camera_state.enabled;
          robots[0].camera->enable_camera(_camera_enable);
      }
      // Filling Camera state
      if (camera_state.enabled && robots[0].camera != NULL && robots[0].camera->last_update > camera_state.timestamp_us) {

          float *depth_data_camera = (float *) robots[0].camera->FrameDepth();
          int size = robots[0].camera->getWidth() * robots[0].camera->getHeight();
          if (size && depth_data_camera) {
              camera_state.depth_data.resize(size);
              std::copy(depth_data_camera, depth_data_camera + size, camera_state.depth_data.begin());
          }
          camera_state.timestamp_us = robots[0].camera->last_update;
          //camera_state.clock_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
          camera_state.clock_us = robots[0].camera->last_clock_update;
          unsigned char *rgb_data_camera = (unsigned char *) robots[0].camera->FrameColor();
          if (size && rgb_data_camera) {
              camera_state.rgb_data.resize(size * 4);
              std::copy(rgb_data_camera, rgb_data_camera + size * 4, camera_state.rgb_data.begin());
          }
          camera_state.timestamp_us = robots[0].camera->last_update;
      }


      const double interval((double)world->sim_interval / 1e6);
      if (drive_mode==Stg::ModelPosition::DRIVE_DIFFERENTIAL){
          // Filling robot state
          differential_drive_robot_state.timestamp_us = world->SimTimeNow();
          //differential_drive_robot_state.clock_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
          differential_drive_robot_state.clock_us = robots[0].position->last_clock_update;

          double left_velocity = velocity.x - 0.5*(velocity.a * differential_drive_robot_state.wheel_distance);
          double right_velocity = velocity.x + 0.5*(velocity.a * differential_drive_robot_state.wheel_distance);
          differential_drive_robot_state.traction_left_distance_mm += (left_velocity * interval) * 1e3;
          differential_drive_robot_state.traction_right_distance_mm += (right_velocity * interval) * 1e3;

          differential_drive_robot_state.angular_velocity_rad_per_sec = velocity.a;
          differential_drive_robot_state.heading_angle_rad += velocity.a * interval;

          // Sending motor command
          double brainos_forward_speed = world_gui->linear * 1.5;
          double brainos_angular_speed = -world_gui->angular * 1.;

          if (world->SimTimeNow() > buffer_motor_command_us && differential_drive_command.timestamp_us > world->SimTimeNow() - buffer_motor_command_us){
              brainos_forward_speed = (differential_drive_command.traction_left_wheel_speed + differential_drive_command.traction_right_wheel_speed) / 2.;
              brainos_angular_speed = (differential_drive_command.traction_right_wheel_speed - differential_drive_command.traction_left_wheel_speed) / differential_drive_robot_state.wheel_distance;
          }
          robots[0].position->SetSpeed(brainos_forward_speed, 0., brainos_angular_speed);
      }else if(drive_mode==Stg::ModelPosition::DRIVE_CAR){
          // Filling robot state
          tricycle_drive_robot_state.timestamp_us = world->SimTimeNow();
          //tricycle_drive_robot_state.clock_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
          tricycle_drive_robot_state.clock_us = robots[0].position->last_clock_update;
          tricycle_drive_robot_state.angular_velocity_rad_per_sec = velocity.a;
          tricycle_drive_robot_state.heading_angle_rad += velocity.a * interval;

          tricycle_drive_robot_state.steering_angle = state.a;
          tricycle_drive_robot_state.traction_encoder += state.x * interval * tricycle_drive_robot_state.tick_per_meter;
          tricycle_drive_robot_state.traction_distance_mm += state.x * interval * 1e3;

          // Sending motor command
          double brainos_traction_speed_command = world_gui->linear * 1.5; //+ abs(world_gui->angular);
          double brainos_steering_angle_command = -world_gui->angular * 1.57;
          //if(world_gui->linear>0) brainos_steering_angle_command /= 2.;

          if (world->SimTimeNow() > buffer_motor_command_us && tricycle_drive_command.timestamp_us > world->SimTimeNow() - buffer_motor_command_us){
              brainos_traction_speed_command = tricycle_drive_command.linear_velocity;
              brainos_steering_angle_command = tricycle_drive_command.steering_angle ;
          }
          robots[0].position->SetSpeed(brainos_traction_speed_command, 0., brainos_steering_angle_command);
      }
  }

protected:
  unsigned int population_size;
  Robot *robots;

  Stg::ModelRanger *rgr;
  int buffer_motor_command_us = 200000; // Buffer of 200 ms
  bool _camera_enable;


public:
    DifferentialDriveState differential_drive_robot_state;
    DifferentialDriveCommand differential_drive_command;

    TricycleDriveState tricycle_drive_robot_state;
    TricycleDriveCommand tricycle_drive_command;

    LidarState lidar_state;
    CameraState camera_state;
    FiducialState fiducial_state;
    std::string world_file;

    Stg::Pose odom_pose;
    Stg::Pose global_pose;
    Stg::Velocity velocity;
    Stg::Pose state;

    Stg::ModelPosition::DriveMode drive_mode;

};



struct StageSimulator
{
    StageSimulator(std::string world_file): world_file(world_file) {

        // initialize libstage
        int number_of_arguments = 3;
        char* parameters[3];
        parameters[0] = "stage";
        parameters[1] = (char *) world_file.c_str();
        parameters[2] = "1";
        char ** cast_parameters = parameters;
        int popsize = 1;
        pthread_t cThread;
        Stg::Init(&number_of_arguments, (char***)&cast_parameters);

        // create the world
        world = new Stg::WorldGui(800, 700, "Stage Benchmark Program");
        timestamp_us = world->SimTimeNow();

        // normal posix pthread C function pointer
        typedef void *(*func_ptr)(void *);

        logic = new Logic(1, world_file);

        std::pair<Stg::WorldGui *, Logic * > * params = new std::pair<Stg::WorldGui *, Logic * >(world, logic);
        pthread_create(&cThread, NULL,  (func_ptr)StageSimulator::run, params);
        //sleep(1);

    } // added constructor

    static void run(std::pair<Stg::WorldGui *, Logic * > * params){

        Stg::WorldGui * world = params->first;
        Logic * logic = params->second;

        // create the world
        world->Load(logic->world_file.c_str());

        // create the logic and connect it to the world
        logic->connect(world);
        world->Run();
    }


    py::dict get_odom(){
        py::dict odom;
        Stg::Velocity velocity = logic->velocity;
        Stg::Pose odom_pose = logic->odom_pose;
        Stg::Pose global_pose = logic->global_pose;

        odom["velocity"]=py::make_tuple(velocity.x, velocity.y, velocity.a);
        odom["pose"]=py::make_tuple(odom_pose.x, odom_pose.y, odom_pose.a);
        odom["global_pose"]=py::make_tuple(global_pose.x, global_pose.y, global_pose.z);

        return odom;
    }

    py::dict get_scan_data() {
        py::dict scan_data_dict;
        std::vector<double> ranges = logic->lidar_state.ranges;
        std::vector<double> intensities = logic->lidar_state.intensities;
        std::vector<double> bearings = logic->lidar_state.bearings;
        scan_data_dict["ranges"] = NULL;
        scan_data_dict["intensities"] = NULL;
        scan_data_dict["bearings"] = NULL;

        if (ranges.size() > 0){
            py::safe_array<double> np_ranges = py::cast(ranges);
            scan_data_dict["ranges"] = np_ranges;
        }
        if (intensities.size() > 0){
            py::safe_array<double> np_intensities = py::cast(intensities);
            scan_data_dict["intensities"] = np_intensities;
        }
        if (bearings.size() > 0){
            py::safe_array<double> np_bearings = py::cast(bearings);
            scan_data_dict["bearings"] = np_bearings;
        }
        scan_data_dict["fov"] = logic->lidar_state.fov;
        scan_data_dict["timestamp_us"] = logic->lidar_state.timestamp_us;
        scan_data_dict["clock_us"] = logic->lidar_state.clock_us;
        return scan_data_dict;
    }


    py::dict get_depth_data(){
        py::dict depth_data_dict;
        std::vector<float> depth =  logic->camera_state.depth_data;
        depth_data_dict["timestamp_us"] = logic->camera_state.timestamp_us;
        depth_data_dict["clock_us"] = logic->camera_state.clock_us;
        depth_data_dict["width"] = logic->camera_state.camera_width;
        depth_data_dict["height"] = logic->camera_state.camera_height;
        depth_data_dict["data"] = NULL;
        if (depth.size() > 0){
            py::safe_array<float> np_depth = py::cast(depth);
            depth_data_dict["data"] = np_depth;
        }

        return depth_data_dict;
    }

    py::dict get_rgb_data(){
        py::dict rgb_data_dict;
        std::vector<unsigned char> rgb_data =  logic->camera_state.rgb_data;
        rgb_data_dict["timestamp_us"] = logic->camera_state.timestamp_us;
        rgb_data_dict["clock_us"] = logic->camera_state.clock_us;
        rgb_data_dict["width"] = logic->camera_state.camera_width;
        rgb_data_dict["height"] = logic->camera_state.camera_height;
        rgb_data_dict["data"] = NULL;
        if (rgb_data.size() > 0) {
            py::safe_array<unsigned char> np_rgb_data = py::cast(rgb_data);
            rgb_data_dict["data"] = np_rgb_data;
        }
        return rgb_data_dict;
    }

    py::dict get_robot_state( )
    {
        py::dict robot_state_dict;

        robot_state_dict["timestamp_us"] = logic->differential_drive_robot_state.timestamp_us;
        robot_state_dict["clock_us"] = logic->differential_drive_robot_state.clock_us;
        robot_state_dict["traction_left_distance_mm"] = logic->differential_drive_robot_state.traction_left_distance_mm;
        robot_state_dict["traction_right_distance_mm"] = logic->differential_drive_robot_state.traction_right_distance_mm;
        robot_state_dict["heading_angle_rad"] = logic->differential_drive_robot_state.heading_angle_rad;
        robot_state_dict["angular_velocity_rad_per_sec"] =  logic->differential_drive_robot_state.angular_velocity_rad_per_sec;
        robot_state_dict["wheel_distance"] =  logic->differential_drive_robot_state.wheel_distance;
        return robot_state_dict;
    }

    py::dict get_home_marker( )
    {
        py::dict home_marker_dict;

        home_marker_dict["timestamp_us"] = logic->fiducial_state.timestamp_us;
        home_marker_dict["detected"] = logic->fiducial_state.detected;
        home_marker_dict["heading"] = logic->fiducial_state.heading;
        home_marker_dict["range"] = logic->fiducial_state.range;
        home_marker_dict["bearing"] = logic->fiducial_state.bearing;
        return home_marker_dict;
    }

    Stg::usec_t get_robot_state_timestamp_us(){return logic->differential_drive_robot_state.timestamp_us;}
    Stg::usec_t get_depth_data_timestamp_us(){return logic->camera_state.timestamp_us;}
    Stg::usec_t get_rgb_data_timestamp_us(){return logic->camera_state.timestamp_us;}
    Stg::usec_t get_scan_data_timestamp_us(){return logic->lidar_state.timestamp_us;}

    int get_camera_width(){
        return logic->camera_state.camera_width;
    }

    int get_camera_height(){
        return logic->camera_state.camera_height;
    }

    void send_command(double traction_left_wheel_speed, double traction_right_wheel_speed) {
        logic->differential_drive_command.timestamp_us = world->SimTimeNow();
        logic->differential_drive_command.traction_left_wheel_speed = traction_left_wheel_speed;
        logic->differential_drive_command.traction_right_wheel_speed = traction_right_wheel_speed;
    }

    bool step_simulation_async(int number_of_ms) {

        if (world->has_more_step_to_run()) return false;

        int sim_interval_ms = world->sim_interval / 1000;
        int num_steps = number_of_ms /  sim_interval_ms;
        if (number_of_ms % sim_interval_ms != 0){
            num_steps ++;
            PRINT_ERR2("%d ms is not a fraction of sim interval %d", number_of_ms, sim_interval_ms);
            PRINT_ERR2("Step simulation of %d steps (%d ms)", num_steps, num_steps*sim_interval_ms);
        }
        world->UnpauseForNumSteps(num_steps);
        return true;
    }

    bool step_simulation_sync(int number_of_ms, bool wait_after_trigger) {

        while (world->has_more_step_to_run()){usleep(1000);} // Make sure the previous step is done

        int sim_interval_ms = world->sim_interval / 1000;
        int num_steps = number_of_ms /  sim_interval_ms;
        if (number_of_ms % sim_interval_ms != 0){
            num_steps ++;
            PRINT_ERR2("%d ms is not a fraction of sim interval %d", number_of_ms, sim_interval_ms);
            PRINT_ERR2("Step simulation of %d steps (%d ms)", num_steps, num_steps*sim_interval_ms);
        }
        world->UnpauseForNumSteps(num_steps);
        if (wait_after_trigger) {
            while (world->has_more_step_to_run()) { usleep(1000); } // Make sure this step is done
        }
        return true;
    }

    bool has_simulation_stepped() {
        return not world->has_more_step_to_run();
    }

    void release_simulation(){
        world->Unlock();
    }

    void lock_simulation(){
        world->Lock();
    }

    void start_simulation(){
        world->Start();
    }

    void stop_simulation(){
        world->Stop();
    }

    Stg::usec_t get_timestamp_us(){
        timestamp_us = world->SimTimeNow();
        return timestamp_us;
    }

    void enable_camera(){
        logic->camera_state.enabled = true;
    }

    void disable_camera(){
        logic->camera_state.enabled = false;
    }


    std::string world_file;
    Stg::WorldGui * world;
    Logic * logic;
    Stg::usec_t timestamp_us;
};



/**
 * @brief pybind module
 * @details pybind module for stage simulator
 *
 */
PYBIND11_MODULE(stagesim, m)
{
    m.doc() = "Python wrapper for Stage Simulator";
    py::class_<StageSimulator>(m, "StageSimulator")
        .def(py::init<std::string>(), "config_filename"_a)
        .def("get_odom", &StageSimulator::get_odom)
        .def("get_scan_data", &StageSimulator::get_scan_data)
        .def("get_scan_data_timestamp_us", &StageSimulator::get_scan_data_timestamp_us)

        .def("get_camera_width", &StageSimulator::get_camera_width)
        .def("get_camera_height", &StageSimulator::get_camera_height)

        .def("enable_camera", &StageSimulator::enable_camera)
        .def("disable_camera", &StageSimulator::disable_camera)
        .def("get_depth_data", &StageSimulator::get_depth_data)
        .def("get_depth_data_timestamp_us", &StageSimulator::get_depth_data_timestamp_us)

        .def("get_rgb_data", &StageSimulator::get_rgb_data)
        .def("get_rgb_data_timestamp_us", &StageSimulator::get_rgb_data_timestamp_us)

        .def("get_robot_state", &StageSimulator::get_robot_state)
        .def("get_robot_state_timestamp_us", &StageSimulator::get_robot_state_timestamp_us)

        .def("send_command", &StageSimulator::send_command)
        .def("step_simulation_async", &StageSimulator::step_simulation_async)
        .def("step_simulation_sync", &StageSimulator::step_simulation_sync)
        .def("has_simulation_stepped", &StageSimulator::has_simulation_stepped)
        .def("release_simulation", &StageSimulator::release_simulation)
        .def("lock_simulation", &StageSimulator::lock_simulation)
        .def("start_simulation", &StageSimulator::start_simulation)
        .def("stop_simulation", &StageSimulator::stop_simulation)
        .def("get_timestamp_us", &StageSimulator::get_timestamp_us)
        .def("get_home_marker", &StageSimulator::get_home_marker);
}
