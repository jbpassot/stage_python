/////////////////////////////////
// File: stest.c
// Desc: Stage library test program
// Created: 2004.9.15
// Author: Richard Vaughan <vaughan@sfu.ca>
// CVS: $Id: stest.cc,v 1.1 2008-01-15 01:29:10 rtv Exp $
// License: GPL
/////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <sstream>
#include <iostream>
#include <cmath>

#include "stage.hh"
#include <boost/python.hpp>
#include <numpy/ndarrayobject.h>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
using namespace boost::python;

typedef std::vector<double> DoubleVectorType;
typedef std::vector<int> IntVectorType;


class DifferentialDriveCommand{
public:
    Stg::usec_t timestamp_us;
    double traction_left_wheel_speed;
    double traction_right_wheel_speed;

    DifferentialDriveCommand(): timestamp_us(0), traction_left_wheel_speed(0.), traction_right_wheel_speed(0.)
    {}
};

class DifferentialDriveState
{
public:
    Stg::usec_t timestamp_us;
    double traction_left_distance_mm;
    double traction_right_distance_mm;
    double heading_angle_rad;
    double angular_velocity_rad_per_sec;
    double wheel_distance;

    DifferentialDriveState() : wheel_distance(0.413), timestamp_us(0), traction_left_distance_mm(0.),
        traction_right_distance_mm(0.), heading_angle_rad(0.), angular_velocity_rad_per_sec(0.)
    {}
};

class LidarState{
public:
    Stg::usec_t timestamp_us;
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
    fov(0.1), timestamp_us(0),
    sample_count(1), ranges(), intensities(), bearings()
    {}
};

class CameraState{
public:
    Stg::usec_t timestamp_us;
    std::vector<float> depth_data;
    std::vector<float> rgb_data;
    int camera_width = 0;
    int camera_height = 0;

    CameraState():
        depth_data(),rgb_data(),camera_width(0),camera_height(0), timestamp_us(0)
    {}
};

boost::python::object stdDoubleVecToNumpyArray(std::vector<double> const& vec )
{
   npy_intp size = vec.size();

    /* const_cast is rather horrible but we need a writable pointer
       in C++11, vec.data() will do the trick
       but you will still need to const_cast
     */

    double * data = size ? const_cast<double *>(&vec[0])
                         : static_cast<double *>(NULL);

    // create a PyObject * from pointer and data

    npy_intp dims[1];
    dims[0] = size;

    PyObject * pyObj = PyArray_SimpleNewFromData( 1, dims, NPY_DOUBLE, data );

    boost::python::handle<> handle( pyObj );

    boost::python::numeric::array arr( handle );

    /* The problem of returning arr is twofold: firstly the user can modify
      the data which will betray the const-correctness
      Secondly the lifetime of the data is managed by the C++ API and not the
      lifetime of the numpy array whatsoever. But we have a simple solution..
     */

    return arr.copy(); // copy the object. numpy owns the copy now.
}

boost::python::object stdFloatVecToNumpyArray( std::vector<float> const& vec )
{
    npy_intp size = vec.size();

    /* const_cast is rather horrible but we need a writable pointer
       in C++11, vec.data() will do the trick
       but you will still need to const_cast
     */

    float * data = size ? const_cast<float *>(&vec[0])
                         : static_cast<float *>(NULL);

    // create a PyObject * from pointer and data

    npy_intp dims[1];
    dims[0] = size;

    PyObject * pyObj = PyArray_SimpleNewFromData( 1, dims, NPY_FLOAT, data );

    boost::python::handle<> handle( pyObj );

    boost::python::numeric::array arr( handle );

    /* The problem of returning arr is twofold: firstly the user can modify
      the data which will betray the const-correctness
      Secondly the lifetime of the data is managed by the C++ API and not the
      lifetime of the numpy array whatsoever. But we have a simple solution..
     */

    return arr.copy(); // copy the object. numpy owns the copy now.
}



class Robot {
public:
  Stg::ModelPosition *position;
  Stg::ModelRanger *ranger;
  Stg::ModelCamera *camera;
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


    explicit Logic(unsigned int popsize)
      : population_size(popsize), robots(new Robot[population_size]), last_timestamp_us_command(0)
  {
  }

  Stg::Velocity get_odometry_data(){
    return velocity;
  }

    void connect(Stg::World *world)
  {
    // connect the first population_size robots to this controller
    //for (unsigned int idx = 0; idx < population_size; idx++) {
        int idx = 0;
      // the robots' models are named r0 .. r1999
      std::stringstream name;
      name << "r" << idx;

      // get the robot's model and subscribe to it
      Stg::ModelPosition *posmod =
          reinterpret_cast<Stg::ModelPosition *>(world->GetModel(name.str()));
      assert(posmod != 0);


      robot_state.wheel_distance = posmod->wheeldistance;
      robots[idx].position = posmod;
      robots[idx].position->Subscribe();

        // get the robot's camera model and subscribe to it
        Stg::ModelCamera *cammod =
                reinterpret_cast<Stg::ModelCamera *>(robots[idx].position->GetChild("camera:0"));
        robots[idx].camera = cammod;


        if (cammod)
            camera_state.camera_width = robots[idx].camera->getWidth();
            camera_state.camera_height = robots[idx].camera->getHeight();
            robots[idx].camera->Subscribe();

                // get the robot's ranger model and subscribe to it
      Stg::ModelRanger *rngmod =
          reinterpret_cast<Stg::ModelRanger *>(robots[idx].position->GetChild("ranger:1"));
      assert(rngmod != 0);
      robots[idx].ranger = rngmod;
      robots[idx].ranger->Subscribe();
    //}
      Stg::WorldGui *world_gui = dynamic_cast<Stg::WorldGui *>(world);

    world_gui->AddMoveCallback(Logic::MoveCallback, reinterpret_cast<void *>(this));
    // register with the world
    world->AddUpdateCallback(Logic::Callback, reinterpret_cast<void *>(this));
  }


  ~Logic() { delete[] robots; }
  void Tick(Stg::World * world) {
      bool debug = false;

      //Step 1. Read status of the robot
      velocity = robots[0].position->GetVelocity();

      if (debug){
          PRINT_ERR1("%s", world->ClockString().c_str());
          PRINT_ERR2("Robot Speed (%f, %f)", velocity.x, velocity.a);
      }

      // Filling scan state
      lidar_state.timestamp_us = robots[0].ranger->last_update;
      lidar_state.ranges = robots[0].ranger->GetSensors()[0].ranges;
      lidar_state.intensities = robots[0].ranger->GetSensors()[0].intensities;
      lidar_state.bearings = robots[0].ranger->GetSensors()[0].bearings;
      lidar_state.fov = robots[0].ranger->GetSensors()[0].fov;
      lidar_state.sample_count = robots[0].ranger->GetSensors()[0].sample_count;

      // Filling robot state
      const double interval((double)world->sim_interval / 1e6);
      robot_state.timestamp_us = world->SimTimeNow();
      robot_state.traction_left_distance_mm += robot_state.wheel_distance* (2*velocity.x + velocity.a) / 2.;
      robot_state.traction_right_distance_mm += robot_state.wheel_distance* (2*velocity.x - velocity.a) / 2.;
      robot_state.angular_velocity_rad_per_sec = velocity.a;
      robot_state.heading_angle_rad += velocity.a * interval;

      // Filling Camera state
      if (robots[0].camera != NULL) {
          float *depth_data_camera = (float *) robots[0].camera->FrameDepth();
          float *rgb_data_camera = (float *) robots[0].camera->FrameColor();
          int size = robots[0].camera->getWidth() * robots[0].camera->getHeight();
          if (size && depth_data_camera) {
              camera_state.depth_data.resize(size);
              std::copy(depth_data_camera, depth_data_camera + size, camera_state.depth_data.begin());
          }
          camera_state.timestamp_us = robots[0].camera->last_update;
      }

      Stg::WorldGui *world_gui = dynamic_cast<Stg::WorldGui *>(world);

      double brainos_forward_speed = world_gui->linear * 1.5;
      double brainos_angular_speed = -world_gui->angular * 0.5;

      if (drive_command.timestamp_us > last_timestamp_us_command){
          last_timestamp_us_command = drive_command.timestamp_us;
          brainos_forward_speed = (drive_command.traction_left_wheel_speed + drive_command.traction_right_wheel_speed) / 2.;
          brainos_angular_speed = (drive_command.traction_right_wheel_speed - drive_command.traction_left_wheel_speed) / robot_state.wheel_distance;
      }
      robots[0].position->SetSpeed(brainos_forward_speed, 0., brainos_angular_speed);
      //world_gui->Redraw();
  }

protected:
  unsigned int population_size;
  Robot *robots;

    Stg::Velocity velocity;
    Stg::ModelRanger *rgr;
    Stg::usec_t last_timestamp_us_command;


public:
    DifferentialDriveState robot_state;
    DifferentialDriveCommand drive_command;
    LidarState lidar_state;
    CameraState camera_state;
};



struct StageSimulator
{
    StageSimulator(std::string world_file): world_file(world_file) {

        // initialize libstage
        int argc = 3;
        char* argv[3];
        argv[0] = "stage";
        argv[1] = (char *) world_file.c_str();
        argv[2] = "1";
        int popsize = 1;
        char ** a = argv;
        pthread_t cThread;

        Stg::Init(&argc, &a);

        // create the world
        world = new Stg::WorldGui(800, 700, "Stage Benchmark Program");

        timestamp_us = world->SimTimeNow();

        // normal posix pthread C function pointer
        typedef void *(*func_ptr)(void *);

        logic = new Logic(1);

        std::pair<Stg::WorldGui *, Logic * > * params = new std::pair<Stg::WorldGui *, Logic * >(world, logic);
        pthread_create(&cThread, NULL,  (func_ptr)StageSimulator::run, params);
        //sleep(1);

    } // added constructor
    //static void run(void * world_param){
        //Stg::WorldGui * world = reinterpret_cast<Stg::WorldGui *>(world_param);
       //static void run(Stg::WorldGui * world){
       static void run(std::pair<Stg::WorldGui *, Logic * > * params){

        Stg::WorldGui * world = params->first;
        Logic * logic = params->second;

        // create the world
        //world = new Stg::WorldGui(800, 700, "Stage Benchmark Program");
        world->Load("/home/jb/projects/stage4/Stage/worlds/benchmark/hospital_2.world");

        // create the logic and connect it to the world
        //logic = new Logic(1);
        logic->connect(world);
        world->Run();
    }


    DoubleVectorType get_odom(){
        Stg::Velocity velocity = logic->get_odometry_data();
        DoubleVectorType odom;
        odom.push_back(velocity.x);
        odom.push_back(velocity.y);
        odom.push_back(velocity.a);
        return odom;
    }

    boost::python::object get_scan_data() {
        boost::python::dict scan_data_dict;
        std::vector<double> ranges = logic->lidar_state.ranges;
        std::vector<double> intensities = logic->lidar_state.intensities;
        std::vector<double> bearings = logic->lidar_state.bearings;
        scan_data_dict["ranges"] = NULL;
        scan_data_dict["intensities"] = NULL;
        scan_data_dict["bearings"] = NULL;
        if (ranges.size() > 0) scan_data_dict["ranges"] = stdDoubleVecToNumpyArray(ranges);
        if (intensities.size() > 0) scan_data_dict["intensities"] = stdDoubleVecToNumpyArray(intensities);
        if (bearings.size() > 0) scan_data_dict["bearings"] = stdDoubleVecToNumpyArray(bearings);
        scan_data_dict["fov"] = logic->lidar_state.fov;
        scan_data_dict["timestamp_us"] = logic->lidar_state.timestamp_us;
        return scan_data_dict;
    }


    boost::python::object get_depth_data(){
        boost::python::dict depth_data_dict;
        std::vector<float> depth =  logic->camera_state.depth_data;
        depth_data_dict["timestamp_us"] = logic->camera_state.timestamp_us;
        depth_data_dict["width"] = logic->camera_state.camera_width;
        depth_data_dict["height"] = logic->camera_state.camera_height;
        depth_data_dict["data"] = NULL;
        if (depth.size() > 0) depth_data_dict["data"] = stdFloatVecToNumpyArray(depth);

        return depth_data_dict;
    }

    boost::python::dict get_robot_state( )
    {
        boost::python::dict robot_state_dict;

        robot_state_dict["timestamp_us"] = logic->robot_state.timestamp_us;
        robot_state_dict["traction_left_distance_mm"] = logic->robot_state.traction_left_distance_mm;
        robot_state_dict["traction_right_distance_mm"] = logic->robot_state.traction_right_distance_mm;
        robot_state_dict["heading_angle_rad"] = logic->robot_state.heading_angle_rad;
        robot_state_dict["angular_velocity_rad_per_sec"] =  logic->robot_state.angular_velocity_rad_per_sec;
        robot_state_dict["wheel_distance"] =  logic->robot_state.wheel_distance;
        return robot_state_dict;
    }

    int get_camera_width(){
        return logic->camera_state.camera_width;
    }

    int get_camera_height(){
        return logic->camera_state.camera_height;
    }

    void send_command(double traction_left_wheel_speed, double traction_right_wheel_speed) {
        logic->drive_command.timestamp_us = world->SimTimeNow();
        logic->drive_command.traction_left_wheel_speed = traction_left_wheel_speed;
        logic->drive_command.traction_right_wheel_speed = traction_right_wheel_speed;
    }

    bool step_simulation(int number_of_ms) {

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

    void release_simulation(){
        world->Unlock();
    }

    void lock_simulation(){
        world->Lock();
    }

    std::string world_file;
    Stg::WorldGui * world;
    Logic * logic;
    Stg::usec_t timestamp_us;
};




BOOST_PYTHON_MODULE(stagesim)
{

    Py_Initialize();
    import_array()
    numeric::array::set_module_and_type("numpy", "ndarray");

    class_<DoubleVectorType>("DoubleVectorType")
            .def(vector_indexing_suite<DoubleVectorType>() );


    class_<IntVectorType>("IntVectorType")
            .def(vector_indexing_suite<IntVectorType>() );

    class_<StageSimulator>("StageSimulator", init<std::string>())
            .def("run", &StageSimulator::run)
            .def("get_odom", &StageSimulator::get_odom)
            .def("get_scan_data", &StageSimulator::get_scan_data)
            .def("get_depth_data", &StageSimulator::get_depth_data)
            .def("get_camera_width", &StageSimulator::get_camera_width)
            .def("get_camera_height", &StageSimulator::get_camera_height)
            .def("get_robot_state", &StageSimulator::get_robot_state)
            .def("send_command", &StageSimulator::send_command)
            .def("step_simulation", &StageSimulator::step_simulation)
            .def("release_simulation", &StageSimulator::release_simulation)
            .def("lock_simulation", &StageSimulator::lock_simulation)
            ;
}

int main(int argc, char *argv[])
{
  // check and handle the argumets
  if (argc < 3) {
    puts("Usage: stest <worldfile> <number of robots>");
    exit(0);
  }

  const int popsize = atoi(argv[2]);

  // initialize libstage
  Stg::Init(&argc, &argv);

  // create the world
  // Stg::World world;
  Stg::WorldGui world(800, 700, "Stage Benchmark Program");
  world.Load(argv[1]);

  // create the logic and connect it to the world
  Logic logic(popsize);
  logic.connect(&world);

  // and then run the simulation
  world.Run();

  return 0;
}
